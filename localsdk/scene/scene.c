/*
 * scene.c — ISP scene parameter loader and day/night switcher.
 *
 * Replaces libsceneauto.so which cannot be used in non-interactive mode
 * (it reads a "video mode index" from stdin and fails when stdin is not a tty).
 *
 * Parses the vendor sensor.ini scene files at startup and applies ISP
 * parameters via HI_MPI_ISP_Set* calls. On day/night transitions, switches
 * between the two profiles without reinitialising the ISP pipeline.
 *
 * Sections handled from each scene INI file:
 *   [static_ae]        — exposure interval, route-ex flag, gain cap, speed/tolerance/delay
 *   [static_aerouteex] — AE route EX node table (IntTime, AGain, DGain, IspDGain)
 *   [static_ccm]       — CCM op-type, manual matrix, auto color-temp / matrix tables
 *   [static_saturation]— per-ISO saturation curve
 *   [static_nr]        — per-ISO NR fine strength and coring weight
 *   [static_ca]        — chromatic aberration enable flag
 */

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>

#include "hi_type.h"
#include "hi_comm_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"

#include "ini.h"
#include "logger.h"

#include "scene.h"

/* -------------------------------------------------------------------------
 * Internal types
 * ---------------------------------------------------------------------- */

typedef struct {
    /* [static_ae] */
    HI_U8   ae_run_interval;
    HI_BOOL ae_route_ex_valid;
    HI_U32  ae_sys_gain_max;
    HI_U8   ae_speed;
    HI_U8   ae_tolerance;
    HI_U16  ae_black_delay;
    HI_U16  ae_white_delay;

    /* [static_aerouteex] — number of nodes parsed from RouteEXIntTime */
    HI_U32  ae_route_node_num;
    HI_U32  ae_route_int_time[ISP_AE_ROUTE_EX_MAX_NODES];
    HI_U32  ae_route_again[ISP_AE_ROUTE_EX_MAX_NODES];
    HI_U32  ae_route_dgain[ISP_AE_ROUTE_EX_MAX_NODES];
    HI_U32  ae_route_isp_dgain[ISP_AE_ROUTE_EX_MAX_NODES];

    /* [static_ccm] */
    int     ccm_op_type;   /* 0 = auto, 1 = manual */
    HI_U16  ccm_manual[CCM_MATRIX_SIZE];
    HI_U16  ccm_tab_num;
    HI_U16  ccm_color_temp[CCM_MATRIX_NUM];
    HI_U16  ccm_auto[CCM_MATRIX_NUM][CCM_MATRIX_SIZE];

    /* [static_saturation] */
    HI_U8   sat[ISP_AUTO_ISO_STRENGTH_NUM];

    /* [static_nr] */
    HI_U8   nr_fine_str[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  nr_coring_wgt[ISP_AUTO_ISO_STRENGTH_NUM];

    /* [static_ca] */
    HI_BOOL ca_enable;
} scene_params_t;

static scene_params_t g_day;
static scene_params_t g_night;
static int g_initialized = 0;

/* -------------------------------------------------------------------------
 * Value parsing helpers
 *
 * inih strips whitespace around '=' but does NOT strip surrounding double
 * quotes.  INI values in this firmware are always "quoted", so strip_val()
 * removes the outer '"' before further parsing.
 *
 * inih guarantees that the value buffer may be modified by the handler,
 * so casting away const and using strtok() in-place is safe.
 * ---------------------------------------------------------------------- */

static const char *strip_val(const char *value)
{
    char *v = (char *)value;
    if (*v == '"') v++;
    size_t n = strlen(v);
    if (n > 0 && v[n - 1] == '"') v[n - 1] = '\0';
    return v;
}

static int parse_u32_arr(const char *s, HI_U32 *out, int max_n)
{
    char *buf = (char *)s;
    int n = 0;
    for (char *tok = strtok(buf, ","); tok && n < max_n; tok = strtok(NULL, ","))
        out[n++] = (HI_U32)strtoul(tok, NULL, 10);
    return n;
}

static int parse_u16_arr(const char *s, HI_U16 *out, int max_n)
{
    char *buf = (char *)s;
    int n = 0;
    for (char *tok = strtok(buf, ","); tok && n < max_n; tok = strtok(NULL, ","))
        out[n++] = (HI_U16)strtoul(tok, NULL, 10);
    return n;
}

static int parse_u8_arr(const char *s, HI_U8 *out, int max_n)
{
    char *buf = (char *)s;
    int n = 0;
    for (char *tok = strtok(buf, ","); tok && n < max_n; tok = strtok(NULL, ","))
        out[n++] = (HI_U8)strtoul(tok, NULL, 10);
    return n;
}

/* -------------------------------------------------------------------------
 * inih callback — populates one scene_params_t from a scene INI file
 * ---------------------------------------------------------------------- */

static int scene_handler(void *user, const char *section,
                         const char *name, const char *value)
{
    scene_params_t *p = (scene_params_t *)user;
    const char *v = strip_val(value);

    if (strcasecmp(section, "static_ae") == 0) {
        if      (strcasecmp(name, "AERunInterval") == 0)
            p->ae_run_interval   = (HI_U8)atoi(v);
        else if (strcasecmp(name, "AERouteExValid") == 0)
            p->ae_route_ex_valid = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "AutoSysGainMax") == 0)
            p->ae_sys_gain_max   = (HI_U32)strtoul(v, NULL, 10);
        else if (strcasecmp(name, "AutoSpeed") == 0)
            p->ae_speed          = (HI_U8)atoi(v);
        else if (strcasecmp(name, "AutoTolerance") == 0)
            p->ae_tolerance      = (HI_U8)atoi(v);
        else if (strcasecmp(name, "AutoBlackDelayFrame") == 0)
            p->ae_black_delay    = (HI_U16)atoi(v);
        else if (strcasecmp(name, "AutoWhiteDelayFrame") == 0)
            p->ae_white_delay    = (HI_U16)atoi(v);

    } else if (strcasecmp(section, "static_aerouteex") == 0) {
        if (strcasecmp(name, "RouteEXIntTime") == 0)
            p->ae_route_node_num = (HI_U32)parse_u32_arr(v, p->ae_route_int_time,
                                                          ISP_AE_ROUTE_EX_MAX_NODES);
        else if (strcasecmp(name, "RouteEXAGain") == 0)
            parse_u32_arr(v, p->ae_route_again,    ISP_AE_ROUTE_EX_MAX_NODES);
        else if (strcasecmp(name, "RouteEXDGain") == 0)
            parse_u32_arr(v, p->ae_route_dgain,    ISP_AE_ROUTE_EX_MAX_NODES);
        else if (strcasecmp(name, "RouteEXISPDGain") == 0)
            parse_u32_arr(v, p->ae_route_isp_dgain, ISP_AE_ROUTE_EX_MAX_NODES);

    } else if (strcasecmp(section, "static_ccm") == 0) {
        if (strcasecmp(name, "CCMOpType") == 0)
            p->ccm_op_type = atoi(v);
        else if (strcasecmp(name, "ManualCCMTable") == 0)
            parse_u16_arr(v, p->ccm_manual, CCM_MATRIX_SIZE);
        else if (strcasecmp(name, "TotalNum") == 0)
            p->ccm_tab_num = (HI_U16)atoi(v);
        else if (strcasecmp(name, "AutoColorTemp") == 0)
            parse_u16_arr(v, p->ccm_color_temp, CCM_MATRIX_NUM);
        else if (strncasecmp(name, "AutoCCMTable_", 13) == 0) {
            int idx = atoi(name + 13);
            if (idx >= 0 && idx < CCM_MATRIX_NUM)
                parse_u16_arr(v, p->ccm_auto[idx], CCM_MATRIX_SIZE);
        }

    } else if (strcasecmp(section, "static_saturation") == 0) {
        if (strcasecmp(name, "AutoSat") == 0)
            parse_u8_arr(v, p->sat, ISP_AUTO_ISO_STRENGTH_NUM);

    } else if (strcasecmp(section, "static_nr") == 0) {
        if (strcasecmp(name, "FineStr") == 0)
            parse_u8_arr(v, p->nr_fine_str, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "CoringWgt") == 0)
            parse_u16_arr(v, p->nr_coring_wgt, ISP_AUTO_ISO_STRENGTH_NUM);

    } else if (strcasecmp(section, "static_ca") == 0) {
        if (strcasecmp(name, "Enable") == 0)
            p->ca_enable = atoi(v) ? HI_TRUE : HI_FALSE;
    }

    return 1;
}

/* -------------------------------------------------------------------------
 * ISP apply helpers — Get current attr, patch our fields, Set back.
 * Using Get→patch→Set avoids clobbering fields we don't own.
 * ---------------------------------------------------------------------- */

static void apply_ae(const scene_params_t *p)
{
    ISP_EXPOSURE_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetExposureAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetExposureAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.u8AERunInterval                         = p->ae_run_interval;
    attr.bAERouteExValid                         = p->ae_route_ex_valid;
    attr.stAuto.stSysGainRange.u32Max            = p->ae_sys_gain_max;
    attr.stAuto.u8Speed                          = p->ae_speed;
    attr.stAuto.u8Tolerance                      = p->ae_tolerance;
    attr.stAuto.stAEDelayAttr.u16BlackDelayFrame = p->ae_black_delay;
    attr.stAuto.stAEDelayAttr.u16WhiteDelayFrame = p->ae_white_delay;

    ret = HI_MPI_ISP_SetExposureAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetExposureAttr failed 0x%x", (unsigned)ret);

    if (p->ae_route_ex_valid && p->ae_route_node_num > 0) {
        ISP_AE_ROUTE_EX_S route;
        memset(&route, 0, sizeof(route));
        route.u32TotalNum = p->ae_route_node_num;
        for (HI_U32 i = 0; i < p->ae_route_node_num && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
            route.astRouteExNode[i].u32IntTime  = p->ae_route_int_time[i];
            route.astRouteExNode[i].u32Again    = p->ae_route_again[i];
            route.astRouteExNode[i].u32Dgain    = p->ae_route_dgain[i];
            route.astRouteExNode[i].u32IspDgain = p->ae_route_isp_dgain[i];
        }
        ret = HI_MPI_ISP_SetAERouteAttrEx(0, &route);
        if (ret != HI_SUCCESS)
            LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetAERouteAttrEx failed 0x%x", (unsigned)ret);
    }
}

static void apply_ccm(const scene_params_t *p)
{
    ISP_COLORMATRIX_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetCCMAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetCCMAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.enOpType = (p->ccm_op_type == 0) ? OP_TYPE_AUTO : OP_TYPE_MANUAL;
    memcpy(attr.stManual.au16CCM, p->ccm_manual, sizeof(p->ccm_manual));

    /* Clamp tab_num to ISP accepted range [3, CCM_MATRIX_NUM]. */
    HI_U16 tab_num = p->ccm_tab_num;
    if (tab_num < 3) tab_num = 3;
    if (tab_num > CCM_MATRIX_NUM) tab_num = CCM_MATRIX_NUM;
    attr.stAuto.u16CCMTabNum = tab_num;

    for (int i = 0; i < tab_num; i++) {
        attr.stAuto.astCCMTab[i].u16ColorTemp = p->ccm_color_temp[i];
        memcpy(attr.stAuto.astCCMTab[i].au16CCM, p->ccm_auto[i],
               CCM_MATRIX_SIZE * sizeof(HI_U16));
    }

    ret = HI_MPI_ISP_SetCCMAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetCCMAttr op=%d failed 0x%x",
               p->ccm_op_type, (unsigned)ret);
}

/* Apply per-ISO saturation curve AND the CSC-level saturation together.
 *
 * The ISP has two independent saturation controls that multiply:
 *   1. ISP_SATURATION_ATTR_S.stAuto.au8Sat[i]  — per-ISO chroma curve
 *   2. ISP_CSC_ATTR_S.u8Satu                   — global CSC saturation (0-100)
 * If either is zero the output is grayscale.
 *
 * night.c manages (2) only when APP_CFG.night.gray==2.  We manage both here
 * so that day/night color state is consistent regardless of the gray setting.
 */
static void apply_saturation(const scene_params_t *p)
{
    /* Detect night (grayscale) profile: all per-ISO sat values are 0. */
    HI_BOOL grayscale = HI_TRUE;
    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        if (p->sat[i] != 0) { grayscale = HI_FALSE; break; }
    }

    ISP_SATURATION_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetSaturationAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetSaturationAttr failed 0x%x", (unsigned)ret);
        return;
    }
    attr.enOpType = OP_TYPE_AUTO;
    memcpy(attr.stAuto.au8Sat, p->sat, sizeof(p->sat));
    ret = HI_MPI_ISP_SetSaturationAttr(0, &attr);
    LOGGER(LOGGER_LEVEL_DEBUG, "[scene] SetSaturationAttr sat[0]=%u grayscale=%d ret=0x%x",
           p->sat[0], grayscale, (unsigned)ret);

    /* Align the CSC-level saturation: day=100, night=0. */
    ISP_CSC_ATTR_S csc;
    ret = HI_MPI_ISP_GetCSCAttr(0, &csc);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetCSCAttr failed 0x%x", (unsigned)ret);
        return;
    }
    csc.u8Satu = grayscale ? 0 : 100;
    ret = HI_MPI_ISP_SetCSCAttr(0, &csc);
    LOGGER(LOGGER_LEVEL_DEBUG, "[scene] SetCSCAttr u8Satu=%u ret=0x%x",
           csc.u8Satu, (unsigned)ret);
}

static void apply_nr(const scene_params_t *p)
{
    ISP_NR_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetNRAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetNRAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.enOpType = OP_TYPE_AUTO;
    memcpy(attr.stAuto.au8FineStr,    p->nr_fine_str,  sizeof(p->nr_fine_str));
    memcpy(attr.stAuto.au16CoringWgt, p->nr_coring_wgt, sizeof(p->nr_coring_wgt));

    ret = HI_MPI_ISP_SetNRAttr(0, &attr);
    LOGGER(LOGGER_LEVEL_DEBUG, "[scene] SetNRAttr fine[0]=%u coring[0]=%u ret=0x%x",
           p->nr_fine_str[0], p->nr_coring_wgt[0], (unsigned)ret);
}

static void apply_ca(const scene_params_t *p)
{
    ISP_CA_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetCAAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetCAAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable = p->ca_enable;

    ret = HI_MPI_ISP_SetCAAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetCAAttr failed 0x%x", (unsigned)ret);
}

static void apply_scene(const scene_params_t *p)
{
    apply_ae(p);
    apply_ccm(p);
    apply_saturation(p);
    apply_nr(p);
    apply_ca(p);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

int scene_init(const char *day_ini, const char *night_ini)
{
    memset(&g_day,   0, sizeof(g_day));
    memset(&g_night, 0, sizeof(g_night));

    int ret = ini_parse(day_ini, scene_handler, &g_day);
    if (ret < 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[scene] cannot open day INI %s (ret=%d)", day_ini, ret);
        return -1;
    }
    if (ret > 0)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] day INI %s: parse stopped at line %d (long value — ignored)", day_ini, ret);

    ret = ini_parse(night_ini, scene_handler, &g_night);
    if (ret < 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[scene] cannot open night INI %s (ret=%d)", night_ini, ret);
        return -1;
    }
    if (ret > 0)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] night INI %s: parse stopped at line %d (long value — ignored)", night_ini, ret);

    g_initialized = 1;
    LOGGER(LOGGER_LEVEL_INFO, "[scene] initialized: day sat[0]=%u nr_fine[0]=%u gain_max=%u ccm_op=%d",
           g_day.sat[0], g_day.nr_fine_str[0], g_day.ae_sys_gain_max, g_day.ccm_op_type);
    LOGGER(LOGGER_LEVEL_INFO, "[scene] initialized: night sat[0]=%u nr_fine[0]=%u gain_max=%u ccm_op=%d",
           g_night.sat[0], g_night.nr_fine_str[0], g_night.ae_sys_gain_max, g_night.ccm_op_type);
    return 0;
}

int scene_set_day(void)
{
    if (!g_initialized) return -1;
    LOGGER(LOGGER_LEVEL_INFO, "[scene] applying day scene (sat[0]=%u)", g_day.sat[0]);
    apply_scene(&g_day);
    return 0;
}

int scene_set_night(void)
{
    if (!g_initialized) return -1;
    LOGGER(LOGGER_LEVEL_INFO, "[scene] applying night scene (sat[0]=%u)", g_night.sat[0]);
    apply_scene(&g_night);
    return 0;
}
