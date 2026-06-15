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
 *   [static_awb]       — white-balance calibration (static WB, curve, Cr/Cb track)
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

    /* [static_awb] — white-balance calibration */
    HI_BOOL awb_present;
    HI_U16  awb_static_wb[ISP_BAYER_CHN_NUM];
    HI_S32  awb_curve[AWB_CURVE_PARA_NUM];
    HI_U16  awb_speed;
    HI_U16  awb_low_color_temp;
    HI_U16  awb_cr_max[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  awb_cr_min[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  awb_cb_max[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  awb_cb_min[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_BOOL awb_luma_hist_en;

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
static HI_U32 g_target_fps = 20;

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

static int parse_s32_arr(const char *s, HI_S32 *out, int max_n)
{
    char *buf = (char *)s;
    int n = 0;
    for (char *tok = strtok(buf, ","); tok && n < max_n; tok = strtok(NULL, ","))
        out[n++] = (HI_S32)strtol(tok, NULL, 10);
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

    } else if (strcasecmp(section, "static_awb") == 0) {
        p->awb_present = HI_TRUE;
        if      (strcasecmp(name, "AutoStaticWb") == 0)
            parse_u16_arr(v, p->awb_static_wb, ISP_BAYER_CHN_NUM);
        else if (strcasecmp(name, "AutoCurvePara") == 0)
            parse_s32_arr(v, p->awb_curve, AWB_CURVE_PARA_NUM);
        else if (strcasecmp(name, "AutoSpeed") == 0)
            p->awb_speed = (HI_U16)strtoul(v, NULL, 10);
        else if (strcasecmp(name, "AutoLowColorTemp") == 0)
            p->awb_low_color_temp = (HI_U16)strtoul(v, NULL, 10);
        else if (strcasecmp(name, "AutoCrMax") == 0)
            parse_u16_arr(v, p->awb_cr_max, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoCrMin") == 0)
            parse_u16_arr(v, p->awb_cr_min, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoCbMax") == 0)
            parse_u16_arr(v, p->awb_cb_max, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoCbMin") == 0)
            parse_u16_arr(v, p->awb_cb_min, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "LumaHistEnable") == 0)
            p->awb_luma_hist_en = atoi(v) ? HI_TRUE : HI_FALSE;

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
    /* Max integration time for the current sensor fps (one full frame period). */
    HI_U32 max_int_time = (g_target_fps > 0) ? (1000000 / g_target_fps) : 50000;

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
    LOGGER(LOGGER_LEVEL_DEBUG,
           "[scene] SetExposureAttr fps=%u max_int=%u sys_gain_max=%u speed=%u tol=%u ret=0x%x",
           g_target_fps, max_int_time, p->ae_sys_gain_max, p->ae_speed, p->ae_tolerance,
           (unsigned)ret);

    if (p->ae_route_ex_valid && p->ae_route_node_num > 0) {
        ISP_AE_ROUTE_EX_S route;
        memset(&route, 0, sizeof(route));
        route.u32TotalNum = p->ae_route_node_num;
        for (HI_U32 i = 0; i < p->ae_route_node_num && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
            HI_U32 t = p->ae_route_int_time[i];
            if (t > max_int_time) t = max_int_time;
            route.astRouteExNode[i].u32IntTime  = t;
            route.astRouteExNode[i].u32Again    = p->ae_route_again[i];
            route.astRouteExNode[i].u32Dgain    = p->ae_route_dgain[i];
            route.astRouteExNode[i].u32IspDgain = p->ae_route_isp_dgain[i];
        }
        ret = HI_MPI_ISP_SetAERouteAttrEx(0, &route);
        LOGGER(LOGGER_LEVEL_DEBUG,
               "[scene] SetAERouteAttrEx nodes=%u IntTime[0]=%u IntTime[last]=%u ret=0x%x",
               route.u32TotalNum,
               route.astRouteExNode[0].u32IntTime,
               route.astRouteExNode[route.u32TotalNum - 1].u32IntTime,
               (unsigned)ret);
    }
}

/* Apply the white-balance calibration from [static_awb].
 *
 * Without this the auto-AWB algorithm runs on the ISP's default calibration
 * (or the sensor cmos_get_awb_default() anchor), which does not match the JXF22
 * factory calibration in the INI -> visible colour cast (green). We keep
 * enOpType=AUTO and only patch the calibration fields (static WB reference,
 * curve, Cr/Cb tracking) via Get->patch->Set so the rest of the AWB state and
 * stManual are preserved. */
static void apply_awb(const scene_params_t *p)
{
    if (!p->awb_present) return;

    ISP_WB_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetWBAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetWBAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.enOpType = OP_TYPE_AUTO;
    memcpy(attr.stAuto.au16StaticWB,  p->awb_static_wb, sizeof(p->awb_static_wb));
    memcpy(attr.stAuto.as32CurvePara, p->awb_curve,     sizeof(p->awb_curve));
    if (p->awb_speed)          attr.stAuto.u16Speed         = p->awb_speed;
    if (p->awb_low_color_temp) attr.stAuto.u16LowColorTemp  = p->awb_low_color_temp;

    attr.stAuto.stCbCrTrack.bEnable = HI_TRUE;
    memcpy(attr.stAuto.stCbCrTrack.au16CrMax, p->awb_cr_max, sizeof(p->awb_cr_max));
    memcpy(attr.stAuto.stCbCrTrack.au16CrMin, p->awb_cr_min, sizeof(p->awb_cr_min));
    memcpy(attr.stAuto.stCbCrTrack.au16CbMax, p->awb_cb_max, sizeof(p->awb_cb_max));
    memcpy(attr.stAuto.stCbCrTrack.au16CbMin, p->awb_cb_min, sizeof(p->awb_cb_min));
    attr.stAuto.stLumaHist.bEnable  = p->awb_luma_hist_en;

    ret = HI_MPI_ISP_SetWBAttr(0, &attr);
    LOGGER(LOGGER_LEVEL_DEBUG,
           "[scene] SetWBAttr staticWB=[%u,%u,%u,%u] speed=%u lowCT=%u ret=0x%x",
           p->awb_static_wb[0], p->awb_static_wb[1], p->awb_static_wb[2], p->awb_static_wb[3],
           p->awb_speed, p->awb_low_color_temp, (unsigned)ret);
}

static void apply_ccm(const scene_params_t *p)
{
    /* Auto mode (ccm_op=0): ISP/AWB library manages CCM from jxf22_cmos.c
       cmos_get_awb_default() tables — do not call SetCCMAttr.
       The day INI has no ManualCCMTable so stManual.au16CCM would be all-zeros.
       Passing a zero stManual corrupts the ISP color pipeline even when
       enOpType=OP_TYPE_AUTO (the ISP uses stManual internally). */
    if (p->ccm_op_type == 0) {
        LOGGER(LOGGER_LEVEL_DEBUG, "[scene] CCM auto — ISP/AWB manages CCM, no SetCCMAttr");
        return;
    }

    /* Manual mode (ccm_op=1): night grayscale — force identity CCM matrix. */
    ISP_COLORMATRIX_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetCCMAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetCCMAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.enOpType = OP_TYPE_MANUAL;
    attr.stManual.bSatEn = HI_FALSE;
    memcpy(attr.stManual.au16CCM, p->ccm_manual, sizeof(p->ccm_manual));

    ret = HI_MPI_ISP_SetCCMAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetCCMAttr manual failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_DEBUG, "[scene] SetCCMAttr manual OK ccm[0]=%u ccm[4]=%u ccm[8]=%u",
               p->ccm_manual[0], p->ccm_manual[4], p->ccm_manual[8]);
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
    apply_awb(p);
    apply_ccm(p);
    apply_saturation(p);
    apply_nr(p);
    apply_ca(p);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

int scene_init(const char *day_ini, const char *night_ini, HI_U32 target_fps)
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
    g_target_fps = (target_fps > 0) ? target_fps : 20;
    LOGGER(LOGGER_LEVEL_INFO, "[scene] initialized: day sat[0]=%u nr_fine[0]=%u gain_max=%u ccm_op=%d awb=%d staticWB=[%u,%u,%u,%u]",
           g_day.sat[0], g_day.nr_fine_str[0], g_day.ae_sys_gain_max, g_day.ccm_op_type,
           (int)g_day.awb_present,
           g_day.awb_static_wb[0], g_day.awb_static_wb[1], g_day.awb_static_wb[2], g_day.awb_static_wb[3]);
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
