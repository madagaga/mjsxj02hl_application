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
 *   [static_drc]       — dynamic range compression (gated by [module_state])
 *   [static_sharpen]   — edge/texture sharpening, per-ISO tables (gated)
 *   [static_ldci]      — local dynamic contrast improvement, per-ISO auto tables
 *   [static_dpc]       — dynamic defect-pixel correction, per-ISO auto tables
 *   [dynamic_linear_drc] — per-ISO DRC strength, re-applied at runtime by a thread
 *   [module_state]     — per-module flags (bStaticDRC/bStaticSharpen/bDynamicLinearDrc)
 */

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

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
    HI_U8   ae_compensation;  /* AutoCompesation steady-state value; 0 = keep ISP default */

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
    HI_BOOL ccm_iso_act_en;
    HI_BOOL ccm_temp_act_en;
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

    /* [module_state] — only the flags we act on */
    HI_BOOL mod_static_drc;
    HI_BOOL mod_static_sharpen;
    HI_BOOL mod_dyn_lineardrc;

    /* [dynamic_linear_drc] — per-ISO DRC strength (overrides the static strength
       at runtime; this is why the original is not over-bright in daylight). */
    HI_BOOL dld_enable;
    HI_U32  dld_iso_cnt;
    HI_U32  dld_iso_level[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  dld_strength[ISP_AUTO_ISO_STRENGTH_NUM];

    /* [static_drc] */
    HI_BOOL drc_enable;
    int     drc_curve_select;   /* 0 = asymmetry, 1 = cubic, 2 = user curve */
    int     drc_op_type;        /* 0 = auto, 1 = manual */
    HI_U16  drc_auto_str;
    HI_U16  drc_auto_str_min;
    HI_U16  drc_auto_str_max;

    /* [static_sharpen] — auto (per-ISO) tables. 2D = [gain/luma idx][iso]. */
    HI_BOOL shp_enable;
    HI_U8   shp_luma_wgt[ISP_SHARPEN_LUMA_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  shp_texture_str[ISP_SHARPEN_GAIN_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  shp_edge_str[ISP_SHARPEN_GAIN_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  shp_texture_freq[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  shp_edge_freq[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_over_shoot[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_under_shoot[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_shoot_sup_str[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_shoot_sup_adj[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_detail_ctrl[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_edge_filt_str[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_edge_filt_max_cap[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_rgain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_bgain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_ggain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   shp_skin_gain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  shp_max_sharp_gain[ISP_AUTO_ISO_STRENGTH_NUM];

    /* [static_ldci] — local dynamic contrast improvement (per-ISO auto) */
    HI_BOOL ldci_present;
    HI_BOOL ldci_enable;
    HI_U8   ldci_gauss_sigma;
    HI_U8   ldci_pos_wgt[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   ldci_pos_sigma[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   ldci_pos_mean[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   ldci_neg_wgt[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   ldci_neg_sigma[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   ldci_neg_mean[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  ldci_blc_ctrl[ISP_AUTO_ISO_STRENGTH_NUM];

    /* [static_dpc] — dynamic defect-pixel correction (per-ISO auto) */
    HI_BOOL dpc_present;
    HI_BOOL dpc_enable;
    HI_U16  dpc_strength[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  dpc_blend_ratio[ISP_AUTO_ISO_STRENGTH_NUM];
} scene_params_t;

static scene_params_t g_day;
static scene_params_t g_night;
static int g_initialized = 0;
static HI_U32 g_target_fps = 20;

/* Currently active profile (day or night) for the dynamic DRC thread. */
static const scene_params_t *g_active = NULL;
static pthread_t    g_dynThread = 0;
static volatile int g_dynRun    = 0;

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

    } else if (strcasecmp(section, "dynamic_ae") == 0) {
        /* AutoCompesation (INI typo: missing 'n') is a per-exposure-segment AE
         * luma target table, e.g. "82,82,80,80,80,80". The ISP exposes a single
         * scalar u8Compensation; the values are near-constant (80-82) so we apply
         * the steady-state (last, longest-exposure) value statically rather than
         * running a segment-tracking thread. Matches the original (Comp=80). */
        if (strcasecmp(name, "AutoCompesation") == 0) {
            HI_U8 arr[8];
            int n = parse_u8_arr(v, arr, 8);
            if (n > 0)
                p->ae_compensation = arr[n - 1];
        }

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
        else if (strcasecmp(name, "ISOActEn") == 0)
            p->ccm_iso_act_en = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "TempActEn") == 0)
            p->ccm_temp_act_en = atoi(v) ? HI_TRUE : HI_FALSE;
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

    } else if (strcasecmp(section, "module_state") == 0) {
        if      (strcasecmp(name, "bStaticDRC") == 0)
            p->mod_static_drc     = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "bStaticSharpen") == 0)
            p->mod_static_sharpen = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "bDynamicLinearDrc") == 0)
            p->mod_dyn_lineardrc  = atoi(v) ? HI_TRUE : HI_FALSE;

    } else if (strcasecmp(section, "static_drc") == 0) {
        /* Keys before DRCToneMappingValue parse fine; the 200-value tone-mapping
           LUT line (backslash-continued) is not parseable by inih and is unused
           with the asymmetry curve, so it is intentionally ignored. */
        if      (strcasecmp(name, "Enable") == 0)
            p->drc_enable        = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "CurveSelect") == 0)
            p->drc_curve_select  = atoi(v);
        else if (strcasecmp(name, "DRCOpType") == 0)
            p->drc_op_type       = atoi(v);
        else if (strcasecmp(name, "DRCAutoStr") == 0)
            p->drc_auto_str      = (HI_U16)atoi(v);
        else if (strcasecmp(name, "DRCAutoStrMin") == 0)
            p->drc_auto_str_min  = (HI_U16)atoi(v);
        else if (strcasecmp(name, "DRCAutoStrMax") == 0)
            p->drc_auto_str_max  = (HI_U16)atoi(v);

    } else if (strcasecmp(section, "static_sharpen") == 0) {
        /* 2D per-ISO tables: "AutoLumaWgt_<row>" etc., each line = 16 ISO values. */
        if (strcasecmp(name, "Enable") == 0) {
            p->shp_enable = atoi(v) ? HI_TRUE : HI_FALSE;
        } else if (strncasecmp(name, "AutoLumaWgt_", 12) == 0) {
            int r = atoi(name + 12);
            if (r >= 0 && r < ISP_SHARPEN_LUMA_NUM)
                parse_u8_arr(v, p->shp_luma_wgt[r], ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strncasecmp(name, "AutoTextureStr_", 15) == 0) {
            int r = atoi(name + 15);
            if (r >= 0 && r < ISP_SHARPEN_GAIN_NUM)
                parse_u16_arr(v, p->shp_texture_str[r], ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strncasecmp(name, "AutoEdgeStr_", 12) == 0) {
            int r = atoi(name + 12);
            if (r >= 0 && r < ISP_SHARPEN_GAIN_NUM)
                parse_u16_arr(v, p->shp_edge_str[r], ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoTextureFreq") == 0) {
            parse_u16_arr(v, p->shp_texture_freq, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoEdgeFreq") == 0) {
            parse_u16_arr(v, p->shp_edge_freq, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoOverShoot") == 0) {
            parse_u8_arr(v, p->shp_over_shoot, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoUnderShoot") == 0) {
            parse_u8_arr(v, p->shp_under_shoot, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoShootSupStr") == 0) {
            parse_u8_arr(v, p->shp_shoot_sup_str, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoShootSupAdj") == 0) {
            parse_u8_arr(v, p->shp_shoot_sup_adj, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoDetailCtrl") == 0) {
            parse_u8_arr(v, p->shp_detail_ctrl, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoEdgeFiltStr") == 0) {
            parse_u8_arr(v, p->shp_edge_filt_str, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoEdgeFiltMaxCap") == 0) {
            parse_u8_arr(v, p->shp_edge_filt_max_cap, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoRGain") == 0) {
            parse_u8_arr(v, p->shp_rgain, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoBGain") == 0) {
            parse_u8_arr(v, p->shp_bgain, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoGGain") == 0) {
            parse_u8_arr(v, p->shp_ggain, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoSkinGain") == 0) {
            parse_u8_arr(v, p->shp_skin_gain, ISP_AUTO_ISO_STRENGTH_NUM);
        } else if (strcasecmp(name, "AutoMaxSharpGain") == 0) {
            parse_u16_arr(v, p->shp_max_sharp_gain, ISP_AUTO_ISO_STRENGTH_NUM);
        }

    } else if (strcasecmp(section, "dynamic_linear_drc") == 0) {
        if      (strcasecmp(name, "Enable") == 0)
            p->dld_enable    = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "IsoCnt") == 0)
            p->dld_iso_cnt   = (HI_U32)atoi(v);
        else if (strcasecmp(name, "IsoLevel") == 0)
            parse_u32_arr(v, p->dld_iso_level, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "Strength") == 0)
            parse_u16_arr(v, p->dld_strength, ISP_AUTO_ISO_STRENGTH_NUM);

    } else if (strcasecmp(section, "static_ldci") == 0) {
        p->ldci_present = HI_TRUE;
        if      (strcasecmp(name, "Enable") == 0)
            p->ldci_enable      = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "LDCIGaussLPFSigma") == 0)
            p->ldci_gauss_sigma = (HI_U8)atoi(v);
        else if (strcasecmp(name, "AutoHePosWgt") == 0)
            parse_u8_arr(v, p->ldci_pos_wgt,   ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoHePosSigma") == 0)
            parse_u8_arr(v, p->ldci_pos_sigma, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoHePosMean") == 0)
            parse_u8_arr(v, p->ldci_pos_mean,  ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoHeNegWgt") == 0)
            parse_u8_arr(v, p->ldci_neg_wgt,   ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoHeNegSigma") == 0)
            parse_u8_arr(v, p->ldci_neg_sigma, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoHeNegMean") == 0)
            parse_u8_arr(v, p->ldci_neg_mean,  ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "AutoBlcCtrl") == 0)
            parse_u16_arr(v, p->ldci_blc_ctrl, ISP_AUTO_ISO_STRENGTH_NUM);

    } else if (strcasecmp(section, "static_dpc") == 0) {
        p->dpc_present = HI_TRUE;
        if      (strcasecmp(name, "DpcEnable") == 0)
            p->dpc_enable = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "Strength") == 0)
            parse_u16_arr(v, p->dpc_strength,    ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "BlendRatio") == 0)
            parse_u16_arr(v, p->dpc_blend_ratio, ISP_AUTO_ISO_STRENGTH_NUM);
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
    if (p->ae_compensation > 0)                  /* brighter luma target (INI AutoCompesation) */
        attr.stAuto.u8Compensation               = p->ae_compensation;

    ret = HI_MPI_ISP_SetExposureAttr(0, &attr);
    LOGGER(LOGGER_LEVEL_DEBUG,
           "[scene] SetExposureAttr fps=%u max_int=%u sys_gain_max=%u speed=%u tol=%u comp=%u ret=0x%x",
           g_target_fps, max_int_time, p->ae_sys_gain_max, p->ae_speed, p->ae_tolerance,
           p->ae_compensation, (unsigned)ret);

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

    /* Match HI_SCENE_SetStaticAWB (3516e ref): do NOT force enOpType, and do NOT
       force stCbCrTrack.bEnable — only fill the calibration arrays. Forcing
       Cr/Cb tracking ON constrained the AWB to a wrong white point (R-heavy gains
       -> magenta). Leave the rest of the WB state as the ISP returned it. */
    memcpy(attr.stAuto.au16StaticWB,  p->awb_static_wb, sizeof(p->awb_static_wb));
    memcpy(attr.stAuto.as32CurvePara, p->awb_curve,     sizeof(p->awb_curve));
    attr.stAuto.u16Speed        = p->awb_speed;
    attr.stAuto.u16LowColorTemp = p->awb_low_color_temp;

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
    ISP_COLORMATRIX_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetCCMAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetCCMAttr failed 0x%x", (unsigned)ret);
        return;
    }

    /* Auto mode (ccm_op=0): apply the INI's per-color-temperature CCM tables.
       Leaving CCM at the ISP/cmos defaults produced a magenta cast. We use
       Get→patch→Set and only touch stAuto + enOpType, leaving stManual as the
       ISP returned it (a zeroed stManual is what caused the historical magenta;
       Get→patch→Set never zeroes it). */
    if (p->ccm_op_type == 0) {
        if (p->ccm_tab_num < 3) {   /* SDK requires u16CCMTabNum in [3,7] */
            LOGGER(LOGGER_LEVEL_WARNING,
                   "[scene] CCM auto: only %u tables (<3), leaving ISP default", p->ccm_tab_num);
            return;
        }
        attr.enOpType            = OP_TYPE_AUTO;
        attr.stAuto.bISOActEn    = p->ccm_iso_act_en;
        attr.stAuto.bTempActEn   = p->ccm_temp_act_en;
        attr.stAuto.u16CCMTabNum = p->ccm_tab_num;
        for (HI_U16 i = 0; i < p->ccm_tab_num && i < CCM_MATRIX_NUM; i++) {
            attr.stAuto.astCCMTab[i].u16ColorTemp = p->ccm_color_temp[i];
            memcpy(attr.stAuto.astCCMTab[i].au16CCM, p->ccm_auto[i], sizeof(p->ccm_auto[i]));
        }
        ret = HI_MPI_ISP_SetCCMAttr(0, &attr);
        LOGGER(LOGGER_LEVEL_DEBUG,
               "[scene] SetCCMAttr auto num=%u ct[0]=%u ccm0=[%u,%u,%u] ret=0x%x",
               p->ccm_tab_num, p->ccm_color_temp[0],
               p->ccm_auto[0][0], p->ccm_auto[0][1], p->ccm_auto[0][2], (unsigned)ret);
        return;
    }

    /* Manual mode (ccm_op=1): night grayscale — force identity CCM matrix. */
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

/* Dynamic Range Compression (local tone mapping) — recovers highlight/shadow
   detail in high-contrast scenes. Faithful to libsceneauto HI_SCENE_SetStaticDRC:
   Get→patch enable/curve/op-type/auto-strength→Set, leaving stManual,
   stAsymmetryCurve and the tone-mapping LUT at their ISP defaults. */
static void apply_drc(const scene_params_t *p)
{
    if (!p->mod_static_drc)
        return;  /* [module_state] bStaticDRC = 0 → leave DRC untouched */

    ISP_DRC_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetDRCAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetDRCAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable               = p->drc_enable;
    attr.enCurveSelect         = (ISP_DRC_CURVE_SELECT_E)p->drc_curve_select;
    attr.enOpType              = (ISP_OP_TYPE_E)p->drc_op_type;
    attr.stAuto.u16Strength    = p->drc_auto_str;
    attr.stAuto.u16StrengthMin = p->drc_auto_str_min;
    attr.stAuto.u16StrengthMax = p->drc_auto_str_max;

    ret = HI_MPI_ISP_SetDRCAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetDRCAttr failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO,
               "[scene] DRC en=%d curve=%d op=%d str=%u/%u/%u",
               (int)attr.bEnable, (int)attr.enCurveSelect, (int)attr.enOpType,
               attr.stAuto.u16Strength, attr.stAuto.u16StrengthMin,
               attr.stAuto.u16StrengthMax);
}

/* Edge/texture sharpening — recovers the crispness the original firmware has.
   Faithful to libsceneauto HI_SCENE_SetStaticSharpen: Get→fill the per-ISO auto
   tables→Set. enOpType, skin range, DetailCtrlThr and WeakDetailGain keep their
   ISP defaults from Get (the reference does not touch them). */
static void apply_sharpen(const scene_params_t *p)
{
    if (!p->mod_static_sharpen)
        return;  /* [module_state] bStaticSharpen = 0 → leave sharpen untouched */

    ISP_SHARPEN_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetIspSharpenAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetIspSharpenAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable = p->shp_enable;

    for (int i = 0; i < ISP_SHARPEN_GAIN_NUM; i++) {
        for (int j = 0; j < ISP_AUTO_ISO_STRENGTH_NUM; j++) {
            attr.stAuto.au8LumaWgt[i][j]    = p->shp_luma_wgt[i][j];
            attr.stAuto.au16TextureStr[i][j] = p->shp_texture_str[i][j];
            attr.stAuto.au16EdgeStr[i][j]    = p->shp_edge_str[i][j];
        }
    }

    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        attr.stAuto.au16TextureFreq[i]   = p->shp_texture_freq[i];
        attr.stAuto.au16EdgeFreq[i]      = p->shp_edge_freq[i];
        attr.stAuto.au8OverShoot[i]      = p->shp_over_shoot[i];
        attr.stAuto.au8UnderShoot[i]     = p->shp_under_shoot[i];
        attr.stAuto.au8ShootSupStr[i]    = p->shp_shoot_sup_str[i];
        attr.stAuto.au8ShootSupAdj[i]    = p->shp_shoot_sup_adj[i];
        attr.stAuto.au8DetailCtrl[i]     = p->shp_detail_ctrl[i];
        attr.stAuto.au8EdgeFiltStr[i]    = p->shp_edge_filt_str[i];
        attr.stAuto.au8EdgeFiltMaxCap[i] = p->shp_edge_filt_max_cap[i];
        attr.stAuto.au8RGain[i]          = p->shp_rgain[i];
        attr.stAuto.au8BGain[i]          = p->shp_bgain[i];
        attr.stAuto.au8GGain[i]          = p->shp_ggain[i];
        attr.stAuto.au8SkinGain[i]       = p->shp_skin_gain[i];
        attr.stAuto.au16MaxSharpGain[i]  = p->shp_max_sharp_gain[i];
    }

    ret = HI_MPI_ISP_SetIspSharpenAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetIspSharpenAttr failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO,
               "[scene] Sharpen en=%d textureStr[0][0]=%u edgeStr[0][0]=%u maxGain[0]=%u",
               (int)attr.bEnable, attr.stAuto.au16TextureStr[0][0],
               attr.stAuto.au16EdgeStr[0][0], attr.stAuto.au16MaxSharpGain[0]);
}

/* Local dynamic contrast improvement — faithful to HI_SCENE_SetStaticLDCI:
   Get→fill the per-ISO auto tables (7 arrays × 16 ISO nodes)→Set.
   enOpType kept at OP_TYPE_AUTO (LDCIOpType=0 in the INI). */
static void apply_ldci(const scene_params_t *p)
{
    if (!p->ldci_present)
        return;

    ISP_LDCI_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetLDCIAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetLDCIAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable         = p->ldci_enable;
    attr.u8GaussLPFSigma = p->ldci_gauss_sigma;
    attr.enOpType        = OP_TYPE_AUTO;

    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        attr.stAuto.astHeWgt[i].stHePosWgt.u8Wgt   = p->ldci_pos_wgt[i];
        attr.stAuto.astHeWgt[i].stHePosWgt.u8Sigma = p->ldci_pos_sigma[i];
        attr.stAuto.astHeWgt[i].stHePosWgt.u8Mean  = p->ldci_pos_mean[i];
        attr.stAuto.astHeWgt[i].stHeNegWgt.u8Wgt   = p->ldci_neg_wgt[i];
        attr.stAuto.astHeWgt[i].stHeNegWgt.u8Sigma = p->ldci_neg_sigma[i];
        attr.stAuto.astHeWgt[i].stHeNegWgt.u8Mean  = p->ldci_neg_mean[i];
        attr.stAuto.au16BlcCtrl[i]                  = p->ldci_blc_ctrl[i];
    }

    ret = HI_MPI_ISP_SetLDCIAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetLDCIAttr failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO,
               "[scene] LDCI en=%d sigma=%u posWgt[4]=%u blcCtrl[4]=%u",
               (int)attr.bEnable, attr.u8GaussLPFSigma,
               attr.stAuto.astHeWgt[4].stHePosWgt.u8Wgt,
               attr.stAuto.au16BlcCtrl[4]);
}

/* Dynamic defect-pixel correction — faithful to HI_SCENE_SetStaticDPC:
   Get→fill per-ISO auto Strength and BlendRatio arrays→Set (OP_TYPE_AUTO). */
static void apply_dpc(const scene_params_t *p)
{
    if (!p->dpc_present)
        return;

    ISP_DP_DYNAMIC_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetDPDynamicAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetDPDynamicAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable  = p->dpc_enable;
    attr.enOpType = OP_TYPE_AUTO;

    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        attr.stAuto.au16Strength[i]   = p->dpc_strength[i];
        attr.stAuto.au16BlendRatio[i] = p->dpc_blend_ratio[i];
    }

    ret = HI_MPI_ISP_SetDPDynamicAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetDPDynamicAttr failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO,
               "[scene] DPC en=%d strength[8]=%u blendRatio[10]=%u",
               (int)attr.bEnable,
               attr.stAuto.au16Strength[8], attr.stAuto.au16BlendRatio[10]);
}

/* Dynamic linear DRC — the original modulates the DRC strength per-ISO at
   runtime ([dynamic_linear_drc]); without it our fixed static strength (512)
   over-brightens in daylight (low ISO), washing the image and degrading IVP
   detection. Faithful to libsceneauto HI_SCENE_SetDynamicLinearDrc: interpolate
   the strength from the per-ISO table and write it to the DRC attr (manual or
   auto, matching enOpType). The smart-exposure term is omitted (≈0 here: at
   ISO 2730 the table alone gives 179, matching the original's /proc dump). */
static HI_U16 dld_interp(const scene_params_t *p, HI_U32 iso)
{
    HI_U32 n = p->dld_iso_cnt;
    if (n == 0) return 0;
    if (n > ISP_AUTO_ISO_STRENGTH_NUM) n = ISP_AUTO_ISO_STRENGTH_NUM;
    if (iso <= p->dld_iso_level[0])     return p->dld_strength[0];
    if (iso >= p->dld_iso_level[n - 1]) return p->dld_strength[n - 1];
    for (HI_U32 i = 1; i < n; i++) {
        if (iso <= p->dld_iso_level[i]) {
            HI_S32 lo = (HI_S32)p->dld_iso_level[i - 1], hi = (HI_S32)p->dld_iso_level[i];
            HI_S32 slo = p->dld_strength[i - 1], shi = p->dld_strength[i];
            if (hi == lo) return (HI_U16)slo;
            return (HI_U16)(slo + (shi - slo) * ((HI_S32)iso - lo) / (hi - lo));
        }
    }
    return p->dld_strength[n - 1];
}

static void apply_dynamic_drc(const scene_params_t *p, HI_U32 iso)
{
    if (!p || !p->mod_dyn_lineardrc || !p->dld_enable) return;

    ISP_DRC_ATTR_S attr;
    if (HI_MPI_ISP_GetDRCAttr(0, &attr) != HI_SUCCESS) return;

    HI_U16 s = dld_interp(p, iso);
    if (s > 512) s = 512; /* SDK clamp */
    /* Write to whichever slot the current op-type uses. */
    attr.stManual.u16Strength = s;
    attr.stAuto.u16Strength   = s;
    HI_MPI_ISP_SetDRCAttr(0, &attr);
}

/* Polls the ISP exposure ISO once per second and re-applies the per-ISO DRC
   strength for the active (day/night) profile. */
static void *scene_dyn_thread(void *arg)
{
    (void)arg;
    while (g_dynRun) {
        const scene_params_t *p = g_active;
        if (p) {
            ISP_EXP_INFO_S exp;
            if (HI_MPI_ISP_QueryExposureInfo(0, &exp) == HI_SUCCESS)
                apply_dynamic_drc(p, exp.u32ISO);
        }
        sleep(1);
    }
    return NULL;
}

static void apply_scene(const scene_params_t *p)
{
    apply_ae(p);
    apply_awb(p);
    apply_ccm(p);
    apply_saturation(p);
    apply_nr(p);
    apply_ca(p);
    apply_ldci(p);
    apply_dpc(p);
    apply_drc(p);
    apply_sharpen(p);
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
    g_active = &g_day;

    /* Start the per-ISO dynamic DRC thread (if the day profile enables it). */
    if (!g_dynRun && g_day.mod_dyn_lineardrc) {
        g_dynRun = 1;
        if (pthread_create(&g_dynThread, NULL, scene_dyn_thread, NULL) != 0) {
            g_dynRun = 0;
            LOGGER(LOGGER_LEVEL_WARNING, "[scene] dynamic DRC thread start failed");
        } else {
            LOGGER(LOGGER_LEVEL_INFO, "[scene] dynamic linear DRC active (iso-adaptive strength)");
        }
    }
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
    g_active = &g_day;
    apply_scene(&g_day);
    return 0;
}

int scene_set_night(void)
{
    if (!g_initialized) return -1;
    LOGGER(LOGGER_LEVEL_INFO, "[scene] applying night scene (sat[0]=%u)", g_night.sat[0]);
    g_active = &g_night;
    apply_scene(&g_night);
    return 0;
}
