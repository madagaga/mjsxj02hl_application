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
#include "gamma_jxf22.h"

/* -------------------------------------------------------------------------
 * Internal types
 * ---------------------------------------------------------------------- */

/* [module_state] flags, in the order of libsceneauto's SCENE_LoadModuleState
   (indices confirmed by RE, see re_notes/libsceneauto_spec.md §1). A key that is
   absent from the INI leaves its flag at 0, i.e. the module is off — exactly as
   in the original. The key "bDyanamic3DNR" is misspelt in the original too. */
#define SCENE_MODULE_LIST(X) \
    X(DEBUG, "bDebug") X(AEWEIGHTTAB, "bAeWeightTab") X(STATIC_AE, "bStaticAE") \
    X(STATIC_GLOBALCAC, "bStaticGlobalCac") X(STATIC_LOCALCAC, "bStaticLocalCac") X(STATIC_DPC, "bStaticDPC") \
    X(STATIC_WDREXPOSURE, "bStaticWdrExposure") X(STATIC_FSWDR, "bStaticFsWdr") X(STATIC_AWB, "bStaticAWB") \
    X(STATIC_AWBEX, "bStaticAWBEx") X(STATIC_DRC, "bStaticDRC") X(STATIC_DEHAZE, "bStaticDehaze") \
    X(STATIC_LDCI, "bStaticLdci") X(STATIC_SATURATION, "bStaticSaturation") X(STATIC_CCM, "bStaticCCM") \
    X(STATIC_NR, "bStaticNr") X(STATIC_SHADING, "bStaticShading") X(STATIC_CSC, "bStaticCSC") \
    X(STATIC_SHARPEN, "bStaticSharpen") X(STATIC_CLUT, "bStaticClut") X(STATIC_CA, "bStaticCa") \
    X(STATIC_CROSSTALK, "bStaticCrosstalk") X(STATIC_3DNR, "bStatic3DNR") X(STATIC_VENC, "bStaticVenc") \
    X(DYN_FPS, "bDynamicFps") X(DYN_AE, "bDynamicAE") X(DYN_WDREXPOSURE, "bDynamicWdrExposure") \
    X(DYN_ISOVENC, "bDynamicIsoVenc") X(DYN_FSWDR, "bDynamicFSWDR") X(DYN_DRC, "bDynamicDrc") \
    X(DYN_LINEARDRC, "bDynamicLinearDrc") X(DYN_DEHAZE, "bDynamicDehaze") X(DYN_LDCI, "bDynamicLdci") \
    X(DYN_NR, "bDynamicNr") X(DYN_DPC, "bDynamicDpc") X(DYN_BLC, "bDynamicBLC") \
    X(DYN_CA, "bDynamicCA") X(DYN_SHADING, "bDynamicShading") X(DYN_GAMMA, "bDynamicGamma") \
    X(DYN_FALSECOLOR, "bDynamicFalseColor") X(DYN_3DNR, "bDyanamic3DNR")

enum {
#define X(id, key) MOD_##id,
    SCENE_MODULE_LIST(X)
#undef X
    MOD_COUNT
};

static const char *const k_module_key[MOD_COUNT] = {
#define X(id, key) key,
    SCENE_MODULE_LIST(X)
#undef X
};

/* Maximum number of ISO / exposure levels kept for the dynamic tables. */
#define SCENE_DYN_LEVELS 8

/* Test a [module_state] flag of a profile. */
#define MOD_ON(p, id) ((p)->mod[MOD_##id] != 0)

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
    HI_S32  ca_iso_ratio[ISP_AUTO_ISO_STRENGTH_NUM];

    /* [static_dehaze] — differs day/night (day: flat pass-through LUT 255x256;
       night: real ramp 107->255). [dynamic_dehaze] — strength ramps by raw
       exposure (u32Exposure, not ISO), re-applied continuously like DRC.
       Verified by RE of libsceneauto.so: HI_SCENE_SetStaticDeHaze applies the
       LUT once; HI_SCENE_SetDynamicDehaze re-interpolates stManual.u8strength
       on every exposure change. Our old code forced a flat LUT + fixed
       strength=90 always -> correct-ish in daylight, wrong at night. */
    HI_BOOL dehaze_enable;
    HI_BOOL dehaze_user_lut;
    int     dehaze_op_type;
    HI_U8   dehaze_lut[256];
    int     dehaze_lut_fill;      /* parse-time cursor across continuation lines */
    HI_U32  dehaze_exp_cnt;
    HI_U32  dehaze_exp_thresh[8];
    HI_U8   dehaze_str[8];

    /* [static_aeweight] — AE metering zone weight table (15 rows x 17 cols).
       Center-weighted in both day/night INIs; the ISP defaults to a flat
       table when this is never applied, which is the AE daytime brightness
       gap vs the original (verified: HI_SCENE_SetAeWeightTab in libsceneauto.so
       does exactly Get->patch au8Weight->Set, no unKey bit involved). */
    HI_BOOL ae_weight_present;
    HI_U8   ae_weight[AE_ZONE_ROW][AE_ZONE_COLUMN];

    /* [module_state] — all 41 flags of the original, indexed by MOD_* */
    HI_U8   mod[MOD_COUNT];

    /* [dynamic_linear_drc] — per-ISO DRC strength (overrides the static strength
       at runtime; this is why the original is not over-bright in daylight). */
    HI_BOOL dld_enable;
    HI_U32  dld_iso_cnt;
    HI_U32  dld_iso_level[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  dld_strength[ISP_AUTO_ISO_STRENGTH_NUM];
    /* [dynamic_linear_drc] local dynamic-contrast tables (per-ISO). These shape
       shadow/highlight local contrast and the dark/bright gain limits; without
       them the DRC only had our per-ISO Strength and the ISP defaults for the
       rest, leaving the image flat with lifted shadows. */
    HI_BOOL dld_lm_present;
    HI_U8   dld_lm_bright_max[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_lm_bright_min[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_lm_dark_max[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_lm_dark_min[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_bright_gain_lmt[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_bright_gain_lmt_step[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_dark_gain_lmt_y[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_dark_gain_lmt_c[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_contrast_ctrl[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_detail_adjust[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_asymmetry[ISP_AUTO_ISO_STRENGTH_NUM];    /* asymmetry tone-curve params */
    HI_U8   dld_second_pole[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_compress[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   dld_stretch[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_BOOL dld_curve_present;

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

    /* [dynamic_ae] - per exposure-level AE luma target and max hist offset
       (exposure64 = ExpTime * ISO / 100, thresholds "aeExpLtoHThresh"). */
    HI_U32  dae_cnt;
    HI_U64  dae_thr[SCENE_DYN_LEVELS];
    HI_U8   dae_comp[SCENE_DYN_LEVELS];
    HI_U8   dae_hist[SCENE_DYN_LEVELS];

    /* [dynamic_nr] - per-ISO luma-dependent coring ratio LUT (33 entries) and
       per-level WDR frame strengths. */
    HI_U32  dnr_cnt;
    HI_U64  dnr_iso[SCENE_DYN_LEVELS];
    HI_U16  dnr_coring[SCENE_DYN_LEVELS][HI_ISP_BAYERNR_LUT_LENGTH];
    HI_U8   dnr_short[SCENE_DYN_LEVELS];
    HI_U8   dnr_long[SCENE_DYN_LEVELS];

    /* [dynamic_blc] - per-ISO black level (R, Gr, Gb, B). */
    HI_U32  dblc_cnt;
    HI_U64  dblc_iso[SCENE_DYN_LEVELS];
    HI_U16  dblc_val[ISP_BAYER_CHN_NUM][SCENE_DYN_LEVELS];

    /* [dynamic_ca] - per-ISO luma CA ratio LUT (128 entries per level; the INI
       continues each row over several lines, one handler call per line). */
    HI_U32  dca_cnt;
    HI_U64  dca_iso[SCENE_DYN_LEVELS];
    HI_U16  dca_lut[SCENE_DYN_LEVELS][HI_ISP_CA_YRATIO_LUT_LENGTH];
    int     dca_fill[SCENE_DYN_LEVELS];   /* parse-time cursor per row */

    /* [static_awbex] */
    HI_BOOL awbex_present;
    HI_U8   awbex_bypass;
    HI_U8   awbex_tolerance;
    HI_U32  awbex_out_thresh;
    HI_U16  awbex_low_stop;
    HI_U16  awbex_high_start;
    HI_U16  awbex_high_stop;
    HI_BOOL awbex_multi_ls_en;
    HI_U16  awbex_multi_ct_wt[AWB_MULTI_CT_NUM];
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

static int parse_u64_arr(const char *s, HI_U64 *out, int max_n)
{
    char *buf = (char *)s;
    int n = 0;
    for (char *tok = strtok(buf, ","); tok && n < max_n; tok = strtok(NULL, ","))
        out[n++] = (HI_U64)strtoull(tok, NULL, 10);
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
        /* Applied by apply_dynamic_ae(): the original interpolates both the
           compensation and the max hist offset by exposure level. */
        if      (strcasecmp(name, "aeExpCount") == 0)
            p->dae_cnt = (HI_U32)atoi(v);
        else if (strcasecmp(name, "aeExpLtoHThresh") == 0)
            parse_u64_arr(v, p->dae_thr, SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "AutoCompesation") == 0)   /* sic */
            parse_u8_arr(v, p->dae_comp, SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "AutoHistOffset") == 0)
            parse_u8_arr(v, p->dae_hist, SCENE_DYN_LEVELS);

    } else if (strcasecmp(section, "dynamic_nr") == 0) {
        int idx;
        if      (strcasecmp(name, "CoringRatioCount") == 0)
            p->dnr_cnt = (HI_U32)atoi(v);
        else if (strcasecmp(name, "CoringRatioIso") == 0)
            parse_u64_arr(v, p->dnr_iso, SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "FrameShortStr") == 0)
            parse_u8_arr(v, p->dnr_short, SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "FrameLongStr") == 0)
            parse_u8_arr(v, p->dnr_long, SCENE_DYN_LEVELS);
        else if (sscanf(name, "CoringRatio_%d", &idx) == 1 && idx >= 0 && idx < SCENE_DYN_LEVELS)
            parse_u16_arr(v, p->dnr_coring[idx], HI_ISP_BAYERNR_LUT_LENGTH);

    } else if (strcasecmp(section, "dynamic_blc") == 0) {
        if      (strcasecmp(name, "BLCCount") == 0)
            p->dblc_cnt = (HI_U32)atoi(v);
        else if (strcasecmp(name, "IsoThresh") == 0)
            parse_u64_arr(v, p->dblc_iso, SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "R") == 0)
            parse_u16_arr(v, p->dblc_val[0], SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "Gr") == 0)
            parse_u16_arr(v, p->dblc_val[1], SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "Gb") == 0)
            parse_u16_arr(v, p->dblc_val[2], SCENE_DYN_LEVELS);
        else if (strcasecmp(name, "B") == 0)
            parse_u16_arr(v, p->dblc_val[3], SCENE_DYN_LEVELS);

    } else if (strcasecmp(section, "dynamic_ca") == 0) {
        int idx;
        if      (strcasecmp(name, "CACount") == 0)
            p->dca_cnt = (HI_U32)atoi(v);
        else if (strcasecmp(name, "IsoThresh") == 0)
            parse_u64_arr(v, p->dca_iso, SCENE_DYN_LEVELS);
        else if (sscanf(name, "CAYRatioLut_%d", &idx) == 1 && idx >= 0 && idx < SCENE_DYN_LEVELS) {
            char *buf = (char *)v;
            size_t len = strlen(buf);
            if (len > 0 && buf[len - 1] == '\\')       /* trailing continuation mark */
                buf[len - 1] = '\0';
            int remaining = HI_ISP_CA_YRATIO_LUT_LENGTH - p->dca_fill[idx];
            if (remaining > 0)
                p->dca_fill[idx] += parse_u16_arr(buf, p->dca_lut[idx] + p->dca_fill[idx], remaining);
        }

    } else if (strcasecmp(section, "static_awbex") == 0) {
        p->awbex_present = HI_TRUE;
        if      (strcasecmp(name, "ByPass") == 0)
            p->awbex_bypass = (HI_U8)atoi(v);
        else if (strcasecmp(name, "Tolerance") == 0)
            p->awbex_tolerance = (HI_U8)atoi(v);
        else if (strcasecmp(name, "OutThresh") == 0)
            p->awbex_out_thresh = (HI_U32)strtoul(v, NULL, 10);
        else if (strcasecmp(name, "LowStop") == 0)
            p->awbex_low_stop = (HI_U16)atoi(v);
        else if (strcasecmp(name, "HighStart") == 0)
            p->awbex_high_start = (HI_U16)atoi(v);
        else if (strcasecmp(name, "HighStop") == 0)
            p->awbex_high_stop = (HI_U16)atoi(v);
        else if (strcasecmp(name, "MultiLightSourceEn") == 0)
            p->awbex_multi_ls_en = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "MultiCTWt") == 0)
            parse_u16_arr(v, p->awbex_multi_ct_wt, AWB_MULTI_CT_NUM);

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
        else if (strcasecmp(name, "IsoRatio") == 0)
            parse_s32_arr(v, p->ca_iso_ratio, ISP_AUTO_ISO_STRENGTH_NUM);

    } else if (strcasecmp(section, "static_dehaze") == 0) {
        if (strcasecmp(name, "Enable") == 0)
            p->dehaze_enable = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "DehazeUserLutEnable") == 0)
            p->dehaze_user_lut = atoi(v) ? HI_TRUE : HI_FALSE;
        else if (strcasecmp(name, "DehazeOpType") == 0)
            p->dehaze_op_type = atoi(v);
        else if (strcasecmp(name, "DehazeLut") == 0) {
            /* 256 values continued across ~16 lines (inih calls the handler once
               per continuation line, each terminated by a literal '\' that is
               not part of the data — strip it before tokenizing). */
            char *buf = (char *)v;
            size_t len = strlen(buf);
            if (len > 0 && buf[len - 1] == '\\')
                buf[len - 1] = '\0';
            int remaining = 256 - p->dehaze_lut_fill;
            if (remaining > 0)
                p->dehaze_lut_fill += parse_u8_arr(buf, p->dehaze_lut + p->dehaze_lut_fill, remaining);
        }

    } else if (strcasecmp(section, "dynamic_dehaze") == 0) {
        if (strcasecmp(name, "ExpThreshCnt") == 0)
            p->dehaze_exp_cnt = (HI_U32)atoi(v);
        else if (strcasecmp(name, "ExpThreshLtoH") == 0)
            parse_u32_arr(v, p->dehaze_exp_thresh, 8);
        else if (strcasecmp(name, "ManualDehazeStr") == 0)
            parse_u8_arr(v, p->dehaze_str, 8);

    } else if (strcasecmp(section, "module_state") == 0) {
        for (int i = 0; i < MOD_COUNT; i++) {
            if (strcasecmp(name, k_module_key[i]) == 0) {
                p->mod[i] = atoi(v) ? 1 : 0;
                break;
            }
        }

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
        else if (strcasecmp(name, "LocalMixingBrightMax") == 0)
            { parse_u8_arr(v, p->dld_lm_bright_max, ISP_AUTO_ISO_STRENGTH_NUM); p->dld_lm_present = HI_TRUE; }
        else if (strcasecmp(name, "LocalMixingBrightMin") == 0)
            parse_u8_arr(v, p->dld_lm_bright_min, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "LocalMixingDarkMax") == 0)
            parse_u8_arr(v, p->dld_lm_dark_max, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "LocalMixingDarkMin") == 0)
            parse_u8_arr(v, p->dld_lm_dark_min, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "BrightGainLmt") == 0)
            parse_u8_arr(v, p->dld_bright_gain_lmt, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "BrightGainLmtStep") == 0)
            parse_u8_arr(v, p->dld_bright_gain_lmt_step, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "DarkGainLmtY") == 0)
            parse_u8_arr(v, p->dld_dark_gain_lmt_y, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "DarkGainLmtC") == 0)
            parse_u8_arr(v, p->dld_dark_gain_lmt_c, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "ContrastControl") == 0)
            parse_u8_arr(v, p->dld_contrast_ctrl, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "DetailAdjustFactor") == 0)
            parse_u8_arr(v, p->dld_detail_adjust, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "Asymmetry") == 0)
            { parse_u8_arr(v, p->dld_asymmetry, ISP_AUTO_ISO_STRENGTH_NUM); p->dld_curve_present = HI_TRUE; }
        else if (strcasecmp(name, "SecondPole") == 0)
            parse_u8_arr(v, p->dld_second_pole, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "Compress") == 0)
            parse_u8_arr(v, p->dld_compress, ISP_AUTO_ISO_STRENGTH_NUM);
        else if (strcasecmp(name, "Stretch") == 0)
            parse_u8_arr(v, p->dld_stretch, ISP_AUTO_ISO_STRENGTH_NUM);

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

    } else if (strcasecmp(section, "static_aeweight") == 0) {
        p->ae_weight_present = HI_TRUE;
        if (strncasecmp(name, "ExpWeight_", 10) == 0) {
            int idx = atoi(name + 10);
            if (idx >= 0 && idx < AE_ZONE_ROW)
                parse_u8_arr(v, p->ae_weight[idx], AE_ZONE_COLUMN);
        }

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
 * (1) comes from the scene INI; (2) keeps the ISP default in day and is forced
 * to 0 in night, see below.
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

    /* CSC-level saturation. The stock scene never touches the CSC in day
     * (bStaticCSC=0, [static_csc] Enable=0), so day must keep the ISP default
     * (u8Satu=50 is the neutral point, 100 doubles the chroma). Only night
     * forces u8Satu=0 (grayscale, redundant with the all-zero sat curve).
     * The ISP default is captured on the first call and restored for day. */
    static ISP_CSC_ATTR_S csc_default;
    static int            csc_default_valid = 0;
    ISP_CSC_ATTR_S csc;
    ret = HI_MPI_ISP_GetCSCAttr(0, &csc);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetCSCAttr failed 0x%x", (unsigned)ret);
        return;
    }
    if (!csc_default_valid) {
        csc_default       = csc;
        csc_default_valid = 1;
        LOGGER(LOGGER_LEVEL_INFO,
               "[scene] CSC default: en=%d luma=%u contr=%u hue=%u satu=%u limitedRange=%d",
               (int)csc.bEnable, csc.u8Luma, csc.u8Contr, csc.u8Hue, csc.u8Satu,
               (int)csc.bLimitedRangeEn);
    }

    csc = csc_default;
    if (grayscale) csc.u8Satu = 0;
    ret = HI_MPI_ISP_SetCSCAttr(0, &csc);
    LOGGER(LOGGER_LEVEL_DEBUG, "[scene] SetCSCAttr u8Satu=%u limitedRange=%d ret=0x%x",
           csc.u8Satu, (int)csc.bLimitedRangeEn, (unsigned)ret);
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

    /* au16CoarseStr is intentionally left untouched here: it's sensor-intrinsic
       (no [static_nr] CoarseStr key in either INI), now supplied byte-exact by
       jxf22_cmos.c cmos_get_isp_default() via the CMOS-default unKey channel —
       Get above already picked up that value, so re-Set below just writes it
       straight back unmodified. */
    ret = HI_MPI_ISP_SetNRAttr(0, &attr);
    LOGGER(LOGGER_LEVEL_DEBUG, "[scene] SetNRAttr fine[0]=%u coring[0]=%u ret=0x%x",
           p->nr_fine_str[0], p->nr_coring_wgt[0], (unsigned)ret);
}

/* Gamma tone curve — the stock vendor curve (1025 nodes, from libsns_f22.so).
   The ISP default gamma lifts shadows/mids more than the vendor curve, making
   our image flat/washed out (measured: std 48 vs 66, blacks lifted, highlights
   compressed). Apply the extracted curve via the runtime channel so we get the
   original's contrast (deeper blacks, brighter highlights). */
static void apply_gamma(void)
{
    ISP_GAMMA_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetGammaAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetGammaAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable     = HI_TRUE;
    attr.enCurveType = ISP_GAMMA_CURVE_USER_DEFINE;
    memcpy(attr.u16Table, g_jxf22_gamma, sizeof(g_jxf22_gamma));

    ret = HI_MPI_ISP_SetGammaAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetGammaAttr failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO, "[scene] Gamma applied (vendor 1025-node, mid=%u)",
               g_jxf22_gamma[512]);
}

/* Dehaze — the original firmware runs dehaze (bStaticDehaze=1, bDynamicDehaze=1)
   but it does NOT appear in /proc/umap/isp, which is why our "all /proc params
   match" comparison missed it. Verified by RE of libsceneauto.so
   (HI_SCENE_SetStaticDeHaze / HI_SCENE_SetDynamicDehaze): the LUT is per-profile
   (day: flat pass-through 255x256; night: a real 107->255 ramp) and the manual
   strength is re-interpolated continuously from [dynamic_dehaze] by raw exposure
   (u32Exposure), not fixed at 90 — night targets 120-130, not 90. This function
   now applies the static half (enable/LUT/op-type); apply_dynamic_dehaze()
   below re-applies the strength on every exposure change, same pattern as DRC. */
static void apply_dehaze(const scene_params_t *p)
{
    ISP_DEHAZE_ATTR_S attr;
    HI_S32 ret = HI_MPI_ISP_GetDehazeAttr(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetDehazeAttr failed 0x%x", (unsigned)ret);
        return;
    }

    attr.bEnable        = p->dehaze_enable;
    attr.bUserLutEnable = p->dehaze_user_lut;
    memcpy(attr.au8DehazeLut, p->dehaze_lut, sizeof(attr.au8DehazeLut));
    attr.enOpType = p->dehaze_op_type ? OP_TYPE_MANUAL : OP_TYPE_AUTO;
    if (p->dehaze_exp_cnt > 0)
        attr.stManual.u8strength = p->dehaze_str[0];

    ret = HI_MPI_ISP_SetDehazeAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetDehazeAttr failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO, "[scene] Dehaze enabled=%d lut[0]=%u strength=%u",
               (int)attr.bEnable, attr.au8DehazeLut[0], attr.stManual.u8strength);
}

/* Re-interpolates the manual dehaze strength from [dynamic_dehaze] ExpThreshLtoH
   / ManualDehazeStr by raw exposure (ISP_EXP_INFO_S.u32Exposure), mirroring
   dld_interp() for DRC. Called from scene_dyn_thread on every exposure change. */
static void apply_dynamic_dehaze(const scene_params_t *p, HI_U32 exposure)
{
    HI_U32 n = p->dehaze_exp_cnt;
    if (n == 0) return;
    if (n > 8) n = 8;

    HI_U8 strength;
    if (exposure <= p->dehaze_exp_thresh[0])
        strength = p->dehaze_str[0];
    else if (exposure >= p->dehaze_exp_thresh[n - 1])
        strength = p->dehaze_str[n - 1];
    else {
        strength = p->dehaze_str[n - 1];
        for (HI_U32 i = 1; i < n; i++) {
            if (exposure <= p->dehaze_exp_thresh[i]) {
                HI_S32 lo = (HI_S32)p->dehaze_exp_thresh[i - 1], hi = (HI_S32)p->dehaze_exp_thresh[i];
                HI_S32 slo = p->dehaze_str[i - 1], shi = p->dehaze_str[i];
                strength = (hi == lo) ? (HI_U8)slo
                                      : (HI_U8)(slo + (shi - slo) * ((HI_S32)exposure - lo) / (hi - lo));
                break;
            }
        }
    }

    ISP_DEHAZE_ATTR_S attr;
    if (HI_MPI_ISP_GetDehazeAttr(0, &attr) != HI_SUCCESS)
        return;
    attr.stManual.u8strength = strength;
    HI_MPI_ISP_SetDehazeAttr(0, &attr);
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
    /* [static_ca] IsoRatio — sensor-per-ISO CA correction ratio (Q1.10, 1024 =
       neutral). Verified via RE of libsceneauto.so HI_SCENE_SetStaticCA: it
       patches only bEnable + this table, never stCA.au32YRatioLut. Without it
       we stayed at the ISP's neutral default (1024) instead of the day INI's
       {1300,1300,1250,...} ramp — negligible visual impact (edge fringing
       only) but now byte-faithful to the original. */
    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++)
        attr.stCA.as32ISORatio[i] = p->ca_iso_ratio[i];

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
    if (!MOD_ON(p, STATIC_DRC))
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
    if (!MOD_ON(p, STATIC_SHARPEN))
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

/* AE metering zone weight table — the original always applies this (day AND
   night INI both carry [static_aeweight], center-weighted). We never called
   it, so our AE ran on the ISP's flat default weighting -> averages in more
   of the frame edges than the original -> our AE target luma reads higher
   than the original's at the same Compensation (the scorecard's last daytime
   gap). Faithful to libsceneauto HI_SCENE_SetAeWeightTab: Get->patch
   au8Weight[15][17]->Set, nothing else touched (no unKey bit gates this). */
static void apply_ae_weight(const scene_params_t *p)
{
    if (!p->ae_weight_present)
        return;

    ISP_STATISTICS_CFG_S attr;
    HI_S32 ret = HI_MPI_ISP_GetStatisticsConfig(0, &attr);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetStatisticsConfig failed 0x%x", (unsigned)ret);
        return;
    }

    memcpy(attr.stAECfg.au8Weight, p->ae_weight, sizeof(attr.stAECfg.au8Weight));

    ret = HI_MPI_ISP_SetStatisticsConfig(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetStatisticsConfig failed 0x%x", (unsigned)ret);
    else
        LOGGER(LOGGER_LEVEL_INFO, "[scene] AE weight table applied, center=%u",
               p->ae_weight[7][8]);
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

/* Generic per-ISO linear interpolation over a HI_U8 table (same node grid as
   dld_interp), for the [dynamic_linear_drc] local-contrast parameters. */
static HI_U8 dld_interp_u8(const scene_params_t *p, const HI_U8 *val, HI_U32 iso)
{
    HI_U32 n = p->dld_iso_cnt;
    if (n == 0) return 0;
    if (n > ISP_AUTO_ISO_STRENGTH_NUM) n = ISP_AUTO_ISO_STRENGTH_NUM;
    if (iso <= p->dld_iso_level[0])     return val[0];
    if (iso >= p->dld_iso_level[n - 1]) return val[n - 1];
    for (HI_U32 i = 1; i < n; i++) {
        if (iso <= p->dld_iso_level[i]) {
            HI_S32 lo = (HI_S32)p->dld_iso_level[i - 1], hi = (HI_S32)p->dld_iso_level[i];
            HI_S32 vlo = val[i - 1], vhi = val[i];
            if (hi == lo) return (HI_U8)vlo;
            return (HI_U8)(vlo + (vhi - vlo) * ((HI_S32)iso - lo) / (hi - lo));
        }
    }
    return val[n - 1];
}

static void apply_dynamic_drc(const scene_params_t *p, HI_U32 iso)
{
    if (!p || !MOD_ON(p, DYN_LINEARDRC) || !p->dld_enable) return;

    ISP_DRC_ATTR_S attr;
    if (HI_MPI_ISP_GetDRCAttr(0, &attr) != HI_SUCCESS) return;

    HI_U16 s = dld_interp(p, iso);
    if (s > 512) s = 512; /* SDK clamp */
    /* Write to whichever slot the current op-type uses. */
    attr.stManual.u16Strength = s;
    attr.stAuto.u16Strength   = s;

    /* Local dynamic-contrast fields (top-level ISP_DRC_ATTR_S) — the stock
       firmware modulates these per-ISO too. DarkGainLmtY limits shadow gain
       (deepens blacks), ContrastControl sets contrast, LocalMixingDark/Bright
       shape local contrast. Without them the DRC was flat with lifted shadows. */
    if (p->dld_lm_present) {
        attr.u8LocalMixingBrightMax = dld_interp_u8(p, p->dld_lm_bright_max, iso);
        attr.u8LocalMixingBrightMin = dld_interp_u8(p, p->dld_lm_bright_min, iso);
        attr.u8LocalMixingDarkMax   = dld_interp_u8(p, p->dld_lm_dark_max, iso);
        attr.u8LocalMixingDarkMin   = dld_interp_u8(p, p->dld_lm_dark_min, iso);
        attr.u8BrightGainLmt        = dld_interp_u8(p, p->dld_bright_gain_lmt, iso);
        attr.u8BrightGainLmtStep    = dld_interp_u8(p, p->dld_bright_gain_lmt_step, iso);
        attr.u8DarkGainLmtY         = dld_interp_u8(p, p->dld_dark_gain_lmt_y, iso);
        attr.u8DarkGainLmtC         = dld_interp_u8(p, p->dld_dark_gain_lmt_c, iso);
        attr.u8ContrastControl      = dld_interp_u8(p, p->dld_contrast_ctrl, iso);
        attr.s8DetailAdjustFactor   = (HI_S8)dld_interp_u8(p, p->dld_detail_adjust, iso);
    }

    /* Asymmetry tone-mapping curve (static_drc CurveSelect=0): the stock scene
       sets its four parameters per ISO too (HI_SCENE_SetDynamicLinearDRC); we
       used to leave the ISP defaults. FltScaleCoarse/Fine are not used by this
       chip (Hi3559A/3519A only). */
    if (p->dld_curve_present) {
        attr.stAsymmetryCurve.u8Asymmetry  = dld_interp_u8(p, p->dld_asymmetry, iso);
        attr.stAsymmetryCurve.u8SecondPole = dld_interp_u8(p, p->dld_second_pole, iso);
        attr.stAsymmetryCurve.u8Compress   = dld_interp_u8(p, p->dld_compress, iso);
        attr.stAsymmetryCurve.u8Stretch    = dld_interp_u8(p, p->dld_stretch, iso);
    }

    HI_MPI_ISP_SetDRCAttr(0, &attr);
}

/* -------------------------------------------------------------------------
 * Dynamic tables (libsceneauto: HI_SCENE_SetDynamicAE / Nr / BLC / CA)
 *
 * All four pick a level with GetLevelLtoH (first threshold >= value, clamped
 * to the last one), use row 0 as is for level 0 and otherwise interpolate
 * linearly between rows l-1 and l, clamping outside [thr[l-1], thr[l]].
 * ---------------------------------------------------------------------- */

static HI_U32 dyn_interp(HI_U64 x, HI_U64 xl, HI_U32 yl, HI_U64 xr, HI_U32 yr)
{
    if (x <= xl) return yl;
    if (x >= xr) return yr;
    HI_U64 k = xr - xl;
    return (HI_U32)(((xr - x) * yl + (x - xl) * yr + (k >> 1)) / k);
}

/* Selects the rows to blend; lo == hi for level 0. */
static void dyn_rows(HI_U64 v, const HI_U64 *thr, HI_U32 n, HI_U32 *lo, HI_U32 *hi)
{
    if (n > SCENE_DYN_LEVELS) n = SCENE_DYN_LEVELS;
    HI_U32 l = 0;
    while (l < n && v > thr[l]) l++;
    if (l >= n) l = n - 1;
    *lo = (l == 0) ? 0 : l - 1;
    *hi = l;
}

#define DYN_VAL(v, thr, tab, lo, hi) \
    (((lo) == (hi)) ? (HI_U32)(tab)[hi] : dyn_interp((v), (thr)[lo], (tab)[lo], (thr)[hi], (tab)[hi]))

/* AE: compensation + max hist offset by exposure64 (HI_SCENE_SetDynamicAE,
   linear branch). The smart-exposure part of the original only runs when
   something else enabled SmartExposure; not handled here. */
static void apply_dynamic_ae(const scene_params_t *p, HI_U64 exposure)
{
    if (p->dae_cnt == 0) return;
    HI_U32 lo, hi;
    dyn_rows(exposure, p->dae_thr, p->dae_cnt, &lo, &hi);

    ISP_EXPOSURE_ATTR_S attr;
    if (HI_MPI_ISP_GetExposureAttr(0, &attr) != HI_SUCCESS) return;
    attr.stAuto.u8Compensation  = (HI_U8)DYN_VAL(exposure, p->dae_thr, p->dae_comp, lo, hi);
    attr.stAuto.u8MaxHistOffset = (HI_U8)DYN_VAL(exposure, p->dae_thr, p->dae_hist, lo, hi);
    HI_S32 ret = HI_MPI_ISP_SetExposureAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] dynamic AE SetExposureAttr failed 0x%x", (unsigned)ret);
}

/* NR: coring ratio LUT by ISO; WDR frame strengths follow the lower row. */
static void apply_dynamic_nr(const scene_params_t *p, HI_U32 iso)
{
    if (p->dnr_cnt == 0) return;
    HI_U32 lo, hi;
    dyn_rows(iso, p->dnr_iso, p->dnr_cnt, &lo, &hi);

    ISP_NR_ATTR_S attr;
    if (HI_MPI_ISP_GetNRAttr(0, &attr) != HI_SUCCESS) return;
    for (int i = 0; i < HI_ISP_BAYERNR_LUT_LENGTH; i++) {
        HI_U32 a = p->dnr_coring[lo][i], b = p->dnr_coring[hi][i];
        attr.au16CoringRatio[i] = (HI_U16)((lo == hi) ? a : dyn_interp(iso, p->dnr_iso[lo], a, p->dnr_iso[hi], b));
    }
    attr.stWdr.au8WDRFrameStr[0]    = p->dnr_short[lo];
    attr.stWdr.au8WDRFrameStr[1]    = p->dnr_long[lo];
    attr.stWdr.au8FusionFrameStr[0] = p->dnr_short[lo];
    attr.stWdr.au8FusionFrameStr[1] = p->dnr_long[lo];
    HI_S32 ret = HI_MPI_ISP_SetNRAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] dynamic NR SetNRAttr failed 0x%x", (unsigned)ret);
}

/* BLC: manual black level by ISO (R, Gr, Gb, B). */
static void apply_dynamic_blc(const scene_params_t *p, HI_U32 iso)
{
    if (p->dblc_cnt == 0) return;
    HI_U32 lo, hi;
    dyn_rows(iso, p->dblc_iso, p->dblc_cnt, &lo, &hi);

    ISP_BLACK_LEVEL_S attr;
    if (HI_MPI_ISP_GetBlackLevelAttr(0, &attr) != HI_SUCCESS) return;
    attr.enOpType = OP_TYPE_MANUAL;
    for (int c = 0; c < ISP_BAYER_CHN_NUM; c++)
        attr.au16BlackLevel[c] = (HI_U16)DYN_VAL(iso, p->dblc_iso, p->dblc_val[c], lo, hi);
    HI_S32 ret = HI_MPI_ISP_SetBlackLevelAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] dynamic BLC SetBlackLevelAttr failed 0x%x", (unsigned)ret);
}

/* CA: luma ratio LUT by ISO. A table that the INI parser could not fill
   completely (continuation lines not indented) is ignored. */
static void apply_dynamic_ca(const scene_params_t *p, HI_U32 iso)
{
    if (p->dca_cnt == 0) return;
    HI_U32 lo, hi;
    dyn_rows(iso, p->dca_iso, p->dca_cnt, &lo, &hi);
    if (p->dca_fill[lo] < HI_ISP_CA_YRATIO_LUT_LENGTH || p->dca_fill[hi] < HI_ISP_CA_YRATIO_LUT_LENGTH)
        return;

    ISP_CA_ATTR_S attr;
    if (HI_MPI_ISP_GetCAAttr(0, &attr) != HI_SUCCESS) return;
    for (int i = 0; i < HI_ISP_CA_YRATIO_LUT_LENGTH; i++) {
        HI_U32 a = p->dca_lut[lo][i], b = p->dca_lut[hi][i];
        attr.stCA.au32YRatioLut[i] = (lo == hi) ? a : dyn_interp(iso, p->dca_iso[lo], a, p->dca_iso[hi], b);
    }
    HI_S32 ret = HI_MPI_ISP_SetCAAttr(0, &attr);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] dynamic CA SetCAAttr failed 0x%x", (unsigned)ret);
}

/* Static AWBEX (HI_SCENE_SetStaticAWBEX): WB bypass flag, then the extended
   AWB tolerance / indoor-outdoor thresholds / multi light source weights. */
static void apply_awbex(const scene_params_t *p)
{
    if (!p->awbex_present) return;

    ISP_WB_ATTR_S wb;
    HI_S32 ret = HI_MPI_ISP_GetWBAttr(0, &wb);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetWBAttr failed 0x%x", (unsigned)ret);
        return;
    }
    wb.bByPass = p->awbex_bypass ? HI_TRUE : HI_FALSE;
    ret = HI_MPI_ISP_SetWBAttr(0, &wb);
    if (ret != HI_SUCCESS)
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] SetWBAttr(bypass) failed 0x%x", (unsigned)ret);

    ISP_AWB_ATTR_EX_S ex;
    ret = HI_MPI_ISP_GetAWBAttrEx(0, &ex);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] GetAWBAttrEx failed 0x%x", (unsigned)ret);
        return;
    }
    ex.u8Tolerance             = p->awbex_tolerance;
    ex.stInOrOut.u32OutThresh  = p->awbex_out_thresh;
    ex.stInOrOut.u16LowStop    = p->awbex_low_stop;
    ex.stInOrOut.u16HighStart  = p->awbex_high_start;
    ex.stInOrOut.u16HighStop   = p->awbex_high_stop;
    ex.bMultiLightSourceEn     = p->awbex_multi_ls_en;
    for (int i = 0; i < AWB_MULTI_CT_NUM; i++)
        ex.au16MultiCTWt[i] = p->awbex_multi_ct_wt[i];
    ret = HI_MPI_ISP_SetAWBAttrEx(0, &ex);
    LOGGER(LOGGER_LEVEL_INFO, "[scene] AWBEX bypass=%u tol=%u outThresh=%u ct=[%u,%u,%u] multi=%d ret=0x%x",
           (unsigned)p->awbex_bypass, p->awbex_tolerance, p->awbex_out_thresh, p->awbex_low_stop,
           p->awbex_high_start, p->awbex_high_stop, (int)p->awbex_multi_ls_en, (unsigned)ret);
}

/* Last values seen by the dynamic setters; g_dyn_force makes the next pass
   re-apply everything (set after every static re-apply, since SetSceneMode
   resets what they patched). */
static HI_U32 g_last_iso_nr, g_last_iso_ca, g_last_iso_blc;
static HI_U64 g_last_exp_ae;
static volatile int g_dyn_force = 1;

/* One pass of the dynamic setters, like the original's Normal/Luminance threads
   but at our 1 s cadence. Each setter fires on a change of its input. */
static void run_dynamic(const scene_params_t *p)
{
    ISP_EXP_INFO_S exp;
    if (HI_MPI_ISP_QueryExposureInfo(0, &exp) != HI_SUCCESS)
        return;
    HI_U32 iso = exp.u32ISO;
    HI_U64 exposure64 = (HI_U64)exp.u32ExpTime * iso / 100;
    int force = g_dyn_force;
    g_dyn_force = 0;

    apply_dynamic_drc(p, iso);
    if (MOD_ON(p, DYN_DEHAZE))
        apply_dynamic_dehaze(p, exp.u32Exposure);
    if (MOD_ON(p, DYN_AE) && (force || exposure64 != g_last_exp_ae)) {
        g_last_exp_ae = exposure64;
        apply_dynamic_ae(p, exposure64);
    }
    if (MOD_ON(p, DYN_NR) && (force || iso != g_last_iso_nr)) {
        g_last_iso_nr = iso;
        apply_dynamic_nr(p, iso);
    }
    if (MOD_ON(p, DYN_CA) && (force || iso != g_last_iso_ca)) {
        g_last_iso_ca = iso;
        apply_dynamic_ca(p, iso);
    }
    if (MOD_ON(p, DYN_BLC) && (force || iso != g_last_iso_blc)) {
        g_last_iso_blc = iso;
        apply_dynamic_blc(p, iso);
    }
}

/* Polls the ISP exposure once per second and runs the dynamic setters for the
   active (day/night) profile. */
static void *scene_dyn_thread(void *arg)
{
    (void)arg;
    while (g_dynRun) {
        const scene_params_t *p = g_active;
        if (p)
            run_dynamic(p);
        sleep(1);
    }
    return NULL;
}

/* Each static module is applied only when its [module_state] flag is set, as in
   libsceneauto (HI_SCENE_SetStatic* test the flag themselves; a missing flag
   means off). Gamma follows bDynamicGamma: the original only ever programs it
   from the luminance thread (HI_SCENE_SetDynamicVideoGamma). */
static void apply_scene(const scene_params_t *p)
{
    if (MOD_ON(p, STATIC_AE))         apply_ae(p);
    if (MOD_ON(p, AEWEIGHTTAB))       apply_ae_weight(p);
    if (MOD_ON(p, STATIC_AWB))        apply_awb(p);
    if (MOD_ON(p, STATIC_AWBEX))      apply_awbex(p);
    if (MOD_ON(p, STATIC_CCM))        apply_ccm(p);
    if (MOD_ON(p, STATIC_SATURATION)) apply_saturation(p);
    if (MOD_ON(p, STATIC_NR))         apply_nr(p);
    if (MOD_ON(p, DYN_GAMMA))         apply_gamma();
    if (MOD_ON(p, STATIC_DEHAZE))     apply_dehaze(p);
    if (MOD_ON(p, STATIC_CA))         apply_ca(p);
    if (MOD_ON(p, STATIC_LDCI))       apply_ldci(p);
    if (MOD_ON(p, STATIC_DPC))        apply_dpc(p);
    if (MOD_ON(p, STATIC_DRC))        apply_drc(p);
    if (MOD_ON(p, STATIC_SHARPEN))    apply_sharpen(p);

    /* The static pass above reset what the dynamic setters patch: run them now
       instead of waiting for the next 1 s tick. */
    g_dyn_force = 1;
    run_dynamic(p);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/* One-line dump of the enabled [module_state] flags of a profile. */
static void log_modules(const char *tag, const scene_params_t *p)
{
    char buf[512];
    size_t n = 0;
    buf[0] = '\0';
    for (int i = 1; i < MOD_COUNT && n < sizeof(buf) - 32; i++)
        if (p->mod[i])
            n += (size_t)snprintf(buf + n, sizeof(buf) - n, " %s", k_module_key[i]);
    LOGGER(LOGGER_LEVEL_INFO, "[scene] %s modules on:%s", tag, buf);
}

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
    if (!g_dynRun && (MOD_ON(&g_day, DYN_LINEARDRC) || MOD_ON(&g_night, DYN_LINEARDRC) ||
                      MOD_ON(&g_day, DYN_DEHAZE) || MOD_ON(&g_night, DYN_DEHAZE) ||
                      MOD_ON(&g_day, DYN_AE) || MOD_ON(&g_night, DYN_AE) ||
                      MOD_ON(&g_day, DYN_NR) || MOD_ON(&g_night, DYN_NR) ||
                      MOD_ON(&g_day, DYN_CA) || MOD_ON(&g_day, DYN_BLC))) {
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
    log_modules("day", &g_day);
    log_modules("night", &g_night);
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
