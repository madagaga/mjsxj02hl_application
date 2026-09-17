/* JXF22 CMOS sensor driver — AE/AWB/ISP callbacks + ISP_SNS_OBJ_S export.
 * Ported from soi_f22__soc_v3 (glutinium) to hi3516ev200 ISP_SNS_OBJ_S model.
 *
 * API changes vs old model:
 *   - All pfn_cmos_* take VI_PIPE as first argument.
 *   - pfnRegisterCallback(VI_PIPE, AE_LIB, AWB_LIB) replaces sensor_register_callback().
 *   - Per-frame AE/gain updates fill g_stSnsRegsInfo.astI2cData[] shadow regs;
 *     the ISP kernel driver writes I2C at VD interrupt time.
 *   - ISP_CMOS_DEFAULT_S fields that don't map to hi3516ev200 structs are NULLed;
 *     ISP uses built-in defaults for those. Demosaic/BayerNR/AntiFalseColor are
 *     populated with tables extracted byte-exact from the vendor libsns_f22.so
 *     cmos_get_isp_default() (Ghidra decompile @0x11808) — these are sensor-
 *     intrinsic noise/optics calibration, not scene (day/night) config, so they
 *     belong here rather than in scene.c. Gamma/Sharpen/Ldci/Drc/Dehaze are left
 *     unset: the vendor also ships static defaults for those, but scene.c already
 *     applies the real per-firmware values at runtime via HI_MPI_ISP_Set*Attr
 *     (validated against /proc/umap/isp — see CLAUDE.md), which would immediately
 *     override a sensor-static duplicate here.
 */

#include <stdio.h>
#include <string.h>

#include "hi_comm_sns.h"
#include "hi_comm_video.h"
#include "hi_sns_ctrl.h"
#include "mpi_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"

#include "jxf22.h"

#define JXF22_ID  22

/* ---- AE state ---------------------------------------------------------- */

static HI_U32 gu32FullLinesStd = VMAX_1080P30_LINEAR;
static HI_U32 gu32FullLines    = VMAX_1080P30_LINEAR;
static HI_BOOL bInit           = HI_FALSE;

static HI_U16 g_au16InitWBGain[3]  = {0};
static HI_U16 g_au16SampleRgain    = 0;
static HI_U16 g_au16SampleBgain    = 0;

/* I2C bus stored by pfnSetBusInfo, passed to sensor_i2c_init */
static HI_S8 g_i2cBus = 0;

/* ---- AE callbacks ------------------------------------------------------- */

static HI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    (void)ViPipe;
    if (!pstAeSnsDft) return HI_FAILURE;

    memset(&pstAeSnsDft->stAERouteAttr, 0, sizeof(ISP_AE_ROUTE_S));

    pstAeSnsDft->u32LinesPer500ms = gu32FullLinesStd * 30 / 2;
    pstAeSnsDft->u32FullLinesStd  = gu32FullLinesStd;
    pstAeSnsDft->u32FlickerFreq   = 0;
    pstAeSnsDft->u32FullLinesMax  = FULL_LINES_MAX;

    pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
    pstAeSnsDft->stIntTimeAccu.f32Accuracy = 1;
    pstAeSnsDft->stIntTimeAccu.f32Offset   = 0;

    pstAeSnsDft->stAgainAccu.enAccuType  = AE_ACCURACY_TABLE;
    pstAeSnsDft->stAgainAccu.f32Accuracy = 1;

    pstAeSnsDft->stDgainAccu.enAccuType  = AE_ACCURACY_LINEAR;
    pstAeSnsDft->stDgainAccu.f32Accuracy = 0.0625f;

    pstAeSnsDft->u32MaxDgain = 16;
    pstAeSnsDft->u32MinDgain = 16;

    pstAeSnsDft->u32ISPDgainShift      = 8;
    pstAeSnsDft->u32MinISPDgainTarget  = 1 << pstAeSnsDft->u32ISPDgainShift;
    pstAeSnsDft->u32MaxISPDgainTarget  = 8 << pstAeSnsDft->u32ISPDgainShift; /* original=8192 (8×) vs default 4× */

    pstAeSnsDft->enMaxIrisFNO = ISP_IRIS_F_NO_1_0;
    pstAeSnsDft->enMinIrisFNO = ISP_IRIS_F_NO_32_0;

    pstAeSnsDft->bAERouteExValid          = HI_FALSE;
    pstAeSnsDft->stAERouteAttr.u32TotalNum   = 0;
    pstAeSnsDft->stAERouteAttrEx.u32TotalNum = 0;

    pstAeSnsDft->u32InitExposure = 921600;

    /* Linear 1080p30 */
    pstAeSnsDft->au8HistThresh[0] = 0x0d;
    pstAeSnsDft->au8HistThresh[1] = 0x28;
    pstAeSnsDft->au8HistThresh[2] = 0x60;
    pstAeSnsDft->au8HistThresh[3] = 0x80;
    pstAeSnsDft->u8AeCompensation = 0x32;

    gu32FullLinesStd = VMAX_1080P30_LINEAR;
    pstAeSnsDft->u32MaxIntTime       = gu32FullLinesStd - 4;
    pstAeSnsDft->u32MinIntTime       = 2;
    pstAeSnsDft->u32MaxIntTimeTarget = gu32FullLinesStd - 2; /* cap at ~1 frame; original=VMAX-2 */
    pstAeSnsDft->u32MinIntTimeTarget = 2;
    pstAeSnsDft->u32MaxAgain         = 15872; /* 15.5× */
    pstAeSnsDft->u32MinAgain         = 1024;
    pstAeSnsDft->u32MaxAgainTarget   = pstAeSnsDft->u32MaxAgain;
    pstAeSnsDft->u32MinAgainTarget   = pstAeSnsDft->u32MinAgain;
    pstAeSnsDft->u32MaxDgainTarget   = pstAeSnsDft->u32MaxDgain;
    pstAeSnsDft->u32MinDgainTarget   = pstAeSnsDft->u32MinDgain;
    pstAeSnsDft->u32LinesPer500ms    = gu32FullLinesStd * 30 / 2;

    return HI_SUCCESS;
}

static HI_VOID cmos_fps_set(VI_PIPE ViPipe, HI_FLOAT f32Fps,
                             AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    (void)ViPipe;

    if (SENSOR_1080P_30FPS_MODE == gu8SensorImageMode) {
        if (f32Fps > 0.5f && f32Fps <= 30.0f)
            gu32FullLinesStd = (HI_U32)((VMAX_1080P30_LINEAR * 30.0f) / f32Fps);
        else {
            printf("[sns][jxf22] unsupported fps %.1f\n", f32Fps);
            return;
        }
    }

    gu32FullLinesStd = (gu32FullLinesStd > FULL_LINES_MAX) ? FULL_LINES_MAX : gu32FullLinesStd;

    g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_L].u32Data = gu32FullLinesStd & 0xff;
    g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_H].u32Data = (gu32FullLinesStd >> 8) & 0xff;

    pstAeSnsDft->f32Fps              = f32Fps;
    pstAeSnsDft->u32LinesPer500ms    = (HI_U32)(gu32FullLinesStd * f32Fps / 2.0f);
    pstAeSnsDft->u32MaxIntTime       = gu32FullLinesStd - 4;
    pstAeSnsDft->u32MaxIntTimeTarget = gu32FullLinesStd - 2;
    pstAeSnsDft->u32FullLinesStd     = gu32FullLinesStd;
    gu32FullLines = gu32FullLinesStd;
    pstAeSnsDft->u32FullLines     = gu32FullLines;
}

static HI_VOID cmos_slow_framerate_set(VI_PIPE ViPipe, HI_U32 u32FullLines,
                                        AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    (void)ViPipe;

    if (WDR_MODE_NONE != genSensorMode) {
        pstAeSnsDft->u32FullLines = gu32FullLinesStd;
        return;
    }

    u32FullLines = (u32FullLines > FULL_LINES_MAX) ? FULL_LINES_MAX : u32FullLines;
    gu32FullLines = u32FullLines;
    pstAeSnsDft->u32FullLines = gu32FullLines;

    g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_L].u32Data = u32FullLines & 0xff;
    g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_H].u32Data = (u32FullLines >> 8) & 0xff;

    pstAeSnsDft->u32MaxIntTime = gu32FullLines - 4;
}

static HI_VOID cmos_inttime_update(VI_PIPE ViPipe, HI_U32 u32IntTime)
{
    (void)ViPipe;
    g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_L].u32Data  = u32IntTime & 0xff;
    g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_L].bUpdate  = HI_TRUE;
    g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_H].u32Data  = (u32IntTime >> 8) & 0xff;
    g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_H].bUpdate  = HI_TRUE;
}

/* Analog gain table: sensor AGC register value = (gain_lin>>6) mapped to 5-bit field. */
static HI_VOID cmos_again_calc_table(VI_PIPE ViPipe,
                                      HI_U32 *pu32AgainLin, HI_U32 *pu32AgainDb)
{
    HI_U32 again;
    HI_U32 shift = 0;
    (void)ViPipe;

    again = *pu32AgainLin >> 6; /* 10-bit linear → 4-bit */
    while (again > 31) {
        again >>= 1;
        shift++;
    }
    if (again > 15)
        again -= 16;

    *pu32AgainDb = (shift << 4) | again;
}

static HI_VOID cmos_gains_update(VI_PIPE ViPipe,
                                  HI_U32 u32Again, HI_U32 u32Dgain)
{
    (void)ViPipe;
    (void)u32Dgain; /* sensor has no digital gain register */
    g_stSnsRegsInfo.astI2cData[SNS_REG_AGC].u32Data  = u32Again & 0xff;
    g_stSnsRegsInfo.astI2cData[SNS_REG_AGC].bUpdate  = HI_TRUE;
}

static HI_S32 cmos_init_ae_exp_function(AE_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    memset(pstExpFuncs, 0, sizeof(AE_SENSOR_EXP_FUNC_S));
    pstExpFuncs->pfn_cmos_get_ae_default     = cmos_get_ae_default;
    pstExpFuncs->pfn_cmos_fps_set            = cmos_fps_set;
    pstExpFuncs->pfn_cmos_slow_framerate_set = cmos_slow_framerate_set;
    pstExpFuncs->pfn_cmos_inttime_update     = cmos_inttime_update;
    pstExpFuncs->pfn_cmos_gains_update       = cmos_gains_update;
    pstExpFuncs->pfn_cmos_again_calc_table   = cmos_again_calc_table;
    return HI_SUCCESS;
}

/* ---- AWB callbacks ------------------------------------------------------- */

/* CCM tables extracted from vendor firmware sensor.ini (AutoCCMTable_0..4,
 * AutoColorTemp = "7526, 6500, 5359, 3925, 2665"). Values are HI_U16 in
 * Format:8.8 (256 = 1.0); bit 15 set means negative magnitude. Replacing
 * the glutinium tables which had R diagonal ≈2.44 vs vendor 1.80, causing
 * a violet tint on this specific camera. */
static AWB_CCM_S g_stAwbCcm = {
    .u16CCMTabNum = 5,
    .astCCMTab = {
        { 7526, { 0x01D9, 0x80CC, 0x800D,
                  0x803E, 0x01F6, 0x80B8,
                  0x8016, 0x80AE, 0x01C4 } },
        { 6500, { 0x020A, 0x80F9, 0x8011,
                  0x8051, 0x020B, 0x80BA,
                  0x801C, 0x8083, 0x019F } },
        { 5359, { 0x01CC, 0x80C6, 0x8006,
                  0x8052, 0x0205, 0x80B3,
                  0x8021, 0x810B, 0x022C } },
        { 3925, { 0x01DA, 0x80C1, 0x8019,
                  0x8045, 0x01F3, 0x80AE,
                  0x8014, 0x8150, 0x0264 } },
        { 2665, { 0x0215, 0x8101, 0x8014,
                  0x8064, 0x0184, 0x8020,
                  0x8089, 0x8207, 0x0390 } },
    },
};

static AWB_AGC_TABLE_S g_stAwbAgcTable = {
    .bValid = 1,
    .au8Saturation = {118, 115, 110, 100, 90, 85, 75, 65, 65, 65, 70, 64, 64, 56, 56, 56},
};

#define GOLDEN_RGAIN 0
#define GOLDEN_BGAIN 0

/* AWB sensor calibration — INTRINSIC to the JXF22, extracted byte-exact from the
 * vendor libsns_f22.so cmos_get_awb_default (Ghidra/objdump @0x1614). These are
 * sensor-driver data (same for any camera with this sensor), NOT the camera
 * scene tuning: the INI [static_awb] (AutoStaticWb/AutoCurvePara) is a separate
 * thing applied at runtime via scene.c apply_awb (ISP_WB_ATTR_S). Our earlier
 * values (gain offset B=455, refTemp 6500, wrong curve) made the AWB converge
 * wrong -> magenta. */
static const HI_U16 g_awbGainOffset[4] = { 410, 256, 256, 350 };
static const HI_S32 g_awbWbPara[6]     = { 59, 197, 0, 170828, 128, -118550 };

static HI_S32 cmos_get_awb_default(VI_PIPE ViPipe, AWB_SENSOR_DEFAULT_S *pstAwbSnsDft)
{
    (void)ViPipe;
    if (!pstAwbSnsDft) return HI_FAILURE;

    memset(pstAwbSnsDft, 0, sizeof(AWB_SENSOR_DEFAULT_S));

    pstAwbSnsDft->u16WbRefTemp        = 4970;   /* libsns_f22 value */
    memcpy(pstAwbSnsDft->au16GainOffset, g_awbGainOffset, sizeof(g_awbGainOffset));
    memcpy(pstAwbSnsDft->as32WbPara,     g_awbWbPara,     sizeof(g_awbWbPara));

    pstAwbSnsDft->u16GoldenRgain = GOLDEN_RGAIN;
    pstAwbSnsDft->u16GoldenBgain = GOLDEN_BGAIN;

    memcpy(&pstAwbSnsDft->stCcm,    &g_stAwbCcm,    sizeof(AWB_CCM_S));
    memcpy(&pstAwbSnsDft->stAgcTbl, &g_stAwbAgcTable, sizeof(AWB_AGC_TABLE_S));

    pstAwbSnsDft->u16SampleRgain = g_au16SampleRgain;
    pstAwbSnsDft->u16SampleBgain = g_au16SampleBgain;
    pstAwbSnsDft->u16InitRgain   = g_au16InitWBGain[0];
    pstAwbSnsDft->u16InitGgain   = g_au16InitWBGain[1];
    pstAwbSnsDft->u16InitBgain   = g_au16InitWBGain[2];

    pstAwbSnsDft->u8AWBRunInterval = 4;

    return HI_SUCCESS;
}

static HI_S32 cmos_init_awb_exp_function(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    memset(pstExpFuncs, 0, sizeof(AWB_SENSOR_EXP_FUNC_S));
    pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;
    return HI_SUCCESS;
}

/* ---- ISP default --------------------------------------------------------- */

/* Demosaic per-ISO tuning, byte-exact from vendor libsns_f22.so cmos_get_isp_default
 * (.rodata @0x5280). Sensor-intrinsic (single table spanning the whole ISO range,
 * no day/night variant) — this is why previous attempts to tune this via scene.c
 * apply_demosaic() interpolation only ever approximated it (no INI section exists
 * for demosaic). au8DetailSmoothRange ramping to 7 at high ISO (not 4, as our old
 * 2-point interpolation guessed) is the fix for the night-grain regression. */
static ISP_CMOS_DEMOSAIC_S g_stIspDemosaic = {
    .bEnable                   = HI_TRUE,
    .au8NonDirStr              = {64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64},
    .au8NonDirMFDetailEhcStr   = {32,32,32,36,36,36,36,32,22,20,18,18,18,18,18,18},
    .au8NonDirHFDetailEhcStr   = { 3, 3, 4, 5, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7},
    .au8DetailSmoothRange      = { 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 7, 7, 7, 7, 7, 7},
};

/* BayerNR tuning, byte-exact from vendor libsns_f22.so cmos_get_isp_default
 * (.rodata @0x4f7c). Sensor-intrinsic, same reasoning as Demosaic above.
 * au16CoarseStr ramps to 170 at high ISO (our previous scene.c apply_nr() guess
 * capped at 128) — under-denoising at night was the "grain superieur a l'original"
 * regression. au8LutFineStr/au16LutCoringWgt here are the sensor defaults; scene.c
 * apply_nr() still overrides FineStr/CoringWgt per day/night from the INI
 * ([static_nr] has no CoarseStr section, confirming it's sensor- not scene-owned). */
static ISP_CMOS_BAYERNR_S g_stIspBayerNr = {
    .bEnable              = HI_TRUE,
    .bBnrMonoSensorEn     = HI_FALSE,
    .bNrLscEnable         = HI_FALSE,
    .u8BnrLscMaxGain      = 0x60,
    .u16BnrLscCmpStrength = 0x100,
    .au8LutFineStr        = {80,80,90,90,90,90,80,70,60,50,50,50,50,50,40,40},
    .au8ChromaStr         = {
        {1,1,1,1,2,2,2,3,3,3,3,3,3,3,3,3},  /* R  */
        {0,0,0,0,1,1,1,2,2,2,2,2,2,2,2,2},  /* Gr */
        {0,0,0,0,1,1,1,2,2,2,2,2,2,2,2,2},  /* Gb */
        {1,1,1,1,2,2,2,3,3,3,3,3,3,3,3,3},  /* B  */
    },
    .au8WDRFrameStr       = {0, 0, 0, 0},
    .au8FusionFrameStr    = {0, 0, 0, 0},
    .au16CoarseStr        = {
        {90,100,100,110,110,120,130,150,160,170,170,170,170,170,170,170},  /* R  */
        {90,100,100,110,110,120,130,150,160,170,170,170,170,170,170,170},  /* Gr */
        {90,100,100,110,110,120,130,150,160,170,170,170,170,170,170,170},  /* Gb */
        {90,100,100,110,110,120,130,150,160,170,170,170,170,170,170,170},  /* B  */
    },
    .au16LutCoringWgt     = {40,40,35,30,30,30,30,30,30,30,30,30,30,30,30,30},
    .au16LutCoringRatio   = {
        90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,
        91,92,93,94,95,96,97,98,99,100,100,100,100,100,100,100,
        100,
    },
};

/* AntiFalseColor, byte-exact from vendor libsns_f22.so cmos_get_isp_default
 * (.rodata @0x4f58). Sensor-intrinsic, same reasoning as above. */
static ISP_CMOS_ANTIFALSECOLOR_S g_stIspAntiFalseColor = {
    .bEnable                       = HI_TRUE,
    .au8AntiFalseColorThreshold    = {10,10,8,8,7,7,7,6,6,6,5,4,3,2,1,0},
    .au8AntiFalseColorStrength     = { 8, 8,8,8,7,7,7,6,6,6,5,4,3,2,1,0},
};

static HI_S32 cmos_get_isp_default(VI_PIPE ViPipe, ISP_CMOS_DEFAULT_S *pstDef)
{
    (void)ViPipe;
    if (!pstDef) return HI_FAILURE;

    memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));

    /* unKey bits matching the vendor binary exactly for these three modules
     * (Ghidra decompile: unKey.u64Key |= 0x3b | (0x47 << 8), masked to just
     * Demosaic/BayerNr/AntiFalseColor here — the ISP silently ignores any
     * pst* pointer whose bit isn't set, which is why setting these pointers
     * alone (the pre-RE state of this file) had zero effect). */
    pstDef->unKey.bit1Demosaic      = 1;
    pstDef->unKey.bit1BayerNr       = 1;
    pstDef->unKey.bit1AntiFalseColor = 1;

    pstDef->pstDemosaic       = &g_stIspDemosaic;
    pstDef->pstBayerNr        = &g_stIspBayerNr;
    pstDef->pstAntiFalseColor = &g_stIspAntiFalseColor;

    /* Sensor resolution for ISP */
    pstDef->stSensorMaxResolution.u32MaxWidth  = 1920;
    pstDef->stSensorMaxResolution.u32MaxHeight = 1080;

    return HI_SUCCESS;
}

static HI_S32 cmos_get_isp_black_level(VI_PIPE ViPipe,
                                         ISP_CMOS_BLACK_LEVEL_S *pstBlackLevel)
{
    (void)ViPipe;
    if (!pstBlackLevel) return HI_FAILURE;

    pstBlackLevel->bUpdate = HI_TRUE;
    /* JXF22 black level = 64 (R/Gr/Gb/B). Confirmed against the original firmware's
       running ISP state (/proc/umap/isp "Black Level Actual" = 64) and the vendor
       INI [dynamic_blc] (R/Gr/Gb/B = 64). The earlier "×4 = 256" assumption was
       wrong and over-subtracted the black point, producing a magenta cast (the
       extra subtraction is amplified per channel by the WB gains, blue most). */
    pstBlackLevel->au16BlackLevel[0] = 64;
    pstBlackLevel->au16BlackLevel[1] = 64;
    pstBlackLevel->au16BlackLevel[2] = 64;
    pstBlackLevel->au16BlackLevel[3] = 64;

    return HI_SUCCESS;
}

static HI_S32 cmos_get_sns_regs_info(VI_PIPE ViPipe,
                                       ISP_SNS_REGS_INFO_S *pstSnsRegsInfo)
{
    (void)ViPipe;
    if (!pstSnsRegsInfo) return HI_FAILURE;

    if (HI_FALSE == g_stSnsRegsInfo.bConfig) {
        g_stSnsRegsInfo.enSnsType          = ISP_SNS_I2C_TYPE;
        g_stSnsRegsInfo.unComBus.s8I2cDev  = g_i2cBus;
        g_stSnsRegsInfo.u32RegNum          = SNS_REG_NUM;
        g_stSnsRegsInfo.u8Cfg2ValidDelayMax = 2;

#define INIT_I2C_REG(idx, reg) do { \
    g_stSnsRegsInfo.astI2cData[idx].bUpdate        = HI_TRUE; \
    g_stSnsRegsInfo.astI2cData[idx].u8DelayFrmNum  = 0; \
    g_stSnsRegsInfo.astI2cData[idx].u8IntPos       = 0; \
    g_stSnsRegsInfo.astI2cData[idx].u8DevAddr      = (HI_U8)sensor_i2c_addr; \
    g_stSnsRegsInfo.astI2cData[idx].u32RegAddr     = (reg); \
    g_stSnsRegsInfo.astI2cData[idx].u32AddrByteNum = sensor_addr_byte; \
    g_stSnsRegsInfo.astI2cData[idx].u32DataByteNum = sensor_data_byte; \
} while (0)

        INIT_I2C_REG(SNS_REG_EXPO_L, EXPOSURE_ADDR_L);
        INIT_I2C_REG(SNS_REG_EXPO_H, EXPOSURE_ADDR_H);
        INIT_I2C_REG(SNS_REG_AGC,    AGC_ADDR);
        INIT_I2C_REG(SNS_REG_VMAX_L, VMAX_ADDR_L);
        INIT_I2C_REG(SNS_REG_VMAX_H, VMAX_ADDR_H);
#undef INIT_I2C_REG

        /* Default values matching sensor init */
        g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_L].u32Data = 0x6A; /* ~half VMAX */
        g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_H].u32Data = 0x00;
        g_stSnsRegsInfo.astI2cData[SNS_REG_AGC].u32Data    = 0x00;
        g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_L].u32Data = VMAX_1080P30_LINEAR & 0xff;
        g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_H].u32Data = (VMAX_1080P30_LINEAR >> 8) & 0xff;

        g_stSnsRegsInfo.bConfig = HI_TRUE;
    }

    memcpy(pstSnsRegsInfo, &g_stSnsRegsInfo, sizeof(ISP_SNS_REGS_INFO_S));
    memcpy(&g_stPreSnsRegsInfo, &g_stSnsRegsInfo, sizeof(ISP_SNS_REGS_INFO_S));

    /* Clear update flags for next frame */
    g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_L].bUpdate = HI_FALSE;
    g_stSnsRegsInfo.astI2cData[SNS_REG_EXPO_H].bUpdate = HI_FALSE;
    g_stSnsRegsInfo.astI2cData[SNS_REG_AGC].bUpdate    = HI_FALSE;
    g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_L].bUpdate = HI_FALSE;
    g_stSnsRegsInfo.astI2cData[SNS_REG_VMAX_H].bUpdate = HI_FALSE;

    return HI_SUCCESS;
}

static HI_S32 cmos_set_image_mode(VI_PIPE ViPipe,
                                    ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
    (void)ViPipe;
    if (!pstSensorImageMode) return HI_FAILURE;

    if (pstSensorImageMode->u16Width  == 1920 &&
        pstSensorImageMode->u16Height == 1080) {
        gu8SensorImageMode = SENSOR_1080P_30FPS_MODE;
        return HI_SUCCESS;
    }

    printf("[sns][jxf22] unsupported mode %ux%u\n",
           pstSensorImageMode->u16Width, pstSensorImageMode->u16Height);
    return HI_FAILURE;
}

static HI_S32 cmos_set_wdr_mode(VI_PIPE ViPipe, HI_U8 u8Mode)
{
    (void)ViPipe;
    if (u8Mode != WDR_MODE_NONE) {
        printf("[sns][jxf22] WDR not supported\n");
        return HI_FAILURE;
    }
    genSensorMode = WDR_MODE_NONE;
    return HI_SUCCESS;
}

static HI_VOID cmos_set_pixel_detect(VI_PIPE ViPipe, HI_BOOL bEnable)
{
    (void)ViPipe;
    (void)bEnable;
}

static HI_VOID sensor_global_init(VI_PIPE ViPipe)
{
    (void)ViPipe;
    gu8SensorImageMode = SENSOR_1080P_30FPS_MODE;
    genSensorMode      = WDR_MODE_NONE;
    gu32FullLinesStd   = VMAX_1080P30_LINEAR;
    bInit              = HI_FALSE;
    bSensorInit        = HI_FALSE;
    memset(&g_stSnsRegsInfo,    0, sizeof(ISP_SNS_REGS_INFO_S));
    memset(&g_stPreSnsRegsInfo, 0, sizeof(ISP_SNS_REGS_INFO_S));
}

/* ---- ISP_SNS_OBJ_S glue -------------------------------------------------- */

static HI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    memset(pstExpFuncs, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));
    pstExpFuncs->pfn_cmos_sensor_init        = sensor_init;
    pstExpFuncs->pfn_cmos_sensor_exit        = sensor_exit;
    pstExpFuncs->pfn_cmos_sensor_global_init = sensor_global_init;
    pstExpFuncs->pfn_cmos_set_image_mode     = cmos_set_image_mode;
    pstExpFuncs->pfn_cmos_set_wdr_mode       = cmos_set_wdr_mode;
    pstExpFuncs->pfn_cmos_get_isp_default    = cmos_get_isp_default;
    pstExpFuncs->pfn_cmos_get_isp_black_level = cmos_get_isp_black_level;
    pstExpFuncs->pfn_cmos_get_sns_reg_info   = cmos_get_sns_regs_info;
    pstExpFuncs->pfn_cmos_set_pixel_detect   = cmos_set_pixel_detect;
    return HI_SUCCESS;
}

static HI_S32 sensor_register_callback(VI_PIPE ViPipe,
                                         ALG_LIB_S *pstAeLib,
                                         ALG_LIB_S *pstAwbLib)
{
    ISP_SNS_ATTR_INFO_S   stSnsAttrInfo  = { .eSensorId = JXF22_ID };
    ISP_SENSOR_REGISTER_S stIspRegister;
    AE_SENSOR_REGISTER_S  stAeRegister;
    AWB_SENSOR_REGISTER_S stAwbRegister;
    HI_S32 ret;

    cmos_init_sensor_exp_function(&stIspRegister.stSnsExp);
    ret = HI_MPI_ISP_SensorRegCallBack(ViPipe, &stSnsAttrInfo, &stIspRegister);
    if (ret) {
        printf("[sns][jxf22] ISP_SensorRegCallBack failed: 0x%x\n", ret);
        return ret;
    }

    cmos_init_ae_exp_function(&stAeRegister.stSnsExp);
    ret = HI_MPI_AE_SensorRegCallBack(ViPipe, pstAeLib, &stSnsAttrInfo, &stAeRegister);
    if (ret) {
        printf("[sns][jxf22] AE_SensorRegCallBack failed: 0x%x\n", ret);
        return ret;
    }

    cmos_init_awb_exp_function(&stAwbRegister.stSnsExp);
    ret = HI_MPI_AWB_SensorRegCallBack(ViPipe, pstAwbLib, &stSnsAttrInfo, &stAwbRegister);
    if (ret) {
        printf("[sns][jxf22] AWB_SensorRegCallBack failed: 0x%x\n", ret);
        return ret;
    }

    return HI_SUCCESS;
}

static HI_S32 sensor_unregister_callback(VI_PIPE ViPipe,
                                           ALG_LIB_S *pstAeLib,
                                           ALG_LIB_S *pstAwbLib)
{
    HI_S32 ret;

    ret = HI_MPI_ISP_SensorUnRegCallBack(ViPipe, JXF22_ID);
    if (ret) printf("[sns][jxf22] ISP_SensorUnRegCallBack failed: 0x%x\n", ret);

    ret = HI_MPI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, JXF22_ID);
    if (ret) printf("[sns][jxf22] AE_SensorUnRegCallBack failed: 0x%x\n", ret);

    ret = HI_MPI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, JXF22_ID);
    if (ret) printf("[sns][jxf22] AWB_SensorUnRegCallBack failed: 0x%x\n", ret);

    return HI_SUCCESS;
}

static HI_S32 sensor_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
    (void)ViPipe;
    g_i2cBus = unSNSBusInfo.s8I2cDev;
    /* Update shadow reg bus field if already configured */
    g_stSnsRegsInfo.unComBus.s8I2cDev = g_i2cBus;
    return HI_SUCCESS;
}

/* pfnMirrorFlip: writes I2C regs 0x12/0x27/0x28 directly.
 * Reverse-engineered from libsns_f22.so (cmos_set_mirror_flip_ex).
 * reg0x12 bits[5:4] → {flip, mirror}; regs 0x27/0x28 adjust analog offsets. */
static HI_VOID cmos_set_mirror_flip(VI_PIPE ViPipe,
                                     ISP_SNS_MIRRORFLIP_TYPE_E eMirrorFlip)
{
    HI_S32 reg12;
    (void)ViPipe;

    reg12 = sensor_read_register(0x12);
    if (reg12 < 0) reg12 = 0;
    reg12 &= 0xcf; /* clear bits[5:4] */

    switch (eMirrorFlip) {
    case ISP_SNS_NORMAL:
        sensor_write_register(0x12, reg12);
        sensor_write_register(0x27, 0xc9);
        sensor_write_register(0x28, 0x18);
        break;
    case ISP_SNS_MIRROR:
        sensor_write_register(0x12, reg12 | 0x10);
        sensor_write_register(0x27, 0xc9);
        sensor_write_register(0x28, 0x18);
        break;
    case ISP_SNS_FLIP:
        sensor_write_register(0x12, reg12 | 0x20);
        sensor_write_register(0x27, 0xd2);
        sensor_write_register(0x28, 0x15);
        break;
    case ISP_SNS_MIRROR_FLIP:
        sensor_write_register(0x12, reg12 | 0x30);
        sensor_write_register(0x27, 0xd2);
        sensor_write_register(0x28, 0x15);
        break;
    default:
        break;
    }
}

static HI_S32 sensor_write_reg(VI_PIPE ViPipe, HI_S32 s32Addr, HI_S32 s32Data)
{
    (void)ViPipe;
    return sensor_write_register(s32Addr, s32Data);
}

static HI_S32 sensor_read_reg(VI_PIPE ViPipe, HI_S32 s32Addr)
{
    (void)ViPipe;
    return sensor_read_register(s32Addr);
}

static HI_S32 sensor_set_init(VI_PIPE ViPipe, ISP_INIT_ATTR_S *pstInitAttr)
{
    (void)ViPipe;
    if (!pstInitAttr) return HI_FAILURE;

    g_au16InitWBGain[0] = pstInitAttr->u16WBRgain;
    g_au16InitWBGain[1] = pstInitAttr->u16WBGgain;
    g_au16InitWBGain[2] = pstInitAttr->u16WBBgain;
    g_au16SampleRgain   = pstInitAttr->u16SampleRgain;
    g_au16SampleBgain   = pstInitAttr->u16SampleBgain;

    return HI_SUCCESS;
}

/* ---- Public sensor object ------------------------------------------------ */

ISP_SNS_OBJ_S stSnsSoiSensorObj = {
    .pfnRegisterCallback   = sensor_register_callback,
    .pfnUnRegisterCallback = sensor_unregister_callback,
    .pfnSetBusInfo         = sensor_set_bus_info,
    .pfnStandby            = NULL,
    .pfnRestart            = NULL,
    .pfnMirrorFlip         = cmos_set_mirror_flip,
    .pfnWriteReg           = sensor_write_reg,
    .pfnReadReg            = sensor_read_reg,
    .pfnSetInit            = sensor_set_init,
};
