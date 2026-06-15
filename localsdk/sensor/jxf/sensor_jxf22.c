/* SOI JXF22 sensor module — owns the sensor-coupled MPP bring-up.
 *
 * All MIPI/VI/ISP attributes captured from the original liblocalsdk.so runtime
 * trace (trace.txt) and cross-checked against the 3516ev200 sample
 * (DEV_ATTR_SC2231 / PIPE_ATTR_1920x1080_RAW10).
 * 2-lane MIPI, RAW10, 1080p30 linear, VI online -> VPSS online.
 *
 * This module is generic/reusable: it knows nothing of the board. The board
 * pushes the couple-specific config (fps, orientation) via
 * sensor_jxf22_configure() and triggers sensor_jxf22_bringup().
 */

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include "sensor_jxf22.h"

/* Hisilicon MPP */
#include "hi_mipi.h"
#include "hi_comm_sys.h"
#include "hi_comm_vi.h"
#include "hi_comm_isp.h"
#include "hi_comm_3a.h"
#include "hi_ae_comm.h"
#include "hi_awb_comm.h"
#include "hi_sns_ctrl.h"
#include "mpi_sys.h"
#include "mpi_vi.h"
#include "mpi_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"

/* OSS sensor driver (port of libsns_f22.so) */
extern ISP_SNS_OBJ_S stSnsSoiSensorObj;
#include "jxf22.h"

#include "../../../logger/logger.h"

/* ------------------------------------------------------------------------- */
/* Sensor attributes (private to this module)                                */
/* ------------------------------------------------------------------------- */

static const combo_dev_attr_t s_mipi_attr = {
    .devno      = 0,
    .input_mode = INPUT_MODE_MIPI,
    .data_rate  = MIPI_DATA_RATE_X1,
    .img_rect   = {0, 0, JXF22_WIDTH, JXF22_HEIGHT},
    .mipi_attr  = {
        DATA_TYPE_RAW_10BIT,
        HI_MIPI_WDR_MODE_NONE,
        {0, 2, -1, -1}
    }
};

static const VI_DEV_ATTR_S s_vi_dev_attr = {
    VI_MODE_MIPI,
    VI_WORK_MODE_1Multiplex,
    {0xFFC00000, 0x0},
    VI_SCAN_PROGRESSIVE,
    { -1, -1, -1, -1},
    VI_DATA_SEQ_YUYV,
    {
        VI_VSYNC_PULSE, VI_VSYNC_NEG_LOW,
        VI_HSYNC_VALID_SINGNAL, VI_HSYNC_NEG_HIGH,
        VI_VSYNC_VALID_SINGAL, VI_VSYNC_VALID_NEG_HIGH,
        {
            0, JXF22_WIDTH, 0,
            0, JXF22_HEIGHT, 0,
            0, 0, 0
        }
    },
    VI_DATA_TYPE_RGB,
    HI_FALSE,
    {JXF22_WIDTH, JXF22_HEIGHT},
    {
        { {JXF22_WIDTH, JXF22_HEIGHT} },
        { VI_REPHASE_MODE_NONE, VI_REPHASE_MODE_NONE }
    },
    {
        WDR_MODE_NONE,
        JXF22_HEIGHT
    },
    DATA_RATE_X1
};

static const VI_PIPE_ATTR_S s_vi_pipe_attr = {
    VI_PIPE_BYPASS_NONE, HI_FALSE, HI_FALSE,
    JXF22_WIDTH, JXF22_HEIGHT,
    PIXEL_FORMAT_RGB_BAYER_10BPP,
    COMPRESS_MODE_NONE,
    DATA_BITWIDTH_10,
    HI_TRUE,  /* bNrEn: VI-pipe level NR */
    {
        PIXEL_FORMAT_YVU_SEMIPLANAR_420,
        DATA_BITWIDTH_8,
        VI_NR_REF_FROM_RFR,
        COMPRESS_MODE_NONE
    },
    HI_FALSE,
    { -1, -1 }
};

static const ISP_PUB_ATTR_S s_isp_pub_attr = {
    .stWndRect    = {0, 0, JXF22_WIDTH, JXF22_HEIGHT},
    .stSnsSize    = {JXF22_WIDTH, JXF22_HEIGHT},
    .f32FrameRate = (float)JXF22_FPS_NATIVE,  /* native; app retimes via VPSS */
    .enBayer      = BAYER_BGGR,  /* confirmed correct with vendor libsns_f22.so */
    .enWDRMode    = WDR_MODE_NONE,
    .u8SnsMode    = 0
};

/* ------------------------------------------------------------------------- */
/* Module state                                                              */
/* ------------------------------------------------------------------------- */

static sensor_config_t s_cfg = { .fps = JXF22_FPS_NATIVE, .mirror = HI_FALSE, .flip = HI_FALSE };

static VI_DEV   s_viDev  = 0;
static VI_PIPE  s_viPipe = 0;
static VI_CHN   s_viChn  = 0;
static pthread_t s_ispThread = 0;

/* ------------------------------------------------------------------------- */
/* MIPI / VI / ISP bring-up                                                  */
/* ------------------------------------------------------------------------- */

/* MIPI bring-up sequence, ported faithfully from SAMPLE_COMM_VI_StartMIPI:
   HsMode -> EnableMipiClock -> ResetMipi -> EnableSensorClock -> ResetSensor
   -> SetDevAttr -> UnresetMipi -> UnresetSensor. */
static HI_S32 sensor_mipi_init(void) {
    int fd;
    lane_divide_mode_t hsMode = LANE_DIVIDE_MODE_0;
    combo_dev_t  devno  = 0;
    sns_clk_source_t snsClk = 0;
    sns_rst_source_t snsRst = 0;
    combo_dev_attr_t stComboAttr;

    fd = open("/dev/hi_mipi", O_RDWR);
    if (fd < 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] open /dev/hi_mipi failed");
        return HI_FAILURE;
    }

    if (ioctl(fd, HI_MIPI_SET_HS_MODE, &hsMode) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_SET_HS_MODE failed");
        close(fd);
        return HI_FAILURE;
    }
    if (ioctl(fd, HI_MIPI_ENABLE_MIPI_CLOCK, &devno) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_ENABLE_MIPI_CLOCK failed");
        close(fd);
        return HI_FAILURE;
    }
    if (ioctl(fd, HI_MIPI_RESET_MIPI, &devno) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_RESET_MIPI failed");
        close(fd);
        return HI_FAILURE;
    }
    if (ioctl(fd, HI_MIPI_ENABLE_SENSOR_CLOCK, &snsClk) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_ENABLE_SENSOR_CLOCK failed");
        close(fd);
        return HI_FAILURE;
    }
    if (ioctl(fd, HI_MIPI_RESET_SENSOR, &snsRst) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_RESET_SENSOR failed");
        close(fd);
        return HI_FAILURE;
    }

    memcpy(&stComboAttr, &s_mipi_attr, sizeof(stComboAttr));
    stComboAttr.devno = 0;
    if (ioctl(fd, HI_MIPI_SET_DEV_ATTR, &stComboAttr) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_SET_DEV_ATTR failed");
        close(fd);
        return HI_FAILURE;
    }

    if (ioctl(fd, HI_MIPI_UNRESET_MIPI, &devno) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_UNRESET_MIPI failed");
        close(fd);
        return HI_FAILURE;
    }
    if (ioctl(fd, HI_MIPI_UNRESET_SENSOR, &snsRst) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MIPI_UNRESET_SENSOR failed");
        close(fd);
        return HI_FAILURE;
    }

    close(fd);
    return HI_SUCCESS;
}

/* VI bring-up in the pipeline model (matches libmpi.so exports):
   SetDevAttr -> EnableDev -> SetDevBindPipe -> CreatePipe -> StartPipe
   -> SetChnAttr -> EnableChn. */
static HI_S32 sensor_vi_start(void) {
    VI_DEV_ATTR_S      stViDevAttr;
    VI_PIPE_ATTR_S     stPipeAttr;
    VI_CHN_ATTR_S      stChnAttr;
    VI_DEV_BIND_PIPE_S stDevBindPipe;
    HI_S32 result;

    if (sensor_mipi_init() != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] MIPI init failed");
        return HI_FAILURE;
    }

    /* --- VI device --- */
    memcpy(&stViDevAttr, &s_vi_dev_attr, sizeof(stViDevAttr));
    result = HI_MPI_VI_SetDevAttr(s_viDev, &stViDevAttr);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_SetDevAttr failed: 0x%x", result);
        return HI_FAILURE;
    }
    result = HI_MPI_VI_EnableDev(s_viDev);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_EnableDev failed: 0x%x", result);
        return HI_FAILURE;
    }

    /* --- bind dev -> pipe --- */
    memset(&stDevBindPipe, 0, sizeof(stDevBindPipe));
    stDevBindPipe.u32Num     = 1;
    stDevBindPipe.PipeId[0]  = s_viPipe;
    result = HI_MPI_VI_SetDevBindPipe(s_viDev, &stDevBindPipe);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_SetDevBindPipe failed: 0x%x", result);
        HI_MPI_VI_DisableDev(s_viDev);
        return HI_FAILURE;
    }

    /* --- VI pipe --- */
    memcpy(&stPipeAttr, &s_vi_pipe_attr, sizeof(stPipeAttr));
    result = HI_MPI_VI_CreatePipe(s_viPipe, &stPipeAttr);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_CreatePipe failed: 0x%x", result);
        HI_MPI_VI_DisableDev(s_viDev);
        return HI_FAILURE;
    }
    result = HI_MPI_VI_StartPipe(s_viPipe);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_StartPipe failed: 0x%x", result);
        HI_MPI_VI_DestroyPipe(s_viPipe);
        HI_MPI_VI_DisableDev(s_viDev);
        return HI_FAILURE;
    }

    /* --- VI physical channel --- */
    memset(&stChnAttr, 0, sizeof(stChnAttr));
    stChnAttr.stSize.u32Width  = JXF22_WIDTH;
    stChnAttr.stSize.u32Height = JXF22_HEIGHT;
    stChnAttr.enPixelFormat    = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stChnAttr.enDynamicRange   = DYNAMIC_RANGE_SDR8;
    stChnAttr.enVideoFormat    = VIDEO_FORMAT_LINEAR;
    stChnAttr.enCompressMode   = COMPRESS_MODE_NONE;
    /* In VI_ONLINE_VPSS_ONLINE the VI channel is a pass-through; bMirror/bFlip
       are not applied at this level. Orientation is corrected at the sensor. */
    stChnAttr.bMirror          = HI_FALSE;
    stChnAttr.bFlip            = HI_FALSE;
    stChnAttr.u32Depth         = 0;
    stChnAttr.stFrameRate.s32SrcFrameRate = -1;
    stChnAttr.stFrameRate.s32DstFrameRate = -1;

    result = HI_MPI_VI_SetChnAttr(s_viPipe, s_viChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_SetChnAttr failed: 0x%x", result);
        HI_MPI_VI_StopPipe(s_viPipe);
        HI_MPI_VI_DestroyPipe(s_viPipe);
        HI_MPI_VI_DisableDev(s_viDev);
        return HI_FAILURE;
    }

    result = HI_MPI_VI_EnableChn(s_viPipe, s_viChn);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_VI_EnableChn failed: 0x%x", result);
        HI_MPI_VI_StopPipe(s_viPipe);
        HI_MPI_VI_DestroyPipe(s_viPipe);
        HI_MPI_VI_DisableDev(s_viDev);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

static void *sensor_isp_thread(void *arg) {
    (void)arg;
    HI_MPI_ISP_Run(0);
    return NULL;
}

static HI_S32 sensor_register(void) {
    ALG_LIB_S stAeLib, stAwbLib;

    memset(&stAeLib,  0, sizeof(stAeLib));
    memset(&stAwbLib, 0, sizeof(stAwbLib));
    stAeLib.s32Id  = 0;
    stAwbLib.s32Id = 0;
    strncpy(stAeLib.acLibName,  HI_AE_LIB_NAME,  sizeof(stAeLib.acLibName)  - 1);
    strncpy(stAwbLib.acLibName, HI_AWB_LIB_NAME, sizeof(stAwbLib.acLibName) - 1);

    /* Tell the sensor driver which I2C bus to use (i2c-0 for JXF22). */
    if (stSnsSoiSensorObj.pfnSetBusInfo) {
        ISP_SNS_COMMBUS_U unBus;
        unBus.s8I2cDev = 0;
        stSnsSoiSensorObj.pfnSetBusInfo(0, unBus);
    }

    if (!stSnsSoiSensorObj.pfnRegisterCallback) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] pfnRegisterCallback is NULL");
        return HI_FAILURE;
    }

    HI_S32 result = stSnsSoiSensorObj.pfnRegisterCallback(0, &stAeLib, &stAwbLib);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] pfnRegisterCallback failed: 0x%x", result);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

static HI_S32 sensor_isp_init(void) {
    HI_S32 result;
    ISP_PUB_ATTR_S stPubAttr;
    ALG_LIB_S stLib;

    result = sensor_register();
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] sensor registration failed");
        return HI_FAILURE;
    }

    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AE_LIB_NAME, sizeof(stLib.acLibName) - 1);
    result = HI_MPI_AE_Register(0, &stLib);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_AE_Register failed: 0x%x", result);
        return HI_FAILURE;
    }

    memset(&stLib, 0, sizeof(stLib));
    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AWB_LIB_NAME, sizeof(stLib.acLibName) - 1);
    result = HI_MPI_AWB_Register(0, &stLib);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_AWB_Register failed: 0x%x", result);
        return HI_FAILURE;
    }

    result = HI_MPI_ISP_MemInit(0);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_ISP_MemInit failed: 0x%x", result);
        return HI_FAILURE;
    }

    memcpy(&stPubAttr, &s_isp_pub_attr, sizeof(stPubAttr));
    result = HI_MPI_ISP_SetPubAttr(0, &stPubAttr);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_ISP_SetPubAttr failed: 0x%x", result);
        return HI_FAILURE;
    }

    result = HI_MPI_ISP_Init(0);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] HI_MPI_ISP_Init failed: 0x%x", result);
        return HI_FAILURE;
    }

    if (pthread_create(&s_ispThread, NULL, sensor_isp_thread, NULL) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] Failed to create ISP thread");
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void sensor_jxf22_configure(const sensor_config_t *cfg) {
    if (cfg) s_cfg = *cfg;
}

HI_S32 sensor_jxf22_bringup(void) {
    /* Order matters: bring up MIPI + VI dev/pipe/chn FIRST, then ISP. The ISP
       attaches to the VI pipe, and the sensor i2c writes happen during ISP init
       using the sensor clock enabled by the MIPI bring-up. */
    if (sensor_vi_start() != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] VI start failed");
        return HI_FAILURE;
    }
    if (sensor_isp_init() != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[sensor][jxf22] ISP init failed");
        return HI_FAILURE;
    }

    /* Wait for the ISP thread to call pfnCmosSensorInit and open the I2C bus
       before writing the VMAX / mirror-flip registers over i2c. */
    usleep(300000);

    /* Apply target fps (VMAX reprogramming) and orientation at the sensor. */
    sensor_set_fps(s_cfg.fps);

    ISP_SNS_MIRRORFLIP_TYPE_E eMf =
        (ISP_SNS_MIRRORFLIP_TYPE_E)((s_cfg.flip ? ISP_SNS_FLIP : 0) |
                                    (s_cfg.mirror ? ISP_SNS_MIRROR : 0));
    if (stSnsSoiSensorObj.pfnMirrorFlip)
        stSnsSoiSensorObj.pfnMirrorFlip(0, eMf);

    LOGGER(LOGGER_LEVEL_INFO, "[sensor][jxf22] bring-up ok: fps=%u mirror=%d flip=%d",
           s_cfg.fps, (int)s_cfg.mirror, (int)s_cfg.flip);
    return HI_SUCCESS;
}

void sensor_jxf22_get_vi(VI_DEV *dev, VI_PIPE *pipe, VI_CHN *chn) {
    if (dev)  *dev  = s_viDev;
    if (pipe) *pipe = s_viPipe;
    if (chn)  *chn  = s_viChn;
}

void sensor_jxf22_teardown_isp(void) {
    if (s_ispThread != 0) {
        HI_MPI_ISP_Exit(0);
        pthread_join(s_ispThread, NULL);
        s_ispThread = 0;
    }
    if (stSnsSoiSensorObj.pfnUnRegisterCallback) {
        ALG_LIB_S stAeLib = {0}, stAwbLib = {0};
        strncpy(stAeLib.acLibName,  HI_AE_LIB_NAME,  sizeof(stAeLib.acLibName)  - 1);
        strncpy(stAwbLib.acLibName, HI_AWB_LIB_NAME, sizeof(stAwbLib.acLibName) - 1);
        stSnsSoiSensorObj.pfnUnRegisterCallback(0, &stAeLib, &stAwbLib);
    }
}
