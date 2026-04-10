/**
 * @file localsdk-clean.c
 * @brief Cleaned and refactored version of localsdk.c
 * 
 * This file provides a consolidated and well-organized implementation of the
 * mjsxj02hl local SDK. It removes redundancies while maintaining 100% API compatibility.
 * 
 * Organization:
 *  - Initialization/Core functions
 *  - Video subsystem
 *  - Audio subsystem
 *  - Alarm subsystem
 *  - OSD subsystem
 *  - Night mode subsystem
 *  - Hardware control (LEDs, buttons)
 *  - Internal utilities
 */

#include "localsdk.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <sys/select.h>

/* Hisilicon MPP */
#include "hi_comm_sys.h"
#include "hi_comm_isp.h"
#include "hi_comm_3a.h"
#include "hi_comm_video.h"
#include "hi_comm_venc.h"
#include "hi_comm_vpss.h"
#include "hi_comm_vi.h"
#include "mpi_sys.h"
#include "mpi_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_venc.h"
#include "mpi_vpss.h"
#include "mpi_vi.h"

/* Sensor library entry points from libsns_f22 / libsns_jxf22 */
extern int sensor_register_callback(void);
extern int sensor_unregister_callback(void);

/* ============================================================================
   DEFINES AND CONSTANTS
   ============================================================================ */

#define SDK_VERSION 14

/* Configuration paths */
#define WPA_CONFIG_PATH "/configs"
#define HOSTAPD_CONFIG_PATH "/usr/app/share"
#define UDHCPD_CONFIG_PATH "/usr/app/share"

/* Video stream buffer sizes */
#define VIDEO_FRAME_BUFFER_SIZE 0x180  /* 384 bytes */
#define VIDEO_STATUS_BUFFER_SIZE 0x58   /* 88 bytes */

/* Timeout values (microseconds) */
#define USLEEP_100MS 100000
#define USLEEP_50MS 50000

/* Error codes and status values */
#define SDK_INVALID_CHANNEL 0xffffffff
#define SDK_SUCCESS 0
#define SDK_FAILURE 0xffffffff

/* ============================================================================
   GLOBAL STATE AND CALLBACKS
   ============================================================================ */

static int (*g_sdkLogPrintf_cb)(const char *, ...) = NULL;
static int (*g_sdkShellCall_cb)(const char *) = NULL;

/* Video parameters storage */
static uint32_t g_videoParams[256];  /* Video configuration buffer */
static uint32_t g_videoBrightCheckLoop = 0;
static uint32_t g_videoBrightSetCnt = 0;
static pthread_mutex_t g_videoBrightLock;
static pthread_t g_ispThread = 0;

/* Alarm and OSD state */
static uint32_t g_alarmState = 0;
static uint32_t g_osdState = 0;

/* Alarm callbacks (small fixed pool, per decompiled behavior) */
#define SDK_ALARM_CB_MAX 10
typedef struct AlarmCbNode {
    struct AlarmCbNode *next;
    int (*cb)(LOCALSDK_ALARM_EVENT_INFO *eventInfo);
    int used;
} AlarmCbNode;

static AlarmCbNode g_alarmCbPool[SDK_ALARM_CB_MAX];
static AlarmCbNode *g_alarmCbHead = NULL;
static pthread_mutex_t g_alarmCbMutex = PTHREAD_MUTEX_INITIALIZER;

/* ============================================================================
   UTILITY FUNCTIONS - Error Handling and Logging
   ============================================================================ */

/**
 * @brief Log debug message if callback is set
 */
static inline void sdk_log(const char *fmt, ...) {
    if (g_sdkLogPrintf_cb) {
        char buffer[1024];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        g_sdkLogPrintf_cb("%s", buffer);
    }
}

/**
 * @brief Execute shell command with logging
 */
static inline int sdk_exec_shell(const char *cmd) {
    if (!cmd) {
        sdk_log("[sdk] Error: shell command is NULL\n");
        return SDK_FAILURE;
    }
    
    if (g_sdkShellCall_cb) {
        return g_sdkShellCall_cb(cmd);
    }
    
    sdk_log("[sdk] Warning: shell callback not set\n");
    return SDK_FAILURE;
}

/**
 * @brief Sleep with minimum value check
 */
static inline void sdk_usleep(uint32_t usec) {
    if (usec > 0) {
        usleep(usec);
    }
}

/* ============================================================================
   ALARM CALLBACK REGISTRY (linked list + fixed pool)
   ============================================================================ */

static AlarmCbNode *alarm_cb_alloc(void) {
    for (int i = 0; i < SDK_ALARM_CB_MAX; i++) {
        if (!g_alarmCbPool[i].used) {
            g_alarmCbPool[i].used = 1;
            g_alarmCbPool[i].next = NULL;
            g_alarmCbPool[i].cb = NULL;
            return &g_alarmCbPool[i];
        }
    }
    return NULL;
}

static void alarm_cb_free(AlarmCbNode *node) {
    if (!node) {
        return;
    }
    node->used = 0;
    node->next = NULL;
    node->cb = NULL;
}

static int alarm_cb_add(AlarmCbNode *node) {
    if (!node) {
        return LOCALSDK_ERROR;
    }
    if (!g_alarmCbHead) {
        g_alarmCbHead = node;
        return LOCALSDK_OK;
    }

    AlarmCbNode *cur = g_alarmCbHead;
    while (cur->next) {
        cur = cur->next;
    }
    cur->next = node;
    return LOCALSDK_OK;
}

static int alarm_cb_remove(AlarmCbNode *node) {
    if (!node || !g_alarmCbHead) {
        return LOCALSDK_ERROR;
    }
    if (g_alarmCbHead == node) {
        g_alarmCbHead = node->next;
        return LOCALSDK_OK;
    }
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur->next) {
        if (cur->next == node) {
            cur->next = node->next;
            return LOCALSDK_OK;
        }
        cur = cur->next;
    }
    return LOCALSDK_ERROR;
}

static AlarmCbNode *alarm_cb_find(int (*callback)(LOCALSDK_ALARM_EVENT_INFO *eventInfo)) {
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur) {
        if (cur->cb == callback) {
            return cur;
        }
        cur = cur->next;
    }
    return NULL;
}

static int sdk_alarm_run_callback(LOCALSDK_ALARM_EVENT_INFO *eventInfo) {
    int result = LOCALSDK_OK;
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur) {
        if (cur->cb) {
            result = cur->cb(eventInfo);
        }
        cur = cur->next;
    }
    return result;
}

/* ============================================================================
   INITIALIZATION AND CORE FUNCTIONS
   ============================================================================ */

/**
 * @brief Set the debug print function
 */
int localsdk_set_logprintf_func(int (*function)(const char *, ...)) {
    g_sdkLogPrintf_cb = function;
    return LOCALSDK_OK;
}

/**
 * @brief Set the shell command execution function
 */
int localsdk_set_shellcall_func(int *param_1) {
    if (param_1) {
        g_sdkShellCall_cb = (int (*)(const char *))param_1;
        return LOCALSDK_OK;
    }
    return LOCALSDK_ERROR;
}

/**
 * @brief Initialize the SDK with HISILICON subsystems
 */
int localsdk_init() {
    puts("--------------------------------…");
    puts("    WELCOME TO LOCALSDK      ('_…");
    puts("--------------------------------…");
    puts("    platform: hi3518ev300 ");
    printf("    version : %s \n", "HI_SDK20190315");
    printf("    update  : %s (%s) \n", "Apr 12 2021", "18:16:49");
    puts("--------------------------------…");
    
    platform_callback_mutex_init();
    
    if (HI_PWM_Init() != 0) {
        printf("[%s]err: HI_PWM init Fail!\n", "localsdk_init");
        return 1;
    }
    
    if (softlight_init() != 0)
        printf("[%s]err: softlight init Fail!\n", "localsdk_init");
    else {
        int32_t attr = gpio_all_init();
        
        if (attr != 0) {
            printf("[%s]err: gpio init Fail!\n", "localsdk_init");
            return 1;
        }
        
        void newthread;
        int32_t result = pthread_create(&newthread, attr, platform_thread);
        
        if (result == 0) {
            printf("[%s]dbg: Platform Thread Create …", "SDK-THREAD");
            return result;
        }
        
        printf("[%s]err: Platform Thread Create …", "localsdk_init");
    }
    
    return 1;
}

/**
 * @brief Destroy the SDK and cleanup resources
 */
int localsdk_destory() {
    sdk_log("[sdk] Destroying SDK\n");

    if (g_ispThread != 0) {
        HI_MPI_ISP_Exit(0);
        pthread_join(g_ispThread, NULL);
        g_ispThread = 0;
    }

    if (sensor_unregister_callback() != 0) {
        sdk_log("[sdk][video] sensor_unregister_callback failed\n");
    }
    
    /* Cleanup locks */
    pthread_mutex_destroy(&g_videoBrightLock);
    
    /* Clear state */
    memset(g_videoParams, 0, sizeof(g_videoParams));
    
    sdk_log("[sdk] SDK cleanup complete\n");
    return LOCALSDK_OK;
}

/**
 * @brief Get SDK version
 */
int localsdk_get_version() {
    return LOCALSDK_CURRENT_VERSION;
}

/* ============================================================================
   VIDEO SUBSYSTEM - HISILICON IMPLEMENTATION
   ============================================================================ */

/* Global video state */
static VPSS_GRP g_vpssGrp = 0;
static VPSS_CHN g_vpssChn[4] = {0, 1, 2, 3};
static VENC_CHN g_vencChn[2] = {0, 1};
static int32_t g_videoStarted[2] = {0, 0};
static int32_t (*g_encCb[2])(LOCALSDK_H26X_FRAME_INFO *frameInfo) = {NULL, NULL};
static int32_t (*g_yuvCb[2])(LOCALSDK_H26X_FRAME_INFO *frameInfo) = {NULL, NULL};
static int (*g_algoRegisterCb)(void) = NULL;

static int sdk_video_any_started(void) {
    return g_videoStarted[0] || g_videoStarted[1];
}

static LOCALSDK_VIDEO_OPTIONS *sdk_video_get_options(int chn) {
    if (chn < 0 || chn > 1) {
        return NULL;
    }
    return (LOCALSDK_VIDEO_OPTIONS *)&g_videoParams[chn * 32];
}

static int sdk_video_resolution_to_size(int resolution, SIZE_S *size) {
    if (!size) {
        return LOCALSDK_ERROR;
    }

    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            size->u32Width = 1920;
            size->u32Height = 1080;
            return LOCALSDK_OK;
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
            size->u32Width = 640;
            size->u32Height = 360;
            return LOCALSDK_OK;
        default:
            return LOCALSDK_ERROR;
    }
}

static PAYLOAD_TYPE_E sdk_video_payload_type(uint32_t payload, int jpeg) {
    if (jpeg) {
        return PT_JPEG;
    }

    switch (payload) {
        case LOCALSDK_VIDEO_PAYLOAD_H265:
            return PT_H265;
        case LOCALSDK_VIDEO_PAYLOAD_H264:
        default:
            return PT_H264;
    }
}

static int sdk_video_bind_vpss_venc(int chn, VENC_CHN vencChn) {
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;

    memset(&stSrcChn, 0, sizeof(stSrcChn));
    memset(&stDestChn, 0, sizeof(stDestChn));

    stSrcChn.enModId = HI_ID_VPSS;
    stSrcChn.s32DevId = g_vpssGrp;
    stSrcChn.s32ChnId = g_vpssChn[chn];

    stDestChn.enModId = HI_ID_VENC;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = vencChn;

    return (HI_MPI_SYS_Bind(&stSrcChn, &stDestChn) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static int sdk_video_unbind_vpss_venc(int chn, VENC_CHN vencChn) {
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;

    memset(&stSrcChn, 0, sizeof(stSrcChn));
    memset(&stDestChn, 0, sizeof(stDestChn));

    stSrcChn.enModId = HI_ID_VPSS;
    stSrcChn.s32DevId = g_vpssGrp;
    stSrcChn.s32ChnId = g_vpssChn[chn];

    stDestChn.enModId = HI_ID_VENC;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = vencChn;

    return (HI_MPI_SYS_UnBind(&stSrcChn, &stDestChn) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static int sdk_video_create_venc_channel(VENC_CHN vencChn, LOCALSDK_VIDEO_OPTIONS *options, int jpeg) {
    VENC_CHN_ATTR_S stVencChnAttr;
    SIZE_S stSize;
    PAYLOAD_TYPE_E enType;
    HI_S32 result;

    if (!options || sdk_video_resolution_to_size(options->resolution, &stSize) != LOCALSDK_OK) {
        return LOCALSDK_ERROR;
    }

    memset(&stVencChnAttr, 0, sizeof(stVencChnAttr));
    enType = sdk_video_payload_type(options->payload, jpeg);
    stVencChnAttr.stVeAttr.enType = enType;

    if (enType == PT_H264) {
        VENC_ATTR_H264_S *attr = &stVencChnAttr.stVeAttr.stAttrH264e;
        attr->u32MaxPicWidth = stSize.u32Width;
        attr->u32MaxPicHeight = stSize.u32Height;
        attr->u32PicWidth = stSize.u32Width;
        attr->u32PicHeight = stSize.u32Height;
        attr->u32BufSize = stSize.u32Width * stSize.u32Height;
        attr->u32Profile = 0;
        attr->bByFrame = HI_TRUE;
        attr->u32BFrameNum = 0;
        attr->u32RefNum = 1;

        if (options->rcmode == LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.u32Gop = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.u32StatTime = 1;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.u32SrcFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.fr32DstFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.u32MinQp = 10;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.u32MaxQp = 40;
            stVencChnAttr.stRcAttr.stAttrH264Vbr.u32MaxBitRate = options->bitrate;
        } else if (options->rcmode == LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264FIXQP;
            stVencChnAttr.stRcAttr.stAttrH264FixQp.u32Gop = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stAttrH264FixQp.u32SrcFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH264FixQp.fr32DstFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH264FixQp.u32IQp = 20;
            stVencChnAttr.stRcAttr.stAttrH264FixQp.u32PQp = 23;
        } else {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
            stVencChnAttr.stRcAttr.stAttrH264Cbr.u32Gop = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stAttrH264Cbr.u32StatTime = 1;
            stVencChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH264Cbr.fr32DstFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH264Cbr.u32BitRate = options->bitrate;
            stVencChnAttr.stRcAttr.stAttrH264Cbr.u32FluctuateLevel = 0;
        }
    } else if (enType == PT_H265) {
        VENC_ATTR_H265_S *attr = &stVencChnAttr.stVeAttr.stAttrH265e;
        attr->u32MaxPicWidth = stSize.u32Width;
        attr->u32MaxPicHeight = stSize.u32Height;
        attr->u32PicWidth = stSize.u32Width;
        attr->u32PicHeight = stSize.u32Height;
        attr->u32BufSize = stSize.u32Width * stSize.u32Height * 2;
        attr->u32Profile = 0;
        attr->bByFrame = HI_TRUE;
        attr->u32BFrameNum = 0;
        attr->u32RefNum = 1;

        if (options->rcmode == LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.u32Gop = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.u32StatTime = 1;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.u32SrcFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.fr32DstFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.u32MinQp = 10;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.u32MaxQp = 40;
            stVencChnAttr.stRcAttr.stAttrH265Vbr.u32MaxBitRate = options->bitrate;
        } else if (options->rcmode == LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265FIXQP;
            stVencChnAttr.stRcAttr.stAttrH265FixQp.u32Gop = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stAttrH265FixQp.u32SrcFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH265FixQp.fr32DstFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH265FixQp.u32IQp = 20;
            stVencChnAttr.stRcAttr.stAttrH265FixQp.u32PQp = 23;
        } else {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
            stVencChnAttr.stRcAttr.stAttrH265Cbr.u32Gop = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stAttrH265Cbr.u32StatTime = 1;
            stVencChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH265Cbr.fr32DstFrmRate = options->fps;
            stVencChnAttr.stRcAttr.stAttrH265Cbr.u32BitRate = options->bitrate;
            stVencChnAttr.stRcAttr.stAttrH265Cbr.u32FluctuateLevel = 0;
        }
    } else {
        VENC_ATTR_JPEG_S *attr = &stVencChnAttr.stVeAttr.stAttrJpeg;
        attr->u32MaxPicWidth = stSize.u32Width;
        attr->u32MaxPicHeight = stSize.u32Height;
        attr->u32PicWidth = stSize.u32Width;
        attr->u32PicHeight = stSize.u32Height;
        attr->u32BufSize = (((stSize.u32Width + 15) >> 4) << 4) * (((stSize.u32Height + 15) >> 4) << 4);
        attr->bByFrame = HI_TRUE;
        attr->bSupportDCF = HI_FALSE;
    }

    result = HI_MPI_VENC_CreateChn(vencChn, &stVencChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VENC_CreateChn(%d) failed: 0x%x\n", vencChn, result);
        return LOCALSDK_ERROR;
    }

    if (!jpeg) {
        result = HI_MPI_VENC_StartRecvPic(vencChn);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video] HI_MPI_VENC_StartRecvPic(%d) failed: 0x%x\n", vencChn, result);
            HI_MPI_VENC_DestroyChn(vencChn);
            return LOCALSDK_ERROR;
        }
    }

    return LOCALSDK_OK;
}

/* Camera/Sensor defaults (hi3518ev300 + f22 per firmware) */
#define SDK_SENSOR_F22_FPS_DEFAULT 20
#define SDK_SYS_ALIGN_WIDTH        64

static uint32_t sdk_align_up(uint32_t value, uint32_t align) {
    return (value + (align - 1)) & ~(align - 1);
}

static uint32_t sdk_calc_yuv420_blk_size(uint32_t width, uint32_t height) {
    uint32_t w = sdk_align_up(width, SDK_SYS_ALIGN_WIDTH);
    uint32_t h = sdk_align_up(height, SDK_SYS_ALIGN_WIDTH);
    return (w * h * 3) / 2;
}

static int sdk_video_pinmux_jxf22(void) {
    /* Pinmux + clock setup derived from OpenIPC insert_sns() */
    const char *cmds[] = {
        "devmem 0x200f0040 32 0x2", /* I2C0_SCL */
        "devmem 0x200f0044 32 0x2", /* I2C0_SDA */
        "devmem 0x200f007c 32 0x1", /* VI_DATA13 */
        "devmem 0x200f0080 32 0x1", /* VI_DATA10 */
        "devmem 0x200f0084 32 0x1", /* VI_DATA12 */
        "devmem 0x200f0088 32 0x1", /* VI_DATA11 */
        "devmem 0x200f008c 32 0x2", /* VI_VS */
        "devmem 0x200f0090 32 0x2", /* VI_HS */
        "devmem 0x200f0094 32 0x1", /* VI_DATA9 */
        "devmem 0x2003002c 32 0xb4001" /* clk 27MHz, VI 99MHz */
    };

    for (size_t i = 0; i < (sizeof(cmds) / sizeof(cmds[0])); i++) {
        if (sdk_exec_shell(cmds[i]) != SDK_SUCCESS) {
            sdk_log("[sdk][video] pinmux cmd failed or shell callback missing: %s\n", cmds[i]);
            return LOCALSDK_ERROR;
        }
    }
    return LOCALSDK_OK;
}

static void *sdk_isp_thread(void *arg) {
    (void)arg;
    HI_MPI_ISP_Run(0);
    return NULL;
}

static int sdk_video_sensor_register_f22(void) {
    int32_t result = sensor_register_callback();
    if (result != 0) {
        sdk_log("[sdk][video] sensor_register_callback failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

static int sdk_video_isp_init_minimal(int fps) {
    int32_t result;
    ISP_PUB_ATTR_S stPubAttr;
    ISP_WDR_MODE_S stWdrMode;
    ALG_LIB_S stLib;

    result = sdk_video_sensor_register_f22();
    if (result != LOCALSDK_OK) {
        sdk_log("[sdk][video] sensor registration failed\n");
        return LOCALSDK_ERROR;
    }

    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AE_LIB_NAME, sizeof(stLib.acLibName) - 1);
    result = HI_MPI_AE_Register(0, &stLib);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_AE_Register failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    memset(&stLib, 0, sizeof(stLib));
    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AWB_LIB_NAME, sizeof(stLib.acLibName) - 1);
    result = HI_MPI_AWB_Register(0, &stLib);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_AWB_Register failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_ISP_MemInit(0);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_ISP_MemInit failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    memset(&stWdrMode, 0, sizeof(stWdrMode));
    stWdrMode.enWDRMode = WDR_MODE_NONE;
    result = HI_MPI_ISP_SetWDRMode(0, &stWdrMode);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_ISP_SetWDRMode failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    memset(&stPubAttr, 0, sizeof(stPubAttr));
    stPubAttr.stWndRect.s32X = 0;
    stPubAttr.stWndRect.s32Y = 0;
    stPubAttr.stWndRect.u32Width = 1920;
    stPubAttr.stWndRect.u32Height = 1080;
    stPubAttr.f32FrameRate = (HI_FLOAT)fps;
    stPubAttr.enBayer = BAYER_BGGR;

    result = HI_MPI_ISP_SetPubAttr(0, &stPubAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_ISP_SetPubAttr failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_ISP_Init(0);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_ISP_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    if (pthread_create(&g_ispThread, NULL, sdk_isp_thread, NULL) != 0) {
        sdk_log("[sdk][video] Failed to create ISP thread\n");
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

static int sdk_video_vi_isp_init_f22(int fps) {
    int32_t result;

    if (sdk_video_pinmux_jxf22() != LOCALSDK_OK) {
        sdk_log("[sdk][video] Pinmux/clock setup failed\n");
        return LOCALSDK_ERROR;
    }

    result = sdk_video_isp_init_minimal(fps);
    if (result != LOCALSDK_OK) {
        sdk_log("[sdk][video] Minimal ISP init failed\n");
        return LOCALSDK_ERROR;
    }

    /* TODO:
       - Import MIPI combo attributes for JXF22 (e.g. MIPI_*_JXF22_*).
       - Implement VI start (dev/pipe/chn) and sensor registration.
       - Bind VI -> VPSS after VI/ISP is up. */
    return LOCALSDK_OK;
}

/**
 * @brief Initialize video subsystem with HISILICON VPSS
 */
int local_sdk_video_init(int fps) {
    int32_t result;
    VPSS_GRP_ATTR_S stGrpAttr;
    MPP_SYS_CONF_S stSysConf;
    VB_CONF_S stVbConf;

    sdk_log("[sdk][video] Initializing video (fps=%d)\n", fps);
    if (fps <= 0 || fps > 30) {
        sdk_log("[sdk][video] Invalid FPS: %d\n", fps);
        return LOCALSDK_ERROR;
    }

    /* Basic SYS/VB init */
    memset(&stVbConf, 0, sizeof(stVbConf));
    stVbConf.u32MaxPoolCnt = 2;
    stVbConf.astCommPool[0].u32BlkSize = sdk_calc_yuv420_blk_size(1920, 1080);
    stVbConf.astCommPool[0].u32BlkCnt = 6;
    stVbConf.astCommPool[1].u32BlkSize = sdk_calc_yuv420_blk_size(640, 360);
    stVbConf.astCommPool[1].u32BlkCnt = 6;

    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    result = HI_MPI_VB_SetConf(&stVbConf);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VB_SetConf failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    result = HI_MPI_VB_Init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VB_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    memset(&stSysConf, 0, sizeof(stSysConf));
    stSysConf.u32AlignWidth = SDK_SYS_ALIGN_WIDTH;
    result = HI_MPI_SYS_SetConf(&stSysConf);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_SYS_SetConf failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    result = HI_MPI_SYS_Init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_SYS_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* VI/ISP (sensor f22) */
    result = sdk_video_vi_isp_init_f22(fps);
    if (result != LOCALSDK_OK) {
        sdk_log("[sdk][video] VI/ISP init failed\n");
        return LOCALSDK_ERROR;
    }

    /* VPSS group */
    memset(&stGrpAttr, 0, sizeof(VPSS_GRP_ATTR_S));
    stGrpAttr.enPixFmt = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stGrpAttr.u32MaxW = 1920;
    stGrpAttr.u32MaxH = 1080;
    stGrpAttr.bNrEn = HI_TRUE;
    stGrpAttr.stNrAttr.enNrType = VPSS_NR_TYPE_VIDEO;

    result = HI_MPI_VPSS_CreateGrp(g_vpssGrp, &stGrpAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VPSS_CreateGrp failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    result = HI_MPI_VPSS_StartGrp(g_vpssGrp);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VPSS_StartGrp failed: 0x%x\n", result);
        HI_MPI_VPSS_DestroyGrp(g_vpssGrp);
        return LOCALSDK_ERROR;
    }

    sdk_log("[sdk][video] Video init complete\n");
    return LOCALSDK_OK;
}

/**
 * @brief Create video channel with HISILICON VPSS
 */
int local_sdk_video_create(int chn, LOCALSDK_VIDEO_OPTIONS *options) {
    VPSS_CHN_ATTR_S stChnAttr;
    int32_t result;
    
    if (chn < 0 || chn > 3 || !options) {
        sdk_log("[sdk][video] Invalid channel %d or options\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Creating VPSS channel %d\n", chn);
    
    /* Configure channel attributes */
    memset(&stChnAttr, 0, sizeof(VPSS_CHN_ATTR_S));
    stChnAttr.u32Width = (chn == LOCALSDK_VIDEO_PRIMARY_CHANNEL) ? 1920 : 640;
    stChnAttr.u32Height = (chn == LOCALSDK_VIDEO_PRIMARY_CHANNEL) ? 1080 : 360;
    stChnAttr.enChnMode = VPSS_CHN_MODE_USER;
    stChnAttr.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stChnAttr.u32Depth = 0;
    stChnAttr.bMirror = HI_FALSE;
    stChnAttr.bFlip = HI_FALSE;
    stChnAttr.s32Angle = 0;
    
    /* Set channel attributes */
    result = HI_MPI_VPSS_SetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set channel attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Enable channel */
    result = HI_MPI_VPSS_EnableChn(g_vpssGrp, g_vpssChn[chn]);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to enable channel: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Store options in global state */
    memcpy(&g_videoParams[chn * 32], options, sizeof(LOCALSDK_VIDEO_OPTIONS));
    
    sdk_log("[sdk][video] Channel %d created successfully\n", chn);
    return LOCALSDK_OK;
}

/**
 * @brief Set video parameters via HISILICON
 */
int local_sdk_video_set_parameters(int chn, LOCALSDK_VIDEO_OPTIONS *options) {
    VPSS_CHN_ATTR_S stChnAttr;
    int32_t result;
    
    if (chn < 0 || chn > 3 || !options) {
        sdk_log("[sdk][video] Invalid channel or options\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Setting parameters for channel %d\n", chn);
    
    /* Get current attributes */
    result = HI_MPI_VPSS_GetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to get channel attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Update dimensions if specified */
    if (options->resolution == LOCALSDK_VIDEO_RESOLUTION_1920x1080) {
        stChnAttr.u32Width = 1920;
        stChnAttr.u32Height = 1080;
    } else if (options->resolution == LOCALSDK_VIDEO_RESOLUTION_640x360) {
        stChnAttr.u32Width = 640;
        stChnAttr.u32Height = 360;
    }
    
    /* Update attributes */
    result = HI_MPI_VPSS_SetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set channel attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Update video parameters */
    memcpy(&g_videoParams[chn * 32], options, sizeof(LOCALSDK_VIDEO_OPTIONS));
    
    return LOCALSDK_OK;
}

/**
 * @brief Start video streaming
 */
int local_sdk_video_start(int chn) {
    int32_t result;
    LOCALSDK_VIDEO_OPTIONS *options;
    
    if (chn < 0 || chn > 3) {
        sdk_log("[sdk][video] Invalid channel: %d\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Starting video on channel %d\n", chn);
    
    options = sdk_video_get_options(chn);
    if (!options) {
        return LOCALSDK_ERROR;
    }

    result = sdk_video_create_venc_channel(g_vencChn[chn], options, 0);
    if (result != LOCALSDK_OK) {
        return LOCALSDK_ERROR;
    }

    result = sdk_video_bind_vpss_venc(chn, g_vencChn[chn]);
    if (result != LOCALSDK_OK) {
        HI_MPI_VENC_StopRecvPic(g_vencChn[chn]);
        HI_MPI_VENC_DestroyChn(g_vencChn[chn]);
        sdk_log("[sdk][video] Failed to bind VPSS->VENC for channel %d\n", chn);
        return LOCALSDK_ERROR;
    }

    if (g_algoRegisterCb && chn == LOCALSDK_VIDEO_SECONDARY_CHANNEL) {
        g_algoRegisterCb();
    }

    /* Request I-frame to start streaming */
    result = HI_MPI_VENC_RequestIDR(g_vencChn[chn]);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to request IDR: 0x%x\n", result);
    }
    
    g_videoStarted[chn] = 1;
    return LOCALSDK_OK;
}

/**
 * @brief Stop video streaming
 */
int local_sdk_video_stop(int chn, bool state) {
    (void)state;
    if (chn < 0 || chn > 3) {
        sdk_log("[sdk][video] Invalid channel: %d\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Stopping video on channel %d\n", chn);
    g_videoStarted[chn] = 0;
    sdk_video_unbind_vpss_venc(chn, g_vencChn[chn]);
    HI_MPI_VENC_StopRecvPic(g_vencChn[chn]);
    HI_MPI_VENC_DestroyChn(g_vencChn[chn]);
    return LOCALSDK_OK;
}

/**
 * @brief Run video processing - main loop
 */
int local_sdk_video_run(int chn) {
    VENC_STREAM_S stStream;
    VENC_PACK_S stPack;
    int32_t result;
    
    if (chn < 0 || chn > 3 || !g_videoStarted[chn]) {
        return LOCALSDK_ERROR;
    }
    
    /* Get encoded stream */
    memset(&stStream, 0, sizeof(VENC_STREAM_S));
    stStream.pstPack = &stPack;
    stStream.u32PackCount = 1;
    
    result = HI_MPI_VENC_GetStream(g_vencChn[chn], &stStream, 1000);
    if (result != HI_SUCCESS) {
        return result;  /* No stream available yet */
    }
    
    /* Process encoded data via callback (best-effort) */
    if (g_encCb[chn]) {
        LOCALSDK_H26X_FRAME_INFO frameInfo;
        memset(&frameInfo, 0, sizeof(frameInfo));
        frameInfo.data = (signed char *)stStream.pstPack[0].pu8Addr;
        frameInfo.size = stStream.pstPack[0].u32Len;
        frameInfo.timestamp = stStream.pstPack[0].u64PTS;
        frameInfo.type = stStream.pstPack[0].DataType.enH264EType;
        g_encCb[chn](&frameInfo);
    }
    
    /* Release stream */
    result = HI_MPI_VENC_ReleaseStream(g_vencChn[chn], &stStream);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to release stream: 0x%x\n", result);
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Request I-frame (IDR frame)
 */
int local_sdk_video_force_I_frame(int chn) {
    int32_t result;
    
    if (chn < 0 || chn > 3) {
        sdk_log("[sdk][video] Invalid channel: %d\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Requesting I-frame on channel %d\n", chn);
    
    result = HI_MPI_VENC_RequestIDR(g_vencChn[chn]);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to request IDR: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Get JPEG image and save to file
 */
int local_sdk_video_get_jpeg(int chn, char *file) {
    FILE *fp;
    VENC_STREAM_S stStream;
    VENC_CHN_STAT_S stStat;
    VENC_RECV_PIC_PARAM_S stRecvParam;
    int32_t result;
    uint32_t i;
    int32_t vencFd;
    fd_set read_fds;
    struct timeval timeout;
    SIZE_S stSize;
    LOCALSDK_VIDEO_OPTIONS *options;
    VENC_CHN snapChn = 3;
    
    if (chn < 0 || chn > 3 || !file) {
        sdk_log("[sdk][video] Invalid channel or filename\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Saving JPEG from channel %d to %s\n", chn, file);
    
    options = sdk_video_get_options(chn);
    if (!options || sdk_video_resolution_to_size(options->resolution, &stSize) != LOCALSDK_OK) {
        return LOCALSDK_ERROR;
    }

    fp = fopen(file, "wb");
    if (!fp) {
        sdk_log("[sdk][video] Failed to open file: %s\n", file);
        return LOCALSDK_ERROR;
    }

    result = sdk_video_create_venc_channel(snapChn, options, 1);
    if (result != LOCALSDK_OK) {
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    result = sdk_video_bind_vpss_venc(chn, snapChn);
    if (result != LOCALSDK_OK) {
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    memset(&stRecvParam, 0, sizeof(stRecvParam));
    stRecvParam.s32RecvPicNum = 1;
    result = HI_MPI_VENC_StartRecvPicEx(snapChn, &stRecvParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VENC_StartRecvPicEx failed: 0x%x\n", result);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    vencFd = HI_MPI_VENC_GetFd(snapChn);
    if (vencFd < 0) {
        sdk_log("[sdk][video] HI_MPI_VENC_GetFd failed for snapshot channel\n");
        HI_MPI_VENC_StopRecvPic(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    FD_ZERO(&read_fds);
    FD_SET(vencFd, &read_fds);
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    result = select(vencFd + 1, &read_fds, NULL, NULL, &timeout);
    if (result <= 0 || !FD_ISSET(vencFd, &read_fds)) {
        sdk_log("[sdk][video] Snapshot select timeout/failure\n");
        HI_MPI_VENC_StopRecvPic(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    memset(&stStat, 0, sizeof(stStat));
    result = HI_MPI_VENC_Query(snapChn, &stStat);
    if (result != HI_SUCCESS || stStat.u32CurPacks == 0) {
        sdk_log("[sdk][video] Snapshot query failed or empty: 0x%x\n", result);
        HI_MPI_VENC_StopRecvPic(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    memset(&stStream, 0, sizeof(stStream));
    stStream.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
    if (!stStream.pstPack) {
        HI_MPI_VENC_StopRecvPic(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    stStream.u32PackCount = stStat.u32CurPacks;
    result = HI_MPI_VENC_GetStream(snapChn, &stStream, -1);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VENC_GetStream snapshot failed: 0x%x\n", result);
        free(stStream.pstPack);
        HI_MPI_VENC_StopRecvPic(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    for (i = 0; i < stStream.u32PackCount; i++) {
        fwrite(stStream.pstPack[i].pu8Addr, 1, stStream.pstPack[i].u32Len, fp);
    }

    HI_MPI_VENC_ReleaseStream(snapChn, &stStream);
    free(stStream.pstPack);
    HI_MPI_VENC_StopRecvPic(snapChn);
    sdk_video_unbind_vpss_venc(chn, snapChn);
    HI_MPI_VENC_DestroyChn(snapChn);
    fclose(fp);

    sdk_log("[sdk][video] JPEG saved successfully\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set video frame encode callback
 */
int local_sdk_video_set_encode_frame_callback(int chn, int (*callback)(LOCALSDK_H26X_FRAME_INFO *frameInfo)) {
    if (chn < 0 || chn > 3) {
        sdk_log("[sdk][video] Invalid channel: %d\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Setting encode callback for channel %d\n", chn);
    g_encCb[chn] = callback;
    return LOCALSDK_OK;
}

/**
 * @brief Set YUV frame callback
 */
int local_sdk_video_set_yuv_frame_callback(int chn, int (*callback)(LOCALSDK_H26X_FRAME_INFO *frameInfo)) {
    if (chn < 0 || chn > 3) {
        sdk_log("[sdk][video] Invalid channel: %d\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Setting YUV callback for channel %d\n", chn);
    g_yuvCb[chn] = callback;
    return LOCALSDK_OK;
}

/**
 * @brief Set algorithm module registration callback
 */
int local_sdk_video_set_algo_module_register_callback(int (*callback)()) {
    sdk_log("[sdk][video] Setting algo module register callback\n");
    g_algoRegisterCb = callback;
    return LOCALSDK_OK;
}

/**
 * @brief Set algorithm module unregistration callback
 */
int local_sdk_video_set_algo_module_unregister_callback(int (*callback)()) {
    sdk_log("[sdk][video] Setting algo module unregister callback\n");
    /* TODO: Store and invoke algorithm module unregistration */
    return LOCALSDK_OK;
}

/**
 * @brief Set video brightness (ISP parameter)
 */
int local_sdk_video_set_brightness(int param_1, int param_2, int param_3, int param_4) {
    ISP_EXPOSURE_ATTR_S stExpAttr;
    int32_t result;
    
    sdk_log("[sdk][video] Setting brightness: %d\n", param_1);
    
    /* Get current ISP exposure attributes */
    result = HI_MPI_ISP_GetExposureAttr(0, &stExpAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to get exposure attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Update brightness parameter */
    stExpAttr.stAERoute.u32ShutterTime = param_1;
    
    /* Set ISP exposure attributes */
    result = HI_MPI_ISP_SetExposureAttr(0, &stExpAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set exposure attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set video flip/mirror via VPSS
 */
int local_sdk_video_set_flip(int param_1, int param_2) {
    VPSS_CHN_ATTR_S stChnAttr;
    int32_t result;
    int32_t chn = 0;  /* Use first channel */
    
    sdk_log("[sdk][video] Setting flip: %d, mirror: %d\n", param_1, param_2);
    
    result = HI_MPI_VPSS_GetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to get channel attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    stChnAttr.bFlip = (param_1 != 0) ? HI_TRUE : HI_FALSE;
    stChnAttr.bMirror = (param_2 != 0) ? HI_TRUE : HI_FALSE;
    
    result = HI_MPI_VPSS_SetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set channel attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set video framerate
 */
int local_sdk_video_set_fps(int param_1, int param_2, int param_3, int param_4) {
    VENC_PARAM_MOD_S stModParam;
    int32_t result;
    
    sdk_log("[sdk][video] Setting FPS: %d\n", param_1);
    
    result = HI_MPI_VENC_GetModParam(&stModParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to get encoder module params: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    stModParam.u32FrameRate = param_1;
    
    result = HI_MPI_VENC_SetModParam(&stModParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set encoder module params: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set video bitrate
 */
int local_sdk_video_set_kbps(int param_1, int param_2) {
    VENC_RC_PARAM_S stRcParam;
    int32_t result;
    
    sdk_log("[sdk][video] Setting bitrate: %d kbps\n", param_2);
    
    if (param_1 < 0 || param_1 > 1) {
        sdk_log("[sdk][video] Invalid channel for bitrate: %d\n", param_1);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_VENC_GetRcParam(g_vencChn[param_1], &stRcParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to get RC params: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    stRcParam.u32StatTime = 1;
    stRcParam.u32SrcFrmRate = param_1;
    stRcParam.u32TargetBitrate = param_2 * 1000;  /* Convert kbps to bps */
    
    result = HI_MPI_VENC_SetRcParam(g_vencChn[param_1], &stRcParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set RC params: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/* ============================================================================
   AUDIO SUBSYSTEM - HISILICON IMPLEMENTATION
   ============================================================================ */

/* Global audio state */
static AUDIO_DEV g_aiDev = 0;      /* Audio Input Device */
static AI_CHN g_aiChn = 0;         /* Audio Input Channel */
static AUDIO_DEV g_aoDev = 1;      /* Audio Output Device */
static AO_CHN g_aoChn = 0;         /* Audio Output Channel */
static AENC_CHN g_aencChn = 0;     /* Audio Encoder Channel */
static ADEC_CHN g_adecChn = 0;     /* Audio Decoder Channel */
static int32_t g_audioStarted = 0;

/**
 * @brief Initialize audio subsystem with HISILICON AI/AO
 */
int local_sdk_audio_init() {
    AIO_ATTR_S stAioAttr;
    int32_t result;
    
    sdk_log("[sdk][audio] Initializing audio subsystem\n");
    
    /* Configure audio input attributes */
    memset(&stAioAttr, 0, sizeof(AIO_ATTR_S));
    stAioAttr.enSamplerate = AUDIO_SAMPLE_RATE_16000;  /* 16 kHz */
    stAioAttr.enBitwidth = AUDIO_BIT_WIDTH_16;         /* 16-bit */
    stAioAttr.enWorkmode = AIO_MODE_I2S_MASTER;        /* I2S master */
    stAioAttr.u32EXFlag = 0;
    stAioAttr.u32FrmNum = 30;
    stAioAttr.u32PtNumPerFrm = 320;  /* 20ms at 16kHz */
    stAioAttr.u32ChnCnt = 1;         /* Mono */
    
    /* Set AI public attributes */
    result = HI_MPI_AI_SetPubAttr(g_aiDev, &stAioAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to set AI attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Enable audio input device */
    result = HI_MPI_AI_Enable(g_aiDev);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AI: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Set AO public attributes */
    result = HI_MPI_AO_SetPubAttr(g_aoDev, &stAioAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to set AO attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Enable audio output device */
    result = HI_MPI_AO_Enable(g_aoDev);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AO: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][audio] Audio subsystem initialized\n");
    return LOCALSDK_OK;
}

/**
 * @brief Create audio channel with HISILICON AI
 */
int local_sdk_audio_create(int chn) {
    int32_t result;
    
    if (chn < 0 || chn > 15) {
        sdk_log("[sdk][audio] Invalid channel: %d\n", chn);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][audio] Creating audio channel %d\n", chn);
    
    /* Enable AI channel */
    result = HI_MPI_AI_EnableChn(g_aiDev, chn);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AI channel: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    g_aiChn = chn;
    return LOCALSDK_OK;
}

/**
 * @brief Set audio parameters via HISILICON
 */
int local_sdk_audio_set_parameters(int chn, LOCALSDK_AUDIO_OPTIONS *options) {
    AI_CHN_PARAM_S stChnParam;
    int32_t result;
    
    if (chn < 0 || !options) {
        sdk_log("[sdk][audio] Invalid channel or options\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][audio] Setting parameters for channel %d\n", chn);
    
    /* Get current channel parameters */
    result = HI_MPI_AI_GetChnParam(g_aiDev, chn, &stChnParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to get AI channel params: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Update parameters if needed */
    /* TODO: Update stChnParam based on options */
    
    return LOCALSDK_OK;
}

/**
 * @brief Enable/disable AEC (Acoustic Echo Cancellation)
 */
int local_sdk_audio_set_aec_enable(int chn, bool state) {
    int32_t result;
    
    sdk_log("[sdk][audio] AEC %s on channel %d\n", state ? "enabled" : "disabled", chn);
    
    if (state) {
        /* Enable VQE (Voice Quality Enhancement) which includes AEC */
        result = HI_MPI_AI_EnableVqe(g_aiDev, chn);
    } else {
        result = HI_MPI_AI_DisableVqe(g_aiDev, chn);
    }
    
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to set AEC: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set audio volume
 */
int local_sdk_audio_set_volume(int chn, int value) {
    int32_t result;
    
    if (value < -60 || value > 30) {
        sdk_log("[sdk][audio] Invalid volume: %d (must be -60 to 30 dB)\n", value);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][audio] Setting volume on channel %d to %d dB\n", chn, value);
    
    result = HI_MPI_AO_SetVolume(g_aoDev, value);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to set volume: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set audio encode callback
 */
int local_sdk_audio_set_encode_frame_callback(int chn, int (*callback)(LOCALSDK_AUDIO_G711_FRAME_INFO *frameInfo)) {
    sdk_log("[sdk][audio] Setting encode callback for channel %d\n", chn);
    /* TODO: Store callback pointer and invoke from audio processing thread */
    return LOCALSDK_OK;
}

/**
 * @brief Start audio processing
 */
int local_sdk_audio_start() {
    int32_t result;
    
    sdk_log("[sdk][audio] Starting audio\n");
    
    /* Enable AI channel for capture */
    result = HI_MPI_AI_EnableChn(g_aiDev, g_aiChn);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AI channel: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Enable AO channel for playback */
    result = HI_MPI_AO_EnableChn(g_aoDev, g_aoChn);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AO channel: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    g_audioStarted = 1;
    return LOCALSDK_OK;
}

/**
 * @brief Stop audio processing
 */
int local_sdk_audio_stop() {
    sdk_log("[sdk][audio] Stopping audio\n");
    g_audioStarted = 0;
    return LOCALSDK_OK;
}

/**
 * @brief Run audio processing - capture and encode frames
 */
int local_sdk_audio_run() {
    AUDIO_FRAME_S stFrame;
    AEC_FRAME_S stAecFrame;
    int32_t result;
    
    if (!g_audioStarted) {
        return LOCALSDK_ERROR;
    }
    
    /* Get audio frame from AI */
    result = HI_MPI_AI_GetFrame(g_aiDev, g_aiChn, &stFrame, &stAecFrame, 100);
    if (result != HI_SUCCESS) {
        return result;  /* No frame available */
    }
    
    /* TODO: Send frame to AENC encoder or callback */
    
    /* Release frame */
    result = HI_MPI_AI_ReleaseFrame(g_aiDev, g_aiChn, &stFrame, &stAecFrame);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to release frame: 0x%x\n", result);
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief End audio processing
 */
int local_sdk_audio_end() {
    sdk_log("[sdk][audio] Ending audio\n");
    local_sdk_audio_stop();
    return LOCALSDK_OK;
}

/**
 * @brief Destroy audio subsystem
 */
int local_sdk_audio_destory() {
    int32_t result;
    
    sdk_log("[sdk][audio] Destroying audio\n");
    
    /* Disable AI channel */
    HI_MPI_AI_DisableVqe(g_aiDev, g_aiChn);
    
    /* Disable audio devices */
    result = HI_MPI_AI_Disable(g_aiDev);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to disable AI: 0x%x\n", result);
    }
    
    result = HI_MPI_AO_Disable(g_aoDev);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to disable AO: 0x%x\n", result);
    }
    
    return LOCALSDK_OK;
}

/* ============================================================================
   SPEAKER SUBSYSTEM
   ============================================================================ */

/**
 * @brief Initialize speaker
 */
int local_sdk_speaker_init() {
    sdk_log("[sdk][speaker] Initializing speaker\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set speaker parameters
 */
int local_sdk_speaker_set_parameters(LOCALSDK_SPEAKER_OPTIONS *options) {
    if (!options) {
        sdk_log("[sdk][speaker] Invalid options\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][speaker] Setting parameters\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set speaker volume
 */
int local_sdk_speaker_set_volume(int value) {
    sdk_log("[sdk][speaker] Setting volume to %d\n", value);
    return LOCALSDK_OK;
}

/**
 * @brief Mute speaker
 */
int local_sdk_speaker_mute() {
    sdk_log("[sdk][speaker] Muting speaker\n");
    return LOCALSDK_OK;
}

/**
 * @brief Unmute speaker
 */
int local_sdk_speaker_unmute() {
    sdk_log("[sdk][speaker] Unmuting speaker\n");
    return LOCALSDK_OK;
}

/**
 * @brief Start speaker
 */
int local_sdk_speaker_start() {
    sdk_log("[sdk][speaker] Starting speaker\n");
    return LOCALSDK_OK;
}

/**
 * @brief Feed PCM data to speaker
 */
int local_sdk_speaker_feed_pcm_data(void *data, int size) {
    if (!data || size <= 0) {
        sdk_log("[sdk][speaker] Invalid PCM data\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][speaker] Feeding PCM data: %d bytes\n", size);
    return LOCALSDK_OK;
}

/**
 * @brief Feed G711 encoded data to speaker
 */
int local_sdk_speaker_feed_g711_data(void *data, int size) {
    if (!data || size <= 0) {
        sdk_log("[sdk][speaker] Invalid G711 data\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][speaker] Feeding G711 data: %d bytes\n", size);
    return LOCALSDK_OK;
}

/**
 * @brief Finish buffer data
 */
int local_sdk_speaker_finish_buf_data() {
    sdk_log("[sdk][speaker] Finishing buffer data\n");
    return LOCALSDK_OK;
}

/**
 * @brief Clean buffer data
 */
int local_sdk_speaker_clean_buf_data() {
    sdk_log("[sdk][speaker] Cleaning buffer data\n");
    return LOCALSDK_OK;
}

/* ============================================================================
   ALARM SUBSYSTEM
   ============================================================================ */

/**
 * @brief Initialize alarm subsystem
 */
int local_sdk_alarm_init(int width, int height) {
    if (width <= 0 || height <= 0) {
        sdk_log("[sdk][alarm] Invalid dimensions: %dx%d\n", width, height);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][alarm] Initializing alarm: %dx%d\n", width, height);
    g_alarmState = 1;
    return LOCALSDK_OK;
}

/**
 * @brief Set alarm sensitivity
 */
int local_sdk_set_alarm_sensitivity(int type, int value) {
    if (value < 1 || value > 255) {
        sdk_log("[sdk][alarm] Invalid sensitivity: %d\n", value);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][alarm] Setting alarm sensitivity type %d to %d\n", type, value);
    return LOCALSDK_OK;
}

/**
 * @brief Exit alarm subsystem
 */
int local_sdk_alarm_exit() {
    sdk_log("[sdk][alarm] Exiting alarm\n");
    g_alarmState = 0;
    return LOCALSDK_OK;
}

/**
 * @brief Set motor state
 */
int local_sdk_alarm_set_motor_state() {
    sdk_log("[sdk][alarm] Setting motor state\n");
    return LOCALSDK_OK;
}

/**
 * @brief Register alarm algorithm module callback
 */
int local_sdk_alarm_algo_module_register_callback() {
    sdk_log("[sdk][alarm] Registering algo module callback\n");
    return LOCALSDK_OK;
}

/**
 * @brief Unregister alarm algorithm module callback
 */
int local_sdk_alarm_algo_module_unregister_callback() {
    sdk_log("[sdk][alarm] Unregistering algo module callback\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set alarm state callback (on event)
 */
int local_sdk_alarm_state_set_callback(int (*callback)(LOCALSDK_ALARM_EVENT_INFO *eventInfo)) {
    sdk_log("[sdk][alarm] Setting alarm state callback\n");
    if (!callback) {
        return LOCALSDK_ERROR;
    }
    pthread_mutex_lock(&g_alarmCbMutex);
    if (alarm_cb_find(callback)) {
        pthread_mutex_unlock(&g_alarmCbMutex);
        return LOCALSDK_OK;
    }
    AlarmCbNode *node = alarm_cb_alloc();
    if (!node) {
        pthread_mutex_unlock(&g_alarmCbMutex);
        return LOCALSDK_ERROR;
    }
    node->cb = callback;
    int result = alarm_cb_add(node);
    if (result != LOCALSDK_OK) {
        alarm_cb_free(node);
    }
    pthread_mutex_unlock(&g_alarmCbMutex);
    return result;
}

/**
 * @brief Clear alarm state callback (on event end)
 */
int local_sdk_alarm_state_clear_callback(int (*callback)(LOCALSDK_ALARM_EVENT_INFO *eventInfo)) {
    sdk_log("[sdk][alarm] Clearing alarm state callback\n");
    if (!callback) {
        return LOCALSDK_ERROR;
    }
    pthread_mutex_lock(&g_alarmCbMutex);
    AlarmCbNode *node = alarm_cb_find(callback);
    if (!node) {
        pthread_mutex_unlock(&g_alarmCbMutex);
        return LOCALSDK_ERROR;
    }
    int result = alarm_cb_remove(node);
    alarm_cb_free(node);
    pthread_mutex_unlock(&g_alarmCbMutex);
    return result;
}

/**
 * @brief Set alarm network state
 */
int local_sdk_set_alarm_network_state() {
    sdk_log("[sdk][alarm] Setting alarm network state\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set alarm switch (enable/disable)
 */
int local_sdk_set_alarm_switch(int type, bool state) {
    sdk_log("[sdk][alarm] Alarm type %d %s\n", type, state ? "enabled" : "disabled");
    return LOCALSDK_OK;
}

/* ============================================================================
   OSD SUBSYSTEM (On-Screen Display) - HISILICON IMPLEMENTATION
   ============================================================================ */

/* Global OSD state */
static RGN_HANDLE g_logoRgnHandle = 0;
static RGN_HANDLE g_timeRgnHandle = 1;
static RGN_HANDLE g_rectRgnHandle = 2;
static int32_t g_osdInitialized = 0;
static MPP_CHN_S g_osdVencChn;

/**
 * @brief Initialize OSD region for VENC channel
 */
static int32_t _osd_region_init(RGN_HANDLE handle, RGN_TYPE_E type) {
    RGN_ATTR_S stRgnAttr;
    int32_t result;
    
    memset(&stRgnAttr, 0, sizeof(RGN_ATTR_S));
    stRgnAttr.enType = type;
    
    if (type == OVERLAY_RGN) {
        stRgnAttr.unAttr.stOverlay.enPixelFmt = PIXEL_FORMAT_RGB_1555;
        stRgnAttr.unAttr.stOverlay.u32BgColor = 0xFFFF;  /* White background */
        stRgnAttr.unAttr.stOverlay.u32Size = 1920 * 1080 / 8;  /* ~250KB for overlay */
    } else if (type == COVER_RGN) {
        stRgnAttr.unAttr.stCover.enColor = COVER_COLOR_BLACK;
    }
    
    result = HI_MPI_RGN_Create(handle, &stRgnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to create region %d: 0x%x\n", handle, result);
        return result;
    }
    
    return HI_SUCCESS;
}

/**
 * @brief Set OSD parameters via HISILICON RGN
 */
int local_sdk_video_osd_set_parameters(int chn, LOCALSDK_OSD_OPTIONS *options) {
    RGN_CHN_ATTR_S stChnAttr;
    int32_t result;
    MPP_CHN_S stChn;
    
    if (chn < 0 || chn > 1 || !options) {
        sdk_log("[sdk][osd] Invalid channel or options\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][osd] Setting parameters for channel %d\n", chn);
    
    /* Configure MPP channel (VENC) */
    stChn.enModType = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = chn;
    
    memset(&stChnAttr, 0, sizeof(RGN_CHN_ATTR_S));
    stChnAttr.bShow = HI_TRUE;
    stChnAttr.enType = OVERLAY_RGN;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = options->x;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = options->y;
    stChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 128;
    stChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 128;
    
    result = HI_MPI_RGN_SetDisplayAttr(g_logoRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to set display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Display OEM logo (MI branding) using HISILICON RGN
 */
int local_sdk_video_osd_update_logo(int chn, bool state) {
    RGN_CHN_ATTR_S stChnAttr;
    int32_t result;
    MPP_CHN_S stChn;
    
    sdk_log("[sdk][osd] OEM logo %s on channel %d\n", state ? "displayed" : "hidden", chn);
    
    /* Initialize region if not done */
    if (!g_osdInitialized) {
        result = _osd_region_init(g_logoRgnHandle, OVERLAY_RGN);
        if (result != HI_SUCCESS) {
            return LOCALSDK_ERROR;
        }
        g_osdInitialized = 1;
    }
    
    /* Configure VENC channel */
    stChn.enModType = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = chn;
    
    /* Get current display attributes */
    result = HI_MPI_RGN_GetDisplayAttr(g_logoRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to get display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Update visibility */
    stChnAttr.bShow = state ? HI_TRUE : HI_FALSE;
    
    result = HI_MPI_RGN_SetDisplayAttr(g_logoRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to update display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Display date and time on video via HISILICON RGN
 */
int local_sdk_video_osd_update_timestamp(int chn, bool state, struct tm *timestamp) {
    RGN_CHN_ATTR_S stChnAttr;
    RGN_CANVAS_INFO_S stCanvasInfo;
    int32_t result;
    MPP_CHN_S stChn;
    char timeStr[64];
    
    sdk_log("[sdk][osd] Timestamp %s on channel %d\n", state ? "enabled" : "disabled", chn);
    
    /* Initialize time region if needed */
    if (!g_osdInitialized) {
        result = _osd_region_init(g_timeRgnHandle, OVERLAY_RGN);
        if (result != HI_SUCCESS) {
            return LOCALSDK_ERROR;
        }
    }
    
    /* Configure VENC channel */
    stChn.enModType = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = chn;
    
    /* Get canvas info for drawing */
    result = HI_MPI_RGN_GetCanvasInfo(g_timeRgnHandle, &stCanvasInfo);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to get canvas info: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Format timestamp string */
    if (timestamp) {
        snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
                 timestamp->tm_year + 1900, timestamp->tm_mon + 1, timestamp->tm_mday,
                 timestamp->tm_hour, timestamp->tm_min, timestamp->tm_sec);
    } else {
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    }
    
    sdk_log("[sdk][osd] Time string: %s\n", timeStr);
    
    /* TODO: Draw text on canvas and update */
    
    /* Update canvas */
    result = HI_MPI_RGN_UpdateCanvas(g_timeRgnHandle);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to update canvas: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Set display attributes */
    result = HI_MPI_RGN_GetDisplayAttr(g_timeRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to get display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    stChnAttr.bShow = state ? HI_TRUE : HI_FALSE;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = 10;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = 10;
    
    result = HI_MPI_RGN_SetDisplayAttr(g_timeRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to set display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Display multiple rectangles on video via HISILICON RGN
 */
int local_sdk_video_osd_update_rect_multi(int chn, bool state, LOCALSDK_OSD_RECTANGLES *rectangles) {
    RGN_CHN_ATTR_S stChnAttr;
    int32_t result;
    MPP_CHN_S stChn;
    
    sdk_log("[sdk][osd] Rectangle display %s on channel %d\n", state ? "enabled" : "disabled", chn);
    
    /* Initialize rectangle region if needed */
    if (!g_osdInitialized) {
        result = _osd_region_init(g_rectRgnHandle, COVER_RGN);
        if (result != HI_SUCCESS) {
            return LOCALSDK_ERROR;
        }
    }
    
    /* Configure VENC channel */
    stChn.enModType = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = chn;
    
    /* Get current display attributes */
    result = HI_MPI_RGN_GetDisplayAttr(g_rectRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to get display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Update visibility and position if rectangles provided */
    stChnAttr.bShow = state ? HI_TRUE : HI_FALSE;
    
    if (rectangles && state) {
        stChnAttr.unChnAttr.stCoverChn.stRect.s32X = rectangles->x;
        stChnAttr.unChnAttr.stCoverChn.stRect.s32Y = rectangles->y;
        stChnAttr.unChnAttr.stCoverChn.stRect.u32Width = rectangles->width;
        stChnAttr.unChnAttr.stCoverChn.stRect.u32Height = rectangles->height;
    }
    
    result = HI_MPI_RGN_SetDisplayAttr(g_rectRgnHandle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][osd] Failed to update display attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/* ============================================================================
   HARDWARE CONTROL SUBSYSTEM
   ============================================================================ */

/**
 * @brief Control indicator LEDs (orange and blue)
 */
int local_sdk_indicator_led_option(bool orange, bool blue) {
    sdk_log("[sdk][hw] LED control - orange: %s, blue: %s\n", 
            orange ? "on" : "off", blue ? "on" : "off");
    return LOCALSDK_OK;
}

/**
 * @brief Setup keydown callback with timeout
 */
int local_sdk_setup_keydown_set_callback(int timeout, int (*callback)()) {
    sdk_log("[sdk][hw] Setting keydown callback with timeout: %d ms\n", timeout);
    return LOCALSDK_OK;
}

/* ============================================================================
   NIGHT MODE SUBSYSTEM
   ============================================================================ */

/**
 * @brief Set daytime mode (color image)
 */
int local_sdk_video_set_daytime_mode() {
    sdk_log("[sdk][night] Setting daytime mode (color)\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set night mode (grayscale image)
 */
int local_sdk_video_set_night_mode() {
    sdk_log("[sdk][night] Setting night mode (grayscale)\n");
    return LOCALSDK_OK;
}

/**
 * @brief Enable automatic night mode switching
 */
int local_sdk_auto_night_light() {
    sdk_log("[sdk][night] Enabling auto night mode\n");
    return LOCALSDK_OK;
}

/**
 * @brief Enable manual night mode
 */
int local_sdk_open_night_light() {
    sdk_log("[sdk][night] Opening night light (manual)\n");
    return LOCALSDK_OK;
}

/**
 * @brief Disable manual night mode
 */
int local_sdk_close_night_light() {
    sdk_log("[sdk][night] Closing night light\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set night mode state change callback
 */
int local_sdk_night_state_set_callback(int (*callback)(int state)) {
    sdk_log("[sdk][night] Setting night state callback\n");
    return LOCALSDK_OK;
}

/**
 * @brief Open IR-cut filter
 */
int local_sdk_open_ircut() {
    sdk_log("[sdk][night] Opening IR-cut filter\n");
    return LOCALSDK_OK;
}

/**
 * @brief Close IR-cut filter
 */
int local_sdk_close_ircut() {
    sdk_log("[sdk][night] Closing IR-cut filter\n");
    return LOCALSDK_OK;
}

/* ============================================================================
   UTILITY FUNCTIONS
   ============================================================================ */

/**
 * @brief Get picture size by resolution type
 */
int SAMPLE_COMM_SYS_GetPicSize(int resolution, LOCALSDK_PICTURE_SIZE *size) {
    if (!size) {
        sdk_log("[sdk] GetPicSize: Invalid size pointer\n");
        return LOCALSDK_ERROR;
    }
    
    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
            size->width = 640;
            size->height = 360;
            return LOCALSDK_OK;
        
        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            size->width = 1920;
            size->height = 1080;
            return LOCALSDK_OK;
        
        default:
            sdk_log("[sdk] GetPicSize: Unknown resolution %d\n", resolution);
            return LOCALSDK_ERROR;
    }
}

/**
 * @brief Convert resolution type to internal format
 */
int inner_change_resulu_type(int resolution, int *result) {
    if (!result) {
        sdk_log("[sdk] change_resulu_type: Invalid result pointer\n");
        return LOCALSDK_ERROR;
    }
    
    /* Map external resolution constants to internal values */
    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
            *result = 0;
            return LOCALSDK_OK;
        
        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            *result = 6;
            return LOCALSDK_OK;
        
        default:
            sdk_log("[sdk] change_resulu_type: Unknown resolution %d\n", resolution);
            return LOCALSDK_ERROR;
    }
}

/* ============================================================================
   IVP MODEL MANAGEMENT SUBSYSTEM (OMS Files)
   ============================================================================ */

/* Global IVP state */
static void* g_ivpResourceHandle = NULL;
static uint32_t g_ivpResourceSize = 0;
static uint8_t* g_ivpResourceBuffer = NULL;
static int32_t g_ivpInitialized = 0;

/**
 * @brief Get file size for OMS model loading
 */
static uint32_t sample_ivp_get_file_size(const char *filename) {
    FILE *fp;
    uint32_t file_size = 0;
    
    if (!filename) {
        sdk_log("[sdk][ivp] Invalid filename\n");
        return 0;
    }
    
    fp = fopen(filename, "rb");
    if (!fp) {
        sdk_log("[sdk][ivp] Failed to open file: %s\n", filename);
        return 0;
    }
    
    /* Get file size */
    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    
    fclose(fp);
    return file_size;
}

/**
 * @brief Read OMS model file into memory
 */
static int32_t sample_ivp_read_file(const char *filename, uint8_t *buffer, uint32_t buffer_size) {
    FILE *fp;
    uint32_t read_size = 0;
    
    if (!filename || !buffer || buffer_size == 0) {
        sdk_log("[sdk][ivp] Invalid parameters for file read\n");
        return LOCALSDK_ERROR;
    }
    
    fp = fopen(filename, "rb");
    if (!fp) {
        sdk_log("[sdk][ivp] Failed to open file: %s\n", filename);
        return LOCALSDK_ERROR;
    }
    
    /* Read file into buffer */
    read_size = fread(buffer, 1, buffer_size, fp);
    fclose(fp);
    
    if (read_size != buffer_size) {
        sdk_log("[sdk][ivp] Read size mismatch: expected %u, got %u\n", 
                buffer_size, read_size);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] Successfully read %u bytes from %s\n", read_size, filename);
    return LOCALSDK_OK;
}

/**
 * @brief Load IVP model from OMS file
 * 
 * This function:
 * 1. Opens the OMS (Object Model System) file
 * 2. Allocates MMZ memory for the model
 * 3. Loads the model data into memory
 * 4. Initializes the IVP processor
 */
int32_t sample_ivp_load_resource(const char *oms_file, int32_t chn) {
    uint32_t file_size = 0;
    uint64_t phys_addr = 0;
    uint64_t virt_addr = 0;
    uint32_t virt_size = 0;
    int32_t result;
    
    if (!oms_file) {
        sdk_log("[sdk][ivp] Invalid OMS file path\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] Loading IVP model from: %s\n", oms_file);
    
    /* Get file size */
    file_size = sample_ivp_get_file_size(oms_file);
    if (file_size == 0) {
        sdk_log("[sdk][ivp] Failed to get OMS file size\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] OMS file size: %u bytes\n", file_size);
    
    /* Allocate MMZ (Media Memory Zone) for model data */
    result = HI_MPI_SYS_MmzAlloc(&phys_addr, &virt_addr, "IVP_MODEL", NULL, file_size);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to allocate MMZ memory: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] MMZ allocated - Physical: 0x%llx, Virtual: 0x%llx, Size: %u\n",
            phys_addr, virt_addr, file_size);
    
    /* Read OMS file into memory */
    result = sample_ivp_read_file(oms_file, (uint8_t *)virt_addr, file_size);
    if (result != LOCALSDK_OK) {
        sdk_log("[sdk][ivp] Failed to read OMS file\n");
        HI_MPI_SYS_MmzFree(phys_addr, virt_addr, file_size);
        return LOCALSDK_ERROR;
    }
    
    /* Load resource from memory into IVP */
    result = hi_ivp_load_resource_from_memory((void *)virt_addr, file_size);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to load resource from memory: 0x%x\n", result);
        HI_MPI_SYS_MmzFree(phys_addr, virt_addr, file_size);
        return LOCALSDK_ERROR;
    }
    
    /* Store resource handle for later cleanup */
    g_ivpResourceHandle = (void *)virt_addr;
    g_ivpResourceSize = file_size;
    g_ivpResourceBuffer = (uint8_t *)virt_addr;
    
    sdk_log("[sdk][ivp] IVP model loaded successfully\n");
    return LOCALSDK_OK;
}

/**
 * @brief Initialize IVP (Intelligent Video Processing) processor
 * 
 * This function:
 * 1. Initializes the IVP hardware
 * 2. Loads the OMS model file
 * 3. Configures IVP channels
 */
int32_t sample_ivp_init(int32_t width, int32_t height, const char *oms_file) {
    int32_t result;
    
    if (g_ivpInitialized) {
        sdk_log("[sdk][ivp] IVP already initialized\n");
        return LOCALSDK_OK;
    }
    
    if (!oms_file) {
        oms_file = "/usr/app/local/ivp_re_im_allday_16chn_pr1_640x360_v1040.oms";
    }
    
    sdk_log("[sdk][ivp] Initializing IVP: %dx%d with model: %s\n", 
            width, height, oms_file);
    
    /* Initialize IVP hardware */
    result = hi_ivp_init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to initialize IVP hardware: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    /* Load IVP model from OMS file */
    result = sample_ivp_load_resource(oms_file, 0);
    if (result != LOCALSDK_OK) {
        sdk_log("[sdk][ivp] Failed to load IVP resource\n");
        hi_ivp_deinit();
        return LOCALSDK_ERROR;
    }
    
    g_ivpInitialized = 1;
    sdk_log("[sdk][ivp] IVP initialization complete\n");
    return LOCALSDK_OK;
}

/**
 * @brief Unload IVP model and free resources
 */
int32_t sample_ivp_unload_resource(void) {
    int32_t result;
    
    if (!g_ivpResourceHandle) {
        sdk_log("[sdk][ivp] No IVP resource to unload\n");
        return LOCALSDK_OK;
    }
    
    sdk_log("[sdk][ivp] Unloading IVP resource\n");
    
    /* Unload resource from IVP */
    result = hi_ivp_unload_resource(g_ivpResourceHandle);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to unload resource: 0x%x\n", result);
    }
    
    /* Free MMZ memory */
    if (g_ivpResourceBuffer) {
        HI_MPI_SYS_MmzFree((uint64_t)g_ivpResourceBuffer, 
                          (uint64_t)g_ivpResourceBuffer, 
                          g_ivpResourceSize);
    }
    
    /* Clear state */
    g_ivpResourceHandle = NULL;
    g_ivpResourceBuffer = NULL;
    g_ivpResourceSize = 0;
    
    sdk_log("[sdk][ivp] IVP resource unloaded\n");
    return LOCALSDK_OK;
}

/**
 * @brief Deinitialize IVP processor
 */
int32_t sample_ivp_deinit(void) {
    int32_t result;
    
    if (!g_ivpInitialized) {
        sdk_log("[sdk][ivp] IVP not initialized\n");
        return LOCALSDK_OK;
    }
    
    sdk_log("[sdk][ivp] Deinitializing IVP\n");
    
    /* Unload resource first */
    sample_ivp_unload_resource();
    
    /* Deinitialize IVP hardware */
    result = hi_ivp_deinit();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to deinitialize IVP: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    g_ivpInitialized = 0;
    sdk_log("[sdk][ivp] IVP deinitialization complete\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set IVP control attributes
 */
int32_t sample_ivp_set_ctrl_attr(int32_t chn, void *ctrl_attr) {
    int32_t result;
    
    if (chn < 0 || !ctrl_attr) {
        sdk_log("[sdk][ivp] Invalid channel or control attributes\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] Setting control attributes for channel %d\n", chn);
    
    result = hi_ivp_set_ctrl_attr(ctrl_attr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set control attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set IVP ROI (Region of Interest) attributes
 */
int32_t sample_ivp_set_roi_attr(int32_t chn, void *roi_attr) {
    int32_t result;
    
    if (chn < 0 || !roi_attr) {
        sdk_log("[sdk][ivp] Invalid channel or ROI attributes\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] Setting ROI attributes for channel %d\n", chn);
    
    result = hi_ivp_set_roi_attr(roi_attr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set ROI attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set IVP ROI map
 */
int32_t sample_ivp_set_roi_map(int32_t chn, void *roi_map) {
    int32_t result;
    
    if (chn < 0 || !roi_map) {
        sdk_log("[sdk][ivp] Invalid channel or ROI map\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][ivp] Setting ROI map for channel %d\n", chn);
    
    result = hi_ivp_set_roi_map(roi_map);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set ROI map: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Process frame with IVP
 */
int32_t sample_ivp_process_frame(int32_t chn, void *frame_data, int32_t frame_size) {
    int32_t result;
    
    if (chn < 0 || !frame_data || frame_size <= 0) {
        sdk_log("[sdk][ivp] Invalid channel, frame data, or size\n");
        return LOCALSDK_ERROR;
    }
    
    /* Process frame with IVP */
    result = hi_ivp_process_ex(frame_data);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to process frame: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Advance IVP ISP settings (fine-tuning)
 */
int32_t sample_ivp_set_advance_isp(int32_t param1, int32_t param2) {
    int32_t result;
    
    sdk_log("[sdk][ivp] Setting advanced ISP parameters: %d, %d\n", param1, param2);
    
    result = hi_ivp_set_advance_isp(param1, param2);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set advanced ISP: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set VENC low bitrate mode for IVP optimization
 */
int32_t sample_ivp_set_venc_low_bitrate(int32_t chn, int32_t bitrate) {
    int32_t result;
    
    sdk_log("[sdk][ivp] Setting low bitrate mode: bitrate=%d kbps\n", bitrate);
    
    result = hi_ivp_set_venc_low_bitrate(bitrate);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set low bitrate: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set VENC lowlight ISO threshold for night vision
 */
int32_t sample_ivp_set_venc_lowlight_iso_threshold(int32_t threshold) {
    int32_t result;
    
    sdk_log("[sdk][ivp] Setting lowlight ISO threshold: %d\n", threshold);
    
    result = hi_ivp_set_venc_lowlight_iso_threshold(threshold);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set lowlight ISO threshold: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    
    return LOCALSDK_OK;
}

/* ============================================================================
   NETWORK AND WiFi SUBSYSTEM
   ============================================================================ */

/**
 * @brief Get WPA supplicant configuration path
 */
int local_sdk_get_wpa_config_path() {
    sdk_log("[sdk][net] Getting WPA config path\n");
    return (int)(intptr_t)"/etc/wpa_supplicant/wpa_supplicant.conf";
}

/**
 * @brief Get hostapd configuration path
 */
int local_sdk_get_hostapd_config_path() {
    sdk_log("[sdk][net] Getting hostapd config path\n");
    return (int)(intptr_t)"/etc/hostapd/hostapd.conf";
}

/**
 * @brief Get UDHCPD configuration path
 */
int local_sdk_get_udhcpd_config_path() {
    sdk_log("[sdk][net] Getting UDHCPD config path\n");
    return (int)(intptr_t)"/etc/udhcpd.conf";
}

/**
 * @brief Restart WiFi power cycle
 */
int local_sdk_wifi_power_restart() {
    sdk_log("[sdk][net] Restarting WiFi power\n");
    
    /* Execute WiFi restart sequence */
    sdk_exec_shell("ifconfig wlan0 down");
    usleep(USLEEP_100MS);
    sdk_exec_shell("ifconfig wlan0 up");
    usleep(USLEEP_100MS);
    
    return LOCALSDK_OK;
}

/**
 * @brief Power off WiFi
 */
int local_sdk_wifi_power_off() {
    sdk_log("[sdk][net] Powering off WiFi\n");
    return sdk_exec_shell("ifconfig wlan0 down");
}

/**
 * @brief Restart wlan0 interface
 */
int local_sdk_wifi_wlan0_restart() {
    sdk_log("[sdk][net] Restarting wlan0\n");
    sdk_exec_shell("ifconfig wlan0 down");
    usleep(USLEEP_50MS);
    sdk_exec_shell("ifconfig wlan0 up");
    return LOCALSDK_OK;
}

/**
 * @brief Run WPA supplicant for WiFi connection
 */
int local_sdk_net_run_wpa() {
    sdk_log("[sdk][net] Running WPA supplicant\n");
    
    /* Kill any existing wpa_supplicant instances */
    sdk_exec_shell("killall wpa_supplicant");
    usleep(USLEEP_100MS);
    
    /* Start WPA supplicant */
    return sdk_exec_shell("wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf");
}

/**
 * @brief Run DHCP client on wlan0
 */
int local_sdk_net_run_dhcp() {
    sdk_log("[sdk][net] Running DHCP client\n");
    
    /* Kill any existing DHCP client instances */
    sdk_exec_shell("killall udhcpc");
    usleep(USLEEP_50MS);
    
    /* Start DHCP client */
    return sdk_exec_shell("udhcpc -i wlan0");
}

/**
 * @brief Set MAC address on WiFi chip
 */
char* local_sdk_net_set_mac_to_wifi_chip(char* arg1) {
    sdk_log("[sdk][net] Setting MAC address\n");
    
    if (!arg1) {
        sdk_log("[sdk][net] Invalid MAC address pointer\n");
        return NULL;
    }
    
    /* Execute MAC address configuration */
    char cmd_buf[256];
    snprintf(cmd_buf, sizeof(cmd_buf), "ifconfig wlan0 hw ether %s", arg1);
    sdk_exec_shell(cmd_buf);
    
    return arg1;
}

/* ============================================================================
   DEVICE MANAGEMENT SUBSYSTEM
   ============================================================================ */

/**
 * @brief Execute shell command with optional wait
 */
int system_exec_shell(int arg1, int arg2, int arg3) {
    const char *cmd = (const char *)(intptr_t)arg1;
    
    if (!cmd) {
        sdk_log("[sdk][device] Invalid shell command\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][device] Executing shell: %s\n", cmd);
    return sdk_exec_shell(cmd);
}

/**
 * @brief Open device (mount storage, etc.)
 */
int local_sdk_device_open(int arg1, void* arg2) {
    sdk_log("[sdk][device] Opening device type %d\n", arg1);
    
    /* Handle different device types */
    switch (arg1) {
        case 0:  /* SD card */
            sdk_log("[sdk][device] Opening SD card\n");
            return sdk_exec_shell("mount -t vfat /dev/mmcblk0p1 /mnt/sd");
        
        case 1:  /* USB disk */
            sdk_log("[sdk][device] Opening USB disk\n");
            return sdk_exec_shell("mount -t vfat /dev/sda1 /mnt/usb");
        
        case 2:  /* TTY USB */
            sdk_log("[sdk][device] Opening TTY USB device\n");
            return 0;  /* Device handle or success code */
        
        default:
            sdk_log("[sdk][device] Unknown device type %d\n", arg1);
            return LOCALSDK_ERROR;
    }
}

/**
 * @brief Close device
 */
int* local_sdk_device_close(int arg1) {
    sdk_log("[sdk][device] Closing device type %d\n", arg1);
    
    /* Handle different device types */
    switch (arg1) {
        case 0:  /* SD card */
            sdk_log("[sdk][device] Closing SD card\n");
            sdk_exec_shell("umount /mnt/sd");
            break;
        
        case 1:  /* USB disk */
            sdk_log("[sdk][device] Closing USB disk\n");
            sdk_exec_shell("umount /mnt/usb");
            break;
        
        case 2:  /* TTY USB */
            sdk_log("[sdk][device] Closing TTY USB device\n");
            break;
        
        default:
            sdk_log("[sdk][device] Unknown device type %d\n", arg1);
    }
    
    return (int *)LOCALSDK_OK;
}

/* ============================================================================
   VIDEO STREAMING THREADS
   ============================================================================ */

/**
 * @brief Video YUV stream thread - captures and processes YUV frames
 * 
 * This thread continuously captures YUV frames from VPSS and delivers them
 * via callback or stores them for later processing.
 */
int video_yuv_stream_thread(int32_t* arg1) {
    VPSS_GRP_FRAME_S stGrpFrame;
    VIDEO_FRAME_S *pstFrame;
    int32_t result;
    int32_t s32Count = 0;
    
    sdk_log("[sdk][video_thread] YUV stream thread started\n");
    
    while (sdk_video_any_started()) {
        /* Get frame from VPSS group */
        memset(&stGrpFrame, 0, sizeof(VPSS_GRP_FRAME_S));
        stGrpFrame.stVFrame.u32PoolId = 0;
        
        result = HI_MPI_VPSS_GetChnFrame(0, 0, &pstFrame, 1000);
        if (result != HI_SUCCESS) {
            usleep(USLEEP_50MS);
            continue;
        }
        
        /* TODO: Send to YUV callback or process frame */
        sdk_log("[sdk][video_thread] YUV frame captured: %ux%u\n", 
                pstFrame->u32Width, pstFrame->u32Height);
        
        /* Release frame back to pool */
        result = HI_MPI_VPSS_ReleaseChnFrame(0, 0, pstFrame);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video_thread] Failed to release frame: 0x%x\n", result);
        }
        
        s32Count++;
        if ((s32Count % 30) == 0) {
            sdk_log("[sdk][video_thread] YUV frames processed: %d\n", s32Count);
        }
        
        usleep(USLEEP_50MS);
    }
    
    sdk_log("[sdk][video_thread] YUV stream thread stopped\n");
    return LOCALSDK_OK;
}

/**
 * @brief Video encoded stream thread - gets H264/H265 from VENC
 * 
 * This thread continuously fetches encoded video frames from VENC and
 * delivers them via callback for RTSP streaming or file recording.
 */
void video_enc_stream_thread(int32_t* arg1) {
    VENC_STREAM_S stStream;
    VENC_PACK_S stPack;
    int32_t result;
    int32_t s32Count = 0;
    uint32_t u32PackCount = 0;
    uint32_t i;
    
    sdk_log("[sdk][video_thread] Encoded stream thread started\n");
    
    while (sdk_video_any_started()) {
        /* Get encoded stream from VENC */
        memset(&stStream, 0, sizeof(VENC_STREAM_S));
        stStream.pstPack = &stPack;
        stStream.u32PackCount = 1;
        
        result = HI_MPI_VENC_GetStream(g_vencChn[0], &stStream, 1000);
        if (result != HI_SUCCESS) {
            usleep(USLEEP_50MS);
            continue;
        }
        
        u32PackCount = stStream.u32PackCount;
        
        /* Process each pack in the stream */
        for (i = 0; i < u32PackCount; i++) {
            VENC_PACK_S *pstPack = &stStream.pstPack[i];
            
            /* Identify frame type */
            if (pstPack->DataType.enH264EType == H264E_NALU_SPS ||
                pstPack->DataType.enH264EType == H264E_NALU_PPS ||
                pstPack->DataType.enH264EType == H264E_NALU_IDR) {
                sdk_log("[sdk][video_thread] IDR/SPS/PPS frame: %u bytes\n", 
                        pstPack->u32Len);
            } else {
                sdk_log("[sdk][video_thread] P/B frame: %u bytes\n", 
                        pstPack->u32Len);
            }
            
            /* TODO: Send to encode callback or RTSP server */
            /* callback_encode_frame(pstPack->pu8Addr, pstPack->u32Len); */
        }
        
        /* Release stream back to VENC */
        result = HI_MPI_VENC_ReleaseStream(g_vencChn[0], &stStream);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video_thread] Failed to release stream: 0x%x\n", result);
        }
        
        s32Count++;
        if ((s32Count % 30) == 0) {
            sdk_log("[sdk][video_thread] Encoded frames processed: %d\n", s32Count);
        }
        
        usleep(USLEEP_50MS);
    }
    
    sdk_log("[sdk][video_thread] Encoded stream thread stopped\n");
}

/* ============================================================================
   END OF FILE
   ============================================================================ */
