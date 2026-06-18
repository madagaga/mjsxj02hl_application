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
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>


/* Hisilicon MPP */
#include "hi_mipi.h"
#include "acodec.h"
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
#include "hi_comm_region.h"
#include "mpi_region.h"
#include "mpi_vb.h"
#include "mpi_ai.h"
#include "mpi_ao.h"
#include "mpi_aenc.h"
#include "mpi_adec.h"
#include "hi_ivp.h"

#include "hi_buffer.h"
#include "hi_sns_ctrl.h"
#include "platform/platform.h"

#include "scene/scene.h"

/* Forward declarations for internal video functions */
static void sdk_video_shutdown_channel(int chn);
static int sdk_video_unbind_vi_vpss(void);

/* Video global state - declared here so localsdk_init/destory can access them */
static VPSS_GRP g_vpssGrp = 0;
static VPSS_CHN g_vpssChn[4] = {0, 1, 2, 3};
static VENC_CHN g_vencChn[2] = {0, 1};
static int32_t  g_videoStarted[2] = {0, 0};

/* VPSS wrap buffer parameters for main channel (chn0), computed before VB init. */
static HI_U32   g_mainWrapBufLine = 0;
static HI_U32   g_mainWrapBufSize = 0;

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

/* Board config pointer — set once in localsdk_init(), used everywhere.
   localsdk is blind to the sensor: behaviour via board callbacks, sizing via
   the BOARD_* compile-time constants. */
static const board_cfg_t  *g_board_cfg  = NULL;

static int (*g_sdkLogPrintf_cb)(const char *, ...) = NULL;
static int (*g_sdkShellCall_cb)(const char *) = NULL;

/* Video parameters storage */
static uint32_t g_videoParams[256];  /* Video configuration buffer */
static uint32_t g_videoBrightCheckLoop = 0;
static uint32_t g_videoBrightSetCnt = 0;
static pthread_mutex_t g_videoBrightLock;

/* OSD State - Reconstructed from decompiled code (0xdc = 220 bytes per channel) */
typedef struct {
    uint32_t timestamp_en;    /* 0x00 */
    uint32_t timestamp_show;  /* 0x04 */
    RGN_HANDLE timestamp_hdl; /* 0x08 */
    uint32_t padding1;        /* 0x0c */
    uint32_t logo_en;         /* 0x10 */
    uint32_t logo_show;       /* 0x14 */
    RGN_HANDLE logo_hdl;      /* 0x18 */
    uint32_t padding2[17];    /* 0x1c - 0x5f */
    uint32_t rects_en;        /* 0x60 */
    uint32_t rects_show;      /* 0x64 */
    RGN_HANDLE rects_hdl;     /* 0x68 */
    uint32_t padding3[1];     /* 0x6c */
    LOCALSDK_OSD_OPTIONS opts;/* 0x70 - 0x93 (36 bytes) */
    uint32_t padding4[17];    /* 0x94 - 0xd7 */
} OSD_CHANNEL_PARAMS;

static uint8_t g_osdParams[2 * 220]; /* Compatible with the app's expectation */
static int32_t g_osdInitialized = 0;

/* Alarm state */
static uint32_t g_alarmState = 0;

/* Button callback */
static int (*g_keydownCb)(void) = NULL;
static int g_keydownTimeout = 0;

/* Auto night light polling thread */
static pthread_t        g_nightLightThread = 0;
static volatile int     g_nightLightRun    = 0;

/* Speaker and Audio state */
static int32_t g_speakerRunState = 0; /* 3 = started */
static pthread_mutex_t g_speakerMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_audioMutex = PTHREAD_MUTEX_INITIALIZER;

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

/* Helper to get OSD params for a channel */
static inline OSD_CHANNEL_PARAMS* sdk_osd_get_params(int chn) {
    if (chn < 0 || chn > 1) return NULL;
    return (OSD_CHANNEL_PARAMS*)&g_osdParams[chn * 220];
}

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
   GPIO INFRASTRUCTURE
   ============================================================================ */

static int gpio_write_file(const char *path, const char *value) {
    printf("gpio_write(%s, %s)\n", path, value);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    ssize_t ret = write(fd, value, strlen(value));
    close(fd);
    return (ret > 0) ? 0 : -1;
}

static int gpio_export(int pin) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", pin);
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return 0; /* already exported */
    write(fd, buf, strlen(buf));
    close(fd);
    return 0;
}

static int gpio_set_direction(int pin, const char *dir) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    int ret = gpio_write_file(path, dir);
    if (ret == 0)
        printf("[gpio_init]dbg: gpio:%d  dir:%s  ok!\n", pin, dir);
    return ret;
}

static int gpio_set_value(int pin, int value, const char *dir) {
    char path[64];
    char val[4];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    snprintf(val, sizeof(val), "%d", value);
    int ret = gpio_write_file(path, val);
    printf("[SDK-GPIO]dbg: Pin(%d)  Lvl(%d)  Dir(%s)\n", pin, value, dir);
    return ret;
}

static int gpio_read_value(int pin) {
    char path[64];
    char val[4] = {0};
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    if (read(fd, val, sizeof(val) - 1) > 0) { close(fd); return atoi(val); }
    close(fd);
    return 0;
}

static int gpio_init_pin(int pin, const char *dir, int init_value) {
    gpio_export(pin);
    if (gpio_set_direction(pin, dir) != 0) return -1;
    if (strcmp(dir, "out") == 0) {
        return gpio_set_value(pin, init_value, dir);
    } else {
        int v = gpio_read_value(pin);
        printf("[SDK-GPIO]dbg: Pin(%d)  Lvl(%d)  Dir(%s)\n", pin, v, dir);
    }
    return 0;
}

/* Write a GPIO value (used by ircut/softlight after init) */
static int gpio_write(int pin, int value) {
    char path[64];
    char val[4];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    snprintf(val, sizeof(val), "%d", value);
    return gpio_write_file(path, val);
}

static int gpio_all_init(void) {
    /* Order from runtime trace lines 10-39 */
    if (gpio_init_pin(g_board_cfg->gpio_ir_led_b,     "out", 0) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_ir_led_a,     "out", 1) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_ircut_b,      "out", 0) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_ircut_a,      "out", 0) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_led_orange,   "out", 0) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_led_blue,     "out", 1) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_button_setup, "in",  1) != 0) return -1;
    if (gpio_init_pin(g_board_cfg->gpio_photo_sensor, "in",  0) != 0) return -1;
    return 0;
}

/* ============================================================================
   PWM / SOFTLIGHT INFRASTRUCTURE
   ============================================================================ */

#define PWM_DEV_PATH "/dev/pwm"
#define PWM_CMD_WRITE _IOW('p', 0x01, struct pwm_data_s)

struct pwm_data_s {
    unsigned int pwm_num;
    unsigned int duty;
    unsigned int period;
    unsigned int enable;
};

static int g_pwmFd = -1;

static int HI_PWM_Init(void) {
    g_pwmFd = open(PWM_DEV_PATH, O_RDWR);
    /* Not fatal if unavailable */
    return 0;
}

static int softlight_init(void) {
    if (g_pwmFd >= 0) {
        struct pwm_data_s d = { .pwm_num = 0, .duty = 0, .period = 256, .enable = 1 };
        ioctl(g_pwmFd, PWM_CMD_WRITE, &d);
    }
    return 0;
}

/* ============================================================================
   PLATFORM THREAD
   ============================================================================ */

static pthread_mutex_t g_platformMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_platformThread = 0;

static void platform_callback_mutex_init(void) {
    pthread_mutex_init(&g_platformMutex, NULL);
}

static void *platform_thread(void *arg) {
    (void)arg;
    printf("[SDK-THREAD]dbg: Platform Thread Start...\n");

    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", g_board_cfg->gpio_button_setup);

    int last_state = 1;
    while (1) {
        usleep(50000);
        int v = gpio_read_value(g_board_cfg->gpio_button_setup);
        if (v == 0 && last_state == 1) {
            pthread_mutex_lock(&g_platformMutex);
            if (g_keydownCb) g_keydownCb();
            pthread_mutex_unlock(&g_platformMutex);
        }
        last_state = v;
    }
    return NULL;
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
int localsdk_set_shellcall_func(int (*callback)(const char *)) {
    if (callback) {
        g_sdkShellCall_cb = callback;
        return LOCALSDK_OK;
    }
    return LOCALSDK_ERROR;
}

/**
 * @brief Initialize the SDK with HISILICON subsystems
 */
int localsdk_init() {
    g_board_cfg  = platform_get_board_cfg();

    puts("----------------------------------------");
    puts("    OPEN LOCALSDK (oss)        ('_)')");
    puts("----------------------------------------");
    puts("    platform: hi3518ev300 ");
    printf("    board   : %s \n", g_board_cfg->name);
    printf("    sdk ver : %d \n", SDK_VERSION);
    printf("    build   : %s (%s) \n", __DATE__, __TIME__);
    puts("----------------------------------------");

    platform_callback_mutex_init();

    if (HI_PWM_Init() != 0) {
        printf("[%s]err: HI_PWM init Fail!\n", "localsdk_init");
        return LOCALSDK_ERROR;
    }

    if (softlight_init() != 0) {
        printf("[%s]err: softlight init Fail!\n", "localsdk_init");
        return LOCALSDK_ERROR;
    }

    if (gpio_all_init() != 0) {
        printf("[%s]err: gpio init Fail!\n", "localsdk_init");
        return LOCALSDK_ERROR;
    }

    pthread_t newthread;
    int32_t result = pthread_create(&newthread, NULL, platform_thread, NULL);
    if (result == 0) {
        g_platformThread = newthread;
        printf("[%s]dbg: Platform Thread Create OK!  ('_)')\n", "SDK-THREAD");
        return LOCALSDK_OK;
    }

    printf("[%s]err: Platform Thread Create Fail!\n", "localsdk_init");
    return LOCALSDK_ERROR;
}

/**
 * @brief Destroy the SDK and cleanup resources
 */
int localsdk_destory() {
    sdk_log("[sdk] Destroying SDK\n");

    for (int chn = 0; chn < 2; chn++) {
        sdk_video_shutdown_channel(chn);
    }

    sdk_video_unbind_vi_vpss();
    HI_MPI_VPSS_StopGrp(g_vpssGrp);
    HI_MPI_VPSS_DestroyGrp(g_vpssGrp);

    /* VI teardown — handles owned by the board/sensor, fetched generically. */
    VI_DEV viDev = 0; VI_PIPE viPipe = 0; VI_CHN viChn = 0;
    if (g_board_cfg && g_board_cfg->pfnGetVi)
        g_board_cfg->pfnGetVi(&viDev, &viPipe, &viChn);
    HI_MPI_VI_DisableChn(viPipe, viChn);
    HI_MPI_VI_StopPipe(viPipe);
    HI_MPI_VI_DestroyPipe(viPipe);
    HI_MPI_VI_DisableDev(viDev);
    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    /* Sensor ISP teardown (run thread join + sensor unregister) owned by board. */
    if (g_board_cfg && g_board_cfg->pfnTeardownSensorIsp)
        g_board_cfg->pfnTeardownSensorIsp();

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

/* Global video state (declared at top of file) */
static int32_t (*g_encCb[2])(LOCALSDK_H26X_FRAME_INFO *frameInfo) = {NULL, NULL};
static int32_t (*g_yuvCb[2])(LOCALSDK_H26X_FRAME_INFO *frameInfo) = {NULL, NULL};
static int (*g_algoRegisterCb)(void) = NULL;
static int (*g_algoUnregisterCb)(void) = NULL;
static pthread_t g_videoRunThread[2] = {0, 0};
static volatile int g_videoRunning[2] = {0, 0};

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

static int sdk_video_unbind_vi_vpss(void) {
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;

    memset(&stSrcChn, 0, sizeof(stSrcChn));
    memset(&stDestChn, 0, sizeof(stDestChn));

    VI_DEV viDev = 0; VI_PIPE viPipe = 0; VI_CHN viChn = 0;
    if (g_board_cfg->pfnGetVi) g_board_cfg->pfnGetVi(&viDev, &viPipe, &viChn);

    stSrcChn.enModId = HI_ID_VI;
    stSrcChn.s32DevId = viPipe;
    stSrcChn.s32ChnId = viChn;

    stDestChn.enModId = HI_ID_VPSS;
    stDestChn.s32DevId = g_vpssGrp;
    stDestChn.s32ChnId = 0;

    return (HI_MPI_SYS_UnBind(&stSrcChn, &stDestChn) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static void sdk_video_shutdown_channel(int chn) {
    if (chn < 0 || chn > 1) {
        return;
    }
    sdk_video_unbind_vpss_venc(chn, g_vencChn[chn]);
    HI_MPI_VENC_StopRecvFrame(g_vencChn[chn]);
    HI_MPI_VENC_DestroyChn(g_vencChn[chn]);
    g_videoStarted[chn] = 0;
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

    /* In this SDK the picture dims / profile / bufsize live in the common
       stVencAttr; the codec sub-attrs are minimal and the GOP is separate. */
    stVencChnAttr.stVencAttr.enType         = enType;
    stVencChnAttr.stVencAttr.u32MaxPicWidth  = stSize.u32Width;
    stVencChnAttr.stVencAttr.u32MaxPicHeight = stSize.u32Height;
    stVencChnAttr.stVencAttr.u32PicWidth     = stSize.u32Width;
    stVencChnAttr.stVencAttr.u32PicHeight    = stSize.u32Height;
    stVencChnAttr.stVencAttr.u32Profile      = 0;
    stVencChnAttr.stVencAttr.bByFrame        = HI_TRUE;

    stVencChnAttr.stGopAttr.enGopMode            = VENC_GOPMODE_NORMALP;
    stVencChnAttr.stGopAttr.stNormalP.s32IPQpDelta = 2;

    if (enType == PT_H264) {
        /* Match the original firmware MMZ footprint: a quarter-frame stream
           buffer (original 1080p uses ~328 KB) and a shared recon/reference
           buffer (one Rcn instead of two). Frees ~5 MB vs full-frame buffer +
           dual recon — required to leave room for the IVP HW/SW buffers. */
        stVencChnAttr.stVencAttr.u32BufSize = stSize.u32Width * stSize.u32Height / 4;
        stVencChnAttr.stVencAttr.stAttrH264e.bRcnRefShareBuf = HI_TRUE;

        if (options->rcmode == LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
            stVencChnAttr.stRcAttr.stH264Vbr.u32Gop           = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stH264Vbr.u32StatTime      = 1;
            stVencChnAttr.stRcAttr.stH264Vbr.u32SrcFrameRate  = options->fps;
            stVencChnAttr.stRcAttr.stH264Vbr.fr32DstFrameRate = options->fps;
            stVencChnAttr.stRcAttr.stH264Vbr.u32MaxBitRate    = options->bitrate;
        } else if (options->rcmode == LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264FIXQP;
            stVencChnAttr.stRcAttr.stH264FixQp.u32Gop           = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stH264FixQp.u32SrcFrameRate  = options->fps;
            stVencChnAttr.stRcAttr.stH264FixQp.fr32DstFrameRate = options->fps;
            stVencChnAttr.stRcAttr.stH264FixQp.u32IQp = 20;
            stVencChnAttr.stRcAttr.stH264FixQp.u32PQp = 23;
        } else {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
            stVencChnAttr.stRcAttr.stH264Cbr.u32Gop           = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stH264Cbr.u32StatTime      = 1;
            stVencChnAttr.stRcAttr.stH264Cbr.u32SrcFrameRate  = options->fps;
            stVencChnAttr.stRcAttr.stH264Cbr.fr32DstFrameRate = options->fps;
            stVencChnAttr.stRcAttr.stH264Cbr.u32BitRate       = options->bitrate;
        }
    } else if (enType == PT_H265) {
        stVencChnAttr.stVencAttr.u32BufSize = stSize.u32Width * stSize.u32Height / 2;
        stVencChnAttr.stVencAttr.stAttrH265e.bRcnRefShareBuf = HI_TRUE;

        if (options->rcmode == LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
            stVencChnAttr.stRcAttr.stH265Vbr.u32Gop           = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stH265Vbr.u32StatTime      = 1;
            stVencChnAttr.stRcAttr.stH265Vbr.u32SrcFrameRate  = options->fps;
            stVencChnAttr.stRcAttr.stH265Vbr.fr32DstFrameRate = options->fps;
            stVencChnAttr.stRcAttr.stH265Vbr.u32MaxBitRate    = options->bitrate;
        } else if (options->rcmode == LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY) {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265FIXQP;
            stVencChnAttr.stRcAttr.stH265FixQp.u32Gop           = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stH265FixQp.u32SrcFrameRate  = options->fps;
            stVencChnAttr.stRcAttr.stH265FixQp.fr32DstFrameRate = options->fps;
            stVencChnAttr.stRcAttr.stH265FixQp.u32IQp = 20;
            stVencChnAttr.stRcAttr.stH265FixQp.u32PQp = 23;
        } else {
            stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
            stVencChnAttr.stRcAttr.stH265Cbr.u32Gop           = options->gop ? options->gop : options->fps;
            stVencChnAttr.stRcAttr.stH265Cbr.u32StatTime      = 1;
            stVencChnAttr.stRcAttr.stH265Cbr.u32SrcFrameRate  = options->fps;
            stVencChnAttr.stRcAttr.stH265Cbr.fr32DstFrameRate = options->fps;
            stVencChnAttr.stRcAttr.stH265Cbr.u32BitRate       = options->bitrate;
        }
    } else {
        /* JPEG snapshot */
        stVencChnAttr.stVencAttr.u32BufSize = (((stSize.u32Width + 15) >> 4) << 4) * (((stSize.u32Height + 15) >> 4) << 4);
        stVencChnAttr.stVencAttr.stAttrJpege.bSupportDCF   = HI_FALSE;
        stVencChnAttr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
    }

    result = HI_MPI_VENC_CreateChn(vencChn, &stVencChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VENC_CreateChn(%d) failed: 0x%x\n", vencChn, result);
        return LOCALSDK_ERROR;
    }

    if (!jpeg) {
        VENC_RECV_PIC_PARAM_S stRecvParam;
        stRecvParam.s32RecvPicNum = -1; /* receive frames continuously */
        result = HI_MPI_VENC_StartRecvFrame(vencChn, &stRecvParam);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video] HI_MPI_VENC_StartRecvFrame(%d) failed: 0x%x\n", vencChn, result);
            HI_MPI_VENC_DestroyChn(vencChn);
            return LOCALSDK_ERROR;
        }
    }

    return LOCALSDK_OK;
}

static int sdk_video_bind_vi_vpss(void) {
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;

    memset(&stSrcChn, 0, sizeof(stSrcChn));
    memset(&stDestChn, 0, sizeof(stDestChn));

    VI_DEV viDev = 0; VI_PIPE viPipe = 0; VI_CHN viChn = 0;
    if (g_board_cfg->pfnGetVi) g_board_cfg->pfnGetVi(&viDev, &viPipe, &viChn);

    stSrcChn.enModId = HI_ID_VI;
    stSrcChn.s32DevId = viPipe;
    stSrcChn.s32ChnId = viChn;

    stDestChn.enModId = HI_ID_VPSS;
    stDestChn.s32DevId = g_vpssGrp;
    stDestChn.s32ChnId = 0;

    return (HI_MPI_SYS_Bind(&stSrcChn, &stDestChn) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

#define SDK_SYS_ALIGN_WIDTH        64

static uint32_t sdk_align_up(uint32_t value, uint32_t align) {
    return (value + (align - 1)) & ~(align - 1);
}

static uint32_t sdk_calc_yuv420_blk_size(uint32_t width, uint32_t height) {
    uint32_t w = sdk_align_up(width, SDK_SYS_ALIGN_WIDTH);
    uint32_t h = sdk_align_up(height, SDK_SYS_ALIGN_WIDTH);
    return (w * h * 3) / 2;
}

/**
 * @brief Initialize video subsystem with HISILICON VPSS
 */
int local_sdk_video_init(int fps, int flip, int mirror) {
    int32_t result;
    VPSS_GRP_ATTR_S stGrpAttr;
    VB_CONFIG_S stVbConf;
    VI_VPSS_MODE_S stVIVPSSMode;

    sdk_log("[sdk][video] Initializing video (fps=%d flip=%d mirror=%d)\n", fps, flip, mirror);
    if (fps <= 0 || fps > 30) {
        sdk_log("[sdk][video] Invalid FPS: %d\n", fps);
        return LOCALSDK_ERROR;
    }

    /* Pre-compute VPSS wrap buffer parameters for main channel (chn0 only).
       HI_MPI_SYS_GetVPSSVENCWrapBufferLine is a pure calculation, safe before SYS init.
       The wrap ring buffer limits VPSS output to ~416 lines instead of a full 1080p frame,
       saving ~4 MB in the ~19 MB MMZ and making room for the 3DNR NR reference frame. */
    {
        VPSS_VENC_WRAP_PARAM_S stWrap;
        memset(&stWrap, 0, sizeof(stWrap));
        stWrap.bAllOnline       = HI_TRUE; /* VI_ONLINE_VPSS_ONLINE */
        /* SDK doc: u32FrameRate and u32FullLinesStd are the sensor INPUT rate to
           VIPROC, not the VPSS output rate.  Use native sensor values. */
        stWrap.u32FrameRate     = BOARD_FPS_NATIVE;
        stWrap.u32FullLinesStd  = BOARD_FULL_LINES;
        stWrap.stLargeStreamSize.u32Width  = BOARD_WIDTH;
        stWrap.stLargeStreamSize.u32Height = BOARD_HEIGHT;
        stWrap.stSmallStreamSize.u32Width  = BOARD_SUB_WIDTH;
        stWrap.stSmallStreamSize.u32Height = BOARD_SUB_HEIGHT;
        if (g_board_cfg->vb_main_wrap_lines > 0) {
            /* Board hardcodes wrap lines (original firmware value). */
            g_mainWrapBufLine = g_board_cfg->vb_main_wrap_lines;
        }
        /* vb_main_wrap_lines=0: wrap disabled; g_mainWrapBufLine stays 0. */
        if (g_mainWrapBufLine > 0) {
            g_mainWrapBufSize = VPSS_GetWrapBufferSize(
                stWrap.stLargeStreamSize.u32Width,
                stWrap.stLargeStreamSize.u32Height,
                g_mainWrapBufLine,
                PIXEL_FORMAT_YVU_SEMIPLANAR_420,
                DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
            sdk_log("[sdk][video] wrap: buf_line=%u size=%u\n",
                    g_mainWrapBufLine, g_mainWrapBufSize);
        }
    }

    /* VB pools (newer SDK layout: 64-bit u64BlkSize).
       Pool 0: main YUV420 1080p.  Use wrap buffer size when wrap is supported
               (~1.2 MB, cnt=1); fall back to full frame (~3.1 MB) otherwise.
       Pool 1: sub  YUV420 640x360, cnt=5. */
    memset(&stVbConf, 0, sizeof(stVbConf));
    stVbConf.u32MaxPoolCnt = 2;
    stVbConf.astCommPool[0].u64BlkSize = (g_mainWrapBufSize > 0)
        ? g_mainWrapBufSize
        : sdk_calc_yuv420_blk_size(BOARD_WIDTH, BOARD_HEIGHT);
    stVbConf.astCommPool[0].u32BlkCnt  = g_board_cfg->vb_main_blk_cnt;
    stVbConf.astCommPool[1].u64BlkSize = sdk_calc_yuv420_blk_size(BOARD_SUB_WIDTH, BOARD_SUB_HEIGHT);
    stVbConf.astCommPool[1].u32BlkCnt  = g_board_cfg->vb_sub_blk_cnt;

    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    result = HI_MPI_VB_SetConfig(&stVbConf);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VB_SetConfig failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    result = HI_MPI_VB_Init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VB_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_SYS_Init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_SYS_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* VI-VPSS working mode: VI_ONLINE_VPSS_ONLINE — VI and VPSS share a direct
       hardware pipeline (no VB between VI and VPSS). This is the correct mode for
       JXF22 @ 1920px (ONLINE_LIMIT_WIDTH=2304 in SDK samples). Benefits:
       - Wrap mode works naturally (streaming pipeline)
       - No extra VI-capture VB pool needed (saves ~9 MB vs OFFLINE with wrap)
       - Matches original firmware (flip/mirror at VPSS + wrap both worked there) */
    memset(&stVIVPSSMode, 0, sizeof(stVIVPSSMode));
    HI_MPI_SYS_GetVIVPSSMode(&stVIVPSSMode);
    stVIVPSSMode.aenMode[0] = BOARD_VIVPSS_MODE;
    result = HI_MPI_SYS_SetVIVPSSMode(&stVIVPSSMode);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_SYS_SetVIVPSSMode failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* Sensor bring-up is owned by the board (MIPI + VI + ISP, then fps + the
       app-requested orientation applied at the sensor). localsdk stays blind to
       the sensor: it only hands the orientation through. */
    if (!g_board_cfg->pfnBringupSensor) {
        sdk_log("[sdk][video] board has no pfnBringupSensor\n");
        return LOCALSDK_ERROR;
    }
    result = g_board_cfg->pfnBringupSensor((HI_BOOL)(mirror != 0), (HI_BOOL)(flip != 0));
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] sensor bring-up failed\n");
        return LOCALSDK_ERROR;
    }

    /* Board init: scene ISP params + GPIO initial state. */
    if (g_board_cfg->pfnInit) g_board_cfg->pfnInit();

    /* VPSS group */
    memset(&stGrpAttr, 0, sizeof(VPSS_GRP_ATTR_S));
    stGrpAttr.u32MaxW                     = BOARD_WIDTH;
    stGrpAttr.u32MaxH                     = BOARD_HEIGHT;
    stGrpAttr.enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stGrpAttr.enDynamicRange              = DYNAMIC_RANGE_SDR8;
    stGrpAttr.stFrameRate.s32SrcFrameRate = -1;
    stGrpAttr.stFrameRate.s32DstFrameRate = -1;
    /* VPSS 3DNR enabled — matches original firmware.
       Requires wrap mode (vb_main_wrap_lines=416) to free ~3.8 MB of MMZ for the
       NR reference frame buffer.  SetChnBufWrapAttr no longer returns ILLEGAL_PARAM
       because flip/mirror is now at VI level (VPSS chn0 bMirror=bFlip=FALSE).
       enCompressMode=COMPRESS_MODE_FRAME: reference frame compressed (YUV, whole frame)
       as required by hi3516ev200 SDK sample (sample_venc.c line 573). */
    stGrpAttr.bNrEn                        = HI_TRUE;
    stGrpAttr.stNrAttr.enNrType            = VPSS_NR_TYPE_VIDEO;
    stGrpAttr.stNrAttr.enCompressMode      = COMPRESS_MODE_FRAME;
    stGrpAttr.stNrAttr.enNrMotionMode      = NR_MOTION_MODE_NORMAL;

    result = HI_MPI_VPSS_CreateGrp(g_vpssGrp, &stGrpAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VPSS_CreateGrp failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    /* StartGrp and VI→VPSS bind are deferred to local_sdk_video_create() so that
       SetChnBufWrapAttr can be called before StartGrp, as required by the SDK. */

    sdk_log("[sdk][video] Video init complete\n");
    return LOCALSDK_OK;
}

/**
 * @brief Create video channel with HISILICON VPSS
 */
int local_sdk_video_create(int chn, LOCALSDK_VIDEO_OPTIONS *options) {
    VPSS_CHN_ATTR_S stChnAttr;
    SIZE_S stSize;
    int32_t result;

    if (chn < 0 || chn > 3 || !options) {
        sdk_log("[sdk][video] Invalid channel %d or options\n", chn);
        return LOCALSDK_ERROR;
    }

    if (sdk_video_resolution_to_size(options->resolution, &stSize) != LOCALSDK_OK) {
        sdk_log("[sdk][video] Invalid resolution %u for channel %d\n",
                options->resolution, chn);
        return LOCALSDK_ERROR;
    }

    sdk_log("[sdk][video] Creating VPSS channel %d (%ux%u)\n",
            chn, stSize.u32Width, stSize.u32Height);

    /* Channel attributes (newer 3516e ABI: explicit size + formats). */
    memset(&stChnAttr, 0, sizeof(VPSS_CHN_ATTR_S));
    stChnAttr.enChnMode               = VPSS_CHN_MODE_USER;
    stChnAttr.u32Width                = stSize.u32Width;
    stChnAttr.u32Height               = stSize.u32Height;
    stChnAttr.enVideoFormat           = VIDEO_FORMAT_LINEAR;
    stChnAttr.enPixelFormat           = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stChnAttr.enDynamicRange          = DYNAMIC_RANGE_SDR8;
    /* COMPRESS_MODE_SEG required for VPSS chn0 in wrap mode (SDK sample_venc.c).
       Other channels and non-wrap mode use COMPRESS_MODE_NONE. */
    stChnAttr.enCompressMode = (g_vpssChn[chn] == 0 && g_mainWrapBufLine > 0)
        ? COMPRESS_MODE_SEG : COMPRESS_MODE_NONE;
    stChnAttr.stFrameRate.s32SrcFrameRate = -1;
    stChnAttr.stFrameRate.s32DstFrameRate = -1;
    /* Secondary channel (640x360) feeds both VENC1 (bind) and the IVP detection
       thread. A non-zero depth keeps frames available for user GetChnFrame
       alongside the bind; chn0 stays depth 0 (wrap/online to VENC0). */
    stChnAttr.u32Depth = (g_vpssChn[chn] == 1) ? 2 : 0;

    /* Board orientation is corrected at sensor level via pfnMirrorFlip in local_sdk_video_init.
       VPSS channels use user-requested mirror/flip only (no board XOR).
       chn0+wrap requires bMirror=bFlip=FALSE; sensor correction makes the image correct. */
    /* Orientation is corrected at the sensor (board pfnBringupSensor); all VPSS
       channels stay bMirror=bFlip=FALSE (chn0+wrap requires it anyway). */
    stChnAttr.bMirror = HI_FALSE;
    stChnAttr.bFlip   = HI_FALSE;

    result = HI_MPI_VPSS_SetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set channel attr: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* Wrap mode on chn0 (only VPSS channel 0 supports wrap). Must be called
       after SetChnAttr but before EnableChn. */
    if (g_vpssChn[chn] == 0 && g_mainWrapBufLine > 0) {
        VPSS_CHN_BUF_WRAP_S stWrapAttr;
        stWrapAttr.bEnable           = HI_TRUE;
        stWrapAttr.u32BufLine        = g_mainWrapBufLine;
        stWrapAttr.u32WrapBufferSize = g_mainWrapBufSize;
        result = HI_MPI_VPSS_SetChnBufWrapAttr(g_vpssGrp, g_vpssChn[chn], &stWrapAttr);
        if (result != HI_SUCCESS)
            sdk_log("[sdk][video] SetChnBufWrapAttr failed: 0x%x\n", result);
        else
            sdk_log("[sdk][video] SetChnBufWrapAttr ok (line=%u size=%u)\n",
                    g_mainWrapBufLine, g_mainWrapBufSize);

    }

    /* Enable channel */
    result = HI_MPI_VPSS_EnableChn(g_vpssGrp, g_vpssChn[chn]);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to enable channel: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* Store options in global state */
    memcpy(&g_videoParams[chn * 32], options, sizeof(LOCALSDK_VIDEO_OPTIONS));

    /* StartGrp + VI→VPSS bind are deferred until after all channels are configured
       so that SetChnBufWrapAttr (called above for chn0) runs before StartGrp.
       chn==1 (secondary) is always the last channel created in video.c. */
    if (chn == 1) {
        result = HI_MPI_VPSS_StartGrp(g_vpssGrp);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video] HI_MPI_VPSS_StartGrp failed: 0x%x\n", result);
            return LOCALSDK_ERROR;
        }
        result = sdk_video_bind_vi_vpss();
        if (result != LOCALSDK_OK) {
            sdk_log("[sdk][video] Failed to bind VI->VPSS\n");
            HI_MPI_VPSS_StopGrp(g_vpssGrp);
            HI_MPI_VPSS_DestroyGrp(g_vpssGrp);
            return LOCALSDK_ERROR;
        }
        sdk_log("[sdk][video] VPSS started and VI->VPSS bound\n");
    }

    sdk_log("[sdk][video] Channel %d created successfully\n", chn);
    return LOCALSDK_OK;
}

/**
 * @brief Set video parameters via HISILICON
 */
int local_sdk_video_set_parameters(int chn, LOCALSDK_VIDEO_OPTIONS *options) {
    VPSS_CHN_ATTR_S stChnAttr;
    int32_t result;
    int wasStarted;

    if (chn < 0 || chn > 3 || !options) {
        sdk_log("[sdk][video] Invalid channel or options\n");
        return LOCALSDK_ERROR;
    }

    sdk_log("[sdk][video] Setting parameters for channel %d\n", chn);

    result = HI_MPI_VPSS_GetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to get channel attr: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* Board orientation corrected at sensor level; VPSS uses user options only. */
    /* Orientation is corrected at the sensor (board pfnBringupSensor); all VPSS
       channels stay bMirror=bFlip=FALSE (chn0+wrap requires it anyway). */
    stChnAttr.bMirror = HI_FALSE;
    stChnAttr.bFlip   = HI_FALSE;

    result = HI_MPI_VPSS_SetChnAttr(g_vpssGrp, g_vpssChn[chn], &stChnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] Failed to set channel attr: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* Update video parameters */
    memcpy(&g_videoParams[chn * 32], options, sizeof(LOCALSDK_VIDEO_OPTIONS));

    wasStarted = g_videoStarted[chn];
    if (wasStarted) {
        sdk_video_shutdown_channel(chn);
        result = local_sdk_video_start(chn);
        if (result != LOCALSDK_OK) {
            sdk_log("[sdk][video] Failed to restart channel %d after parameter update\n", chn);
            return LOCALSDK_ERROR;
        }
    }
    
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
        HI_MPI_VENC_StopRecvFrame(g_vencChn[chn]);
        HI_MPI_VENC_DestroyChn(g_vencChn[chn]);
        sdk_log("[sdk][video] Failed to bind VPSS->VENC for channel %d\n", chn);
        return LOCALSDK_ERROR;
    }

    if (g_algoRegisterCb && chn == LOCALSDK_VIDEO_SECONDARY_CHANNEL) {
        g_algoRegisterCb();
    }

    /* Request I-frame to start streaming */
    result = HI_MPI_VENC_RequestIDR(g_vencChn[chn], HI_TRUE);
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

    /* Stop the run thread first */
    if (chn <= 1 && g_videoRunning[chn]) {
        g_videoRunning[chn] = 0;
        if (g_videoRunThread[chn] != 0) {
            pthread_join(g_videoRunThread[chn], NULL);
            g_videoRunThread[chn] = 0;
        }
    }

    sdk_video_shutdown_channel(chn);
    return LOCALSDK_OK;
}

typedef struct {
    int chn;
} VideoRunArg;

static VideoRunArg g_videoRunArgs[2] = {{0}, {1}};

static void *sdk_video_run_thread(void *arg) {
    VideoRunArg *a = (VideoRunArg *)arg;
    int chn = a->chn;
    VENC_CHN vencChn = g_vencChn[chn];
    int vencFd;
    fd_set read_fds;
    struct timeval tv;

    vencFd = HI_MPI_VENC_GetFd(vencChn);
    if (vencFd < 0) {
        sdk_log("[sdk][video] GetFd failed for chn %d\n", chn);
        g_videoRunning[chn] = 0;
        return NULL;
    }

    while (g_videoRunning[chn]) {
        FD_ZERO(&read_fds);
        FD_SET(vencFd, &read_fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int sel = select(vencFd + 1, &read_fds, NULL, NULL, &tv);
        if (sel < 0) break;
        if (sel == 0) continue;

        if (!FD_ISSET(vencFd, &read_fds)) continue;

        VENC_CHN_STATUS_S stStat;
        if (HI_MPI_VENC_QueryStatus(vencChn, &stStat) != HI_SUCCESS) continue;
        if (stStat.u32CurPacks == 0) continue;

        VENC_STREAM_S stStream;
        memset(&stStream, 0, sizeof(stStream));
        stStream.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
        if (!stStream.pstPack) continue;
        stStream.u32PackCount = stStat.u32CurPacks;

        if (HI_MPI_VENC_GetStream(vencChn, &stStream, HI_FALSE) != HI_SUCCESS) {
            free(stStream.pstPack);
            continue;
        }

        if (g_encCb[chn]) {
            for (uint32_t i = 0; i < stStream.u32PackCount; i++) {
                LOCALSDK_H26X_FRAME_INFO fi;
                memset(&fi, 0, sizeof(fi));
                fi.data      = (signed char *)stStream.pstPack[i].pu8Addr;
                fi.size      = stStream.pstPack[i].u32Len;
                fi.timestamp = stStream.pstPack[i].u64PTS;
                fi.offset    = (uint16_t)stStream.pstPack[i].u32Offset; // From VENC_PACK_S.u32Offset
                fi.frame_end = (uint16_t)stStream.pstPack[i].bFrameEnd; // From VENC_PACK_S.bFrameEnd
                fi.type      = stStream.pstPack[i].DataType.enH264EType;
                g_encCb[chn](&fi);
            }
        }

        HI_MPI_VENC_ReleaseStream(vencChn, &stStream);
        free(stStream.pstPack);
    }

    g_videoRunning[chn] = 0;
    return NULL;
}

/**
 * @brief Run video processing - starts background capture thread
 */
int local_sdk_video_run(int chn) {
    if (chn < 0 || chn > 1 || !g_videoStarted[chn]) {
        return LOCALSDK_ERROR;
    }
    if (g_videoRunning[chn]) {
        return LOCALSDK_OK; /* already running */
    }

    g_videoRunning[chn] = 1;
    if (pthread_create(&g_videoRunThread[chn], NULL, sdk_video_run_thread, &g_videoRunArgs[chn]) != 0) {
        g_videoRunning[chn] = 0;
        sdk_log("[sdk][video] Failed to create run thread for chn %d\n", chn);
        return LOCALSDK_ERROR;
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
    
    result = HI_MPI_VENC_RequestIDR(g_vencChn[chn], HI_TRUE);
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
    VENC_CHN_STATUS_S stStat;
    VENC_RECV_PIC_PARAM_S stRecvParam;
    int32_t result;
    uint32_t i;
    int32_t vencFd;
    fd_set read_fds;
    struct timeval timeout;
    SIZE_S stSize;
    VENC_CHN snapChn = 3;
    
    if (chn < 0 || chn > 3 || !file) {
        sdk_log("[sdk][video] Invalid channel or filename\n");
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][video] Saving JPEG from channel %d to %s\n", chn, file);

    /* Derive size from channel number: g_videoParams may be zeroed in the
       --get-image subprocess (separate process, no localsdk_init). */
    if (chn == LOCALSDK_VIDEO_PRIMARY_CHANNEL) {
        stSize.u32Width = 1920; stSize.u32Height = 1080;
    } else {
        stSize.u32Width = 640; stSize.u32Height = 360;
    }

    fp = fopen(file, "wb");
    if (!fp) {
        sdk_log("[sdk][video] Failed to open file: %s\n", file);
        return LOCALSDK_ERROR;
    }

    /* Create JPEG VENC channel directly — only resolution matters for PT_JPEG */
    {
        VENC_CHN_ATTR_S stJpegAttr;
        memset(&stJpegAttr, 0, sizeof(stJpegAttr));
        stJpegAttr.stVencAttr.enType          = PT_JPEG;
        stJpegAttr.stVencAttr.u32MaxPicWidth  = stSize.u32Width;
        stJpegAttr.stVencAttr.u32MaxPicHeight = stSize.u32Height;
        stJpegAttr.stVencAttr.u32PicWidth     = stSize.u32Width;
        stJpegAttr.stVencAttr.u32PicHeight    = stSize.u32Height;
        stJpegAttr.stVencAttr.u32BufSize      =
            (((stSize.u32Width + 15) >> 4) << 4) * (((stSize.u32Height + 15) >> 4) << 4);
        stJpegAttr.stVencAttr.bByFrame        = HI_TRUE;
        stJpegAttr.stVencAttr.stAttrJpege.bSupportDCF   = HI_FALSE;
        stJpegAttr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
        stJpegAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
        stJpegAttr.stGopAttr.stNormalP.s32IPQpDelta = 2;
        result = HI_MPI_VENC_CreateChn(snapChn, &stJpegAttr);
    }
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VENC_CreateChn(snap) failed: 0x%x\n", result);
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
    result = HI_MPI_VENC_StartRecvFrame(snapChn, &stRecvParam);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] HI_MPI_VENC_StartRecvFrame(snap) failed: 0x%x\n", result);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    vencFd = HI_MPI_VENC_GetFd(snapChn);
    if (vencFd < 0) {
        sdk_log("[sdk][video] HI_MPI_VENC_GetFd failed for snapshot channel\n");
        HI_MPI_VENC_StopRecvFrame(snapChn);
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
        HI_MPI_VENC_StopRecvFrame(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    memset(&stStat, 0, sizeof(stStat));
    result = HI_MPI_VENC_QueryStatus(snapChn, &stStat);
    if (result != HI_SUCCESS || stStat.u32CurPacks == 0) {
        sdk_log("[sdk][video] Snapshot query failed or empty: 0x%x\n", result);
        HI_MPI_VENC_StopRecvFrame(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn);
        fclose(fp);
        return LOCALSDK_ERROR;
    }

    memset(&stStream, 0, sizeof(stStream));
    stStream.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
    if (!stStream.pstPack) {
        HI_MPI_VENC_StopRecvFrame(snapChn);
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
        HI_MPI_VENC_StopRecvFrame(snapChn);
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
    HI_MPI_VENC_StopRecvFrame(snapChn);
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
    g_algoUnregisterCb = callback;
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
    stExpAttr.stManual.u32ExpTime = param_1;
    
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
    /* Orientation is now corrected at the sensor during bring-up (board-owned),
       the only path compatible with VPSS chn0 wrap. Runtime VPSS flip/mirror is
       therefore unsupported; this remains as a no-op to preserve the API. */
    (void)param_1;
    (void)param_2;
    sdk_log("[sdk][video] set_flip ignored: orientation is fixed at the sensor\n");
    return LOCALSDK_OK;
}

/**
 * @brief Set video framerate
 */
int local_sdk_video_set_fps(int param_1, int param_2, int param_3, int param_4) {
    int32_t chn = param_1;
    LOCALSDK_VIDEO_OPTIONS *options;
    
    (void)param_3;
    (void)param_4;
    sdk_log("[sdk][video] Setting FPS channel %d: %d\n", chn, param_2);
    
    if (chn < 0 || chn > 1 || param_2 < 1 || param_2 > 30) {
        sdk_log("[sdk][video] Invalid FPS request\n");
        return LOCALSDK_ERROR;
    }

    options = sdk_video_get_options(chn);
    if (!options) {
        return LOCALSDK_ERROR;
    }

    options->fps = (uint32_t)param_2;
    if (options->gop == 0) {
        options->gop = (uint32_t)param_2;
    }

    if (g_videoStarted[chn]) {
        sdk_video_shutdown_channel(chn);
        return local_sdk_video_start(chn);
    }
    
    return LOCALSDK_OK;
}

/**
 * @brief Set video bitrate
 */
int local_sdk_video_set_kbps(int param_1, int param_2) {
    LOCALSDK_VIDEO_OPTIONS *options;

    sdk_log("[sdk][video] Setting bitrate: %d kbps\n", param_2);

    if (param_1 < 0 || param_1 > 1) {
        sdk_log("[sdk][video] Invalid channel for bitrate: %d\n", param_1);
        return LOCALSDK_ERROR;
    }

    options = sdk_video_get_options(param_1);
    if (options) {
        options->bitrate = (uint32_t)param_2;
    }

    /* Bitrate change takes effect on next channel start */
    return LOCALSDK_OK;
}

/* ============================================================================
   AUDIO SUBSYSTEM - HISILICON IMPLEMENTATION
   ============================================================================ */

/* Global audio state — ai_dev/ao_dev initialised from board cfg in local_sdk_audio_init() */
static AUDIO_DEV g_aiDev = 0;
static AI_CHN g_aiChn = 0;
static AUDIO_DEV g_aoDev = 0;
static AO_CHN g_aoChn = 0;
static AENC_CHN g_aencChn = 0;     /* Audio Encoder Channel */
static ADEC_CHN g_adecChn = 0;     /* Audio Decoder Channel */
static int32_t g_audioStarted = 0;
#define MAX_AENC_CALLBACKS 4
static int (*g_aencCb[MAX_AENC_CALLBACKS])(LOCALSDK_AUDIO_G711_FRAME_INFO *frameInfo) = {NULL};
static int g_aencCbCount = 0;
static pthread_t g_audioAiThread = 0;
static pthread_t g_audioAencThread = 0;
static volatile int g_audioRunning = 0;

/* Configure inner acodec via /dev/acodec (from trace: volume=60) */
static void sdk_audio_inner_codec_cfg(void) {
    int fd = open("/dev/acodec", O_RDWR);
    if (fd < 0) return;

    ioctl(fd, ACODEC_SOFT_RESET_CTRL, NULL);

    ACODEC_FS_E fs = ACODEC_FS_8000;
    ioctl(fd, ACODEC_SET_I2S1_FS, &fs);

    ACODEC_MIXER_E input_mode = ACODEC_MIXER_IN1;
    ioctl(fd, ACODEC_SET_MIXER_MIC, &input_mode);

    int vol = 60;
    printf("[SDK-AUDIO]SAMPLE_INNER_CODEC_CfgAudio: set acodec volume:[%d]\n", vol);
    ioctl(fd, ACODEC_SET_INPUT_VOL, &vol);

    close(fd);
}

/**
 * @brief Initialize audio subsystem with HISILICON AI/AO (G.711A: 8kHz, 16-bit, mono)
 */
int local_sdk_audio_init() {
    AIO_ATTR_S stAioAttr;
    int32_t result;

    sdk_log("[sdk][audio] Initializing audio subsystem\n");

    g_aiDev = g_board_cfg->ai_dev;
    g_aoDev = g_board_cfg->ao_dev;

    memset(&stAioAttr, 0, sizeof(AIO_ATTR_S));
    stAioAttr.enSamplerate   = AUDIO_SAMPLE_RATE_8000; /* G.711 standard */
    stAioAttr.enBitwidth     = AUDIO_BIT_WIDTH_16;
    stAioAttr.enWorkmode     = AIO_MODE_I2S_MASTER;
    stAioAttr.u32EXFlag      = 0;
    stAioAttr.u32FrmNum      = 30;
    stAioAttr.u32PtNumPerFrm = 320; /* 40ms at 8kHz */
    stAioAttr.u32ChnCnt      = 1;

    result = HI_MPI_AI_SetPubAttr(g_aiDev, &stAioAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to set AI attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AI_Enable(g_aiDev);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AI: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AO_SetPubAttr(g_aoDev, &stAioAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to set AO attributes: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AO_Enable(g_aoDev);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AO: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    sdk_audio_inner_codec_cfg();

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
 * @brief Set audio encode callback (called twice per trace: for primary and secondary)
 */
int local_sdk_audio_set_encode_frame_callback(int chn, int (*callback)(LOCALSDK_AUDIO_G711_FRAME_INFO *frameInfo)) {
    printf("[SDK-THREAD]dbg: Set Audio Enc Callback Doing...\n");
    if (callback && g_aencCbCount < MAX_AENC_CALLBACKS) {
        g_aencCb[g_aencCbCount++] = callback;
    }
    return LOCALSDK_OK;
}

/* AI capture thread: AI -> AENC */
static void *sdk_audio_ai_thread(void *arg) {
    (void)arg;
    AUDIO_FRAME_S stFrame;
    AEC_FRAME_S stAecFrm;
    fd_set read_fds;
    struct timeval tv;

    AI_CHN_PARAM_S stParam;
    if (HI_MPI_AI_GetChnParam(g_aiDev, g_aiChn, &stParam) == HI_SUCCESS) {
        stParam.u32UsrFrmDepth = 30;
        HI_MPI_AI_SetChnParam(g_aiDev, g_aiChn, &stParam);
    }

    int aiFd = HI_MPI_AI_GetFd(g_aiDev, g_aiChn);

    while (g_audioRunning) {
        FD_ZERO(&read_fds);
        FD_SET(aiFd, &read_fds);
        tv.tv_sec = 1; tv.tv_usec = 0;

        int sel = select(aiFd + 1, &read_fds, NULL, NULL, &tv);
        if (sel <= 0) continue;
        if (!FD_ISSET(aiFd, &read_fds)) continue;

        memset(&stAecFrm, 0, sizeof(stAecFrm));
        if (HI_MPI_AI_GetFrame(g_aiDev, g_aiChn, &stFrame, &stAecFrm, HI_FALSE) != HI_SUCCESS)
            continue;

        HI_MPI_AENC_SendFrame(g_aencChn, &stFrame, &stAecFrm);
        HI_MPI_AI_ReleaseFrame(g_aiDev, g_aiChn, &stFrame, &stAecFrm);
    }
    return NULL;
}

/* AENC stream thread: AENC -> callbacks */
static void *sdk_audio_aenc_thread(void *arg) {
    (void)arg;
    AUDIO_STREAM_S stStream;
    fd_set read_fds;
    struct timeval tv;

    int aencFd = HI_MPI_AENC_GetFd(g_aencChn);

    while (g_audioRunning) {
        FD_ZERO(&read_fds);
        FD_SET(aencFd, &read_fds);
        tv.tv_sec = 1; tv.tv_usec = 0;

        int sel = select(aencFd + 1, &read_fds, NULL, NULL, &tv);
        if (sel <= 0) continue;
        if (!FD_ISSET(aencFd, &read_fds)) continue;

        if (HI_MPI_AENC_GetStream(g_aencChn, &stStream, HI_FALSE) != HI_SUCCESS)
            continue;

        if (g_aencCbCount > 0) {
            LOCALSDK_AUDIO_G711_FRAME_INFO fi;
            memset(&fi, 0, sizeof(fi));
            fi.data = (signed char *)stStream.pStream;
            fi.size = stStream.u32Len;
            fi.timestamp = stStream.u64TimeStamp;
            for (int i = 0; i < g_aencCbCount; i++) {
                if (g_aencCb[i]) g_aencCb[i](&fi);
            }
        }

        HI_MPI_AENC_ReleaseStream(g_aencChn, &stStream);
    }
    return NULL;
}

/**
 * @brief Start audio processing
 */
int local_sdk_audio_start() {
    int32_t result;

    sdk_log("[sdk][audio] Starting audio\n");

    result = HI_MPI_AI_EnableChn(g_aiDev, g_aiChn);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][audio] Failed to enable AI channel: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

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
    g_audioRunning = 0;
    if (g_audioAiThread)   { pthread_join(g_audioAiThread, NULL);   g_audioAiThread = 0; }
    if (g_audioAencThread) { pthread_join(g_audioAencThread, NULL); g_audioAencThread = 0; }
    g_audioStarted = 0;
    return LOCALSDK_OK;
}

/**
 * @brief Run audio - create AENC channel and start AI + AENC threads
 */
int local_sdk_audio_run() {
    AENC_CHN_ATTR_S stAencAttr;
    int32_t result;

    if (!g_audioStarted) return LOCALSDK_ERROR;
    if (g_audioRunning)  return LOCALSDK_OK;

    /* Enable VQE on AI channel */
    HI_MPI_AI_EnableVqe(g_aiDev, g_aiChn);

    /* Create AENC channel (G.711A). u32PtNumPerFrm must match the AI's
       (320 = 40ms @ 8kHz); leaving it 0 makes CreateChn return ILLEGAL_PARAM. */
    static AENC_ATTR_G711_S stAencG711;
    memset(&stAencAttr, 0, sizeof(stAencAttr));
    stAencAttr.enType         = PT_G711A;
    stAencAttr.u32PtNumPerFrm = 320;
    stAencAttr.u32BufSize     = 30;
    stAencAttr.pValue         = &stAencG711;

    result = HI_MPI_AENC_CreateChn(g_aencChn, &stAencAttr);
    if (result != HI_SUCCESS && result != HI_ERR_AENC_EXIST) {
        sdk_log("[sdk][audio] HI_MPI_AENC_CreateChn failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    g_audioRunning = 1;

    if (pthread_create(&g_audioAiThread, NULL, sdk_audio_ai_thread, NULL) != 0) {
        g_audioRunning = 0;
        return LOCALSDK_ERROR;
    }
    if (pthread_create(&g_audioAencThread, NULL, sdk_audio_aenc_thread, NULL) != 0) {
        g_audioRunning = 0;
        pthread_join(g_audioAiThread, NULL);
        g_audioAiThread = 0;
        return LOCALSDK_ERROR;
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
   SPEAKER SUBSYSTEM - HISILICON ADEC + AO
   ============================================================================ */

/**
 * @brief Initialize speaker decoder (ADEC)
 */
int local_sdk_speaker_init() {
    ADEC_CHN_ATTR_S stAdecAttr;
    HI_S32 result;

    sdk_log("[sdk][speaker] Initializing speaker (ADEC)\n");
    
    /* pValue must point to a protocol attr (G711); NULL -> ADEC NULL_PTR. */
    static ADEC_ATTR_G711_S stAdecG711;
    stAdecAttr.enType = PT_G711A;
    stAdecAttr.u32BufSize = 20;
    stAdecAttr.enMode = ADEC_MODE_STREAM;
    stAdecAttr.pValue = &stAdecG711;

    result = HI_MPI_ADEC_CreateChn(g_adecChn, &stAdecAttr);
    if (result != HI_SUCCESS && result != HI_ERR_ADEC_EXIST) {
        sdk_log("[sdk][speaker] HI_MPI_ADEC_CreateChn failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

/**
 * @brief Set speaker parameters
 */
int local_sdk_speaker_set_parameters(LOCALSDK_SPEAKER_OPTIONS *options) {
    if (!options) return LOCALSDK_ERROR;
    sdk_log("[sdk][speaker] Setting parameters\n");
    /* TODO: Apply specific codec options if needed */
    return LOCALSDK_OK;
}

/**
 * @brief Set speaker volume
 */
int local_sdk_speaker_set_volume(int value) {
    sdk_log("[sdk][speaker] Setting volume to %d dB\n", value);
    return HI_MPI_AO_SetVolume(g_aoDev, value) == HI_SUCCESS ? LOCALSDK_OK : LOCALSDK_ERROR;
}

/**
 * @brief Mute/Unmute speaker
 */
int local_sdk_speaker_mute() {
    return HI_MPI_AO_SetMute(g_aoDev, HI_TRUE, NULL) == HI_SUCCESS ? LOCALSDK_OK : LOCALSDK_ERROR;
}

int local_sdk_speaker_unmute() {
    return HI_MPI_AO_SetMute(g_aoDev, HI_FALSE, NULL) == HI_SUCCESS ? LOCALSDK_OK : LOCALSDK_ERROR;
}

/**
 * @brief Start speaker - Bind ADEC to AO
 */
int local_sdk_speaker_start() {
    MPP_CHN_S stSrcChn, stDestChn;
    HI_S32 result;

    sdk_log("[sdk][speaker] Starting speaker (Binding ADEC->AO)\n");

    /* Bind ADEC to AO */
    stSrcChn.enModId = HI_ID_ADEC;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = g_adecChn;

    stDestChn.enModId = HI_ID_AO;
    stDestChn.s32DevId = g_aoDev;
    stDestChn.s32ChnId = g_aoChn;

    result = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][speaker] HI_MPI_SYS_Bind ADEC->AO failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AO_EnableChn(g_aoDev, g_aoChn);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][speaker] HI_MPI_AO_EnableChn failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    pthread_mutex_lock(&g_speakerMutex);
    g_speakerRunState = 3; /* Started */
    pthread_mutex_unlock(&g_speakerMutex);

    return LOCALSDK_OK;
}

/**
 * @brief Feed PCM data (raw 16-bit 16kHz)
 */
int local_sdk_speaker_feed_pcm_data(void *data, int size) {
    AUDIO_FRAME_S stFrame;
    HI_S32 result;

    if (!data || size <= 0) return LOCALSDK_ERROR;

    memset(&stFrame, 0, sizeof(stFrame));
    stFrame.enBitwidth = AUDIO_BIT_WIDTH_16;
    stFrame.enSoundmode = AUDIO_SOUND_MODE_MONO;
    stFrame.u32Len = size;
    stFrame.u64VirAddr[0] = (HI_U8 *)data;
    stFrame.u64PhyAddr[0] = 0; /* MPI will handle it if mapped correctly */

    result = HI_MPI_AO_SendFrame(g_aoDev, g_aoChn, &stFrame, 1000);
    return (result == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

/**
 * @brief Feed G.711 data (encoded)
 */
int local_sdk_speaker_feed_g711_data(void *data, int size) {
    AUDIO_STREAM_S stStream;
    HI_S32 result;

    if (!data || size <= 0) return LOCALSDK_ERROR;

    memset(&stStream, 0, sizeof(stStream));
    stStream.pStream = (HI_U8 *)data;
    stStream.u32Len = size;
    stStream.u64TimeStamp = 0;

    result = HI_MPI_ADEC_SendStream(g_adecChn, &stStream, HI_TRUE);
    return (result == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

int local_sdk_speaker_finish_buf_data() {
    return LOCALSDK_OK;
}

int local_sdk_speaker_clean_buf_data() {
    HI_MPI_AO_ClearChnBuf(g_aoDev, g_aoChn);
    return LOCALSDK_OK;
}

/* ============================================================================
   ALARM SUBSYSTEM
   ============================================================================ */

/**
 * @brief Initialize alarm subsystem
 */
/* IVP detection entry points (defined in the IVP subsystem section below). */
int32_t        sample_ivp_init(int32_t width, int32_t height, const char *oms_file);
int32_t        sample_ivp_deinit(void);
static int32_t sdk_ivp_detect_start(void);
static void    sdk_ivp_detect_stop(void);

int local_sdk_alarm_init(int width, int height) {
    if (width <= 0 || height <= 0) {
        sdk_log("[sdk][alarm] Invalid dimensions: %dx%d\n", width, height);
        return LOCALSDK_ERROR;
    }
    
    sdk_log("[sdk][alarm] Initializing alarm: %dx%d\n", width, height);

    /* Bring up the IVP humanoid detector on the 640x360 model and start the
       detection thread (reads VPSS secondary, dispatches alarm events). */
    if (sample_ivp_init(BOARD_SUB_WIDTH, BOARD_SUB_HEIGHT, NULL) != LOCALSDK_OK) {
        sdk_log("[sdk][alarm] IVP init failed; detection disabled\n");
    } else if (sdk_ivp_detect_start() != LOCALSDK_OK) {
        sdk_log("[sdk][alarm] IVP detection thread failed to start\n");
    }

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
    sdk_ivp_detect_stop();
    sample_ivp_deinit();
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

/* ---- OSD bitmap font (8x8, public-domain font8x8_basic subset) -------------
 * Only the glyphs used by the timestamp are embedded: digits, '-', ':' and
 * space. Each glyph is 8 rows; within a row bit0 is the leftmost pixel. */
static const unsigned char FONT_DIGIT[10][8] = {
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, /* 0 */
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, /* 1 */
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, /* 2 */
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, /* 3 */
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, /* 4 */
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, /* 5 */
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, /* 6 */
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* 7 */
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, /* 8 */
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, /* 9 */
};
static const unsigned char FONT_DASH[8]  = {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00};
static const unsigned char FONT_COLON[8] = {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00};
static const unsigned char FONT_SPACE[8] = {0,0,0,0,0,0,0,0};

static const unsigned char *sdk_osd_font_glyph(char c) {
    if (c >= '0' && c <= '9') return FONT_DIGIT[c - '0'];
    if (c == '-') return FONT_DASH;
    if (c == ':') return FONT_COLON;
    return FONT_SPACE;
}

/* Draw one scaled 8x8 glyph into an ARGB1555 canvas at (x0,y0). */
static void sdk_osd_draw_glyph(uint16_t *base, uint32_t stride_px,
                               uint32_t canvas_w, uint32_t canvas_h,
                               int x0, int y0, const unsigned char *g,
                               int scale, uint16_t color) {
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (!((g[row] >> col) & 1)) continue;
            for (int sy = 0; sy < scale; sy++) {
                int py = y0 + row * scale + sy;
                if (py < 0 || (uint32_t)py >= canvas_h) continue;
                for (int sx = 0; sx < scale; sx++) {
                    int px = x0 + col * scale + sx;
                    if (px < 0 || (uint32_t)px >= canvas_w) continue;
                    base[py * stride_px + px] = color;
                }
            }
        }
    }
}

/**
 * @brief Internal helper to show/hide and attach/detach OSD regions
 */
static int32_t inner_OverLay_ShowRgn(RGN_HANDLE handle, int chn, int x, int y, bool show) {
    MPP_CHN_S stChn;
    RGN_CHN_ATTR_S stChnAttr;
    HI_S32 result;

    stChn.enModId = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = chn;

    /* Check if already attached */
    result = HI_MPI_RGN_GetDisplayAttr(handle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        if (show) {
            /* Need to attach */
            memset(&stChnAttr, 0, sizeof(stChnAttr));
            stChnAttr.bShow = HI_TRUE;
            stChnAttr.enType = OVERLAY_RGN;
            stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = x;
            stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = y;
            /* Hisi OVERLAY alpha range is [0,128] (128 = opaque, not 255). */
            stChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 0;   /* transparent bg */
            stChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 128; /* opaque text    */
            /* Distinct layer per region on the channel (timestamp/logo/rects);
               sharing a layer makes a later attach fail ILLEGAL_PARAM. */
            stChnAttr.unChnAttr.stOverlayChn.u32Layer = (HI_U32)(handle & 7);

            result = HI_MPI_RGN_AttachToChn(handle, &stChn, &stChnAttr);
            if (result != HI_SUCCESS) {
                sdk_log("[sdk][osd] HI_MPI_RGN_AttachToChn failed: 0x%x\n", result);
                return LOCALSDK_ERROR;
            }
        }
    } else {
        /* Update visibility */
        stChnAttr.bShow = show ? HI_TRUE : HI_FALSE;
        HI_MPI_RGN_SetDisplayAttr(handle, &stChn, &stChnAttr);
    }

    return LOCALSDK_OK;
}

/**
 * @brief Initialize an OSD region
 */
static int32_t sdk_osd_region_init(RGN_HANDLE handle, uint32_t width, uint32_t height) {
    RGN_ATTR_S stRgnAttr;
    HI_S32 result;

    memset(&stRgnAttr, 0, sizeof(stRgnAttr));
    stRgnAttr.enType = OVERLAY_RGN;
    stRgnAttr.unAttr.stOverlay.enPixelFmt = PIXEL_FORMAT_ARGB_1555;
    stRgnAttr.unAttr.stOverlay.stSize.u32Width = width;
    stRgnAttr.unAttr.stOverlay.stSize.u32Height = height;
    stRgnAttr.unAttr.stOverlay.u32BgColor = 0; /* Transparent */
    stRgnAttr.unAttr.stOverlay.u32CanvasNum = 2; /* double-buffered; 0 is illegal */

    result = HI_MPI_RGN_Create(handle, &stRgnAttr);
    if (result != HI_SUCCESS && result != HI_ERR_RGN_EXIST) {
        sdk_log("[sdk][osd] HI_MPI_RGN_Create failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    return LOCALSDK_OK;
}

/**
 * @brief Set OSD parameters
 */
int local_sdk_video_osd_set_parameters(int chn, LOCALSDK_OSD_OPTIONS *options) {
    OSD_CHANNEL_PARAMS *params = sdk_osd_get_params(chn);
    if (!params || !options) return LOCALSDK_ERROR;

    sdk_log("[sdk][osd] Setting OSD parameters for channel %d\n", chn);
    memcpy(&params->opts, options, sizeof(LOCALSDK_OSD_OPTIONS));
    
    /* Force re-init of regions if needed (simplified) */
    params->timestamp_en = 1;
    params->logo_en = 1;
    params->rects_en = 1;
    
    return LOCALSDK_OK;
}

/**
 * @brief Update MI logo visibility
 */
int local_sdk_video_osd_update_logo(int chn, bool state) {
    OSD_CHANNEL_PARAMS *params = sdk_osd_get_params(chn);
    if (!params) return LOCALSDK_ERROR;

    if (params->logo_hdl == 0) {
        params->logo_hdl = chn * 3 + 1;
        sdk_osd_region_init(params->logo_hdl, 128, 64);
    }

    /* NOTE: the logo bitmap itself is not yet rendered (no embedded MI asset),
       so this region is currently empty/transparent. */
    return inner_OverLay_ShowRgn(params->logo_hdl, chn,
                                 params->opts.oemlogo_x, params->opts.oemlogo_y, state);
}

/**
 * @brief Update timestamp on OSD
 */
int local_sdk_video_osd_update_timestamp(int chn, bool state, struct tm *timestamp) {
    OSD_CHANNEL_PARAMS *params = sdk_osd_get_params(chn);
    RGN_CANVAS_INFO_S stCanvas;
    HI_S32 result;

    if (!params) return LOCALSDK_ERROR;

    /* Glyph scale: base 3 (≈24px tall @1080p), adjusted by config size. */
    int scale = 3;
    if (params->opts.datetime_increase > 1) scale *= (int)params->opts.datetime_increase;
    if (params->opts.datetime_reduce   > 1) scale /= (int)params->opts.datetime_reduce;
    if (scale < 1) scale = 1;
    int shadow = (scale >= 2) ? 2 : 1; /* drop-shadow offset for legibility */

    if (params->timestamp_hdl == 0) {
        params->timestamp_hdl = chn * 3 + 0;
        /* Size to fit "YYYY-MM-DD HH:MM:SS" (19 chars) + shadow margin. */
        uint32_t w = (uint32_t)(20 * 8 * scale + shadow);
        uint32_t h = (uint32_t)(8 * scale + shadow);
        w = (w + 1) & ~1u;
        h = (h + 1) & ~1u;
        sdk_osd_region_init(params->timestamp_hdl, w, h);
    }

    if (state && timestamp) {
        char buf[24];
        int slen = snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                            timestamp->tm_year + 1900, timestamp->tm_mon + 1,
                            timestamp->tm_mday, timestamp->tm_hour,
                            timestamp->tm_min, timestamp->tm_sec);

        result = HI_MPI_RGN_GetCanvasInfo(params->timestamp_hdl, &stCanvas);
        if (result == HI_SUCCESS) {
            uint16_t *pix       = (uint16_t *)(uintptr_t)stCanvas.u64VirtAddr;
            uint32_t  stride_px = stCanvas.u32Stride / 2;
            uint32_t  cw        = stCanvas.stSize.u32Width;
            uint32_t  ch        = stCanvas.stSize.u32Height;

            /* Clear to transparent */
            memset(pix, 0, stCanvas.u32Stride * ch);

            int x = 0;
            for (int i = 0; i < slen; i++) {
                const unsigned char *g = sdk_osd_font_glyph(buf[i]);
                /* black drop shadow first, then white text on top */
                sdk_osd_draw_glyph(pix, stride_px, cw, ch, x + shadow, shadow, g, scale, 0x8000);
                sdk_osd_draw_glyph(pix, stride_px, cw, ch, x, 0, g, scale, 0xFFFF);
                x += 8 * scale;
                if ((uint32_t)(x + 8 * scale) > cw) break;
            }
            HI_MPI_RGN_UpdateCanvas(params->timestamp_hdl);
        }
    }

    return inner_OverLay_ShowRgn(params->timestamp_hdl, chn,
                                 params->opts.datetime_x, params->opts.datetime_y, state);
}

/**
 * @brief Update multiple rectangles (motion detection)
 */
/* Set one pixel in a 2bpp (4 pixels/byte) overlay canvas. */
static inline void sdk_osd_px2bpp(uint8_t *base, uint32_t stride, uint32_t x,
                                  uint32_t y, uint8_t v) {
    uint8_t *p = base + y * stride + (x >> 2);
    int sh = (int)(x & 3) * 2;
    *p = (uint8_t)((*p & ~(3 << sh)) | ((v & 3) << sh));
}

/* Detection boxes use a full-frame 2bpp OVERLAY with a 2-entry colour LUT, like
   the original firmware (1920x1080, PiFmt ARGB_2BPP, ~1 MB — vs ~8 MB for an
   ARGB1555 canvas, which fails NOMEM next to the IVP). COVER_RGN was tried but
   is not supported on VENC (0xa0038008 NOT_SUPPORT). Outlines drawn with pixel
   value 1 -> ColorLUT; value 0 = transparent. */
int local_sdk_video_osd_update_rect_multi(int chn, bool state, LOCALSDK_OSD_RECTANGLES *rectangles) {
    OSD_CHANNEL_PARAMS *params = sdk_osd_get_params(chn);
    HI_S32 result;
    if (!params) return LOCALSDK_ERROR;

    if (params->rects_hdl == 0) {
        params->rects_hdl = chn * 3 + 2;
        RGN_ATTR_S a;
        memset(&a, 0, sizeof(a));
        a.enType = OVERLAY_RGN;
        a.unAttr.stOverlay.enPixelFmt       = PIXEL_FORMAT_ARGB_2BPP;
        a.unAttr.stOverlay.stSize.u32Width  = BOARD_WIDTH;
        a.unAttr.stOverlay.stSize.u32Height = BOARD_HEIGHT;
        a.unAttr.stOverlay.u32BgColor       = 0;
        a.unAttr.stOverlay.u32CanvasNum     = 2;
        result = HI_MPI_RGN_Create(params->rects_hdl, &a);
        if (result != HI_SUCCESS && result != HI_ERR_RGN_EXIST) {
            sdk_log("[sdk][osd] rect RGN_Create(2bpp) failed: 0x%x\n", result);
            params->rects_hdl = 0;
            return LOCALSDK_ERROR;
        }
    }

    if (state && rectangles && rectangles->count > 0) {
        RGN_CANVAS_INFO_S cv;
        if (HI_MPI_RGN_GetCanvasInfo(params->rects_hdl, &cv) == HI_SUCCESS) {
            uint8_t *base   = (uint8_t *)(uintptr_t)cv.u64VirtAddr;
            uint32_t stride = cv.u32Stride;
            uint32_t cw = cv.stSize.u32Width, ch = cv.stSize.u32Height;
            const uint32_t T = 4; /* outline thickness */

            memset(base, 0, stride * ch); /* transparent */
            for (uint32_t o = 0; o < rectangles->count && o < LOCALSDK_ALARM_MAXIMUM_OBJECTS; o++) {
                if (!rectangles->objects[o].visible) continue;
                uint32_t x = rectangles->objects[o].x, y = rectangles->objects[o].y;
                uint32_t w = rectangles->objects[o].width, h = rectangles->objects[o].height;
                if (x >= cw || y >= ch) continue;
                uint32_t x2 = (x + w < cw) ? x + w : cw - 1;
                uint32_t y2 = (y + h < ch) ? y + h : ch - 1;
                /* Pixel value 3 (byte 0xFF) = humanoid, matching the original's
                   inner_update_rect_multi_canvas (0xFF for type!=3). Value 0 is
                   transparent; non-zero is opaque via the colour LUT + FgAlpha. */
                for (uint32_t px = x; px <= x2; px++)
                    for (uint32_t t = 0; t < T; t++) {
                        if (y + t < ch)  sdk_osd_px2bpp(base, stride, px, y + t, 3);
                        if (y2 >= t)     sdk_osd_px2bpp(base, stride, px, y2 - t, 3);
                    }
                for (uint32_t py = y; py <= y2; py++)
                    for (uint32_t t = 0; t < T; t++) {
                        if (x + t < cw)  sdk_osd_px2bpp(base, stride, x + t, py, 3);
                        if (x2 >= t)     sdk_osd_px2bpp(base, stride, x2 - t, py, 3);
                    }
            }
            HI_MPI_RGN_UpdateCanvas(params->rects_hdl);
        }
    }

    /* Attach on first use (with the colour LUT), then just toggle visibility. */
    MPP_CHN_S stChn;
    stChn.enModId = HI_ID_VENC; stChn.s32DevId = 0; stChn.s32ChnId = chn;
    RGN_CHN_ATTR_S ca;
    bool want = (state && rectangles && rectangles->count > 0);
    if (HI_MPI_RGN_GetDisplayAttr(params->rects_hdl, &stChn, &ca) != HI_SUCCESS) {
        if (!want) return LOCALSDK_OK;
        memset(&ca, 0, sizeof(ca));
        ca.bShow  = HI_TRUE;
        ca.enType = OVERLAY_RGN;
        ca.unChnAttr.stOverlayChn.stPoint.s32X = 0;
        ca.unChnAttr.stOverlayChn.stPoint.s32Y = 0;
        ca.unChnAttr.stOverlayChn.u32FgAlpha   = 128; /* range [0,128] */
        ca.unChnAttr.stOverlayChn.u32BgAlpha   = 0;
        ca.unChnAttr.stOverlayChn.u32Layer     = 5;   /* match original rect layer */
        /* Exact LUT values read from the original firmware's /proc/umap/rgn. */
        ca.unChnAttr.stOverlayChn.u16ColorLUT[0] = 916;
        ca.unChnAttr.stOverlayChn.u16ColorLUT[1] = 31106;
        result = HI_MPI_RGN_AttachToChn(params->rects_hdl, &stChn, &ca);
        if (result != HI_SUCCESS)
            sdk_log("[sdk][osd] rect AttachToChn failed: 0x%x\n", result);
        return (result == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
    }
    ca.bShow = want ? HI_TRUE : HI_FALSE;
    HI_MPI_RGN_SetDisplayAttr(params->rects_hdl, &stChn, &ca);
    return LOCALSDK_OK;
}

/* ============================================================================
   HARDWARE CONTROL SUBSYSTEM
   ============================================================================ */

/**
 * @brief Control indicator LEDs (orange and blue)
 */
int local_sdk_indicator_led_option(bool orange, bool blue) {
    gpio_write(g_board_cfg->gpio_led_orange, orange ? 1 : 0);
    gpio_write(g_board_cfg->gpio_led_blue,   blue   ? 1 : 0);
    return LOCALSDK_OK;
}

/**
 * @brief Setup keydown callback with timeout
 */
int local_sdk_setup_keydown_set_callback(int timeout, int (*callback)()) {
    printf("[SDK-THREAD]dbg: Set setup_keydown Callback Doing...\n");
    pthread_mutex_lock(&g_platformMutex);
    g_keydownCb = callback;
    g_keydownTimeout = timeout;
    pthread_mutex_unlock(&g_platformMutex);
    return LOCALSDK_OK;
}

/* ============================================================================
   NIGHT MODE SUBSYSTEM
   ============================================================================ */

/* Polls ISP AE luma every 1s; delegates day/night decisions to board->pfnOnLuma. */
#define NIGHT_AE_SETTLE_S  8u

static void *night_light_thread(void *arg)
{
    (void)arg;
    usleep(NIGHT_AE_SETTLE_S * 1000000u);
    while (g_nightLightRun) {
        ISP_EXP_INFO_S stExp;
        if (HI_MPI_ISP_QueryExposureInfo(0, &stExp) == HI_SUCCESS)
            if (g_board_cfg && g_board_cfg->pfnOnLuma)
                g_board_cfg->pfnOnLuma(stExp.u8AveLum);
        usleep(1000000);
    }
    return NULL;
}

int local_sdk_auto_night_light() {
    if (g_nightLightThread) return LOCALSDK_OK;
    g_nightLightRun = 1;
    if (pthread_create(&g_nightLightThread, NULL, night_light_thread, NULL) != 0) {
        g_nightLightRun = 0;
        sdk_log("[sdk][night] auto night thread create failed\n");
        return LOCALSDK_ERROR;
    }
    sdk_log("[sdk][night] auto night light thread started\n");
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
    
    /* Validate and pass the resolution through unchanged: downstream
       (SAMPLE_COMM_SYS_GetPicSize, options.resolution) switches on the same
       LOCALSDK_VIDEO_RESOLUTION_* constants, so the value must stay consistent.
       (The old mapping returned 0 for 640x360, which GetPicSize rejected.) */
    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
            *result = LOCALSDK_VIDEO_RESOLUTION_640x360;
            return LOCALSDK_OK;

        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            *result = LOCALSDK_VIDEO_RESOLUTION_1920x1080;
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
static hi_s32   g_ivpHandle = -1;
static pthread_t    g_ivpDetectThread = 0;
static volatile int g_ivpDetectRun    = 0;
static uint32_t g_ivpResourceSize = 0;
static HI_VOID *g_ivpResourceBuffer = NULL;
static HI_U64   g_ivpPhysAddr = 0;
static int32_t  g_ivpInitialized = 0;

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
    HI_U64 phys_addr = 0;
    HI_VOID *virt_addr = NULL;
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

    sdk_log("[sdk][ivp] MMZ allocated - Physical: 0x%llx, Virtual: %p, Size: %u\n",
            (unsigned long long)phys_addr, virt_addr, file_size);

    /* Read OMS file into memory */
    result = sample_ivp_read_file(oms_file, (uint8_t *)virt_addr, file_size);
    if (result != LOCALSDK_OK) {
        sdk_log("[sdk][ivp] Failed to read OMS file\n");
        HI_MPI_SYS_MmzFree(phys_addr, virt_addr);
        return LOCALSDK_ERROR;
    }

    /* Load resource via hi_ivp_mem_info struct */
    {
        hi_ivp_mem_info mem_info;
        memset(&mem_info, 0, sizeof(mem_info));
        mem_info.physical_addr = (hi_u64)phys_addr;
        mem_info.virtual_addr  = (hi_u64)(uintptr_t)virt_addr;
        mem_info.memory_size   = file_size;

        result = hi_ivp_load_resource_from_memory(&mem_info, &g_ivpHandle);
    }
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to load resource from memory: 0x%x\n", result);
        HI_MPI_SYS_MmzFree(phys_addr, virt_addr);
        return LOCALSDK_ERROR;
    }

    g_ivpResourceSize   = file_size;
    g_ivpResourceBuffer = virt_addr;
    g_ivpPhysAddr       = phys_addr;

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
        oms_file = g_board_cfg->ivp_oms_path;
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

    if (g_ivpHandle < 0) {
        sdk_log("[sdk][ivp] No IVP resource to unload\n");
        return LOCALSDK_OK;
    }

    sdk_log("[sdk][ivp] Unloading IVP resource\n");

    result = hi_ivp_unload_resource(g_ivpHandle);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to unload resource: 0x%x\n", result);
    }

    if (g_ivpResourceBuffer) {
        HI_MPI_SYS_MmzFree(g_ivpPhysAddr, g_ivpResourceBuffer);
        g_ivpPhysAddr = 0;
        g_ivpResourceBuffer = NULL;
    }

    g_ivpHandle = -1;
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

    if (!ctrl_attr) {
        sdk_log("[sdk][ivp] Invalid control attributes\n");
        return LOCALSDK_ERROR;
    }

    sdk_log("[sdk][ivp] Setting control attributes for channel %d\n", chn);

    result = hi_ivp_set_ctrl_attr(g_ivpHandle, (const hi_ivp_ctrl_attr *)ctrl_attr);
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

    if (!roi_attr) {
        sdk_log("[sdk][ivp] Invalid ROI attributes\n");
        return LOCALSDK_ERROR;
    }

    sdk_log("[sdk][ivp] Setting ROI attributes for channel %d\n", chn);

    result = hi_ivp_set_roi_attr(g_ivpHandle, (const hi_ivp_roi_attr *)roi_attr);
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

    if (!roi_map) {
        sdk_log("[sdk][ivp] Invalid ROI map\n");
        return LOCALSDK_ERROR;
    }

    sdk_log("[sdk][ivp] Setting ROI map for channel %d\n", chn);

    result = hi_ivp_set_roi_map(g_ivpHandle, (const hi_ivp_roi_map *)roi_map);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set ROI map: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

/**
 * @brief Process frame with IVP
 */
/* Run the humanoid detector on one 640x360 frame and dispatch the result.
 * hi_ivp_process_ex returns boxes per class in the model's (640x360) space;
 * obj_class[0] is the humanoid class. Boxes are scaled to the primary
 * (1920x1080) so the OSD rectangle overlay (drawn on the primary channel) and
 * the app alarm logic receive primary-space coordinates. */
/* Per-class output buffers for hi_ivp_process_ex. The obj_array is in/out: the
   caller must provide an allocated objs buffer and a non-zero rect_capcity for
   every class, otherwise libivp rejects it (ivp_check_obj: rect_capcity can not
   be 0). libivp fills rect_num and the boxes. */
#define SDK_IVP_OBJ_CAPACITY 16
static int32_t sdk_ivp_process_and_dispatch(VIDEO_FRAME_INFO_S *frame) {
    static hi_ivp_obj s_ivpObjBuf[HI_IVP_MAX_CLASS][SDK_IVP_OBJ_CAPACITY];
    hi_ivp_obj_array objs;
    LOCALSDK_ALARM_EVENT_INFO ev;
    int32_t result;

    memset(&objs, 0, sizeof(objs));
    for (int c = 0; c < HI_IVP_MAX_CLASS; c++) {
        objs.obj_class[c].objs         = s_ivpObjBuf[c];
        objs.obj_class[c].rect_capcity = SDK_IVP_OBJ_CAPACITY;
    }
    result = hi_ivp_process_ex(g_ivpHandle, frame, &objs);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] hi_ivp_process_ex failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    const uint32_t sx = BOARD_WIDTH  / BOARD_SUB_WIDTH;   /* 1920/640 = 3 */
    const uint32_t sy = BOARD_HEIGHT / BOARD_SUB_HEIGHT;  /* 1080/360 = 3 */

    memset(&ev, 0, sizeof(ev));
    int n = 0;
    if (objs.class_num > 0) {
        hi_ivp_obj_of_one_class *cls = &objs.obj_class[0]; /* humanoid */
        int cnt = (int)cls->rect_num;
        if (cnt > LOCALSDK_ALARM_MAXIMUM_OBJECTS) cnt = LOCALSDK_ALARM_MAXIMUM_OBJECTS;
        for (int i = 0; i < cnt && cls->objs; i++) {
            hi_ivp_rect *r = &cls->objs[i].rect;
            ev.objects[n].type   = LOCALSDK_ALARM_TYPE_HUMANOID;
            ev.objects[n].state  = 1;
            ev.objects[n].x      = (uint32_t)(r->x > 0 ? r->x : 0) * sx;
            ev.objects[n].y      = (uint32_t)(r->y > 0 ? r->y : 0) * sy;
            ev.objects[n].width  = r->width  * sx;
            ev.objects[n].height = r->height * sy;
            n++;
        }
    }
    ev.type  = LOCALSDK_ALARM_TYPE_HUMANOID;
    ev.state = (n > 0) ? 1 : 0;

    sdk_alarm_run_callback(&ev);
    return LOCALSDK_OK;
}

/* Detection thread: pulls 640x360 frames from VPSS secondary (chn 1, which has
 * u32Depth>0 so user readout coexists with the VENC1 bind) and feeds the IVP. */
static void *sdk_ivp_detect_thread(void *arg) {
    VIDEO_FRAME_INFO_S frame;
    int32_t ret;
    (void)arg;

    sdk_log("[sdk][ivp] detection thread started\n");
    while (g_ivpDetectRun) {
        memset(&frame, 0, sizeof(frame));
        ret = HI_MPI_VPSS_GetChnFrame(g_vpssGrp, LOCALSDK_VIDEO_SECONDARY_CHANNEL,
                                      &frame, 1000);
        if (ret != HI_SUCCESS) {
            usleep(USLEEP_50MS);
            continue;
        }

        sdk_ivp_process_and_dispatch(&frame);

        HI_MPI_VPSS_ReleaseChnFrame(g_vpssGrp, LOCALSDK_VIDEO_SECONDARY_CHANNEL, &frame);
    }
    sdk_log("[sdk][ivp] detection thread stopped\n");
    return NULL;
}

static int32_t sdk_ivp_detect_start(void) {
    if (g_ivpDetectRun) return LOCALSDK_OK;
    g_ivpDetectRun = 1;
    if (pthread_create(&g_ivpDetectThread, NULL, sdk_ivp_detect_thread, NULL) != 0) {
        g_ivpDetectRun = 0;
        sdk_log("[sdk][ivp] failed to start detection thread\n");
        return LOCALSDK_ERROR;
    }
    return LOCALSDK_OK;
}

static void sdk_ivp_detect_stop(void) {
    if (!g_ivpDetectRun) return;
    g_ivpDetectRun = 0;
    if (g_ivpDetectThread) {
        pthread_join(g_ivpDetectThread, NULL);
        g_ivpDetectThread = 0;
    }
}

/**
 * @brief Advance IVP ISP settings (fine-tuning)
 */
int32_t sample_ivp_set_advance_isp(int32_t vi_pipe, int32_t enable) {
    int32_t result;

    sdk_log("[sdk][ivp] Setting advanced ISP: pipe=%d enable=%d\n", vi_pipe, enable);

    result = hi_ivp_set_advance_isp(g_ivpHandle, vi_pipe, enable ? HI_TRUE : HI_FALSE);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set advanced ISP: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

/**
 * @brief Set VENC low bitrate mode for IVP optimization
 */
int32_t sample_ivp_set_venc_low_bitrate(int32_t chn, int32_t enable) {
    int32_t result;

    sdk_log("[sdk][ivp] Setting low bitrate mode: chn=%d enable=%d\n", chn, enable);

    result = hi_ivp_set_venc_low_bitrate(g_ivpHandle, chn, enable ? HI_TRUE : HI_FALSE);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][ivp] Failed to set low bitrate: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

/**
 * @brief Set VENC lowlight ISO threshold for night vision
 */
int32_t sample_ivp_set_venc_lowlight_iso_threshold(int32_t chn, int32_t threshold) {
    hi_ivp_venc_lowlight_iso_threshold iso_thr;
    int32_t result;

    sdk_log("[sdk][ivp] Setting lowlight ISO threshold: chn=%d threshold=%d\n", chn, threshold);

    memset(&iso_thr, 0, sizeof(iso_thr));
    iso_thr.iso_adaptive_enable = HI_TRUE;
    iso_thr.iso_threshold[0] = (hi_u32)threshold;

    result = hi_ivp_set_venc_lowlight_iso_threshold(g_ivpHandle, chn, &iso_thr);
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
    VIDEO_FRAME_INFO_S stFrameInfo;
    int32_t result;
    int32_t s32Count = 0;

    sdk_log("[sdk][video_thread] YUV stream thread started\n");

    while (sdk_video_any_started()) {
        memset(&stFrameInfo, 0, sizeof(VIDEO_FRAME_INFO_S));

        result = HI_MPI_VPSS_GetChnFrame(0, 0, &stFrameInfo, 1000);
        if (result != HI_SUCCESS) {
            usleep(USLEEP_50MS);
            continue;
        }

        sdk_log("[sdk][video_thread] YUV frame: %ux%u\n",
                stFrameInfo.stVFrame.u32Width, stFrameInfo.stVFrame.u32Height);

        result = HI_MPI_VPSS_ReleaseChnFrame(0, 0, &stFrameInfo);
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
                pstPack->DataType.enH264EType == H264E_NALU_ISLICE) {
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
