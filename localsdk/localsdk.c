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

#include "hi_buffer.h"
#include "hi_sns_ctrl.h"
#include "platform/platform.h"

#include "scene/scene.h"
#include "video/video.h"

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

/* Board config pointer â€” set once in localsdk_init(), used everywhere.
   localsdk is blind to the sensor: behaviour via board callbacks, sizing via
   the BOARD_* compile-time constants. */
static const board_cfg_t  *g_board_cfg  = NULL;

static int (*g_sdkLogPrintf_cb)(const char *, ...) = NULL;
static int (*g_sdkShellCall_cb)(const char *) = NULL;

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

/* Button callback */
static int (*g_keydownCb)(void) = NULL;
static int g_keydownTimeout = 0;

/* Auto night light polling thread */
static pthread_t        g_nightLightThread = 0;
static volatile int     g_nightLightRun    = 0;

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

    /* Stop VENC capture threads + unbind VPSSâ†’VENC + destroy VENC channels. */
    video_free();

    /* Unbind VIâ†’VPSS, stop VPSS group, destroy VPSS group. */
    video_deinit();

    /* VI teardown â€” handles owned by the board/sensor, fetched generically. */
    const board_cfg_t *board = platform_get_board_cfg();
    VI_DEV viDev = 0; VI_PIPE viPipe = 0; VI_CHN viChn = 0;
    if (board && board->pfnGetVi)
        board->pfnGetVi(&viDev, &viPipe, &viChn);
    HI_MPI_VI_DisableChn(viPipe, viChn);
    HI_MPI_VI_StopPipe(viPipe);
    HI_MPI_VI_DestroyPipe(viPipe);
    HI_MPI_VI_DisableDev(viDev);
    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    /* Sensor ISP teardown (run thread join + sensor unregister) owned by board. */
    if (board && board->pfnTeardownSensorIsp)
        board->pfnTeardownSensorIsp();

    sdk_log("[sdk] SDK cleanup complete\n");
    return LOCALSDK_OK;
}

/**
 * @brief Get SDK version
 */
int localsdk_get_version() {
    return LOCALSDK_CURRENT_VERSION;
}

/* Video subsystem implementation moved to video/video.c (Phase 1 refactoring). */

/* Audio subsystem implementation moved to audio/audio.c (Phase 2 refactoring). */
/* Speaker subsystem implementation moved to speaker/speaker.c (Phase 2 refactoring). */
/* Alarm/IVP subsystem implementation moved to alarm/alarm.c (Phase 3 refactoring). */

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

    /* Glyph scale: base 3 (â‰ˆ24px tall @1080p), adjusted by config size. */
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
   the original firmware (1920x1080, PiFmt ARGB_2BPP, ~1 MB â€” vs ~8 MB for an
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
   END OF FILE
   ============================================================================ */
