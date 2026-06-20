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


/* Button callback */
static int (*g_keydownCb)(void) = NULL;
static int g_keydownTimeout = 0;

/* Auto night light polling thread */
static pthread_t        g_nightLightThread = 0;
static volatile int     g_nightLightRun    = 0;


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

/* OSD subsystem implementation moved to osd/osd.c (Phase 4 refactoring). */

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
