/* Board configuration and runtime logic for Xiaomi MJSXJ02HL.
 * GPIO assignments and hardware layout from runtime trace (trace.txt).
 * The JXF22 sensor is mounted rotated 180° on this PCB.
 *
 * This file owns all board-specific behaviour:
 *   - Sensor configuration (fps, mirror/flip)
 *   - Scene ISP parameters (day/night INI files)
 *   - GPIO control (IR LED, IR-cut motor)
 *   - Day/night state machine (luma thresholds, hysteresis, transitions)
 */

#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "platform.h"
#include "mpi_vi.h"
#include "mpi_sys.h"
#include "mpi_vb.h"
#include "mpi_isp.h"
#include "../sensor/jxf/sensor_jxf22.h"
#include "../scene/scene.h"
#include "../../logger/logger.h"

/* Day/night detection uses the AE demand (ISO), NOT the metered output luma.
 * The stock firmware judges day/night from the AE exposure/gain (libsceneauto
 * photo_sensitive_ev_judge / gain_judge query HI_MPI_ISP_QueryExposureInfo /
 * QueryWBInfo) — there is no light-sensor GPIO/ADC (gpio9 stays 1 in the dark).
 * u8AveLum is unusable: once the night ISP profile is applied it collapses a
 * lit room into ~46-56 (the 30-60 dead zone) and the camera stays stuck in
 * night. ISO reflects the actual light demand and does not collapse:
 * measured on the stock camera — lit room ISO~1086, dark room ISO~5025. */
#define MJSXJ02HL_NIGHT_ISO  3000     /* ISO above this → dark → night candidate */
#define MJSXJ02HL_DAY_ISO    2000     /* ISO below this → lit  → day candidate   */
#define MJSXJ02HL_HYSTERESIS    3     /* consecutive samples required for transition */
#define MJSXJ02HL_SETTLE_S     15     /* seconds to skip sampling after transition */

#define MJSXJ02HL_SCENE_DAY   "/usr/app/local/sensor.ini/config_product_scene_1080p20_linear.ini"
#define MJSXJ02HL_SCENE_NIGHT "/usr/app/local/sensor.ini/config_product_scene_1080p20_linear_ir.ini"

/* Forward declaration — defined below after callbacks */
extern const board_cfg_t g_board_mjsxj02hl;

/* Registered app-level callback (MQTT + alarm_switch). NULL = not registered. */
static void (*s_mode_change_cb)(HI_BOOL) = NULL;

/* Current day/night state (1=day, 0=night) */
static volatile int s_current_mode = 1; /* assume day at startup */

/* Hysteresis counters — only used by pfnOnLuma (single luma thread writer) */
static int s_consec_night = 0;
static int s_consec_day   = 0;
static int s_settling     = 0; /* remaining 1s ticks in settle period */

/* -------------------------------------------------------------------------
 * GPIO helpers
 * ---------------------------------------------------------------------- */

static void board_gpio_write(int gpio, int value)
{
    if (gpio < 0) return; /* pin not present on this board */
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    FILE *f = fopen(path, "w");
    if (f) { fprintf(f, "%d", value); fclose(f); }
}

/* -------------------------------------------------------------------------
 * GPIO boot-time setup helpers (export + direction + read)
 * ---------------------------------------------------------------------- */

static void gpio_export(int pin)
{
    FILE *f = fopen("/sys/class/gpio/export", "w");
    if (f) { fprintf(f, "%d", pin); fclose(f); }
    /* EBUSY = already exported; ignore */
}

static void gpio_set_direction(int pin, const char *dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    FILE *f = fopen(path, "w");
    if (f) { fprintf(f, "%s", dir); fclose(f); }
    printf("[gpio_init]dbg: gpio:%d  dir:%s  ok!\n", pin, dir);
}

static int gpio_read(int pin)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    int v = 0;
    fscanf(f, "%d", &v);
    fclose(f);
    return v;
}

static void gpio_init_pin(int pin, const char *dir, int init_value)
{
    gpio_export(pin);
    gpio_set_direction(pin, dir);
    if (strcmp(dir, "out") == 0) {
        board_gpio_write(pin, init_value);
        printf("[SDK-GPIO]dbg: Pin(%d)  Lvl(%d)  Dir(%s)\n", pin, init_value, dir);
    } else {
        printf("[SDK-GPIO]dbg: Pin(%d)  Lvl(%d)  Dir(%s)\n", pin, gpio_read(pin), dir);
    }
}

static void gpio_all_init(const board_cfg_t *b)
{
    gpio_init_pin(b->gpio_ir_led_b,     "out", 0);
    gpio_init_pin(b->gpio_ir_led_a,     "out", 1);
    gpio_init_pin(b->gpio_ircut_b,      "out", 0);
    gpio_init_pin(b->gpio_ircut_a,      "out", 0);
    gpio_init_pin(b->gpio_led_orange,   "out", 0);
    gpio_init_pin(b->gpio_led_blue,     "out", 1);
    gpio_init_pin(b->gpio_button_setup, "in",  1);
    gpio_init_pin(b->gpio_photo_sensor, "in",  0);
}

/* -------------------------------------------------------------------------
 * PWM / softlight
 * ---------------------------------------------------------------------- */

#define PWM_DEV_PATH  "/dev/pwm"
#define PWM_CMD_WRITE _IOW('p', 0x01, struct pwm_data_s)

struct pwm_data_s {
    unsigned int pwm_num;
    unsigned int duty;
    unsigned int period;
    unsigned int enable;
};

static int g_pwmFd = -1;

static void board_pwm_init(void)
{
    g_pwmFd = open(PWM_DEV_PATH, O_RDWR);
    /* Not fatal if the PWM device is absent */
}

static void board_softlight_init(void)
{
    if (g_pwmFd >= 0) {
        struct pwm_data_s d = { .pwm_num = 0, .duty = 0, .period = 256, .enable = 1 };
        ioctl(g_pwmFd, PWM_CMD_WRITE, &d);
    }
}

/* -------------------------------------------------------------------------
 * Button polling thread
 * ---------------------------------------------------------------------- */

static pthread_mutex_t g_keyMutex  = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_keyThread = 0;
static int           (*g_keyCb)(void) = NULL;
static int             g_keyTimeout   = 0;

static void *button_thread(void *arg)
{
    (void)arg;
    const board_cfg_t *b = &g_board_mjsxj02hl;
    printf("[SDK-THREAD]dbg: Platform Thread Start...\n");
    int last_state = 1;
    while (1) {
        usleep(50000);
        int v = gpio_read(b->gpio_button_setup);
        if (v == 0 && last_state == 1) {
            pthread_mutex_lock(&g_keyMutex);
            if (g_keyCb) g_keyCb();
            pthread_mutex_unlock(&g_keyMutex);
        }
        last_state = v;
    }
    return NULL;
}

/* -------------------------------------------------------------------------
 * Public platform API (declared in platform.h)
 * ---------------------------------------------------------------------- */

void board_platform_init(void)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;

    puts("----------------------------------------");
    puts("    OPEN LOCALSDK (oss)        ('_)')");
    puts("----------------------------------------");
    puts("    platform: hi3518ev300 ");
    printf("    board   : %s \n", b->name);
    puts("    sdk ver : 14 ");
    printf("    build   : %s (%s) \n", __DATE__, __TIME__);
    puts("----------------------------------------");

    pthread_mutex_init(&g_keyMutex, NULL);
    board_pwm_init();
    board_softlight_init();
    gpio_all_init(b);

    if (pthread_create(&g_keyThread, NULL, button_thread, NULL) != 0) {
        g_keyThread = 0;
        printf("[board_platform_init]err: Platform Thread Create Fail!\n");
    } else {
        printf("[SDK-THREAD]dbg: Platform Thread Create OK!  ('_)')\n");
    }
}

void board_platform_deinit(void)
{
    /* Cancel button polling thread */
    if (g_keyThread) {
        pthread_cancel(g_keyThread);
        pthread_join(g_keyThread, NULL);
        g_keyThread = 0;
    }
    if (g_pwmFd >= 0) { close(g_pwmFd); g_pwmFd = -1; }

    /* VI teardown */
    const board_cfg_t *b = &g_board_mjsxj02hl;
    VI_DEV dev = 0; VI_PIPE pipe = 0; VI_CHN chn = 0;
    if (b->pfnGetVi) b->pfnGetVi(&dev, &pipe, &chn);
    HI_MPI_VI_DisableChn(pipe, chn);
    HI_MPI_VI_StopPipe(pipe);
    HI_MPI_VI_DestroyPipe(pipe);
    HI_MPI_VI_DisableDev(dev);
    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    /* Sensor ISP teardown */
    if (b->pfnTeardownSensorIsp) b->pfnTeardownSensorIsp();
}

void board_indicator_led(bool orange, bool blue)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;
    board_gpio_write(b->gpio_led_orange, orange ? 1 : 0);
    board_gpio_write(b->gpio_led_blue,   blue   ? 1 : 0);
}

void board_set_button_callback(int timeout, int (*cb)(void))
{
    printf("[SDK-THREAD]dbg: Set setup_keydown Callback Doing...\n");
    pthread_mutex_lock(&g_keyMutex);
    g_keyCb      = cb;
    g_keyTimeout = timeout;
    pthread_mutex_unlock(&g_keyMutex);
}

/* -------------------------------------------------------------------------
 * (end of platform API block)
 * ---------------------------------------------------------------------- */

static void board_ircut_open(void)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;
    board_gpio_write(b->gpio_ircut_a, 1);
    usleep((unsigned)b->gpio_ircut_step_us);
    board_gpio_write(b->gpio_ircut_b, 1);
}

static void board_ircut_close(void)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;
    board_gpio_write(b->gpio_ircut_a, 0);
    usleep((unsigned)b->gpio_ircut_step_us);
    board_gpio_write(b->gpio_ircut_b, 0);
}

/* -------------------------------------------------------------------------
 * Mode transition — GPIO + ISP scene + app notification
 * ---------------------------------------------------------------------- */

static void board_apply_mode(HI_BOOL is_night)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;
    if (is_night) {
        /* Switch ISP to night profile first (grayscale + low gain) before IR floods in */
        scene_set_night();
        board_gpio_write(b->gpio_ir_led_a, 1);
        if (b->gpio_ir_led_b >= 0) board_gpio_write(b->gpio_ir_led_b, 1);
        board_ircut_open();
        LOGGER(LOGGER_LEVEL_INFO, "[board][mjsxj02hl] → NIGHT: scene night, IR LED on, IR-cut open");
    } else {
        /* Close IR-cut first, wait for motor, then restore colour ISP */
        board_ircut_close();
        usleep(1500000); /* 1.5s for IR-cut filter motor to fully insert */
        scene_set_day();
        board_gpio_write(b->gpio_ir_led_a, 0);
        if (b->gpio_ir_led_b >= 0) board_gpio_write(b->gpio_ir_led_b, 0);
        LOGGER(LOGGER_LEVEL_INFO, "[board][mjsxj02hl] → DAY: IR-cut closed, scene day, IR LED off");
    }
    s_current_mode = is_night ? 0 : 1;
    if (s_mode_change_cb) s_mode_change_cb(is_night);
}

/* -------------------------------------------------------------------------
 * Board lifecycle callbacks (wired into board_cfg_t)
 * ---------------------------------------------------------------------- */

/* Physical PCB mount: the JXF22 is rotated 180°, so the board's BASE sensor
   orientation is mirror+flip. The ISP Bayer phase (BAYER_BGGR) is calibrated for
   this mounted readout — running the sensor "normal" both flips the image AND
   corrupts colour (wrong Bayer phase → green tint). The app flip/mirror toggle
   relative to this base (XOR): with .conf flip=0/mirror=0 the image is upright. */
#define MJSXJ02HL_MOUNT_MIRROR  1   /* 180° mount */
#define MJSXJ02HL_MOUNT_FLIP    1

/* Board owns and configures its sensor: composes the app-requested orientation
   over the physical mount, adds its target fps, and brings the sensor up. */
static HI_S32 mjsxj02hl_bringup_sensor(HI_BOOL mirror, HI_BOOL flip)
{
    sensor_config_t cfg = {
        .fps    = BOARD_TARGET_FPS,
        .mirror = (HI_BOOL)(MJSXJ02HL_MOUNT_MIRROR ^ (mirror != 0)),
        .flip   = (HI_BOOL)(MJSXJ02HL_MOUNT_FLIP   ^ (flip   != 0)),
    };
    sensor_jxf22_configure(&cfg);
    LOGGER(LOGGER_LEVEL_INFO,
           "[board][mjsxj02hl] bringup sensor: fps=%u app(mirror=%d flip=%d) -> sensor(mirror=%d flip=%d)",
           BOARD_TARGET_FPS, (int)mirror, (int)flip, (int)cfg.mirror, (int)cfg.flip);
    return sensor_jxf22_bringup();
}

static void mjsxj02hl_get_vi(VI_DEV *dev, VI_PIPE *pipe, VI_CHN *chn)
{
    sensor_jxf22_get_vi(dev, pipe, chn);
}

static void mjsxj02hl_teardown_sensor_isp(void)
{
    sensor_jxf22_teardown_isp();
}

static HI_S32 mjsxj02hl_init(void)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;

    /* GPIO initial state: IR LED off, IR-cut motor relaxed (A=0, B=0). */
    board_gpio_write(b->gpio_ir_led_a, 0);
    if (b->gpio_ir_led_b >= 0) board_gpio_write(b->gpio_ir_led_b, 0);
    board_gpio_write(b->gpio_ircut_a, 0);
    board_gpio_write(b->gpio_ircut_b, 0);

    /* Bistable IR-cut: at cold boot the filter's latched position is unknown.
     * A single day-direction pulse is a silent no-op when the filter is already
     * latched in day (which is why our boot made no "click", unlike the stock
     * firmware). Drive a full night→day cycle so the motor is always actuated
     * into a known day (filter-in) position and audibly seats. Runs before the
     * sensor produces frames, so the brief night excursion is not visible. */
    board_ircut_open();                  /* pulse toward night (filter out) */
    usleep(300000);                      /* let the motor reach the night stop */
    board_ircut_close();                 /* pulse back to day (filter in) */
    LOGGER(LOGGER_LEVEL_INFO, "[board][mjsxj02hl] init: IR-cut boot seat cycle night→day (filter in)");

    HI_S32 ret = scene_init(MJSXJ02HL_SCENE_DAY, MJSXJ02HL_SCENE_NIGHT, BOARD_TARGET_FPS);
    if (ret != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[board][mjsxj02hl] scene_init failed (%d)", ret);
        return HI_FAILURE;
    }
    scene_set_day();
    LOGGER(LOGGER_LEVEL_INFO, "[board][mjsxj02hl] init: scene day loaded, GPIO in day state");
    return HI_SUCCESS;
}

static void mjsxj02hl_on_luma(HI_U8 luma)
{
    if (s_settling > 0) {
        s_settling--;
        return;
    }

    /* Judge day/night from the AE ISO (light demand), not the passed u8AveLum.
     * u8AveLum collapses into the dead zone once the night profile is applied
     * (see the ISO thresholds above); ISO reflects how much the AE has to
     * amplify and stays monotonic with darkness across day/night profiles. */
    ISP_EXP_INFO_S exp;
    if (HI_MPI_ISP_QueryExposureInfo(0, &exp) != HI_SUCCESS)
        return;
    HI_U32 iso = exp.u32Iso;

    LOGGER(LOGGER_LEVEL_DEBUG, "[board][mjsxj02hl] iso=%u luma=%u mode=%s consec_n=%d consec_d=%d",
           iso, luma, s_current_mode ? "day" : "night", s_consec_night, s_consec_day);

    if (iso > MJSXJ02HL_NIGHT_ISO) {          /* dark */
        s_consec_night++;
        s_consec_day = 0;
    } else if (iso < MJSXJ02HL_DAY_ISO) {     /* lit */
        s_consec_day++;
        s_consec_night = 0;
    } else {                                   /* ambiguous gap */
        s_consec_night = 0;
        s_consec_day   = 0;
    }

    if (s_current_mode == 1 && s_consec_night >= MJSXJ02HL_HYSTERESIS) {
        s_consec_night = 0;
        s_settling     = MJSXJ02HL_SETTLE_S;
        board_apply_mode(HI_TRUE);
    } else if (s_current_mode == 0 && s_consec_day >= MJSXJ02HL_HYSTERESIS) {
        s_consec_day = 0;
        s_settling   = MJSXJ02HL_SETTLE_S;
        board_apply_mode(HI_FALSE);
    }
}

static void mjsxj02hl_deinit(void)
{
    const board_cfg_t *b = &g_board_mjsxj02hl;
    board_gpio_write(b->gpio_ir_led_a, 0);
    if (b->gpio_ir_led_b >= 0) board_gpio_write(b->gpio_ir_led_b, 0);
    board_gpio_write(b->gpio_ircut_a, 0);
    board_gpio_write(b->gpio_ircut_b, 0);
    LOGGER(LOGGER_LEVEL_INFO, "[board][mjsxj02hl] deinit: GPIO cleared");
}

/* -------------------------------------------------------------------------
 * Board instance
 * ---------------------------------------------------------------------- */

const board_cfg_t g_board_mjsxj02hl = {
    .name                  = "MJSXJ02HL",

    .pfnBringupSensor      = mjsxj02hl_bringup_sensor,
    .pfnGetVi              = mjsxj02hl_get_vi,
    .pfnTeardownSensorIsp  = mjsxj02hl_teardown_sensor_isp,
    .pfnInit               = mjsxj02hl_init,
    .pfnOnLuma             = mjsxj02hl_on_luma,
    .pfnDeinit             = mjsxj02hl_deinit,

    /* GPIO pin numbers (sysfs index), from runtime trace; -1 = not present */
    .gpio_ir_led_a       = 52,   /* IR LED A */
    .gpio_ir_led_b       = 53,   /* IR LED B */
    .gpio_ircut_a        = 70,   /* IR-cut motor A */
    .gpio_ircut_b        = 68,   /* IR-cut motor B */
    .gpio_led_orange     = 16,   /* Orange indicator */
    .gpio_led_blue       = 55,   /* Blue indicator */
    .gpio_button_setup   = 0,    /* Reset/setup button (input) */
    .gpio_photo_sensor   = 9,    /* Photo-sensitive sensor (input) */
    .gpio_ircut_step_us  = 50000,

    .ai_dev = 0,
    .ao_dev = 0,

    .vb_main_blk_cnt     = 1,
    .vb_sub_blk_cnt      = 5,
    /* 416 lines: original firmware value (inner_GetWrapBufLine_By_SnsType).
       VB block ≈1.17 MB × 1; saves ~1.8 MB vs full-frame (vs cnt=2).
       Requires VI_ONLINE_VPSS_ONLINE. */
    .vb_main_wrap_lines  = 416,

    .ivp_oms_path        = "/usr/app/local/ivp_re_im_allday_16chn_pr1_640x360_v1040.oms",
};

const board_cfg_t *platform_get_board_cfg(void) {
    return &g_board_mjsxj02hl;
}

/* -------------------------------------------------------------------------
 * Board notification / control API (declared in platform.h)
 * ---------------------------------------------------------------------- */

void board_register_mode_change_cb(void (*cb)(HI_BOOL is_night))
{
    s_mode_change_cb = cb;
}

void board_set_mode(HI_BOOL is_night)
{
    board_apply_mode(is_night);
}
