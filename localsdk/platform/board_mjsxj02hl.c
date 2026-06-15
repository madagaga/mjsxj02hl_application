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

#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"
#include "../sensor/jxf/sensor_jxf22.h"
#include "../scene/scene.h"
#include "../../logger/logger.h"

/* Board-specific constants (BOARD_TARGET_FPS lives in board_mjsxj02hl.h) */
#define MJSXJ02HL_NIGHT_LUM    30     /* u8AveLum below this → night candidate */
#define MJSXJ02HL_DAY_LUM      60     /* u8AveLum above this → day candidate */
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

/* Board owns and configures its sensor. The orientation (mirror/flip) comes from
   the app (default in .conf = 180° correction for the PCB mount, user-overridable);
   the board forwards it together with its target fps to the sensor module. */
static HI_S32 mjsxj02hl_bringup_sensor(HI_BOOL mirror, HI_BOOL flip)
{
    sensor_config_t cfg = {
        .fps    = BOARD_TARGET_FPS,
        .mirror = mirror,
        .flip   = flip,
    };
    sensor_jxf22_configure(&cfg);
    LOGGER(LOGGER_LEVEL_INFO, "[board][mjsxj02hl] bringup sensor: fps=%u mirror=%d flip=%d",
           BOARD_TARGET_FPS, (int)mirror, (int)flip);
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

    /* GPIO initial state: day (IR LED off, IR-cut motor relaxed) */
    board_gpio_write(b->gpio_ir_led_a, 0);
    if (b->gpio_ir_led_b >= 0) board_gpio_write(b->gpio_ir_led_b, 0);
    board_gpio_write(b->gpio_ircut_a, 0);
    board_gpio_write(b->gpio_ircut_b, 0);

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

    LOGGER(LOGGER_LEVEL_DEBUG, "[board][mjsxj02hl] luma=%u mode=%s consec_n=%d consec_d=%d",
           luma, s_current_mode ? "day" : "night", s_consec_night, s_consec_day);

    if (luma < MJSXJ02HL_NIGHT_LUM) {
        s_consec_night++;
        s_consec_day = 0;
    } else if (luma > MJSXJ02HL_DAY_LUM) {
        s_consec_day++;
        s_consec_night = 0;
    } else {
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

    .vb_main_blk_cnt     = 2,
    .vb_sub_blk_cnt      = 5,
    /* 416 lines: original firmware value (inner_GetWrapBufLine_By_SnsType).
       VB block ≈1.17 MB × 2; saves ~1.8 MB vs full-frame.
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
