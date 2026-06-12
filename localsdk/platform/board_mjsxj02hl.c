/* Board configuration for Xiaomi MJSXJ02HL.
 * GPIO assignments and hardware layout from runtime trace (trace.txt).
 * The JXF22 sensor is mounted rotated 180° on this PCB, hence default
 * mirror + flip = HI_TRUE (XOR'd with user mirror/flip options).
 */

#include "platform.h"

static const board_cfg_t g_board_mjsxj02hl = {
    .name = "MJSXJ02HL",

    /* GPIO pin numbers (sysfs index), from runtime trace lines 10-39 */
    .gpio_ir_led_a     = 52,   /* IR LED A,         init=1 */
    .gpio_ir_led_b     = 53,   /* IR LED B,         init=0 */
    .gpio_ircut_a      = 70,   /* IR-cut motor A,   init=0 */
    .gpio_ircut_b      = 68,   /* IR-cut motor B,   init=0 */
    .gpio_led_orange   = 16,   /* Orange indicator, init=0 */
    .gpio_led_blue     = 55,   /* Blue indicator,   init=1 */
    .gpio_button_setup = 0,    /* Reset/setup button (input) */
    .gpio_photo_sensor = 9,    /* Photo-sensitive sensor (input) */
    .gpio_ircut_step_us = 50000, /* 50ms between IR-cut motor pulses */

    /* Sensor is soldered upside-down; base 180° correction before user XOR */
    .default_mirror    = HI_TRUE,
    .default_flip      = HI_TRUE,

    /* Inner codec (hi3518ev300): dev 0 is the only valid audio device */
    .ai_dev = 0,
    .ao_dev = 0,

    /* VB pool block counts.
     * MMZ is ~19 MB (mem=45M of 64 MB). Keep counts low to leave headroom
     * for VENC reference/reconstruction frames. */
    .vb_main_blk_cnt = 2,
    .vb_sub_blk_cnt  = 2,

    /* IVP model: must match sub-channel resolution (640x360) */
    .ivp_oms_path = "/usr/app/local/ivp_re_im_allday_16chn_pr1_640x360_v1040.oms",
};

const board_cfg_t *platform_get_board_cfg(void) {
    return &g_board_mjsxj02hl;
}
