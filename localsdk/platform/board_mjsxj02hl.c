/* Board configuration for Xiaomi MJSXJ02HL.
 * GPIO assignments and hardware layout from runtime trace (trace.txt).
 * The JXF22 sensor is mounted rotated 180° on this PCB; default_mirror/flip=TRUE
 * trigger a pfnMirrorFlip(ISP_SNS_MIRROR_FLIP) call in local_sdk_video_init.
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

    /* Sensor is soldered upside-down; pfnMirrorFlip(ISP_SNS_MIRROR_FLIP) applied at ISP init */
    .default_mirror    = HI_TRUE,
    .default_flip      = HI_TRUE,

    /* 20fps: board decision — larger VMAX (1701 vs 1134) allows 50ms max IntTime vs 33ms.
       INI files are named *_1080p20_*, VB pool is sized for 20fps. */
    .target_fps        = 20,

    /* Inner codec (hi3518ev300): dev 0 is the only valid audio device */
    .ai_dev = 0,
    .ao_dev = 0,

    /* VB pool block counts. cnt=2 for pipeline overlap (one block written, one consumed). */
    .vb_main_blk_cnt = 2,
    .vb_sub_blk_cnt  = 5,

    /* Wrap mode: 416 lines matches original firmware (inner_GetWrapBufLine_By_SnsType).
       VB block size = VPSS_GetWrapBufferSize(1920, 1080, 416) = 1198080 B ≈ 1.17 MB
       instead of ~3.14 MB full-frame; 2 blocks saves ~4 MB on VB pool.
       3DNR VPSS(0) Ref buffer takes back 2.2 MB → net savings ~1.8 MB vs full-frame
       without 3DNR. Confirmed on real hardware: MMZ total 19 MB, used 16.6 MB, free 2.4 MB.
       Requires VI_ONLINE_VPSS_ONLINE; in OFFLINE mode wrap output reaches VENC silently
       but VENC receives no frames. */
    .vb_main_wrap_lines = 416,

    /* IVP model: must match sub-channel resolution (640x360) */
    .ivp_oms_path = "/usr/app/local/ivp_re_im_allday_16chn_pr1_640x360_v1040.oms",

    /* libsceneauto.so computes its config path from the binary location.
     * chdir here before sceneauto_init() so the resolution yields this dir. */
    .scene_ini_dir = "/usr/app/local/sensor.ini",
};

const board_cfg_t *platform_get_board_cfg(void) {
    return &g_board_mjsxj02hl;
}
