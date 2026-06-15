#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "hi_type.h"
#include "hi_comm_vi.h"    /* VI_DEV / VI_PIPE / VI_CHN */
#include "hi_comm_aio.h"   /* AUDIO_DEV */

/* -------------------------------------------------------------------------
 * Generic sensor configuration pushed by the board to its sensor.
 * Carries only what the board/sensor couple may tune at bring-up time; the
 * intrinsic sensor attributes (MIPI/VI/ISP) live inside the sensor module.
 * ---------------------------------------------------------------------- */
typedef struct {
    HI_U32  fps;      /* target output fps (sensor VMAX retiming) */
    HI_BOOL mirror;
    HI_BOOL flip;
} sensor_config_t;

/* -------------------------------------------------------------------------
 * Board interface — the ONLY thing localsdk consumes. localsdk is blind to
 * the sensor: it never names the sensor type, only calls these callbacks and
 * reads the BOARD_* compile-time constants (see board_*.h, included below).
 *
 * Two channels board -> localsdk:
 *   - #define BOARD_*  : fixed sizing constants (VB/VPSS), compile-time.
 *   - callbacks below  : runtime behaviour (sensor bring-up, day/night...).
 * ---------------------------------------------------------------------- */
typedef struct {
    const char          *name;

    /* Sensor bring-up — the board owns and configures its sensor.
       Pushes orientation (default = board's physical mount correction, may be
       overridden by the app) + target fps, then brings up MIPI/VI/ISP. */
    HI_S32 (*pfnBringupSensor)(HI_BOOL mirror, HI_BOOL flip);
    /* Expose the VI handles created at bring-up (for VI<->VPSS bind/teardown). */
    void   (*pfnGetVi)(VI_DEV *dev, VI_PIPE *pipe, VI_CHN *chn);
    /* Tear down the sensor ISP (run thread + unregister), called at shutdown. */
    void   (*pfnTeardownSensorIsp)(void);

    /* Board lifecycle */
    HI_S32 (*pfnInit)(void);       /* scene ISP params + GPIO initial state */
    void   (*pfnOnLuma)(HI_U8 luma); /* day/night state machine (1s luma tick) */
    void   (*pfnDeinit)(void);     /* GPIO cleared */

    /* GPIO pin numbers (sysfs index); -1 = not present on this board */
    int            gpio_ir_led_a;
    int            gpio_ir_led_b;
    int            gpio_ircut_a;
    int            gpio_ircut_b;
    int            gpio_led_orange;
    int            gpio_led_blue;
    int            gpio_button_setup;
    int            gpio_photo_sensor;
    int            gpio_ircut_step_us;

    /* Hisilicon audio device indices */
    AUDIO_DEV      ai_dev;
    AUDIO_DEV      ao_dev;

    /* VB pool block counts (tune to available MMZ) */
    HI_U32         vb_main_blk_cnt;
    HI_U32         vb_sub_blk_cnt;
    /* Wrap buffer lines for VPSS chn0 ring mode (0 = disable wrap). */
    HI_U32         vb_main_wrap_lines;

    /* IVP .oms model path (sub-channel resolution must match model) */
    const char    *ivp_oms_path;
} board_cfg_t;

/* Board exported by board_*.c */
const board_cfg_t *platform_get_board_cfg(void);

/* Register a callback invoked by the board on day/night mode transitions.
   night.c registers a function that publishes MQTT + toggles alarm detection.
   Pass NULL to deregister. Only one callback is supported at a time. */
void board_register_mode_change_cb(void (*cb)(HI_BOOL is_night));

/* Force the board into a specific day/night mode immediately (bypasses luma
   detection). Used by night.c for forced-mode config (night.mode=0/1). */
void board_set_mode(HI_BOOL is_night);

/* Active board header: provides BOARD_* sizing constants read by localsdk.
   Included last so the board can relay its sensor's #defines. Swap this (and
   the Makefile rules) to target a different camera. */
#include "board_mjsxj02hl.h"

#endif /* _PLATFORM_H_ */
