#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "hi_type.h"
#include "hi_mipi.h"
#include "hi_comm_vi.h"
#include "hi_comm_isp.h"
#include "hi_sns_ctrl.h"
#include "hi_comm_aio.h"

/* All parameters that are specific to the image sensor fitted on the board.
   Values come from the sensor datasheet + the MIPI/VI/ISP bring-up sequence. */
typedef struct {
    const char        *name;
    combo_dev_attr_t   mipi_attr;
    VI_DEV_ATTR_S      vi_dev_attr;
    VI_PIPE_ATTR_S     vi_pipe_attr;
    ISP_PUB_ATTR_S     isp_pub_attr;   /* includes Bayer pattern, WDR mode, fps */
    ISP_SNS_OBJ_S     *p_sns_obj;      /* libsns_*.so sensor object */
    HI_U32             sensor_fps;     /* application target framerate */
} sensor_cfg_t;

/* All parameters that are specific to the camera board design (PCB/hardware).
   Swap this file to port to a different camera using the same sensor. */
typedef struct {
    const char    *name;
    /* GPIO pin numbers (sysfs index) */
    int            gpio_ir_led_a;
    int            gpio_ir_led_b;
    int            gpio_ircut_a;
    int            gpio_ircut_b;
    int            gpio_led_orange;
    int            gpio_led_blue;
    int            gpio_button_setup;
    int            gpio_photo_sensor;
    int            gpio_ircut_step_us; /* delay between IR-cut motor pulses */
    /* Sensor mounting orientation: base flip/mirror before user config XOR */
    HI_BOOL        default_mirror;
    HI_BOOL        default_flip;
    /* Hisilicon audio device indices */
    AUDIO_DEV      ai_dev;
    AUDIO_DEV      ao_dev;
    /* VB pool block counts (tune to available MMZ) */
    HI_U32         vb_main_blk_cnt;
    HI_U32         vb_sub_blk_cnt;
    /* IVP .oms model path (sub-channel resolution must match model) */
    const char    *ivp_oms_path;
} board_cfg_t;

const sensor_cfg_t *platform_get_sensor_cfg(void);
const board_cfg_t  *platform_get_board_cfg(void);

#endif /* _PLATFORM_H_ */
