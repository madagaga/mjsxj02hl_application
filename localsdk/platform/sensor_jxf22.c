/* SOI JXF22 sensor configuration.
 * Values captured from the original liblocalsdk.so runtime trace (trace.txt)
 * and cross-checked against the 3516ev200 sample (DEV_ATTR_SC2231 /
 * PIPE_ATTR_1920x1080_RAW10).
 * 2-lane MIPI, RAW10, 1080p30 linear, VI online -> VPSS offline.
 */

#include "platform.h"

/* Sensor object exported by libsns_f22.so */
extern ISP_SNS_OBJ_S stSnsSoiSensorObj;

static const sensor_cfg_t g_sensor_jxf22 = {
    .name = "SOI JXF22",

    .mipi_attr = {
        .devno      = 0,
        .input_mode = INPUT_MODE_MIPI,
        .data_rate  = MIPI_DATA_RATE_X1,
        .img_rect   = {0, 0, 1920, 1080},
        .mipi_attr  = {
            DATA_TYPE_RAW_10BIT,
            HI_MIPI_WDR_MODE_NONE,
            {0, 2, -1, -1}
        }
    },

    .vi_dev_attr = {
        VI_MODE_MIPI,
        VI_WORK_MODE_1Multiplex,
        {0xFFC00000, 0x0},
        VI_SCAN_PROGRESSIVE,
        { -1, -1, -1, -1},
        VI_DATA_SEQ_YUYV,
        {
            VI_VSYNC_PULSE, VI_VSYNC_NEG_LOW,
            VI_HSYNC_VALID_SINGNAL, VI_HSYNC_NEG_HIGH,
            VI_VSYNC_VALID_SINGAL, VI_VSYNC_VALID_NEG_HIGH,
            {
                0, 1920, 0,
                0, 1080, 0,
                0, 0,    0
            }
        },
        VI_DATA_TYPE_RGB,
        HI_FALSE,
        {1920, 1080},
        {
            { {1920, 1080} },
            { VI_REPHASE_MODE_NONE, VI_REPHASE_MODE_NONE }
        },
        {
            WDR_MODE_NONE,
            1080
        },
        DATA_RATE_X1
    },

    .vi_pipe_attr = {
        VI_PIPE_BYPASS_NONE, HI_FALSE, HI_FALSE,
        1920, 1080,
        PIXEL_FORMAT_RGB_BAYER_10BPP,
        COMPRESS_MODE_NONE,
        DATA_BITWIDTH_10,
        HI_TRUE,  /* bNrEn: VI-pipe level NR */
        {
            PIXEL_FORMAT_YVU_SEMIPLANAR_420,
            DATA_BITWIDTH_8,
            VI_NR_REF_FROM_RFR,
            COMPRESS_MODE_NONE
        },
        HI_FALSE,
        { -1, -1 }
    },

    .isp_pub_attr = {
        .stWndRect    = {0, 0, 1920, 1080},
        .stSnsSize    = {1920, 1080},
        .f32FrameRate = 30.0f,   /* sensor native; app retimes to 20 fps via VPSS */
        .enBayer      = BAYER_BGGR,  /* TODO: confirm on real image (color cast check) */
        .enWDRMode    = WDR_MODE_NONE,
        .u8SnsMode    = 0
    },

    .p_sns_obj  = &stSnsSoiSensorObj,
    .sensor_fps = 20,
};

const sensor_cfg_t *platform_get_sensor_cfg(void) {
    return &g_sensor_jxf22;
}
