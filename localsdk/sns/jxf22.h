#ifndef __JXF22_H__
#define __JXF22_H__

#include "hi_type.h"
#include "hi_comm_isp.h"
#include "hi_comm_sns.h"
#include "hi_sns_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sensor I2C parameters */
extern const unsigned int sensor_i2c_addr;   /* 0x80 (8-bit, bit0=R/W) */
extern unsigned int sensor_addr_byte;         /* 1 */
extern unsigned int sensor_data_byte;         /* 1 */

/* Sensor mode globals (shared between cmos and ctl) */
extern WDR_MODE_E genSensorMode;
extern HI_U8      gu8SensorImageMode;
extern HI_BOOL    bSensorInit;

/* AE shadow-register buffers filled by cmos_inttime_update / cmos_gains_update.
   The ISP kernel driver reads these at VD interrupt time and writes I2C. */
extern ISP_SNS_REGS_INFO_S g_stSnsRegsInfo;
extern ISP_SNS_REGS_INFO_S g_stPreSnsRegsInfo;

#define SENSOR_1080P_30FPS_MODE (1)
#define VMAX_1080P30_LINEAR     (1134)
#define FULL_LINES_MAX          (0xFFFF)

/* Register addresses */
#define AGC_ADDR        (0x00)
#define EXPOSURE_ADDR_L (0x01)
#define EXPOSURE_ADDR_H (0x02)
#define VMAX_ADDR_L     (0x22)
#define VMAX_ADDR_H     (0x23)

/* Shadow-register indices (must match cmos_get_sns_regs_info init order) */
#define SNS_REG_EXPO_L  0
#define SNS_REG_EXPO_H  1
#define SNS_REG_AGC     2
#define SNS_REG_VMAX_L  3
#define SNS_REG_VMAX_H  4
#define SNS_REG_NUM     5

/* I2C control */
HI_S32 sensor_i2c_init(HI_S8 i2cDev);
HI_S32 sensor_i2c_exit(void);
HI_S32 sensor_write_register(HI_S32 addr, HI_S32 data);
HI_S32 sensor_read_register(HI_S32 addr);

/* Sensor power-on sequence */
HI_VOID sensor_linear_1080p30_init(HI_VOID);
HI_VOID sensor_init(VI_PIPE ViPipe);
HI_VOID sensor_exit(VI_PIPE ViPipe);

#ifdef __cplusplus
}
#endif

#endif /* __JXF22_H__ */
