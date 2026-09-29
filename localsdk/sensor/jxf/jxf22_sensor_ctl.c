/* JXF22 sensor I2C control and power-on register sequence.
 * Ported from soi_f22__soc_v3 (glutinium) to hi3516ev200 ISP_SNS_OBJ_S model.
 * Init table: sensor_linear_1080p30_init — MCLK 24 MHz, 2400×1134, 30 FPS.
 * I2C bus number set by pfnSetBusInfo → stored in g_i2cDev, opened on demand.
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#include "jxf22.h"

const unsigned int sensor_i2c_addr = 0x80; /* 8-bit I2C address (bit0 = R/W) */
unsigned int sensor_addr_byte      = 1;
unsigned int sensor_data_byte      = 1;

WDR_MODE_E genSensorMode    = WDR_MODE_NONE;
HI_U8      gu8SensorImageMode = SENSOR_1080P_30FPS_MODE;
HI_BOOL    bSensorInit      = HI_FALSE;

ISP_SNS_REGS_INFO_S g_stSnsRegsInfo    = {0};
ISP_SNS_REGS_INFO_S g_stPreSnsRegsInfo = {0};

static HI_S8 g_i2cDev = 0;
static int   g_fd     = -1;

HI_S32 sensor_i2c_init(HI_S8 i2cDev)
{
    char devPath[32];
    int ret;

    if (g_fd >= 0)
        return HI_SUCCESS;

    g_i2cDev = i2cDev;
    snprintf(devPath, sizeof(devPath), "/dev/i2c-%d", (int)i2cDev);

    g_fd = open(devPath, O_RDWR);
    if (g_fd < 0) {
        printf("[sns][jxf22] open %s failed\n", devPath);
        return HI_FAILURE;
    }

    ret = ioctl(g_fd, I2C_SLAVE_FORCE, (sensor_i2c_addr >> 1));
    if (ret < 0) {
        printf("[sns][jxf22] I2C_SLAVE_FORCE failed\n");
        close(g_fd);
        g_fd = -1;
        return HI_FAILURE;
    }

    printf("[sns][jxf22] i2c-%d opened (addr=0x%02x)\n",
           (int)i2cDev, sensor_i2c_addr);
    return HI_SUCCESS;
}

HI_S32 sensor_i2c_exit(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    return HI_SUCCESS;
}

HI_S32 sensor_write_register(HI_S32 addr, HI_S32 data)
{
    unsigned char buf[2];
    int ret;

    if (g_fd < 0)
        return HI_FAILURE;

    buf[0] = (unsigned char)(addr & 0xff);
    buf[1] = (unsigned char)(data & 0xff);

    ret = write(g_fd, buf, 2);
    if (ret < 0) {
        printf("[sns][jxf22] I2C write 0x%02x=0x%02x failed\n", addr, data);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

HI_S32 sensor_read_register(HI_S32 addr)
{
    unsigned char regbuf[1];
    unsigned char databuf[1];

    if (g_fd < 0)
        return -1;

    regbuf[0] = (unsigned char)(addr & 0xff);
    if (write(g_fd, regbuf, 1) < 0)
        return -1;
    if (read(g_fd, databuf, 1) < 0)
        return -1;

    return (int)databuf[0];
}

static void delay_ms(int ms)
{
    usleep((unsigned int)ms * 1000);
}

/* 1080p30 linear register table — MCLK 24 MHz, 2400×1134, 30 FPS.
 * Source: soi_f22__soc_v3 / glutinium project. */
HI_VOID sensor_linear_1080p30_init(HI_VOID)
{
    sensor_write_register(0x12, 0x40);
    sensor_write_register(0x0E, 0x11);
    sensor_write_register(0x0F, 0x0C);
    sensor_write_register(0x10, 0x44);
    sensor_write_register(0x11, 0x80);
    sensor_write_register(0x5F, 0x01);
    sensor_write_register(0x60, 0x0A);
    sensor_write_register(0x19, 0x20);
    sensor_write_register(0x48, 0x0A);
    sensor_write_register(0x20, 0xB0);
    sensor_write_register(0x21, 0x04);
    sensor_write_register(0x22, 0x6E); /* VMAX_L: 1134 = 0x046E */
    sensor_write_register(0x23, 0x04); /* VMAX_H */
    sensor_write_register(0x24, 0xC0);
    sensor_write_register(0x25, 0x38);
    sensor_write_register(0x26, 0x43);
    sensor_write_register(0x27, 0xC9);
    sensor_write_register(0x28, 0x18);
    sensor_write_register(0x29, 0x01);
    sensor_write_register(0x2A, 0xC0);
    sensor_write_register(0x2B, 0x21);
    sensor_write_register(0x2C, 0x02);
    sensor_write_register(0x2D, 0x00);
    sensor_write_register(0x2E, 0x16);
    sensor_write_register(0x2F, 0x44);
    sensor_write_register(0x41, 0xD0);
    sensor_write_register(0x42, 0x03);
    sensor_write_register(0x39, 0x90);
    sensor_write_register(0x1D, 0x00);
    sensor_write_register(0x1E, 0x04);
    sensor_write_register(0x6C, 0x40);
    sensor_write_register(0x70, 0x89);
    sensor_write_register(0x71, 0x4A);
    sensor_write_register(0x72, 0x68);
    sensor_write_register(0x73, 0x43);
    sensor_write_register(0x74, 0x52);
    sensor_write_register(0x75, 0x2B);
    sensor_write_register(0x76, 0x60);
    sensor_write_register(0x77, 0x09);
    sensor_write_register(0x78, 0x1B);
    sensor_write_register(0x30, 0x8C);
    sensor_write_register(0x31, 0x0C);
    sensor_write_register(0x32, 0xF0);
    sensor_write_register(0x33, 0x0C);
    sensor_write_register(0x34, 0x1F);
    sensor_write_register(0x35, 0xE3);
    sensor_write_register(0x36, 0x0E);
    sensor_write_register(0x37, 0x34);
    sensor_write_register(0x38, 0x13);
    sensor_write_register(0x3A, 0x08);
    sensor_write_register(0x3B, 0x30);
    sensor_write_register(0x3C, 0xC0);
    sensor_write_register(0x3D, 0x00);
    sensor_write_register(0x3E, 0x00);
    sensor_write_register(0x3F, 0x00);
    sensor_write_register(0x40, 0x00);
    sensor_write_register(0x6F, 0x03);
    sensor_write_register(0x0D, 0x64);
    sensor_write_register(0x56, 0x32);
    sensor_write_register(0x5A, 0x20);
    sensor_write_register(0x5B, 0xB3);
    sensor_write_register(0x5C, 0xF7);
    sensor_write_register(0x5D, 0xF0);
    sensor_write_register(0x62, 0x80);
    sensor_write_register(0x63, 0x80);
    sensor_write_register(0x64, 0x00);
    sensor_write_register(0x67, 0x75);
    sensor_write_register(0x68, 0x00);
    sensor_write_register(0x6A, 0x4D);
    sensor_write_register(0x8F, 0x18);
    sensor_write_register(0x91, 0x04);
    sensor_write_register(0x0C, 0x00);
    sensor_write_register(0x59, 0x97);
    sensor_write_register(0x4A, 0x05);
    sensor_write_register(0x49, 0x10);
    sensor_write_register(0x50, 0x02);
    sensor_write_register(0x47, 0x22);
    sensor_write_register(0x7E, 0xCD);
    sensor_write_register(0x7F, 0x52);
    sensor_write_register(0x7B, 0x57);
    sensor_write_register(0x7C, 0x28);
    sensor_write_register(0x80, 0x00);
    sensor_write_register(0x13, 0x81);
    sensor_write_register(0x74, 0x53);
    sensor_write_register(0x12, 0x00); /* release reset */
    sensor_write_register(0x74, 0x52);
    sensor_write_register(0x93, 0x5C);
    sensor_write_register(0x45, 0x89);
    delay_ms(500);
    sensor_write_register(0x45, 0x09);
    sensor_write_register(0x1F, 0x01);

    printf("[sns][jxf22] sensor_linear_1080p30_init done\n");
}

HI_VOID sensor_init(VI_PIPE ViPipe)
{
    (void)ViPipe;

    sensor_i2c_init(g_i2cDev);

    if (HI_FALSE == bSensorInit) {
        if (WDR_MODE_NONE == genSensorMode &&
            SENSOR_1080P_30FPS_MODE == gu8SensorImageMode) {
            sensor_linear_1080p30_init();
            bSensorInit = HI_TRUE;
        }
    } else {
        if (WDR_MODE_NONE == genSensorMode &&
            SENSOR_1080P_30FPS_MODE == gu8SensorImageMode) {
            sensor_linear_1080p30_init();
        }
    }
}

HI_VOID sensor_exit(VI_PIPE ViPipe)
{
    (void)ViPipe;
    sensor_i2c_exit();
}

HI_VOID sensor_set_fps(HI_U32 fps)
{
    HI_U32 vmax;
    if (fps == 0) return;
    vmax = (HI_U32)(VMAX_1080P30_LINEAR * 30) / fps;
    if (vmax > FULL_LINES_MAX) vmax = FULL_LINES_MAX;
    sensor_write_register(VMAX_ADDR_L, (HI_S32)(vmax & 0xff));
    sensor_write_register(VMAX_ADDR_H, (HI_S32)((vmax >> 8) & 0xff));
    printf("[sns][jxf22] set_fps %u: VMAX=%u (0x%04x) reg[0x22]=0x%02x reg[0x23]=0x%02x\n",
           fps, vmax, vmax, vmax & 0xff, (vmax >> 8) & 0xff);
}
