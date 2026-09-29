#ifndef _SENSOR_JXF22_H_
#define _SENSOR_JXF22_H_

/* SOI JXF22 sensor module — generic & reusable.
 * Owns ALL sensor-coupled MPP bring-up (MIPI + VI + ISP). The board pushes
 * the couple-specific values via sensor_jxf22_configure() then triggers
 * sensor_jxf22_bringup(). localsdk never includes this header. */

#include "../../platform/platform.h"  /* sensor_config_t */
#include "sensor_jxf22_defs.h"         /* JXF22_* defaults */

/* Push the board/sensor-couple configuration (target fps, orientation).
   Must be called before sensor_jxf22_bringup(). */
void   sensor_jxf22_configure(const sensor_config_t *cfg);

/* Full sensor bring-up: MIPI ioctls + VI dev/pipe/chn + ISP register/pub/run
   thread, then apply fps (VMAX) and mirror/flip at the sensor. */
HI_S32 sensor_jxf22_bringup(void);

/* Expose the VI handles created during bring-up (for VI<->VPSS bind/teardown). */
void   sensor_jxf22_get_vi(VI_DEV *dev, VI_PIPE *pipe, VI_CHN *chn);

/* ISP teardown: stop ISP run thread + unregister sensor callback. */
void   sensor_jxf22_teardown_isp(void);

#endif /* _SENSOR_JXF22_H_ */
