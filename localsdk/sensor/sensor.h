#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <stdbool.h>

#include "hi_type.h"

/* -------------------------------------------------------------------------
 * Generic sensor configuration pushed by the board to its sensor.
 * Carries only what the board/sensor couple may tune at bring-up time; the
 * intrinsic sensor attributes (MIPI/VI/ISP) live inside the sensor module.
 *
 * It lives here, not in platform.h: a sensor module must not include the
 * board layer (the board knows the sensor, never the other way round).
 * ---------------------------------------------------------------------- */
typedef struct {
    HI_U32  fps;      /* target output fps (sensor VMAX retiming) */
    HI_BOOL mirror;
    HI_BOOL flip;
} sensor_config_t;

#endif /* _SENSOR_H_ */
