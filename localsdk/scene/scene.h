#ifndef _SCENE_H_
#define _SCENE_H_

#include "hi_type.h"
#include "../platform/platform.h"   /* sensor_calib_t */

/* Parse day and night scene INI files. Must be called once after sensor fps is set.
 * target_fps: board operating framerate (e.g. 20); used to cap AE route EX IntTime. */
int scene_init(const char *day_ini, const char *night_ini, HI_U32 target_fps);

/* Fill the sensor calibration (day [static_awb]) for the board to push into the
 * cmos before sensor registration. Returns 0 on success (scene_init must run first). */
int scene_get_awb_calib(sensor_calib_t *out);

/* Apply day-mode ISP parameters (color, normal AE range). */
int scene_set_day(void);

/* Apply night-mode ISP parameters (grayscale, lower AE gain cap). */
int scene_set_night(void);

#endif /* _SCENE_H_ */
