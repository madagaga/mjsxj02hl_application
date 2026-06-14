#ifndef _SCENE_H_
#define _SCENE_H_

#include "hi_type.h"

/* Parse day and night scene INI files. Must be called once after ISP init.
 * sensor_fps: actual sensor framerate (e.g. 20); used to cap AE max IntTime. */
int scene_init(const char *day_ini, const char *night_ini, HI_U32 sensor_fps);

/* Apply day-mode ISP parameters (color, normal AE range). */
int scene_set_day(void);

/* Apply night-mode ISP parameters (grayscale, lower AE gain cap). */
int scene_set_night(void);

#endif /* _SCENE_H_ */
