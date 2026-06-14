#ifndef _SCENE_H_
#define _SCENE_H_

/* Parse day and night scene INI files. Must be called once after ISP init. */
int scene_init(const char *day_ini, const char *night_ini);

/* Apply day-mode ISP parameters (color, normal AE range). */
int scene_set_day(void);

/* Apply night-mode ISP parameters (grayscale, lower AE gain cap). */
int scene_set_night(void);

#endif /* _SCENE_H_ */
