#ifndef _LOCALSDK_OSD_H_
#define _LOCALSDK_OSD_H_

#include <stdbool.h>
#include <stdint.h>
#include "./../alarm/alarm.h"  /* LOCALSDK_ALARM_EVENT_INFO, LOCALSDK_ALARM_MAXIMUM_OBJECTS */

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define LOCALSDK_OSD_COLOR_GREEN  3
#define LOCALSDK_OSD_COLOR_ORANGE 5

/* ── Types ──────────────────────────────────────────────────────────────────── */

typedef struct {
    uint32_t flags;
    uint32_t datetime_x;
    uint32_t datetime_y;
    uint32_t datetime_reduce;
    uint32_t datetime_increase;
    uint32_t oemlogo_x;
    uint32_t oemlogo_y;
    uint32_t oemlogo_reduce;
    uint32_t oemlogo_increase;
} LOCALSDK_OSD_OPTIONS;

typedef struct {
    uint32_t count;
    struct {
        uint32_t x;
        uint32_t width;
        uint32_t y;
        uint32_t height;
        uint32_t visible;
        uint32_t color;
    } objects[LOCALSDK_ALARM_MAXIMUM_OBJECTS];
} LOCALSDK_OSD_RECTANGLES;

/* ── Functions ──────────────────────────────────────────────────────────────── */

bool osd_is_enabled(void);
bool osd_init(void);
bool osd_postinit(void);
bool osd_free(void);
int  osd_rectangles_callback(LOCALSDK_ALARM_EVENT_INFO *eventInfo);

#endif /* _LOCALSDK_OSD_H_ */
