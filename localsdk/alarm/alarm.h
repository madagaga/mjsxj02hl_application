#ifndef _LOCALSDK_ALARM_H_
#define _LOCALSDK_ALARM_H_

#include <stdbool.h>
#include <stdint.h>
#include "../common.h"

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define LOCALSDK_ALARM_TYPE_MOTION       1
#define LOCALSDK_ALARM_TYPE_HUMANOID     7

#define LOCALSDK_ALARM_MAXIMUM_OBJECTS   4

/* ── Types ──────────────────────────────────────────────────────────────────── */

typedef struct {
    uint32_t state;
    uint32_t type;
    struct {
        uint32_t type;
        uint32_t state;
        uint32_t x;
        uint32_t width;
        uint32_t y;
        uint32_t height;
        uint32_t reserved[11];
    } objects[LOCALSDK_ALARM_MAXIMUM_OBJECTS];
} LOCALSDK_ALARM_EVENT_INFO;

/* ── Functions ──────────────────────────────────────────────────────────────── */

bool alarm_init(void);
bool alarm_switch(bool state);
bool alarm_free(void);

#endif /* _LOCALSDK_ALARM_H_ */
