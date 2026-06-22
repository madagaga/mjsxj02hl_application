#ifndef _LOCALSDK_SPEAKER_H_
#define _LOCALSDK_SPEAKER_H_

#include <stdbool.h>
#include <stdint.h>
#include "../common.h"

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define LOCALSDK_SPEAKER_SAMPLE_RATE 8000
#define LOCALSDK_SPEAKER_BIT_DEPTH   16
#define LOCALSDK_SPEAKER_TRACK_TYPE  1

#define LOCALSDK_SPEAKER_PCM_TYPE    1
#define LOCALSDK_SPEAKER_G711_TYPE   2

#define SPEAKER_MEDIA_STOPPED  0
#define SPEAKER_MEDIA_PLAYING  1
#define SPEAKER_MEDIA_STOPPING 2

/* ── Types ──────────────────────────────────────────────────────────────────── */

typedef struct {
    uint32_t sample_rate;
    uint32_t bit_depth;
    uint32_t points_per_frame;
    uint32_t track_type;
    uint32_t buffer_size;
    uint32_t volume;
    uint32_t pcm_buffer_size;
    uint32_t reserved;
} LOCALSDK_SPEAKER_OPTIONS;

/* ── Functions ──────────────────────────────────────────────────────────────── */

bool speaker_init(void);
bool speaker_free(void);
bool speaker_play_media(char *filename, int type);
int  speaker_status_media(void);
bool speaker_stop_media(void);
bool speaker_set_volume(int value);
int  speaker_get_volume(void);

#endif /* _LOCALSDK_SPEAKER_H_ */
