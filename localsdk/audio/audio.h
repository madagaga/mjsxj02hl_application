#ifndef _LOCALSDK_AUDIO_H_
#define _LOCALSDK_AUDIO_H_

#include <stdbool.h>
#include <stdint.h>
#include "../common.h"

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define LOCALSDK_AUDIO_CHANNEL          0
#define LOCALSDK_AUDIO_SAMPLE_RATE      8000
#define LOCALSDK_AUDIO_BIT_DEPTH        16
#define LOCALSDK_AUDIO_TRACK_TYPE       1

#define LOCALSDK_AUDIO_G711_BUFFER_SIZE 320
#define LOCALSDK_AUDIO_PCM_BUFFER_SIZE  640

#define LOCALSDK_AUDIO_G711_FRAME       2

/* ── Types ──────────────────────────────────────────────────────────────────── */

typedef struct {
    signed char *data;
    uint32_t size;
    uint32_t index;
    uint32_t timestamp;
} LOCALSDK_AUDIO_G711_FRAME_INFO;

typedef struct {
    uint32_t sample_rate;
    uint32_t bit_depth;
    uint32_t points_per_frame;
    uint32_t track_type;
    uint32_t buffer_size;
    uint32_t payload_type;
    uint32_t encoder_param;
    uint32_t reserved_1;
    uint32_t reserved_2;
    uint32_t initial_gain;
    uint32_t volume;
    uint32_t pcm_buffer_size;
    uint32_t g711_buffer_size;
} LOCALSDK_AUDIO_OPTIONS;

/* ── Functions ──────────────────────────────────────────────────────────────── */

bool audio_is_enabled(int channel);
bool audio_init(void);
bool audio_free(void);

/* AO device/channel accessors for speaker.c */
int audio_get_ao_dev(void);
int audio_get_ao_chn(void);

#endif /* _LOCALSDK_AUDIO_H_ */
