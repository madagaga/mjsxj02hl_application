#ifndef _LOCALSDK_H_
#define _LOCALSDK_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/* Board sizing constants (single source of truth for the board+sensor couple).
   Resolved at compile time; provides BOARD_TARGET_FPS used below. */
#include "platform/board_mjsxj02hl.h"

/********************
       GENERAL
********************/

#define LOCALSDK_OK               0
#define LOCALSDK_ERROR            1

#define LOCALSDK_CURRENT_VERSION  14

typedef struct {
    uint32_t width;
    uint32_t height;
} LOCALSDK_PICTURE_SIZE;

// Set printf function for debug messages
int localsdk_set_logprintf_func(int (*function)(const char *, ...));

// Set shell function
// param: pointer to callback function: int (*callback)(const char *command)
int localsdk_set_shellcall_func(int (*callback)(const char *));

// Initialize SDK
int localsdk_init();

// Destory SDK
int localsdk_destory();

// Get SDK version
int localsdk_get_version();

/********************
        VIDEO
********************/

#define LOCALSDK_VIDEO_PAYLOAD_H264            1
#define LOCALSDK_VIDEO_PAYLOAD_H265            2

#define LOCALSDK_VIDEO_RESOLUTION_640x360      3
#define LOCALSDK_VIDEO_RESOLUTION_1920x1080    6

#define LOCALSDK_VIDEO_PRIMARY_CHANNEL         0
#define LOCALSDK_VIDEO_SECONDARY_CHANNEL       1

#define LOCALSDK_VIDEO_PRIMARY_FRAMESIZE       327680
#define LOCALSDK_VIDEO_SECONDARY_FRAMESIZE     81920

/* Stream output framerate = the board's sensor target fps (single source).
   Used by VENC (video.c) and RTP timestamping (rtsp.c); must match the real
   framerate or A/V sync / playback speed is wrong. */
#define LOCALSDK_VIDEO_FRAMERATE               BOARD_TARGET_FPS

#define LOCALSDK_VIDEO_RCMODE_CONSTANT_BITRATE 0
#define LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY 1
#define LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE 2

#define LOCALSDK_VIDEO_H26X_FRAME_I            0
#define LOCALSDK_VIDEO_H26X_FRAME_P            1

typedef struct {
    signed char *data;
    uint32_t size;
    uint32_t index;
    uint32_t timestamp;
    uint16_t offset;      // Offset in buffer (from VENC_PACK_S.u32Offset)
    uint16_t frame_end;   // Frame end flag (from VENC_PACK_S.bFrameEnd)
    uint16_t type;        // Frame type (H264E_NALU_*, H265E_NALU_*)
} LOCALSDK_H26X_FRAME_INFO;

typedef struct {
    uint32_t bitrate;
    uint32_t fps;
    uint32_t resolution;
    uint32_t flip;
    uint32_t mirror;
    uint32_t channel_type;   // Channel type (0=primary 1920x1080, 1=secondary 640x360)
    uint32_t video;
    uint32_t osd;
    uint32_t payload;
    uint32_t rcmode;
    uint32_t gop;
    uint32_t screen_size;
    uint32_t frame_size;
    uint32_t jpeg;
    uint32_t reserved;      // Reserved/padding
} LOCALSDK_VIDEO_OPTIONS;

/********************
        AUDIO
********************/

#define LOCALSDK_AUDIO_CHANNEL          0
#define LOCALSDK_AUDIO_SAMPLE_RATE      8000
#define LOCALSDK_AUDIO_BIT_DEPTH        16
#define LOCALSDK_AUDIO_TRACK_TYPE       1

#define LOCALSDK_AUDIO_G711_BUFFER_SIZE 320
#define LOCALSDK_AUDIO_PCM_BUFFER_SIZE  640

#define LOCALSDK_AUDIO_G711_FRAME       2

typedef struct {
    signed char *data;
    uint32_t size;
    uint32_t index;
    uint32_t timestamp;
} LOCALSDK_AUDIO_G711_FRAME_INFO;

typedef struct {
    uint32_t sample_rate;      // 8000 Hz
    uint32_t bit_depth;        // 16 bits
    uint32_t points_per_frame; // AENC_CHN_ATTR_S.u32PtNumPerFrm (25)
    uint32_t track_type;       // 0=mono, 1=stereo
    uint32_t buffer_size;      // AENC_CHN_ATTR_S.u32BufSize (0 for audio, 30 for speaker)
    uint32_t payload_type;     // AENC_CHN_ATTR_S.enType (2=PT_G711A)
    uint32_t encoder_param;    // AENC_CHN_ATTR_S.pValue pointer/flag (1)
    uint32_t reserved_1;       // Reserved/padding (1)
    uint32_t reserved_2;       // Reserved/padding (2)
    uint32_t initial_gain;     // Initial gain value (20)
    uint32_t volume;           // Volume level
    uint32_t pcm_buffer_size;  // PCM buffer size (640)
    uint32_t g711_buffer_size; // G711 buffer size (320)
} LOCALSDK_AUDIO_OPTIONS;

/* Audio implementation lives in audio/audio.c (Phase 2 refactoring). */

/********************
       SPEAKER
********************/

#define LOCALSDK_SPEAKER_SAMPLE_RATE 8000
#define LOCALSDK_SPEAKER_BIT_DEPTH   16
#define LOCALSDK_SPEAKER_TRACK_TYPE  1

#define LOCALSDK_SPEAKER_PCM_TYPE    1
#define LOCALSDK_SPEAKER_G711_TYPE   2

typedef struct {
    uint32_t sample_rate;      // 8000 Hz
    uint32_t bit_depth;        // 16 bits
    uint32_t points_per_frame; // AENC_CHN_ATTR_S.u32PtNumPerFrm (25)
    uint32_t track_type;       // 0=mono, 1=stereo
    uint32_t buffer_size;      // AENC_CHN_ATTR_S.u32BufSize (30)
    uint32_t volume;           // Volume level
    uint32_t pcm_buffer_size;  // PCM buffer size (640)
    uint32_t reserved;         // Reserved/padding (1)
} LOCALSDK_SPEAKER_OPTIONS;

/* Speaker implementation lives in speaker/speaker.c (Phase 2 refactoring). */

/********************
        ALARM
********************/

#define LOCALSDK_ALARM_TYPE_MOTION       1
#define LOCALSDK_ALARM_TYPE_HUMANOID     7

#define LOCALSDK_ALARM_MAXIMUM_OBJECTS   4

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
        uint32_t reserved[11]; // Replaces unknown[11]
    } objects[LOCALSDK_ALARM_MAXIMUM_OBJECTS];
} LOCALSDK_ALARM_EVENT_INFO;

/* Alarm implementation lives in alarm/alarm.c (Phase 3 refactoring). */

/********************
        OSD
********************/

#define LOCALSDK_OSD_COLOR_GREEN  3
#define LOCALSDK_OSD_COLOR_ORANGE 5

typedef struct {
    uint32_t flags;           // Replaces unknown (value 67 from original binary)
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
        uint32_t visible;        // Replaces unknown (always = 1)
        uint32_t color;
    } objects[LOCALSDK_ALARM_MAXIMUM_OBJECTS];
} LOCALSDK_OSD_RECTANGLES;

/* OSD implementation lives in osd/osd.c (Phase 4 refactoring). */

/********************
        LEDS
********************/

int local_sdk_indicator_led_option(bool orange, bool blue);

/********************
       BUTTON
********************/

int local_sdk_setup_keydown_set_callback(int timeout, int (*callback)());

/********************
     NIGHT MODE
********************/

// Start auto luma polling thread (mode=2); board owns state machine
int local_sdk_auto_night_light();

#ifdef __cplusplus
}
#endif

#endif
