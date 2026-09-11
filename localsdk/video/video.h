#ifndef _LOCALSDK_VIDEO_H_
#define _LOCALSDK_VIDEO_H_

#include <stdbool.h>
#include <stdint.h>
#include "hi_comm_vpss.h"
#include "../common.h"
#include "../platform/platform.h"  /* board_cfg_t, BOARD_TARGET_FPS */

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define LOCALSDK_VIDEO_PAYLOAD_H264            1
#define LOCALSDK_VIDEO_PAYLOAD_H265            2

#define LOCALSDK_VIDEO_RESOLUTION_640x360      3
#define LOCALSDK_VIDEO_RESOLUTION_1920x1080    6

#define LOCALSDK_VIDEO_PRIMARY_CHANNEL         0
#define LOCALSDK_VIDEO_SECONDARY_CHANNEL       1

#define LOCALSDK_VIDEO_PRIMARY_FRAMESIZE       327680
#define LOCALSDK_VIDEO_SECONDARY_FRAMESIZE     81920

/* Stream output framerate = the board's sensor target fps.
   Used by VENC and RTP timestamping; must match the real framerate. */
#define LOCALSDK_VIDEO_FRAMERATE               BOARD_TARGET_FPS

#define LOCALSDK_VIDEO_RCMODE_CONSTANT_BITRATE 0
#define LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY 1
#define LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE 2

#define LOCALSDK_VIDEO_H26X_FRAME_I            0
#define LOCALSDK_VIDEO_H26X_FRAME_P            1

/* ── Types ──────────────────────────────────────────────────────────────────── */

typedef struct {
    uint32_t width;
    uint32_t height;
} LOCALSDK_PICTURE_SIZE;

typedef struct {
    signed char *data;
    uint32_t size;
    uint32_t index;
    uint32_t timestamp;
    uint16_t offset;      /* Offset in buffer (from VENC_PACK_S.u32Offset) */
    uint16_t frame_end;   /* Frame end flag (from VENC_PACK_S.bFrameEnd) */
    uint16_t type;        /* Frame type (H264E_NALU_*, H265E_NALU_*) */
} LOCALSDK_H26X_FRAME_INFO;

typedef struct {
    uint32_t bitrate;
    uint32_t fps;
    uint32_t resolution;
    uint32_t flip;
    uint32_t mirror;
    uint32_t channel_type;   /* 0=primary 1920x1080, 1=secondary 640x360 */
    uint32_t video;
    uint32_t osd;
    uint32_t payload;
    uint32_t rcmode;
    uint32_t gop;
    uint32_t screen_size;
    uint32_t frame_size;
    uint32_t jpeg;
    uint32_t reserved;
} LOCALSDK_VIDEO_OPTIONS;

/* ── Functions ──────────────────────────────────────────────────────────────── */

/* Lifecycle */
bool video_init(void);
bool video_free(void);
void video_deinit(void);

/* Runtime controls */
int  video_force_i_frame(int chn);
int  video_get_jpeg(int chn, char *file);
int  video_set_encode_callback(int chn, int (*cb)(LOCALSDK_H26X_FRAME_INFO *));
int  video_set_algo_register_callback(int (*cb)(void));
int  video_set_kbps(int chn, int kbps);

/* Exported for teardown */
VPSS_GRP video_get_vpss_grp(void);

#endif /* _LOCALSDK_VIDEO_H_ */
