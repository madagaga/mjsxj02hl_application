#ifndef _RTSP_SERVER_H_
#define _RTSP_SERVER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Self-contained RTSP/RTP server (RFC 2326, 3550, 6184, 7798).
   One network thread; producers push encoded frames from their own thread.
   All memory is allocated in rtsp_server_start(): a stalled or slow client
   loses frames (it resumes on the next key frame), it never grows memory. */

#define RTSP_SERVER_MAX_SESSIONS  2
#define RTSP_SERVER_MAX_CLIENTS   4

typedef enum {
    RTSP_CODEC_NONE = 0,
    RTSP_CODEC_H264,
    RTSP_CODEC_H265,
    RTSP_CODEC_PCMA
} rtsp_codec_t;

typedef struct {
    const char  *name;              // URL path: rtsp://<ip>:<port>/<name>
    rtsp_codec_t video;             // H264 or H265
    rtsp_codec_t audio;             // PCMA or NONE
    uint32_t     framerate;         // advertised in the SDP
    uint32_t     video_buffer;      // ring size in bytes
    uint32_t     audio_buffer;      // ring size in bytes
} rtsp_session_config_t;

typedef struct {
    uint16_t    port;
    const char *username;           // Digest auth when username or password is set
    const char *password;
    // Called from the network thread when a client starts playing and no
    // key frame is buffered yet (e.g. request an IDR from the encoder)
    void (*key_frame_needed)(int session);
} rtsp_server_config_t;

bool rtsp_server_start(const rtsp_server_config_t *config, const rtsp_session_config_t *sessions, int count);
void rtsp_server_stop(void);

// Producer side: an Annex-B buffer (one or more NALs); frame_end marks the
// last buffer of a frame. pts in microseconds, same clock for audio and video.
bool rtsp_server_push_video(int session, const uint8_t *data, size_t size, uint64_t pts, bool frame_end);
bool rtsp_server_push_audio(int session, const uint8_t *data, size_t size, uint64_t pts);

#endif
