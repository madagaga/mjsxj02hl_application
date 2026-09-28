#ifndef _RTSP_H_
#define _RTSP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Init RTSP
bool rtsp_init();

// Is enabled
bool rtsp_is_enabled(int channel);

// Free RTSP
bool rtsp_free();

// Send one encoder pack; frame_end marks the last pack of a frame.
// pts in microseconds (MPP clock, shared by audio and video).
bool rtsp_video_frame(int channel, const void *data, size_t size, uint64_t pts, bool frame_end);

// Send one G.711A encoder frame
bool rtsp_audio_frame(int channel, const void *data, size_t size, uint64_t pts);

#endif
