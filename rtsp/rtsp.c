#include <stdlib.h>
#include <stdio.h>

#include "./rtsp.h"
#include "./rtsp_server.h"
#include "./../localsdk/video/video.h"
#include "./../localsdk/audio/audio.h"
#include "./../logger/logger.h"
#include "./../configs/configs.h"

// Ring sizing: one GOP plus one second of margin at the configured bitrate
#define RTSP_VIDEO_BUFFER_MIN   (128 * 1024)
#define RTSP_VIDEO_BUFFER_MAX   (1024 * 1024)
#define RTSP_AUDIO_BUFFER       (16 * 1024)   // G.711: 8 KB/s, 2 s

static int session_of[2] = { -1, -1 };        // channel -> server session

// Is enabled
bool rtsp_is_enabled(int channel) {
    switch(channel) {
        case LOCALSDK_VIDEO_PRIMARY_CHANNEL:
            return (APP_CFG.rtsp.enable && (APP_CFG.rtsp.primary_name && APP_CFG.rtsp.primary_name[0]));
        case LOCALSDK_VIDEO_SECONDARY_CHANNEL:
            return (APP_CFG.rtsp.enable && (APP_CFG.rtsp.secondary_name && APP_CFG.rtsp.secondary_name[0]));
        default:
            return (APP_CFG.rtsp.enable && (rtsp_is_enabled(LOCALSDK_VIDEO_PRIMARY_CHANNEL) || rtsp_is_enabled(LOCALSDK_VIDEO_SECONDARY_CHANNEL)));
    }
}

static uint32_t video_buffer(int bitrate_kbps) {
    int gop_s = APP_CFG.video.gop > 0 ? APP_CFG.video.gop : 1;
    uint32_t size = (uint32_t)(bitrate_kbps > 0 ? bitrate_kbps : 1000) * 125 * (uint32_t)(gop_s + 1);
    if (size < RTSP_VIDEO_BUFFER_MIN) size = RTSP_VIDEO_BUFFER_MIN;
    if (size > RTSP_VIDEO_BUFFER_MAX) size = RTSP_VIDEO_BUFFER_MAX;
    return size;
}

static rtsp_codec_t video_codec(int type) {
    return (type == LOCALSDK_VIDEO_PAYLOAD_H265) ? RTSP_CODEC_H265 : RTSP_CODEC_H264;
}

// A client wants to play and no key frame is buffered: ask the encoder
static void key_frame_needed(int session) {
    for (int channel = 0; channel < 2; channel++) {
        if (session_of[channel] != session) continue;
        if (video_force_i_frame(channel) == LOCALSDK_OK) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "video_force_i_frame()");
        else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "video_force_i_frame()");
    }
}

// Init RTSP
bool rtsp_init() {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if(rtsp_is_enabled(-1)) { // If RTSP enabled
        rtsp_session_config_t sessions[2];
        int count = 0;
        for (int channel = 0; channel < 2; channel++) {
            if (!rtsp_is_enabled(channel)) {
                LOGGER(LOGGER_LEVEL_INFO, "%s channel is disabled in the settings or its name is not set.",
                       channel == LOCALSDK_VIDEO_PRIMARY_CHANNEL ? "Primary" : "Secondary");
                continue;
            }
            bool primary = (channel == LOCALSDK_VIDEO_PRIMARY_CHANNEL);
            rtsp_session_config_t *s = &sessions[count];
            s->name = primary ? APP_CFG.rtsp.primary_name : APP_CFG.rtsp.secondary_name;
            s->video = video_codec(primary ? APP_CFG.video.primary_type : APP_CFG.video.secondary_type);
            s->audio = audio_is_enabled(channel) ? RTSP_CODEC_PCMA : RTSP_CODEC_NONE;
            s->framerate = LOCALSDK_VIDEO_FRAMERATE;
            s->video_buffer = video_buffer(primary ? APP_CFG.video.primary_bitrate : APP_CFG.video.secondary_bitrate);
            s->audio_buffer = RTSP_AUDIO_BUFFER;
            session_of[channel] = count++;
        }

        rtsp_server_config_t config = {
            .port = (uint16_t)APP_CFG.rtsp.port,
            .username = APP_CFG.rtsp.username,
            .password = APP_CFG.rtsp.password,
            .key_frame_needed = key_frame_needed,
        };
        if ((result = rtsp_server_start(&config, sessions, count))) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "rtsp_server_start()");
        else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "rtsp_server_start()");
    } else LOGGER(LOGGER_LEVEL_INFO, "RTSP server is disabled in the settings.");

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

// Free RTSP
bool rtsp_free() {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    if(rtsp_is_enabled(-1)) rtsp_server_stop();
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", "true");
    return true;
}

// HiSilicon AENC prefixes each G.711 frame with a 4-byte header
// (00 01 <len/2 LE16>): it is not part of the RTP payload
static bool strip_hisi_audio_header(const uint8_t **data, size_t *size) {
    const uint8_t *d = *data;
    if (*size > 4 && d[0] == 0x00 && d[1] == 0x01 && (size_t)(d[2] | (d[3] << 8)) * 2 == *size - 4) {
        *data += 4;
        *size -= 4;
        return true;
    }
    return false;
}

// Send video pack
bool rtsp_video_frame(int channel, const void *data, size_t size, uint64_t pts, bool frame_end) {
    if (channel < 0 || channel > 1 || session_of[channel] < 0 || !data || !size) return false;
    return rtsp_server_push_video(session_of[channel], (const uint8_t *)data, size, pts, frame_end);
}

// Send audio frame
bool rtsp_audio_frame(int channel, const void *data, size_t size, uint64_t pts) {
    if (channel < 0 || channel > 1 || session_of[channel] < 0 || !data || !size) return false;
    const uint8_t *buffer = (const uint8_t *)data;
    static bool logged = false;
    bool stripped = strip_hisi_audio_header(&buffer, &size);
    if (!logged) {
        logged = true;
        LOGGER(LOGGER_LEVEL_DEBUG, "G.711 frame of %u bytes, HiSilicon header %s.",
               (unsigned)size, stripped ? "stripped" : "absent");
    }
    return rtsp_server_push_audio(session_of[channel], buffer, size, pts);
}
