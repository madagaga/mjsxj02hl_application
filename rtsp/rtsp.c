#include <stdlib.h>
#include <stdio.h>

#include "./rtsp.h"
#include "./librtsp/include/rtsp_server.h"
#include "./../localsdk/localsdk.h"
#include "./../logger/logger.h"

static rtsp_handle rtsp_server;
static pthread_mutex_t rtsp_mutex;

// Init RTSP
bool rtsp_init() {
    logger("rtsp", "rtsp_init", LOGGER_LEVEL_DEBUG, "Function is called...");

    if(rtsp_server = rtsp_create(RTSP_MAXIMUM_CONNECTIONS, 5, 1)) {
        logger("rtsp", "rtsp_init", LOGGER_LEVEL_INFO, "%s success.", "rtsp_create()");
        if(pthread_mutex_init(&rtsp_mutex, NULL) == 0) {
            logger("rtsp", "rtsp_init", LOGGER_LEVEL_INFO, "%s success.", "pthread_mutex_init()");
        
            rtsp_media_attr_t rtsp_media_attr = { 0 };
	        rtsp_media_attr.video_type[LOCALSDK_VIDEO_PRIMARY_CHANNEL] = RTSP_PAYLOAD_TYPE_H264;
            rtsp_media_attr.video_type[LOCALSDK_VIDEO_SECONDARY_CHANNEL] = RTSP_PAYLOAD_TYPE_H264;
            rtsp_media_attr.video_fps[LOCALSDK_VIDEO_PRIMARY_CHANNEL] = LOCALSDK_VIDEO_FPS;
            rtsp_media_attr.video_fps[LOCALSDK_VIDEO_SECONDARY_CHANNEL] = LOCALSDK_VIDEO_FPS;
            rtsp_media_attr.audio_type = RTSP_PAYLOAD_TYPE_G711_PCMA;
            rtsp_media_attr.audio_sample_rate = LOCALSDK_AUDIO_SAMPLE_RATE;
            rtsp_media_attr.audio_poinum = 160;
            
            rtsp_set_media_attr(rtsp_server, &rtsp_media_attr);

        } else logger("rtsp", "rtsp_init", LOGGER_LEVEL_ERROR, "%s error!", "pthread_mutex_init()");
    } else logger("rtsp", "rtsp_init", LOGGER_LEVEL_ERROR, "%s error!", "rtsp_create()");
    
    logger("rtsp", "rtsp_init", LOGGER_LEVEL_DEBUG, "Function completed.");
    return !!rtsp_server;
}

// Free RTSP
bool rtsp_free() {
    bool result = false;
    logger("rtsp", "rtsp_free", LOGGER_LEVEL_DEBUG, "Function is called...");
    
    // Finish server
    if(rtsp_server) {
        rtsp_finish(rtsp_server);
        result = true;
    } else logger("rtsp", "rtsp_free", LOGGER_LEVEL_ERROR, "%s error!", "rtsp_handle");
    
    // Destroy mutex
    if(&rtsp_mutex) {
        if(pthread_mutex_destroy(&rtsp_mutex) == 0) {
            logger("rtsp", "rtsp_free", LOGGER_LEVEL_INFO, "%s success.", "pthread_mutex_destroy()");
        } else {
            logger("rtsp", "rtsp_free", LOGGER_LEVEL_WARNING, "%s error!", "pthread_mutex_destroy()");
            result = false;
        }
    }
    
    logger("rtsp", "rtsp_free", LOGGER_LEVEL_DEBUG, "Function completed.");
    return result;
}

// Send data frame
bool rtsp_media_frame(signed char *data, size_t size, uint32_t timestamp, int type) {
    bool result = false;

    if(rtsp_server) {
        pthread_mutex_lock(&rtsp_mutex);
        struct timeval time;
        time.tv_sec = timestamp;
        time.tv_usec = 1000 * timestamp;
        if(rtp_send_media(rtsp_server, 0, type, data, size, &time) == 0) {
            result = true;
        } else logger("rtsp", "rtsp_media_frame", LOGGER_LEVEL_ERROR, "%s error!", "rtp_send_media()");
        pthread_mutex_unlock(&rtsp_mutex);
    } else logger("rtsp", "rtsp_media_frame", LOGGER_LEVEL_ERROR, "%s error!", "rtsp_handle");
    
    return result;
}
