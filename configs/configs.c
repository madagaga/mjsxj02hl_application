#include <stdlib.h>
#include <string.h>

#include "./configs.h"
#include "./inih/ini.h"
#include "./../localsdk/localsdk.h"
#include "./../logger/logger.h"

// Default values
APPLICATION_CONFIGURATION APP_CFG = {
    // [logger]
    .logger.level               = LOGGER_LEVEL_WARNING,        // Log level
    .logger.file                = "",                          // Write log to file
    
    // [video]
    .video.type                 = LOCALSDK_VIDEO_PAYLOAD_H265, // Video compression standard (1 = h264, 2 = h265)
    .video.fps                  = 20,                          // Frames per second
    .video.flip                 = false,                       // Flip (true or false)
    .video.mirror               = false,                       // Mirror (true or false)

    // [audio]
    .audio.volume               = 70,                          // Volume (0-100)
    .audio.primary_enable       = true,                        // Enable audio for primary channel
    .audio.secondary_enable     = true,                        // Enable audio for secondary channel
    
    // [speaker]
    .speaker.volume             = 70,                          // Volume (0-100)
    .speaker.type               = LOCALSDK_SPEAKER_PCM_TYPE,   // Default file format (1 = PCM, 2 = G711)
    
    // [alarm]
    .alarm.motion_sens          = 150,                         // Motion sensitivity (1-255)
    .alarm.humanoid_sens        = 150,                         // Humanoid sensitivity (1-255)
    .alarm.motion_timeout       = 60,                          // Motion timeout (in seconds)
    .alarm.humanoid_timeout     = 60,                          // Humanoid timeout (in seconds)
    .alarm.motion_detect_exec   = "",                          // Execute the command when motion is detected
    .alarm.humanoid_detect_exec = "",                          // Execute the command when humanoid is detected
    .alarm.motion_lost_exec     = "",                          // Execute the command when motion is lost
    .alarm.humanoid_lost_exec   = "",                          // Execute the command when humanoid is lost
    
    // [rtsp]
    .rtsp.enable                = true,                        // Enable RTSP server
    .rtsp.port                  = 554,                         // Port number
    .rtsp.username              = "",                          // Username (empty for disable)
    .rtsp.password              = "",                          // Password
    .rtsp.primary_name          = "primary",                   // Name of the primary channel
    .rtsp.secondary_name        = "secondary",                 // Name of the secondary channel
    .rtsp.primary_multicast     = false,                       // Use multicast for primary channel
    .rtsp.secondary_multicast   = false,                       // Use multicast for secondary channel
    
    // [mqtt]
    .mqtt.server                = "",                          // Address (empty for disable)
    .mqtt.port                  = 1883,                        // Port number
    .mqtt.username              = "",                          // Username (empty for anonimous)
    .mqtt.password              = "",                          // Password (empty for disable)
    .mqtt.topic                 = "mjsxj02hl",                 // Topic name
    .mqtt.qos                   = 1,                           // Quality of Service (0, 1 or 2)
    .mqtt.retain                = false,                       // Retained messages
    
    // [night]
    .night.mode                 = 2,                           // Night mode (0 = off, 1 = on, 2 = auto)
    .night.gray                 = 2,                           // Grayscale (0 = off, 1 = on, 2 = auto)
};

// Handler for ini parser
static int parser_handler(void* cfg, const char* section, const char* name, const char* value) {
    bool result = true;

    APPLICATION_CONFIGURATION* config = (APPLICATION_CONFIGURATION*) cfg;
    #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    #define atob(v) strcmp(value, "true") == 0 || strcmp(value, "yes") == 0 || strcmp(value, "on") == 0 || strcmp(value, "1") == 0
    
    // [logger]
    if(MATCH("logger", "level")) {
        config->logger.level = atoi(value);
    } else if(MATCH("logger", "file")) {
        config->logger.file = strdup(value);
    
    // [video]
    } else if(MATCH("video", "type")) {
        config->video.type = atoi(value);
    } else if(MATCH("video", "fps")) {
        config->video.fps = atoi(value);
    } else if(MATCH("video", "flip")) {
        config->video.flip = atob(value);
    } else if(MATCH("video", "mirror")) {
        config->video.mirror = atob(value);

    // [audio]
    } else if(MATCH("audio", "volume")) {
        config->audio.volume = atoi(value);
    } else if(MATCH("audio", "primary_enable")) {
        config->audio.primary_enable = atob(value);
    } else if(MATCH("audio", "secondary_enable")) {
        config->audio.secondary_enable = atob(value);

    // [speaker]
    } else if(MATCH("speaker", "volume")) {
        config->speaker.volume = atoi(value);
    } else if(MATCH("speaker", "type")) {
        config->speaker.type = atoi(value);

    // [alarm]
    } else if(MATCH("alarm", "motion_sens")) {
        config->alarm.motion_sens = atoi(value);
    } else if(MATCH("alarm", "humanoid_sens")) {
        config->alarm.humanoid_sens = atoi(value);
    } else if(MATCH("alarm", "motion_timeout")) {
        config->alarm.motion_timeout = atoi(value);
    } else if(MATCH("alarm", "humanoid_timeout")) {
        config->alarm.humanoid_timeout = atoi(value);
    } else if(MATCH("alarm", "motion_detect_exec")) {
        config->alarm.motion_detect_exec = strdup(value);
    } else if(MATCH("alarm", "humanoid_detect_exec")) {
        config->alarm.humanoid_detect_exec = strdup(value);
    } else if(MATCH("alarm", "motion_lost_exec")) {
        config->alarm.motion_lost_exec = strdup(value);
    } else if(MATCH("alarm", "humanoid_lost_exec")) {
        config->alarm.humanoid_lost_exec = strdup(value);

    // [rtsp]
    } else if(MATCH("rtsp", "enable")) {
        config->rtsp.enable = atob(value);
    } else if(MATCH("rtsp", "port")) {
        config->rtsp.port = atoi(value);
    } else if(MATCH("rtsp", "username")) {
        config->rtsp.username = strdup(value);
    } else if(MATCH("rtsp", "password")) {
        config->rtsp.password = strdup(value);
    } else if(MATCH("rtsp", "primary_name")) {
        config->rtsp.primary_name = strdup(value);
    } else if(MATCH("rtsp", "secondary_name")) {
        config->rtsp.secondary_name = strdup(value);
    } else if(MATCH("rtsp", "primary_multicast")) {
        config->rtsp.primary_multicast = atob(value);
    } else if(MATCH("rtsp", "secondary_multicast")) {
        config->rtsp.secondary_multicast = atob(value);

    // [mqtt]
    } else if(MATCH("mqtt", "server")) {
        config->mqtt.server = strdup(value);
    } else if(MATCH("mqtt", "port")) {
        config->mqtt.port = atoi(value);
    } else if(MATCH("mqtt", "username")) {
        config->mqtt.username = strdup(value);
    } else if(MATCH("mqtt", "password")) {
        config->mqtt.password = strdup(value);
    } else if(MATCH("mqtt", "topic")) {
        config->mqtt.topic = strdup(value);
    } else if(MATCH("mqtt", "qos")) {
        config->mqtt.qos = atoi(value);
    } else if(MATCH("mqtt", "retain")) {
        config->mqtt.retain = atob(value);

    // [night]
    } else if(MATCH("night", "mode")) {
        config->night.mode = atoi(value);
    } else if(MATCH("night", "gray")) {
        config->night.gray = atoi(value);

    // unknown
    } else result = false;

    if(result) {
        logger("configs", "configs_init", LOGGER_LEVEL_FORCED, "%s success. Section: %s, name: %s, value: %s", "parser_handler()", section, name, value);
    } else {
        logger("configs", "configs_init", LOGGER_LEVEL_WARNING, "%s error! Section: %s, name: %s, value: %s", "parser_handler()", section, name, value);
    }
    
    return result;
}

// Init application configs
bool configs_init(char *filename) {
    bool result = true;
    logger("configs", "configs_init", LOGGER_LEVEL_DEBUG, "Function is called...");
    logger("configs", "configs_init", LOGGER_LEVEL_DEBUG, "Filename: %s", filename);

    // Read values from config file
    if(ini_parse(filename, parser_handler, &APP_CFG) >= 0) {
        logger("configs", "configs_init", LOGGER_LEVEL_INFO, "%s success.", "ini_parse()");
    } else {
        logger("configs", "configs_init", LOGGER_LEVEL_ERROR, "%s error!", "ini_parse()");
        logger("configs", "configs_init", LOGGER_LEVEL_ERROR, "Configs filename: %s", filename);
        if(configs_free()) {
            logger("configs", "configs_init", LOGGER_LEVEL_INFO, "%s success.", "configs_free()");
        } else logger("configs", "configs_init", LOGGER_LEVEL_WARNING, "%s error!", "configs_free()");
        result = false;
    }
    
    logger("configs", "configs_init", LOGGER_LEVEL_DEBUG, "Function completed.");
    return result;
}

// Free application configs
bool configs_free() {
    logger("configs", "configs_free", LOGGER_LEVEL_DEBUG, "Function is called...");
    logger("configs", "configs_free", LOGGER_LEVEL_DEBUG, "Function completed.");
    return true;
}
