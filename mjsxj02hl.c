#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sysexits.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>

#include "./logger/logger.h"
#include "./localsdk/init.h"
#include "./localsdk/video/video.h"
#include "./configs/configs.h"
#include "./mqtt/mqtt.h"
#include "./rtsp/rtsp.h"
#include "mpi_sys.h"

// Signal callback
void signal_callback(int signal) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    
    // Enable orange LED
    if(APP_CFG.general.led) {
        board_indicator_led(true, false);
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "board_indicator_led(true, false)");
    }
    
    // MQTT free
    if(mqtt_free(true)) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_free(true)");
    else {
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "mqtt_free(true)");
        signal = EX_SOFTWARE;
    }
    
    // RTSP free
    if(rtsp_free()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "rtsp_free()");
    else {
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "rtsp_free()");
        signal = EX_SOFTWARE;
    }
    
    // All free
    if(all_free()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "all_free()");
    else {
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "all_free()");
        signal = EX_SOFTWARE;
    }
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (signal = %d).", signal);
    exit(signal);
}

// Factory reset callback
int factory_reset_callback() {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    int result = LOCALSDK_OK;
    
    if(system("mjsxj02hl --factory-reset") == EX_OK) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "system(mjsxj02hl --factory-reset)");
    else {
        LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "system(mjsxj02hl --factory-reset)");
        result = LOCALSDK_ERROR;
    }
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result == LOCALSDK_OK ? "LOCALSDK_OK" : "LOCALSDK_ERROR"));
    return result;
}

// Main function
int main(int argc, char **argv) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    
    // Initialize pseudo-random number generator
    srand(time(NULL));
    
    // Default path of config file
    char *config_filename = "/usr/app/share/mjsxj02hl.conf";
    
    // Running with arguments
    if(argc > 1) {
        if(strcmp(argv[1], "--config") == 0) { // Set config path
            if(argc == 3) {
                config_filename = argv[2];
            } else {
                printf("Error: missing filename! Use the --help option for more information.\n");
                return EX_USAGE;
            }
        } else if(strcmp(argv[1], "--factory-reset") == 0) { // Factory reset
            printf("Reset to factory settings...\n");
            system("rm -rf /configs/*");
            system("reboot");
            return EX_OK;
        } else if(strcmp(argv[1], "--get-image") == 0) { // Get image
            if(argc == 3) {
                if(system("pidof -o %PPID mjsxj02hl > /dev/null") == EX_OK) {
                    /* Connect this subprocess to the already-running MPP instance.
                       Reference-counted: safe to call even if main process has it. */
                    HI_MPI_SYS_Init();
                    int lock_fd = open("/tmp/mjsxj02hl_get_image.lock", O_CREAT | O_RDWR, 0666);
                    if(lock_fd < 0) {
                        printf("Error: unable to open snapshot lock file (errno=%d)\n", errno);
                        return EX_CANTCREAT;
                    }

                    int waited_ms = 0;
                    while(flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
                        if(waited_ms >= 2000) {
                            printf("Error: snapshot is busy, try again later.\n");
                            close(lock_fd);
                            return EX_TEMPFAIL;
                        }
                        usleep(100000);
                        waited_ms += 100;
                    }

                    // Best-effort: request I-frame before snapshot
                    video_force_i_frame(LOCALSDK_VIDEO_SECONDARY_CHANNEL);

                    int attempt = 0;
                    for(attempt = 0; attempt < 3; attempt++) {
                        if(video_get_jpeg(LOCALSDK_VIDEO_SECONDARY_CHANNEL, argv[2]) == LOCALSDK_OK) {
                            flock(lock_fd, LOCK_UN);
                            close(lock_fd);
                            return EX_OK;
                        }
                        usleep(150000);
                        video_force_i_frame(LOCALSDK_VIDEO_SECONDARY_CHANNEL);
                    }

                    flock(lock_fd, LOCK_UN);
                    close(lock_fd);
                    printf("Error: video_get_jpeg() failed after retries!\n");
                    return EX_SOFTWARE;
                } else {
                    printf("Error: main thread of mjsxj02hl application is not running!\n");
                    return EX_UNAVAILABLE;
                }
            } else {
                printf("Error: missing filename! Use the --help option for more information.\n");
                return EX_USAGE;
            }
        } else if(strcmp(argv[1], "--help") == 0) { // Help
            printf("Usage: mjsxj02hl [<action> [options...]]\n");
            printf("\n");
            printf("Running without arguments starts the main thread of the application.\n");
            printf("\n");
            printf("  --config <filename>       Specify the location of the configuration file for the main thread of application.\n");
            printf("\n");
            printf("  --factory-reset           Reset device settings to default values. Attention: this action cannot be undone!\n");
            printf("\n");
            printf("  --get-image <filename>    Output the camera image to a file. Requires a running main thread of mjsxj02hl application.\n");
            printf("\n");
            printf("  --help                    Display this message.\n");
            printf("\n");
            printf("Report an error or help in the development of the project you can on the page %s\n", "https://github.com/kasitoru/mjsxj02hl_application");
            return EX_OK;
        } else {
            printf("Error: unknown action! Use the --help option for more information.\n");
            return EX_USAGE;
        }
    }
    
    // Firmware version
    char *fw_ver = firmware_version();
    LOGGER(LOGGER_LEVEL_FORCED, "Firmware version: %s", fw_ver);
    free(fw_ver);
    
    // Device id
    char *dev_id = device_id();
    LOGGER(LOGGER_LEVEL_FORCED, "Device ID: %s", dev_id);
    free(dev_id);
    
    // Main thread
    if(configs_init(config_filename)) { // Init configs
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "configs_init()");
        if(all_init()) { // Init all systems
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "all_init()");
            
            // Register signals
            if(signal(SIGINT, signal_callback) != SIG_ERR) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "signal(SIGINT)"); // SIGINT
            else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "signal(SIGINT)");
            
            if(signal(SIGABRT, signal_callback) != SIG_ERR) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "signal(SIGABRT)"); // SIGABRT
            else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "signal(SIGABRT)");
            
            if(signal(SIGTERM, signal_callback) != SIG_ERR) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "signal(SIGTERM)"); // SIGTERM
            else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "signal(SIGTERM)");
            
            // Onboard LED indicator
            if(APP_CFG.general.led) { // Enable blue LED
                board_indicator_led(false, true);
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "board_indicator_led(false, true)");
            } else { // Disable LEDs
                board_indicator_led(false, false);
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "board_indicator_led(false, false)");
            }

            // Factory reset callback
            board_set_button_callback(3000, factory_reset_callback);
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "board_set_button_callback()");
            
            // RTSP server
            if(rtsp_init()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "rtsp_init()");
            else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "rtsp_init()");
            
            // MQTT client
            if(mqtt_init()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_init()");
            else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "mqtt_init()");
            
            // Endless waiting
            while(true) {
                sleep(1);
            }
            
        } else {
            LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "all_init()");
            return EX_SOFTWARE;
        }
    } else {
        LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "configs_init()");
        return EX_CONFIG;
    }
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (signal = %d).", EX__BASE);
    return EX__BASE;
}
