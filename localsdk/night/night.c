#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>

#include "./night.h"
#include "./../localsdk.h"
#include "./../alarm/alarm.h"
#include "./../platform/platform.h"
#include "./../../logger/logger.h"
#include "./../../configs/configs.h"
#include "./../../yyjson/src/yyjson.h"
#include "./../../mqtt/mqtt.h"

#include "mpi_isp.h"
#include "hi_comm_isp.h"

/* ISP AE settle time before first luma sample (seconds). */
#define NIGHT_AE_SETTLE_S 8u

static pthread_t        g_nightLightThread = 0;
static volatile int     g_nightLightRun    = 0;

/* ── Luma polling thread ─────────────────────────────────────────────────── */

static void *night_light_thread(void *arg)
{
    (void)arg;
    usleep(NIGHT_AE_SETTLE_S * 1000000u);
    const board_cfg_t *board = platform_get_board_cfg();
    while (g_nightLightRun) {
        ISP_EXP_INFO_S stExp;
        if (HI_MPI_ISP_QueryExposureInfo(0, &stExp) == HI_SUCCESS)
            if (board && board->pfnOnLuma)
                board->pfnOnLuma(stExp.u8AveLum);
        usleep(1000000);
    }
    return NULL;
}

/* ── MQTT notification ───────────────────────────────────────────────────── */

static bool night_state_mqtt(bool night, bool gray)
{
    yyjson_mut_doc *doc  = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    yyjson_mut_doc_set_root(doc, root);
    yyjson_mut_obj_add_bool(doc, root, "state", night);
    yyjson_mut_obj_add_bool(doc, root, "gray",  gray);
    const char *json = yyjson_mut_write(doc, 0, NULL);
    bool ok = false;
    if (json) {
        char *topic = mqtt_fulltopic(MQTT_NIGHT_TOPIC);
        ok = mqtt_send(topic, (char *)json);
        free(topic);
        free((void *)json);
    }
    yyjson_mut_doc_free(doc);
    return ok;
}

static void on_mode_change(HI_BOOL is_night)
{
    if (mqtt_is_ready())
        night_state_mqtt(is_night,
            APP_CFG.night.gray == 2 ? is_night : (APP_CFG.night.gray == 1));
    alarm_switch(!is_night);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool night_init(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");

    board_register_mode_change_cb(on_mode_change);

    switch (APP_CFG.night.mode) {
        case 0:
            board_set_mode(HI_FALSE);
            break;
        case 1:
            board_set_mode(HI_TRUE);
            break;
        case 2:
        default:
            g_nightLightRun = 1;
            if (pthread_create(&g_nightLightThread, NULL, night_light_thread, NULL) != 0) {
                g_nightLightRun = 0;
                LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "pthread_create(night_light_thread)");
            } else {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "pthread_create(night_light_thread)");
            }
            break;
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed.");
    return true;
}

bool night_free(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");

    board_register_mode_change_cb(NULL);

    if (g_nightLightThread) {
        g_nightLightRun = 0;
        pthread_cancel(g_nightLightThread);
        pthread_join(g_nightLightThread, NULL);
        g_nightLightThread = 0;
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed.");
    return true;
}
