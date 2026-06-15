#include <stdbool.h>

#include "./night.h"
#include "./../localsdk.h"
#include "./../alarm/alarm.h"
#include "./../platform/platform.h"
#include "./../../logger/logger.h"
#include "./../../configs/configs.h"
#include "./../../yyjson/src/yyjson.h"
#include "./../../mqtt/mqtt.h"

static bool night_state_mqtt(bool night, bool gray) {
    yyjson_mut_doc *doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *root = yyjson_mut_obj(doc);
    yyjson_mut_doc_set_root(doc, root);
    yyjson_mut_obj_add_bool(doc, root, "state", night);
    yyjson_mut_obj_add_bool(doc, root, "gray", gray);
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

static void on_mode_change(HI_BOOL is_night) {
    if (mqtt_is_ready())
        night_state_mqtt(is_night,
            APP_CFG.night.gray == 2 ? is_night : (APP_CFG.night.gray == 1));
    alarm_switch(!is_night);
}

bool night_init() {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");

    board_register_mode_change_cb(on_mode_change);

    switch (APP_CFG.night.mode) {
        case 0: /* force day */
            board_set_mode(HI_FALSE);
            break;
        case 1: /* force night */
            board_set_mode(HI_TRUE);
            break;
        case 2: /* auto luma detection */
        default:
            local_sdk_auto_night_light();
            break;
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed.");
    return true;
}

bool night_free() {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    board_register_mode_change_cb(NULL);
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed.");
    return true;
}
