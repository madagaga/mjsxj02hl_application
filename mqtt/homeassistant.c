#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdio.h>

#include "./homeassistant.h"
#include "./mqtt.h"
#include "./../logger/logger.h"
#include "./../localsdk/init.h"
#include "./../configs/configs.h"
#include "./../yyjson/src/yyjson.h"

// Device info
static bool mqtt_homeassistant_json_device(yyjson_mut_doc *json_doc, yyjson_mut_val *json_root) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;
    
    // Create device object
    yyjson_mut_val *device_object = yyjson_mut_obj(json_doc);
    // Device identifiers
    yyjson_mut_val *identifiers_array = yyjson_mut_arr(json_doc);
    if(result &= yyjson_mut_arr_add_str(json_doc, identifiers_array, mqtt_client_id())) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_arr_add_str(client_id)");
        if(result &= yyjson_mut_obj_add_val(json_doc, device_object, "identifiers", identifiers_array)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_val(identifiers)");
            // Device manufacturer
            if(result &= yyjson_mut_obj_add_str(json_doc, device_object, "manufacturer", MQTT_HOMEASSISTANT_MANUFACTURER)) {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(manufacturer)");
                // Device model
                if(result &= yyjson_mut_obj_add_str(json_doc, device_object, "model", MQTT_HOMEASSISTANT_MODEL)) {
                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(model)");
                    // Device name
                    char *device_name = "";
                    if(result &= (asprintf(&device_name, "%s_%s", mqtt_prepare_string(APP_CFG.mqtt.topic), mqtt_prepare_string(APP_CFG.general.name)) != -1)) {
                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(device_name)");
                        if(result &= yyjson_mut_obj_add_str(json_doc, device_object, "name", device_name)) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(name)");
                            // Device sw_version
                            if(result &= yyjson_mut_obj_add_str(json_doc, device_object, "sw_version", firmware_version())) {
                                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(sw_version)");
                                // Apply device object
                                if(result &= yyjson_mut_obj_add_val(json_doc, json_root, "device", device_object)) {
                                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_val(device)");
                                    // Availability
                                    yyjson_mut_val *availability_array = yyjson_mut_arr(json_doc);
                                    yyjson_mut_val *availability_item = yyjson_mut_arr_add_obj(json_doc, availability_array);
                                    if(result &= yyjson_mut_obj_add_str(json_doc, availability_item, "topic", mqtt_fulltopic(MQTT_STATE_TOPIC))) {
                                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(topic)");
                                        if(result &= yyjson_mut_obj_add_val(json_doc, json_root, "availability", availability_array)) {
                                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_val(availability)");
                                            // Availability_mode
                                            if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "availability_mode", "all")) {
                                                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(availability_mode)");
                                            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(availability_mode)");
                                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_val(availability)");
                                    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(topic)");
                                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_val(device)");
                            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(sw_version)");
                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(name)");
                    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(device_name)");
                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(model)");
            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(manufacturer)");
        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_val(identifiers)");
    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_arr_add_str(client_id)");
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

// Sensor (MQTT_HOMEASSISTANT_SENSOR)
static bool mqtt_homeassistant_json_sensor(yyjson_mut_doc *json_doc, yyjson_mut_val *json_root, char *topic_name, char *json_field, char *device_class, char *unit_of_measurement, bool enabled) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;
    
    // Sensor name
    char *sensor_name = "";
    if(result &= (asprintf(&sensor_name, "%s_%s_%s_%s", mqtt_prepare_string(APP_CFG.mqtt.topic), mqtt_prepare_string(APP_CFG.general.name), topic_name, json_field) != -1)) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(sensor_name)");
        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "name", sensor_name)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(name)");
            // Unique id
            char *unique_id = "";
            if(result &= (asprintf(&unique_id, "%s_%s_%s", mqtt_client_id(), topic_name, json_field) != -1)) {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(unique_id)");
                if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "unique_id", unique_id)) {
                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(unique_id)");
                    // Device class
                    if(device_class != NULL) {
                        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "device_class", device_class)) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(device_class)");
                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(device_class)");
                    }
                    // Units of measurement
                    if(unit_of_measurement != NULL) {
                        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "unit_of_measurement", unit_of_measurement)) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(unit_of_measurement)");
                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(unit_of_measurement)");
                    }
                    // Enable by default
                    if(result &= yyjson_mut_obj_add_bool(json_doc, json_root, "enabled_by_default", enabled)) {
                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_bool(enabled_by_default)");
                        // State topic
                        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "state_topic", mqtt_fulltopic(topic_name))) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_homeassistant_json_binary_sensor(state_topic)");
                            // Value template
                            char *value_template = "";
                            if(result &= (asprintf(&value_template, "{{ value_json.%s }}", json_field) != -1)) {
                                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(value_template)");
                                if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "value_template", value_template)) {
                                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(value_template)");
                                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(value_template)");
                            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(value_template)");
                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(state_topic)");
                    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_bool(enabled_by_default)");
                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(unique_id)");
            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(unique_id)");
        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(name)");
    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(sensor_name)");
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

// Binary sensor (MQTT_HOMEASSISTANT_BINARY_SENSOR)
static bool mqtt_homeassistant_json_binary_sensor(yyjson_mut_doc *json_doc, yyjson_mut_val *json_root, char *topic_name, char *json_field, char *device_class, bool enabled) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;
    
    // Sensor name
    char *sensor_name = "";
    if(result &= (asprintf(&sensor_name, "%s_%s_%s_%s", mqtt_prepare_string(APP_CFG.mqtt.topic), mqtt_prepare_string(APP_CFG.general.name), topic_name, json_field) != -1)) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(sensor_name)");
        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "name", sensor_name)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(name)");
            // Unique id
            char *unique_id = "";
            if(result &= (asprintf(&unique_id, "%s_%s_%s", mqtt_client_id(), topic_name, json_field) != -1)) {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(unique_id)");
                if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "unique_id", unique_id)) {
                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(unique_id)");
                    // Device class
                    if(device_class != NULL) {
                        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "device_class", device_class)) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(device_class)");
                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(device_class)");
                    }
                    // Enable by default
                    if(result &= yyjson_mut_obj_add_bool(json_doc, json_root, "enabled_by_default", enabled)) {
                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_bool(enabled_by_default)");
                        // State topic
                        if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "state_topic", mqtt_fulltopic(topic_name))) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_homeassistant_json_binary_sensor(state_topic)");
                            // Value template
                            char *value_template = "";
                            if(result &= (asprintf(&value_template, "{{ value_json.%s }}", json_field) != -1)) {
                                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(value_template)");
                                if(result &= yyjson_mut_obj_add_str(json_doc, json_root, "value_template", value_template)) {
                                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_str(value_template)");
                                    // Payload values
                                    if(result &= yyjson_mut_obj_add_bool(json_doc, json_root, "payload_on", true)) {
                                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_bool(payload_on)");
                                        if(result &= yyjson_mut_obj_add_bool(json_doc, json_root, "payload_off", false)) {
                                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_bool(payload_off)");
                                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_bool(payload_off)");
                                    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_bool(payload_on)");
                                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(value_template)");
                            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(value_template)");
                        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(state_topic)");
                    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_bool(enabled_by_default)");
                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(unique_id)");
            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(unique_id)");
        } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_str(name)");
    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(sensor_name)");
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

// Add device to Home Assistant over discovery protocol
bool mqtt_homeassistant_discovery(int type, char *topic_name, char *json_field, char *device_class, char *unit_of_measurement, bool enabled) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if(APP_CFG.mqtt.discovery && APP_CFG.mqtt.discovery[0]) {

        // JSON Data
        yyjson_mut_doc *json_doc = yyjson_mut_doc_new(NULL);
        yyjson_mut_val *json_root = yyjson_mut_obj(json_doc);
        yyjson_mut_doc_set_root(json_doc, json_root);
        
        // Device info
        if(result &= mqtt_homeassistant_json_device(json_doc, json_root)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_homeassistant_json_device()");
            
            // Select sensor by type
            char *sensor_type = "";
            switch(type) {
                case MQTT_HOMEASSISTANT_SENSOR:
                    sensor_type = "sensor";
                    if(result &= mqtt_homeassistant_json_sensor(json_doc, json_root, topic_name, json_field, device_class, unit_of_measurement, enabled)) {
                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_homeassistant_json_sensor()");
                    } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "mqtt_homeassistant_json_sensor()");
                    break;
                case MQTT_HOMEASSISTANT_BINARY_SENSOR:
                    sensor_type = "binary_sensor";
                    if(result &= mqtt_homeassistant_json_binary_sensor(json_doc, json_root, topic_name, json_field, device_class, enabled)) {
                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_homeassistant_json_binary_sensor()");
                    } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "mqtt_homeassistant_json_binary_sensor()");
                    break;
                default: result &= false;
            }
            
            // Payload
            if(result) {
                const char *json = yyjson_mut_write(json_doc, 0, NULL);
                if(result &= !!json) {
                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_write()");
                    char *object_id = "";
                    if(result &= (asprintf(&object_id, "%s_%s", topic_name, json_field) != -1)) {
                        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(object_id)");
                        char *discovery_topic = "";
                        char *client_id = mqtt_client_id();
                        if(result &= (asprintf(&discovery_topic, "%s/%s/%s/%s/config", APP_CFG.mqtt.discovery, sensor_type, client_id, object_id) != -1)) {
                            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "asprintf(discovery_topic)");
                            
                            // Send
                            if(result &= mqtt_send(discovery_topic, (char *) json)) {
                                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_send(MQTT_HOMEASSISTANT_DISCOVERY)");
                            } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "mqtt_send(MQTT_HOMEASSISTANT_DISCOVERY)");
                            
                            free(discovery_topic);
                        } LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(discovery_topic)");
                        free(client_id);
                        free(object_id);
                    } LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "asprintf(object_id)");
                    free((void *)json);
                } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_write()");
            }
        } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "mqtt_homeassistant_json_device()");
    
    } else LOGGER(LOGGER_LEVEL_INFO, "Home Assistant Discovery is disabled in the settings or discovery prefix not set.");
    
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

