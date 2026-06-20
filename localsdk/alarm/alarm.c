#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#include "./alarm.h"
#include "./../localsdk.h"
#include "./../platform/platform.h"
#include "./../platform/board_mjsxj02hl.h"
#include "./../video/video.h"
#include "./../osd/osd.h"
#include "./../../logger/logger.h"
#include "./../../configs/configs.h"
#include "./../../yyjson/src/yyjson.h"
#include "./../../mqtt/mqtt.h"

#include "hi_ivp.h"
#include "mpi_vpss.h"
#include "mpi_sys.h"

/* ── Alarm callback pool ─────────────────────────────────────────────────── */

#define ALARM_CB_MAX 10

typedef struct AlarmCbNode {
    struct AlarmCbNode *next;
    int (*cb)(LOCALSDK_ALARM_EVENT_INFO *eventInfo);
    int used;
} AlarmCbNode;

static AlarmCbNode      g_alarmCbPool[ALARM_CB_MAX];
static AlarmCbNode     *g_alarmCbHead  = NULL;
static pthread_mutex_t  g_alarmCbMutex = PTHREAD_MUTEX_INITIALIZER;

/* ── IVP globals ─────────────────────────────────────────────────────────── */

static hi_s32       g_ivpHandle         = -1;
static pthread_t    g_ivpDetectThread   = 0;
static volatile int g_ivpDetectRun      = 0;
static uint32_t     g_ivpResourceSize   = 0;
static HI_VOID     *g_ivpResourceBuffer = NULL;
static HI_U64       g_ivpPhysAddr       = 0;
static int32_t      g_ivpInitialized    = 0;

/* ── High-level state ────────────────────────────────────────────────────── */

static pthread_t timeout_thread;
static int       alarm_time_motion    = 0;
static int       alarm_time_humanoid  = 0;
static bool      alarm_initialized    = false;

/* ── Alarm callback pool helpers ─────────────────────────────────────────── */

static AlarmCbNode *alarm_cb_alloc(void)
{
    for (int i = 0; i < ALARM_CB_MAX; i++) {
        if (!g_alarmCbPool[i].used) {
            g_alarmCbPool[i].used = 1;
            g_alarmCbPool[i].next = NULL;
            g_alarmCbPool[i].cb   = NULL;
            return &g_alarmCbPool[i];
        }
    }
    return NULL;
}

static void alarm_cb_free(AlarmCbNode *node)
{
    if (!node) return;
    node->used = 0;
    node->next = NULL;
    node->cb   = NULL;
}

static int alarm_cb_add(AlarmCbNode *node)
{
    if (!node) return LOCALSDK_ERROR;
    if (!g_alarmCbHead) { g_alarmCbHead = node; return LOCALSDK_OK; }
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur->next) cur = cur->next;
    cur->next = node;
    return LOCALSDK_OK;
}

static int alarm_cb_remove(AlarmCbNode *node)
{
    if (!node || !g_alarmCbHead) return LOCALSDK_ERROR;
    if (g_alarmCbHead == node) { g_alarmCbHead = node->next; return LOCALSDK_OK; }
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur->next) {
        if (cur->next == node) { cur->next = node->next; return LOCALSDK_OK; }
        cur = cur->next;
    }
    return LOCALSDK_ERROR;
}

static AlarmCbNode *alarm_cb_find(int (*cb)(LOCALSDK_ALARM_EVENT_INFO *))
{
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur) {
        if (cur->cb == cb) return cur;
        cur = cur->next;
    }
    return NULL;
}

static int alarm_run_callbacks(LOCALSDK_ALARM_EVENT_INFO *eventInfo)
{
    int result = LOCALSDK_OK;
    AlarmCbNode *cur = g_alarmCbHead;
    while (cur) {
        if (cur->cb) result = cur->cb(eventInfo);
        cur = cur->next;
    }
    return result;
}

static int alarm_register_callback(int (*cb)(LOCALSDK_ALARM_EVENT_INFO *))
{
    if (!cb) return LOCALSDK_ERROR;
    pthread_mutex_lock(&g_alarmCbMutex);
    if (alarm_cb_find(cb)) { pthread_mutex_unlock(&g_alarmCbMutex); return LOCALSDK_OK; }
    AlarmCbNode *node = alarm_cb_alloc();
    if (!node) { pthread_mutex_unlock(&g_alarmCbMutex); return LOCALSDK_ERROR; }
    node->cb = cb;
    int result = alarm_cb_add(node);
    if (result != LOCALSDK_OK) alarm_cb_free(node);
    pthread_mutex_unlock(&g_alarmCbMutex);
    return result;
}

static int alarm_unregister_callback(int (*cb)(LOCALSDK_ALARM_EVENT_INFO *))
{
    if (!cb) return LOCALSDK_ERROR;
    pthread_mutex_lock(&g_alarmCbMutex);
    AlarmCbNode *node = alarm_cb_find(cb);
    if (!node) { pthread_mutex_unlock(&g_alarmCbMutex); return LOCALSDK_ERROR; }
    int result = alarm_cb_remove(node);
    alarm_cb_free(node);
    pthread_mutex_unlock(&g_alarmCbMutex);
    return result;
}

/* ── IVP resource management ─────────────────────────────────────────────── */

static uint32_t ivp_get_file_size(const char *filename)
{
    if (!filename) return 0;
    FILE *fp = fopen(filename, "rb");
    if (!fp) return 0;
    fseek(fp, 0, SEEK_END);
    uint32_t sz = (uint32_t)ftell(fp);
    fclose(fp);
    return sz;
}

static int32_t ivp_read_file(const char *filename, uint8_t *buf, uint32_t size)
{
    if (!filename || !buf || size == 0) return LOCALSDK_ERROR;
    FILE *fp = fopen(filename, "rb");
    if (!fp) return LOCALSDK_ERROR;
    uint32_t n = (uint32_t)fread(buf, 1, size, fp);
    fclose(fp);
    return (n == size) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static int32_t ivp_load_resource(const char *oms_file)
{
    HI_U64   phys_addr = 0;
    HI_VOID *virt_addr = NULL;
    int32_t  result;

    uint32_t file_size = ivp_get_file_size(oms_file);
    if (file_size == 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] Cannot get size of OMS file: %s", oms_file ? oms_file : "(null)");
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_SYS_MmzAlloc(&phys_addr, &virt_addr, "IVP_MODEL", NULL, file_size);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] MmzAlloc failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    if (ivp_read_file(oms_file, (uint8_t *)virt_addr, file_size) != LOCALSDK_OK) {
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] Failed to read OMS file into MMZ");
        HI_MPI_SYS_MmzFree(phys_addr, virt_addr);
        return LOCALSDK_ERROR;
    }

    hi_ivp_mem_info mem_info;
    memset(&mem_info, 0, sizeof(mem_info));
    mem_info.physical_addr = (hi_u64)phys_addr;
    mem_info.virtual_addr  = (hi_u64)(uintptr_t)virt_addr;
    mem_info.memory_size   = file_size;

    result = hi_ivp_load_resource_from_memory(&mem_info, &g_ivpHandle);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] hi_ivp_load_resource_from_memory failed: 0x%x", result);
        HI_MPI_SYS_MmzFree(phys_addr, virt_addr);
        return LOCALSDK_ERROR;
    }

    g_ivpResourceSize   = file_size;
    g_ivpResourceBuffer = virt_addr;
    g_ivpPhysAddr       = phys_addr;
    return LOCALSDK_OK;
}

static void ivp_unload_resource(void)
{
    if (g_ivpHandle >= 0) {
        hi_ivp_unload_resource(g_ivpHandle);
        g_ivpHandle = -1;
    }
    if (g_ivpResourceBuffer) {
        HI_MPI_SYS_MmzFree(g_ivpPhysAddr, g_ivpResourceBuffer);
        g_ivpPhysAddr       = 0;
        g_ivpResourceBuffer = NULL;
    }
    g_ivpResourceSize = 0;
}

static int32_t ivp_mpp_init(int32_t width, int32_t height, const char *oms_file)
{
    (void)width; (void)height;
    if (g_ivpInitialized) return LOCALSDK_OK;

    if (!oms_file) {
        const board_cfg_t *board = platform_get_board_cfg();
        oms_file = board ? board->ivp_oms_path : NULL;
    }

    int32_t result = hi_ivp_init();
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] hi_ivp_init failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    if (ivp_load_resource(oms_file) != LOCALSDK_OK) {
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] ivp_load_resource failed");
        hi_ivp_deinit();
        return LOCALSDK_ERROR;
    }

    g_ivpInitialized = 1;
    LOGGER(LOGGER_LEVEL_DEBUG, "[alarm][ivp] IVP initialized");
    return LOCALSDK_OK;
}

static void ivp_mpp_deinit(void)
{
    if (!g_ivpInitialized) return;
    ivp_unload_resource();
    hi_ivp_deinit();
    g_ivpInitialized = 0;
}

/* ── IVP detection thread ─────────────────────────────────────────────────── */

#define IVP_OBJ_CAPACITY 16

static int32_t ivp_process_and_dispatch(VIDEO_FRAME_INFO_S *frame)
{
    static hi_ivp_obj s_ivpObjBuf[HI_IVP_MAX_CLASS][IVP_OBJ_CAPACITY];
    hi_ivp_obj_array objs;
    LOCALSDK_ALARM_EVENT_INFO ev;
    int32_t result;

    memset(&objs, 0, sizeof(objs));
    for (int c = 0; c < HI_IVP_MAX_CLASS; c++) {
        objs.obj_class[c].objs         = s_ivpObjBuf[c];
        objs.obj_class[c].rect_capcity = IVP_OBJ_CAPACITY;
    }

    result = hi_ivp_process_ex(g_ivpHandle, frame, &objs);
    if (result != HI_SUCCESS) return LOCALSDK_ERROR;

    /* Scale from secondary (640x360) to primary (1920x1080) space */
    const uint32_t sx = BOARD_WIDTH  / BOARD_SUB_WIDTH;
    const uint32_t sy = BOARD_HEIGHT / BOARD_SUB_HEIGHT;

    memset(&ev, 0, sizeof(ev));
    int n = 0;
    if (objs.class_num > 0) {
        hi_ivp_obj_of_one_class *cls = &objs.obj_class[0]; /* humanoid class */
        int cnt = (int)cls->rect_num;
        if (cnt > LOCALSDK_ALARM_MAXIMUM_OBJECTS) cnt = LOCALSDK_ALARM_MAXIMUM_OBJECTS;
        for (int i = 0; i < cnt && cls->objs; i++) {
            hi_ivp_rect *r = &cls->objs[i].rect;
            ev.objects[n].type   = LOCALSDK_ALARM_TYPE_HUMANOID;
            ev.objects[n].state  = 1;
            ev.objects[n].x      = (uint32_t)(r->x > 0 ? r->x : 0) * sx;
            ev.objects[n].y      = (uint32_t)(r->y > 0 ? r->y : 0) * sy;
            ev.objects[n].width  = r->width  * sx;
            ev.objects[n].height = r->height * sy;
            n++;
        }
    }
    ev.type  = LOCALSDK_ALARM_TYPE_HUMANOID;
    ev.state = (n > 0) ? 1 : 0;

    alarm_run_callbacks(&ev);
    return LOCALSDK_OK;
}

static void *ivp_detect_thread(void *arg)
{
    VIDEO_FRAME_INFO_S frame;
    int32_t ret;
    (void)arg;

    LOGGER(LOGGER_LEVEL_DEBUG, "[alarm][ivp] detection thread started");
    while (g_ivpDetectRun) {
        memset(&frame, 0, sizeof(frame));
        ret = HI_MPI_VPSS_GetChnFrame(video_get_vpss_grp(), LOCALSDK_VIDEO_SECONDARY_CHANNEL,
                                      &frame, 1000);
        if (ret != HI_SUCCESS) { usleep(50000); continue; }
        ivp_process_and_dispatch(&frame);
        HI_MPI_VPSS_ReleaseChnFrame(video_get_vpss_grp(), LOCALSDK_VIDEO_SECONDARY_CHANNEL, &frame);
    }
    LOGGER(LOGGER_LEVEL_DEBUG, "[alarm][ivp] detection thread stopped");
    return NULL;
}

static int32_t ivp_detect_start(void)
{
    if (g_ivpDetectRun) return LOCALSDK_OK;
    g_ivpDetectRun = 1;
    if (pthread_create(&g_ivpDetectThread, NULL, ivp_detect_thread, NULL) != 0) {
        g_ivpDetectRun = 0;
        LOGGER(LOGGER_LEVEL_ERROR, "[alarm][ivp] failed to start detection thread");
        return LOCALSDK_ERROR;
    }
    return LOCALSDK_OK;
}

static void ivp_detect_stop(void)
{
    if (!g_ivpDetectRun) return;
    g_ivpDetectRun = 0;
    if (g_ivpDetectThread) {
        pthread_join(g_ivpDetectThread, NULL);
        g_ivpDetectThread = 0;
    }
}

/* ── MQTT alarm notification ─────────────────────────────────────────────── */

static bool alarm_state_mqtt(bool motion, bool humanoid)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    yyjson_mut_doc *json_doc = yyjson_mut_doc_new(NULL);
    yyjson_mut_val *json_root = yyjson_mut_obj(json_doc);
    yyjson_mut_doc_set_root(json_doc, json_root);

    if (yyjson_mut_obj_add_bool(json_doc, json_root, "motion", motion))
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_bool(motion)");
    else
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_bool()");

    if (yyjson_mut_obj_add_bool(json_doc, json_root, "humanoid", humanoid))
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_obj_add_bool(humanoid)");
    else
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_obj_add_bool()");

    const char *json = yyjson_mut_write(json_doc, 0, NULL);
    if (result &= !!json) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "yyjson_mut_write()");

        char *alarm_topic = mqtt_fulltopic(MQTT_ALARM_TOPIC);
        if (result &= mqtt_send(alarm_topic, (char *)json))
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "mqtt_send(MQTT_ALARM_TOPIC)");
        else
            LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "mqtt_send(MQTT_ALARM_TOPIC)");

        free(alarm_topic);
        free((void *)json);
    } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "yyjson_mut_write()");

    yyjson_mut_doc_free(json_doc);

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

/* ── Alarm state timeout thread ──────────────────────────────────────────── */

static void *alarm_state_timeout(void *args)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");

    bool alarm_state_motion   = false;
    bool alarm_state_humanoid = false;

    alarm_time_motion   = 0;
    alarm_time_humanoid = 0;

    do {
        bool alarm_change_motion   = false;
        bool alarm_change_humanoid = false;

        if (alarm_time_motion > 0) {
            if (alarm_state_motion) {
                if ((int)time(NULL) - alarm_time_motion > APP_CFG.alarm.motion_timeout) {
                    alarm_time_motion   = 0;
                    alarm_state_motion  = false;
                    alarm_change_motion = true;
                }
            } else {
                alarm_state_motion  = true;
                alarm_change_motion = true;
            }
        }

        if (alarm_time_humanoid > 0) {
            if (alarm_state_humanoid) {
                if ((int)time(NULL) - alarm_time_humanoid > APP_CFG.alarm.humanoid_timeout) {
                    alarm_time_humanoid   = 0;
                    alarm_state_humanoid  = false;
                    alarm_change_humanoid = true;
                }
            } else {
                alarm_state_humanoid  = true;
                alarm_change_humanoid = true;
            }
        }

        if (alarm_change_motion || alarm_change_humanoid) {
            if (alarm_change_motion) {
                LOGGER(LOGGER_LEVEL_INFO, "Change %s status: %d", "motion", alarm_state_motion);
                if (alarm_state_motion && APP_CFG.alarm.motion_detect_exec && APP_CFG.alarm.motion_detect_exec[0]) {
                    if (system(APP_CFG.alarm.motion_detect_exec) == 0) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "system(motion_detect_exec)");
                    else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "system(motion_detect_exec)");
                } else if (!alarm_state_motion && APP_CFG.alarm.motion_lost_exec && APP_CFG.alarm.motion_lost_exec[0]) {
                    if (system(APP_CFG.alarm.motion_lost_exec) == 0) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "system(motion_lost_exec)");
                    else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "system(motion_lost_exec)");
                }
            }

            if (alarm_change_humanoid) {
                LOGGER(LOGGER_LEVEL_INFO, "Change %s status: %d", "humanoid", alarm_state_humanoid);
                if (alarm_state_humanoid && APP_CFG.alarm.humanoid_detect_exec && APP_CFG.alarm.humanoid_detect_exec[0]) {
                    if (system(APP_CFG.alarm.humanoid_detect_exec) == 0) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "system(humanoid_detect_exec)");
                    else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "system(humanoid_detect_exec)");
                } else if (!alarm_state_humanoid && APP_CFG.alarm.humanoid_lost_exec && APP_CFG.alarm.humanoid_lost_exec[0]) {
                    if (system(APP_CFG.alarm.humanoid_lost_exec) == 0) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "system(humanoid_lost_exec)");
                    else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "system(humanoid_lost_exec)");
                }
            }

            if (mqtt_is_ready()) {
                if (alarm_state_mqtt(alarm_state_motion, alarm_state_humanoid)) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "alarm_state_mqtt()");
                else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "alarm_state_mqtt()");
            }
        }

        sleep(1);
        pthread_testcancel();
    } while (true);

    (void)args;
    return NULL;
}

/* ── Alarm event callback (registered at init) ───────────────────────────── */

static int alarm_state_callback(LOCALSDK_ALARM_EVENT_INFO *eventInfo)
{
    int result = LOCALSDK_OK;

    if (eventInfo) {
        result = osd_rectangles_callback(eventInfo);

        if (eventInfo->state) {
            int ts = (int)time(NULL);
            switch (eventInfo->type) {
                case LOCALSDK_ALARM_TYPE_MOTION:
                    alarm_time_motion = ts;
                    break;
                case LOCALSDK_ALARM_TYPE_HUMANOID:
                    alarm_time_humanoid = ts;
                    break;
                default:
                    LOGGER(LOGGER_LEVEL_INFO, "Change %s status: %d", "unknown", eventInfo->state);
                    result = LOCALSDK_ERROR;
            }
        }
    } else result = LOCALSDK_ERROR;

    return result;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool alarm_switch(bool state)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");

    if (APP_CFG.alarm.enable) {
        LOGGER(LOGGER_LEVEL_INFO, "Alarm %s", state ? "enabled" : "disabled");
        /* IVP-based detection runs continuously once started; the alarm switch
           only gates whether the application acts on events (timeout thread). */
    } else {
        LOGGER(LOGGER_LEVEL_INFO, "Alarm switch ignored, because alarms are disabled");
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", "true");
    return true;
}

bool alarm_init(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (APP_CFG.alarm.enable) {
        /* Start IVP humanoid detector on the 640x360 secondary stream */
        if (result &= (ivp_mpp_init(BOARD_SUB_WIDTH, BOARD_SUB_HEIGHT, NULL) == LOCALSDK_OK)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "ivp_mpp_init()");
            if (result &= (ivp_detect_start() == LOCALSDK_OK)) {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "ivp_detect_start()");
            } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "ivp_detect_start()");
        } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "ivp_mpp_init()");

        /* Register our event dispatch callback */
        if (result &= (alarm_register_callback(alarm_state_callback) == LOCALSDK_OK)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "alarm_register_callback()");
            /* Start the state timeout/MQTT thread */
            if (result &= (pthread_create(&timeout_thread, NULL, alarm_state_timeout, NULL) == 0)) {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "pthread_create(timeout_thread)");
            } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "pthread_create(timeout_thread)");
        } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "alarm_register_callback()");
    } else {
        LOGGER(LOGGER_LEVEL_INFO, "Alarm init skipped, because alarms are disabled");
    }

    if (result) alarm_initialized = true;

    if (!result) {
        if (alarm_free()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "alarm_free()");
        else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "alarm_free()");
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

bool alarm_free(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (!alarm_initialized) {
        LOGGER(LOGGER_LEVEL_DEBUG, "alarm_free() skipped, alarm was not initialized.");
        return result;
    }

    /* Stop the timeout/MQTT thread */
    if (timeout_thread) {
        if (result &= (pthread_cancel(timeout_thread) == 0))
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "pthread_cancel(timeout_thread)");
        else
            LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "pthread_cancel(timeout_thread)");
    }

    /* Remove event callback */
    if (result &= (alarm_unregister_callback(alarm_state_callback) == LOCALSDK_OK))
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "alarm_unregister_callback()");
    else
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "alarm_unregister_callback()");

    /* Stop IVP detection and deinit */
    ivp_detect_stop();
    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "ivp_detect_stop()");

    ivp_mpp_deinit();
    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "ivp_mpp_deinit()");

    alarm_initialized = false;

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}
