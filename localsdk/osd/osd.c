#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#include "./osd.h"
#include "./../localsdk.h"
#include "./../platform/board_mjsxj02hl.h"
#include "./../../logger/logger.h"
#include "./../../configs/configs.h"

#include "hi_comm_region.h"
#include "hi_comm_sys.h"
#include "mpi_region.h"

/* ── Internal OSD channel state ─────────────────────────────────────────── */

typedef struct {
    uint32_t    timestamp_en;
    uint32_t    timestamp_show;
    RGN_HANDLE  timestamp_hdl;
    uint32_t    padding1;
    uint32_t    logo_en;
    uint32_t    logo_show;
    RGN_HANDLE  logo_hdl;
    uint32_t    padding2[17];
    uint32_t    rects_en;
    uint32_t    rects_show;
    RGN_HANDLE  rects_hdl;
    uint32_t    padding3[1];
    LOCALSDK_OSD_OPTIONS opts;
    uint32_t    padding4[17];
} OSD_CHANNEL_PARAMS;

static uint8_t g_osdParams[2 * 220];

static inline OSD_CHANNEL_PARAMS *osd_get_params(int chn)
{
    if (chn < 0 || chn > 1) return NULL;
    return (OSD_CHANNEL_PARAMS *)&g_osdParams[chn * 220];
}

/* ── Bitmap font (8x8, public-domain font8x8_basic subset) ──────────────── */

static const unsigned char FONT_DIGIT[10][8] = {
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, /* 0 */
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, /* 1 */
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, /* 2 */
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, /* 3 */
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, /* 4 */
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, /* 5 */
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, /* 6 */
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* 7 */
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, /* 8 */
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, /* 9 */
};
static const unsigned char FONT_DASH[8]  = {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00};
static const unsigned char FONT_COLON[8] = {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00};
static const unsigned char FONT_SPACE[8] = {0,0,0,0,0,0,0,0};

static const unsigned char *osd_font_glyph(char c)
{
    if (c >= '0' && c <= '9') return FONT_DIGIT[c - '0'];
    if (c == '-') return FONT_DASH;
    if (c == ':') return FONT_COLON;
    return FONT_SPACE;
}

static void osd_draw_glyph(uint16_t *base, uint32_t stride_px,
                            uint32_t canvas_w, uint32_t canvas_h,
                            int x0, int y0, const unsigned char *g,
                            int scale, uint16_t color)
{
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (!((g[row] >> col) & 1)) continue;
            for (int sy = 0; sy < scale; sy++) {
                int py = y0 + row * scale + sy;
                if (py < 0 || (uint32_t)py >= canvas_h) continue;
                for (int sx = 0; sx < scale; sx++) {
                    int px = x0 + col * scale + sx;
                    if (px < 0 || (uint32_t)px >= canvas_w) continue;
                    base[py * stride_px + px] = color;
                }
            }
        }
    }
}

/* Set one pixel in a 2bpp (4 pixels/byte) overlay canvas. */
static inline void osd_px2bpp(uint8_t *base, uint32_t stride,
                               uint32_t x, uint32_t y, uint8_t v)
{
    uint8_t *p = base + y * stride + (x >> 2);
    int sh = (int)(x & 3) * 2;
    *p = (uint8_t)((*p & ~(3 << sh)) | ((v & 3) << sh));
}

/* ── Region helpers ──────────────────────────────────────────────────────── */

static int32_t osd_show_region(RGN_HANDLE handle, int chn, int x, int y, bool show)
{
    MPP_CHN_S      stChn;
    RGN_CHN_ATTR_S stChnAttr;
    HI_S32         result;

    stChn.enModId  = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = chn;

    result = HI_MPI_RGN_GetDisplayAttr(handle, &stChn, &stChnAttr);
    if (result != HI_SUCCESS) {
        if (show) {
            memset(&stChnAttr, 0, sizeof(stChnAttr));
            stChnAttr.bShow  = HI_TRUE;
            stChnAttr.enType = OVERLAY_RGN;
            stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = x;
            stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = y;
            /* Hisilicon OVERLAY alpha range is [0,128] (128 = fully opaque). */
            stChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 0;
            stChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 128;
            /* Distinct layer per handle to avoid ILLEGAL_PARAM on second attach. */
            stChnAttr.unChnAttr.stOverlayChn.u32Layer = (HI_U32)(handle & 7);

            result = HI_MPI_RGN_AttachToChn(handle, &stChn, &stChnAttr);
            if (result != HI_SUCCESS) {
                LOGGER(LOGGER_LEVEL_ERROR, "[osd] RGN_AttachToChn failed: 0x%x", result);
                return LOCALSDK_ERROR;
            }
        }
    } else {
        stChnAttr.bShow = show ? HI_TRUE : HI_FALSE;
        HI_MPI_RGN_SetDisplayAttr(handle, &stChn, &stChnAttr);
    }
    return LOCALSDK_OK;
}

static int32_t osd_region_init(RGN_HANDLE handle, uint32_t width, uint32_t height)
{
    RGN_ATTR_S stRgnAttr;
    HI_S32     result;

    memset(&stRgnAttr, 0, sizeof(stRgnAttr));
    stRgnAttr.enType                          = OVERLAY_RGN;
    stRgnAttr.unAttr.stOverlay.enPixelFmt     = PIXEL_FORMAT_ARGB_1555;
    stRgnAttr.unAttr.stOverlay.stSize.u32Width  = width;
    stRgnAttr.unAttr.stOverlay.stSize.u32Height = height;
    stRgnAttr.unAttr.stOverlay.u32BgColor      = 0;
    stRgnAttr.unAttr.stOverlay.u32CanvasNum    = 2; /* double-buffered */

    result = HI_MPI_RGN_Create(handle, &stRgnAttr);
    if (result != HI_SUCCESS && result != HI_ERR_RGN_EXIST) {
        LOGGER(LOGGER_LEVEL_ERROR, "[osd] RGN_Create failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }
    return LOCALSDK_OK;
}

/* ── OSD MPP operations ──────────────────────────────────────────────────── */

static int osd_set_parameters(int chn, LOCALSDK_OSD_OPTIONS *options)
{
    OSD_CHANNEL_PARAMS *params = osd_get_params(chn);
    if (!params || !options) return LOCALSDK_ERROR;

    memcpy(&params->opts, options, sizeof(LOCALSDK_OSD_OPTIONS));
    params->timestamp_en = 1;
    params->logo_en      = 1;
    params->rects_en     = 1;
    return LOCALSDK_OK;
}

static int osd_update_logo(int chn, bool state)
{
    OSD_CHANNEL_PARAMS *params = osd_get_params(chn);
    if (!params) return LOCALSDK_ERROR;

    if (params->logo_hdl == 0) {
        params->logo_hdl = chn * 3 + 1;
        osd_region_init(params->logo_hdl, 128, 64);
    }
    /* Logo region is intentionally empty (no proprietary MI asset embedded). */
    return osd_show_region(params->logo_hdl, chn,
                           params->opts.oemlogo_x, params->opts.oemlogo_y, state);
}

static int osd_update_timestamp(int chn, bool state, struct tm *timestamp)
{
    OSD_CHANNEL_PARAMS *params = osd_get_params(chn);
    RGN_CANVAS_INFO_S   stCanvas;
    HI_S32              result;

    if (!params) return LOCALSDK_ERROR;

    /* Glyph scale: base 3 (~24 px tall at 1080p), adjusted by config size. */
    int scale = 3;
    if (params->opts.datetime_increase > 1) scale *= (int)params->opts.datetime_increase;
    if (params->opts.datetime_reduce   > 1) scale /= (int)params->opts.datetime_reduce;
    if (scale < 1) scale = 1;
    int shadow = (scale >= 2) ? 2 : 1;

    if (params->timestamp_hdl == 0) {
        params->timestamp_hdl = chn * 3 + 0;
        /* Canvas fits "YYYY-MM-DD HH:MM:SS" (19 chars) + shadow margin. */
        uint32_t w = (uint32_t)(20 * 8 * scale + shadow);
        uint32_t h = (uint32_t)(8 * scale + shadow);
        w = (w + 1) & ~1u;
        h = (h + 1) & ~1u;
        osd_region_init(params->timestamp_hdl, w, h);
    }

    if (state && timestamp) {
        char buf[24];
        int slen = snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                            timestamp->tm_year + 1900, timestamp->tm_mon + 1,
                            timestamp->tm_mday, timestamp->tm_hour,
                            timestamp->tm_min, timestamp->tm_sec);

        result = HI_MPI_RGN_GetCanvasInfo(params->timestamp_hdl, &stCanvas);
        if (result == HI_SUCCESS) {
            uint16_t *pix       = (uint16_t *)(uintptr_t)stCanvas.u64VirtAddr;
            uint32_t  stride_px = stCanvas.u32Stride / 2;
            uint32_t  cw        = stCanvas.stSize.u32Width;
            uint32_t  ch        = stCanvas.stSize.u32Height;

            memset(pix, 0, stCanvas.u32Stride * ch);

            int x = 0;
            for (int i = 0; i < slen; i++) {
                const unsigned char *g = osd_font_glyph(buf[i]);
                /* Black drop shadow, then white text on top. */
                osd_draw_glyph(pix, stride_px, cw, ch, x + shadow, shadow, g, scale, 0x8000);
                osd_draw_glyph(pix, stride_px, cw, ch, x, 0, g, scale, 0xFFFF);
                x += 8 * scale;
                if ((uint32_t)(x + 8 * scale) > cw) break;
            }
            HI_MPI_RGN_UpdateCanvas(params->timestamp_hdl);
        }
    }

    return osd_show_region(params->timestamp_hdl, chn,
                           params->opts.datetime_x, params->opts.datetime_y, state);
}

/* Detection boxes: full-frame 2bpp OVERLAY with a 2-entry colour LUT, matching
   the original firmware (1920x1080, ARGB_2BPP, ~1 MB). COVER_RGN is not
   supported on VENC channels (0xa0038008 NOT_SUPPORT). Outlines drawn with pixel
   value 1 → ColourLUT; value 0 = transparent. */
static int osd_update_rect_multi(int chn, bool state, LOCALSDK_OSD_RECTANGLES *rectangles)
{
    OSD_CHANNEL_PARAMS *params = osd_get_params(chn);
    HI_S32 result;
    if (!params) return LOCALSDK_ERROR;

    if (params->rects_hdl == 0) {
        params->rects_hdl = chn * 3 + 2;
        RGN_ATTR_S a;
        memset(&a, 0, sizeof(a));
        a.enType                             = OVERLAY_RGN;
        a.unAttr.stOverlay.enPixelFmt        = PIXEL_FORMAT_ARGB_2BPP;
        a.unAttr.stOverlay.stSize.u32Width   = BOARD_WIDTH;
        a.unAttr.stOverlay.stSize.u32Height  = BOARD_HEIGHT;
        a.unAttr.stOverlay.u32BgColor        = 0;
        a.unAttr.stOverlay.u32CanvasNum      = 2;
        result = HI_MPI_RGN_Create(params->rects_hdl, &a);
        if (result != HI_SUCCESS && result != HI_ERR_RGN_EXIST) {
            LOGGER(LOGGER_LEVEL_ERROR, "[osd] rect RGN_Create(2bpp) failed: 0x%x", result);
            params->rects_hdl = 0;
            return LOCALSDK_ERROR;
        }
    }

    if (state && rectangles && rectangles->count > 0) {
        RGN_CANVAS_INFO_S cv;
        if (HI_MPI_RGN_GetCanvasInfo(params->rects_hdl, &cv) == HI_SUCCESS) {
            uint8_t *base   = (uint8_t *)(uintptr_t)cv.u64VirtAddr;
            uint32_t stride = cv.u32Stride;
            uint32_t cw = cv.stSize.u32Width, ch = cv.stSize.u32Height;
            const uint32_t T = 4; /* outline thickness in pixels */

            memset(base, 0, stride * ch);
            for (uint32_t o = 0; o < rectangles->count && o < LOCALSDK_ALARM_MAXIMUM_OBJECTS; o++) {
                if (!rectangles->objects[o].visible) continue;
                uint32_t x = rectangles->objects[o].x, y = rectangles->objects[o].y;
                uint32_t w = rectangles->objects[o].width, h = rectangles->objects[o].height;
                if (x >= cw || y >= ch) continue;
                uint32_t x2 = (x + w < cw) ? x + w : cw - 1;
                uint32_t y2 = (y + h < ch) ? y + h : ch - 1;
                /* Value 3 = humanoid (matches original 0xFF for type!=3); 0 = transparent. */
                for (uint32_t px = x; px <= x2; px++)
                    for (uint32_t t = 0; t < T; t++) {
                        if (y + t < ch) osd_px2bpp(base, stride, px, y + t, 3);
                        if (y2 >= t)    osd_px2bpp(base, stride, px, y2 - t, 3);
                    }
                for (uint32_t py = y; py <= y2; py++)
                    for (uint32_t t = 0; t < T; t++) {
                        if (x + t < cw) osd_px2bpp(base, stride, x + t, py, 3);
                        if (x2 >= t)    osd_px2bpp(base, stride, x2 - t, py, 3);
                    }
            }
            HI_MPI_RGN_UpdateCanvas(params->rects_hdl);
        }
    }

    /* Attach on first use (with the colour LUT), then toggle visibility only. */
    MPP_CHN_S  stChn;
    RGN_CHN_ATTR_S ca;
    stChn.enModId = HI_ID_VENC; stChn.s32DevId = 0; stChn.s32ChnId = chn;
    bool want = (state && rectangles && rectangles->count > 0);
    if (HI_MPI_RGN_GetDisplayAttr(params->rects_hdl, &stChn, &ca) != HI_SUCCESS) {
        if (!want) return LOCALSDK_OK;
        memset(&ca, 0, sizeof(ca));
        ca.bShow  = HI_TRUE;
        ca.enType = OVERLAY_RGN;
        ca.unChnAttr.stOverlayChn.stPoint.s32X = 0;
        ca.unChnAttr.stOverlayChn.stPoint.s32Y = 0;
        ca.unChnAttr.stOverlayChn.u32FgAlpha   = 128;
        ca.unChnAttr.stOverlayChn.u32BgAlpha   = 0;
        ca.unChnAttr.stOverlayChn.u32Layer     = 5;
        /* LUT values from original firmware /proc/umap/rgn. */
        ca.unChnAttr.stOverlayChn.u16ColorLUT[0] = 916;
        ca.unChnAttr.stOverlayChn.u16ColorLUT[1] = 31106;
        result = HI_MPI_RGN_AttachToChn(params->rects_hdl, &stChn, &ca);
        if (result != HI_SUCCESS)
            LOGGER(LOGGER_LEVEL_ERROR, "[osd] rect AttachToChn failed: 0x%x", result);
        return (result == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
    }
    ca.bShow = want ? HI_TRUE : HI_FALSE;
    HI_MPI_RGN_SetDisplayAttr(params->rects_hdl, &stChn, &ca);
    return LOCALSDK_OK;
}

/* ── Timestamp timer thread ──────────────────────────────────────────────── */

static pthread_t datetime_thread;

static void *osd_datetime_timer(void *args)
{
    (void)args;
    while (true) {
        time_t      current_time = time(NULL);
        struct tm  *ts           = localtime(&current_time);
        if (osd_update_timestamp(LOCALSDK_VIDEO_PRIMARY_CHANNEL, true, ts) != LOCALSDK_OK)
            LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "osd_update_timestamp(true)");
        sleep(1);
        pthread_testcancel();
    }
    osd_update_timestamp(LOCALSDK_VIDEO_PRIMARY_CHANNEL, false, NULL);
    return NULL;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool osd_is_enabled(void)
{
    return APP_CFG.osd.enable;
}

bool osd_init(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (osd_is_enabled()) {
        LOCALSDK_OSD_OPTIONS opts = {
            .flags            = 67,
            .datetime_x       = APP_CFG.osd.datetime_x,
            .datetime_y       = APP_CFG.osd.datetime_y,
            .datetime_reduce  = (APP_CFG.osd.datetime_size < 0) ? (uint32_t)(abs(APP_CFG.osd.datetime_size) + 1) : 1u,
            .datetime_increase = (APP_CFG.osd.datetime_size > 0) ? (uint32_t)(APP_CFG.osd.datetime_size + 1) : 1u,
            .oemlogo_x        = APP_CFG.osd.oemlogo_x,
            .oemlogo_y        = APP_CFG.osd.oemlogo_y,
            .oemlogo_reduce   = (APP_CFG.osd.oemlogo_size < 0) ? (uint32_t)(abs(APP_CFG.osd.oemlogo_size) + 1) : 1u,
            .oemlogo_increase = (APP_CFG.osd.oemlogo_size > 0) ? (uint32_t)(APP_CFG.osd.oemlogo_size + 1) : 1u,
        };
        if (result &= (osd_set_parameters(LOCALSDK_VIDEO_PRIMARY_CHANNEL, &opts) == LOCALSDK_OK))
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "osd_set_parameters()");
        else
            LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "osd_set_parameters()");

        if (!result) {
            if (osd_free()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "osd_free()");
            else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "osd_free()");
        }
    } else {
        LOGGER(LOGGER_LEVEL_INFO, "OSD is disabled in the settings.");
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

bool osd_postinit(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (osd_is_enabled()) {
        if (APP_CFG.osd.oemlogo) {
            if (result &= (osd_update_logo(LOCALSDK_VIDEO_PRIMARY_CHANNEL, true) == LOCALSDK_OK))
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "osd_update_logo(true)");
            else
                LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "osd_update_logo(true)");
        }

        if (APP_CFG.osd.datetime) {
            if (result &= (pthread_create(&datetime_thread, NULL, osd_datetime_timer, NULL) == 0))
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "pthread_create(datetime_thread)");
            else
                LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "pthread_create(datetime_thread)");
        }
        /* Rectangles are fed on-demand via osd_rectangles_callback(). */
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

bool osd_free(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (osd_is_enabled()) {
        if (APP_CFG.osd.oemlogo) {
            if (result &= (osd_update_logo(LOCALSDK_VIDEO_PRIMARY_CHANNEL, false) == LOCALSDK_OK))
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "osd_update_logo(false)");
            else
                LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "osd_update_logo(false)");
        }

        if (APP_CFG.osd.datetime) {
            if (datetime_thread) {
                if (result &= (pthread_cancel(datetime_thread) == 0))
                    LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "pthread_cancel(datetime_thread)");
                else
                    LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "pthread_cancel(datetime_thread)");
            }
            if (result &= (osd_update_timestamp(LOCALSDK_VIDEO_PRIMARY_CHANNEL, false, NULL) == LOCALSDK_OK))
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "osd_update_timestamp(false)");
            else
                LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "osd_update_timestamp(false)");
        }

        if (APP_CFG.osd.motion || APP_CFG.osd.humanoid) {
            if (result &= (osd_update_rect_multi(LOCALSDK_VIDEO_PRIMARY_CHANNEL, false, NULL) == LOCALSDK_OK))
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "osd_update_rect_multi(false)");
            else
                LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "osd_update_rect_multi(false)");
        }
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

int osd_rectangles_callback(LOCALSDK_ALARM_EVENT_INFO *eventInfo)
{
    if (!osd_is_enabled()) return LOCALSDK_OK;
    if (!APP_CFG.osd.motion && !APP_CFG.osd.humanoid) return LOCALSDK_OK;

    LOCALSDK_OSD_RECTANGLES rectangles;
    rectangles.count = 0;

    for (int i = 0; i < LOCALSDK_ALARM_MAXIMUM_OBJECTS; i++) {
        bool want = (APP_CFG.osd.motion    && eventInfo->objects[i].type == LOCALSDK_ALARM_TYPE_MOTION)
                 || (APP_CFG.osd.humanoid  && eventInfo->objects[i].type == LOCALSDK_ALARM_TYPE_HUMANOID);
        if (!want || !eventInfo->objects[i].state) continue;

        uint32_t n = rectangles.count++;
        rectangles.objects[n].x       = eventInfo->objects[i].x;
        rectangles.objects[n].y       = eventInfo->objects[i].y;
        rectangles.objects[n].width   = eventInfo->objects[i].width;
        rectangles.objects[n].height  = eventInfo->objects[i].height;
        rectangles.objects[n].visible = 1;
        rectangles.objects[n].color   = (eventInfo->objects[i].type == LOCALSDK_ALARM_TYPE_HUMANOID)
                                        ? LOCALSDK_OSD_COLOR_ORANGE
                                        : LOCALSDK_OSD_COLOR_GREEN;
    }

    if (osd_update_rect_multi(LOCALSDK_VIDEO_PRIMARY_CHANNEL, true, &rectangles) != LOCALSDK_OK)
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "osd_update_rect_multi(true)");

    return LOCALSDK_OK;
}
