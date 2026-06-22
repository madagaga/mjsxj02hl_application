#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include "./audio.h"
#include "./../video/video.h"      /* LOCALSDK_VIDEO_PRIMARY/SECONDARY_CHANNEL */
#include "./../platform/platform.h"
#include "./../../logger/logger.h"
#include "./../../configs/configs.h"
#include "./../../rtsp/rtsp.h"

#include "acodec.h"
#include "mpi_ai.h"
#include "mpi_ao.h"
#include "mpi_aenc.h"

/* ── Globals ─────────────────────────────────────────────────────────────── */

static AUDIO_DEV g_aiDev = 0;
static AI_CHN    g_aiChn = 0;
static AUDIO_DEV g_aoDev = 0;
static AO_CHN    g_aoChn = 0;
static AENC_CHN  g_aencChn = 0;

static int32_t   g_audioStarted = 0;
static volatile int g_audioRunning = 0;

#define MAX_AENC_CALLBACKS 4
static int (*g_aencCb[MAX_AENC_CALLBACKS])(LOCALSDK_AUDIO_G711_FRAME_INFO *) = {NULL};
static int g_aencCbCount = 0;

static pthread_t g_audioAiThread   = 0;
static pthread_t g_audioAencThread = 0;

/* ── RTSP capture callbacks ───────────────────────────────────────────────── */

static int g711_capture_callback(int chn, LOCALSDK_AUDIO_G711_FRAME_INFO *frameInfo)
{
    int result = LOCALSDK_OK;
    if (frameInfo && frameInfo->size) {
        if (rtsp_is_enabled(chn)) {
            if (!rtsp_media_frame(chn, frameInfo->data, frameInfo->size,
                                  frameInfo->timestamp, LOCALSDK_AUDIO_G711_FRAME))
                result = LOCALSDK_ERROR;
        }
    }
    return result;
}

static int g711_capture_primary_channel(LOCALSDK_AUDIO_G711_FRAME_INFO *frameInfo)
{
    if (audio_is_enabled(LOCALSDK_VIDEO_PRIMARY_CHANNEL))
        return g711_capture_callback(LOCALSDK_VIDEO_PRIMARY_CHANNEL, frameInfo);
    return LOCALSDK_ERROR;
}

static int g711_capture_secondary_channel(LOCALSDK_AUDIO_G711_FRAME_INFO *frameInfo)
{
    if (audio_is_enabled(LOCALSDK_VIDEO_SECONDARY_CHANNEL))
        return g711_capture_callback(LOCALSDK_VIDEO_SECONDARY_CHANNEL, frameInfo);
    return LOCALSDK_ERROR;
}

/* ── /dev/acodec configuration ───────────────────────────────────────────── */

static void audio_codec_cfg(void)
{
    int fd = open("/dev/acodec", O_RDWR);
    if (fd < 0) return;

    ioctl(fd, ACODEC_SOFT_RESET_CTRL, NULL);

    ACODEC_FS_E fs = ACODEC_FS_8000;
    ioctl(fd, ACODEC_SET_I2S1_FS, &fs);

    ACODEC_MIXER_E input_mode = ACODEC_MIXER_IN1;
    ioctl(fd, ACODEC_SET_MIXER_MIC, &input_mode);

    int vol = 60;
    ioctl(fd, ACODEC_SET_INPUT_VOL, &vol);

    close(fd);
}

/* ── Capture threads ─────────────────────────────────────────────────────── */

static void *audio_ai_thread(void *arg)
{
    (void)arg;
    AUDIO_FRAME_S stFrame;
    AEC_FRAME_S   stAecFrm;
    fd_set read_fds;
    struct timeval tv;

    AI_CHN_PARAM_S stParam;
    if (HI_MPI_AI_GetChnParam(g_aiDev, g_aiChn, &stParam) == HI_SUCCESS) {
        stParam.u32UsrFrmDepth = 30;
        HI_MPI_AI_SetChnParam(g_aiDev, g_aiChn, &stParam);
    }

    int aiFd = HI_MPI_AI_GetFd(g_aiDev, g_aiChn);

    while (g_audioRunning) {
        FD_ZERO(&read_fds);
        FD_SET(aiFd, &read_fds);
        tv.tv_sec = 1; tv.tv_usec = 0;

        int sel = select(aiFd + 1, &read_fds, NULL, NULL, &tv);
        if (sel <= 0) continue;
        if (!FD_ISSET(aiFd, &read_fds)) continue;

        memset(&stAecFrm, 0, sizeof(stAecFrm));
        if (HI_MPI_AI_GetFrame(g_aiDev, g_aiChn, &stFrame, &stAecFrm, HI_FALSE) != HI_SUCCESS)
            continue;

        HI_MPI_AENC_SendFrame(g_aencChn, &stFrame, &stAecFrm);
        HI_MPI_AI_ReleaseFrame(g_aiDev, g_aiChn, &stFrame, &stAecFrm);
    }
    return NULL;
}

static void *audio_aenc_thread(void *arg)
{
    (void)arg;
    AUDIO_STREAM_S stStream;
    fd_set read_fds;
    struct timeval tv;

    int aencFd = HI_MPI_AENC_GetFd(g_aencChn);

    while (g_audioRunning) {
        FD_ZERO(&read_fds);
        FD_SET(aencFd, &read_fds);
        tv.tv_sec = 1; tv.tv_usec = 0;

        int sel = select(aencFd + 1, &read_fds, NULL, NULL, &tv);
        if (sel <= 0) continue;
        if (!FD_ISSET(aencFd, &read_fds)) continue;

        if (HI_MPI_AENC_GetStream(g_aencChn, &stStream, HI_FALSE) != HI_SUCCESS)
            continue;

        if (g_aencCbCount > 0) {
            LOCALSDK_AUDIO_G711_FRAME_INFO fi;
            memset(&fi, 0, sizeof(fi));
            fi.data      = (signed char *)stStream.pStream;
            fi.size      = stStream.u32Len;
            fi.timestamp = stStream.u64TimeStamp;
            for (int i = 0; i < g_aencCbCount; i++) {
                if (g_aencCb[i]) g_aencCb[i](&fi);
            }
        }

        HI_MPI_AENC_ReleaseStream(g_aencChn, &stStream);
    }
    return NULL;
}

/* ── Internal MPP init/deinit ────────────────────────────────────────────── */

static int audio_mpp_init(void)
{
    const board_cfg_t *board = platform_get_board_cfg();
    AIO_ATTR_S stAioAttr;
    HI_S32 result;

    g_aiDev = board ? board->ai_dev : 0;
    g_aoDev = board ? board->ao_dev : 0;

    memset(&stAioAttr, 0, sizeof(stAioAttr));
    stAioAttr.enSamplerate   = AUDIO_SAMPLE_RATE_8000;
    stAioAttr.enBitwidth     = AUDIO_BIT_WIDTH_16;
    stAioAttr.enWorkmode     = AIO_MODE_I2S_MASTER;
    stAioAttr.u32EXFlag      = 0;
    stAioAttr.u32FrmNum      = 30;
    stAioAttr.u32PtNumPerFrm = 320; /* 40 ms @ 8 kHz */
    stAioAttr.u32ChnCnt      = 1;

    result = HI_MPI_AI_SetPubAttr(g_aiDev, &stAioAttr);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AI_SetPubAttr failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AI_Enable(g_aiDev);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AI_Enable failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AO_SetPubAttr(g_aoDev, &stAioAttr);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AO_SetPubAttr failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AO_Enable(g_aoDev);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AO_Enable failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    audio_codec_cfg();
    return LOCALSDK_OK;
}

static int audio_mpp_enable(void)
{
    HI_S32 result;

    result = HI_MPI_AI_EnableChn(g_aiDev, g_aiChn);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AI_EnableChn failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    result = HI_MPI_AO_EnableChn(g_aoDev, g_aoChn);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AO_EnableChn failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    g_audioStarted = 1;
    return LOCALSDK_OK;
}

static int audio_mpp_run(void)
{
    AENC_CHN_ATTR_S stAencAttr;
    HI_S32 result;

    if (!g_audioStarted) return LOCALSDK_ERROR;
    if (g_audioRunning)  return LOCALSDK_OK;

    HI_MPI_AI_EnableVqe(g_aiDev, g_aiChn);

    static AENC_ATTR_G711_S stAencG711;
    memset(&stAencAttr, 0, sizeof(stAencAttr));
    stAencAttr.enType         = PT_G711A;
    stAencAttr.u32PtNumPerFrm = 320;
    stAencAttr.u32BufSize     = 30;
    stAencAttr.pValue         = &stAencG711;

    result = HI_MPI_AENC_CreateChn(g_aencChn, &stAencAttr);
    if (result != HI_SUCCESS && result != HI_ERR_AENC_EXIST) {
        LOGGER(LOGGER_LEVEL_ERROR, "[audio] AENC_CreateChn failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    /* Register RTSP output callbacks (primary + secondary) */
    g_aencCb[0] = g711_capture_primary_channel;
    g_aencCb[1] = g711_capture_secondary_channel;
    g_aencCbCount = 2;

    g_audioRunning = 1;

    if (pthread_create(&g_audioAiThread, NULL, audio_ai_thread, NULL) != 0) {
        g_audioRunning = 0;
        return LOCALSDK_ERROR;
    }
    if (pthread_create(&g_audioAencThread, NULL, audio_aenc_thread, NULL) != 0) {
        g_audioRunning = 0;
        pthread_join(g_audioAiThread, NULL);
        g_audioAiThread = 0;
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

static int audio_mpp_stop(void)
{
    g_audioRunning = 0;
    if (g_audioAiThread)   { pthread_join(g_audioAiThread, NULL);   g_audioAiThread = 0; }
    if (g_audioAencThread) { pthread_join(g_audioAencThread, NULL); g_audioAencThread = 0; }
    g_audioStarted = 0;
    g_aencCbCount  = 0;
    return LOCALSDK_OK;
}

static int audio_mpp_deinit(void)
{
    HI_MPI_AI_DisableVqe(g_aiDev, g_aiChn);
    HI_MPI_AI_Disable(g_aiDev);
    HI_MPI_AO_Disable(g_aoDev);
    return LOCALSDK_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool audio_is_enabled(int channel)
{
    switch (channel) {
        case LOCALSDK_VIDEO_PRIMARY_CHANNEL:   return APP_CFG.audio.primary_enable;
        case LOCALSDK_VIDEO_SECONDARY_CHANNEL: return APP_CFG.audio.secondary_enable;
        default: return false;
    }
}

int audio_get_ao_dev(void) { return (int)g_aoDev; }
int audio_get_ao_chn(void) { return (int)g_aoChn; }

bool audio_init(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (result &= (audio_mpp_init() == LOCALSDK_OK)) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "audio_mpp_init()");
        if (result &= (audio_mpp_enable() == LOCALSDK_OK)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "audio_mpp_enable()");
            if (result &= (audio_mpp_run() == LOCALSDK_OK)) {
                LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "audio_mpp_run()");
            } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "audio_mpp_run()");
        } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "audio_mpp_enable()");
    } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "audio_mpp_init()");

    if (!result) {
        if (audio_free()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "audio_free()");
        else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "audio_free()");
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

bool audio_free(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (result &= (audio_mpp_stop() == LOCALSDK_OK))
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "audio_mpp_stop()");
    else
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "audio_mpp_stop()");

    if (result &= (audio_mpp_deinit() == LOCALSDK_OK))
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "audio_mpp_deinit()");
    else
        LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "audio_mpp_deinit()");

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}
