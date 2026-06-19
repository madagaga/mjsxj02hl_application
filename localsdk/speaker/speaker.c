#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "./speaker.h"
#include "./../localsdk.h"
#include "./../audio/audio.h"
#include "./../../logger/logger.h"
#include "./../../configs/configs.h"

#include "mpi_ao.h"
#include "mpi_adec.h"
#include "mpi_sys.h"

/* ── Globals ─────────────────────────────────────────────────────────────── */

static ADEC_CHN g_adecChn = 0;

int playback_status = SPEAKER_MEDIA_STOPPED;

/* ── Internal MPP helpers ────────────────────────────────────────────────── */

static int speaker_mpp_init(void)
{
    static ADEC_ATTR_G711_S stAdecG711;
    ADEC_CHN_ATTR_S stAdecAttr;
    HI_S32 result;

    stAdecAttr.enType    = PT_G711A;
    stAdecAttr.u32BufSize = 20;
    stAdecAttr.enMode    = ADEC_MODE_STREAM;
    stAdecAttr.pValue    = &stAdecG711;

    result = HI_MPI_ADEC_CreateChn(g_adecChn, &stAdecAttr);
    if (result != HI_SUCCESS && result != HI_ERR_ADEC_EXIST) {
        LOGGER(LOGGER_LEVEL_ERROR, "[speaker] ADEC_CreateChn failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }
    return LOCALSDK_OK;
}

static int speaker_mpp_start(void)
{
    MPP_CHN_S stSrcChn, stDestChn;
    HI_S32 result;
    int ao_dev = audio_get_ao_dev();
    int ao_chn = audio_get_ao_chn();

    stSrcChn.enModId  = HI_ID_ADEC;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = g_adecChn;

    stDestChn.enModId  = HI_ID_AO;
    stDestChn.s32DevId = ao_dev;
    stDestChn.s32ChnId = ao_chn;

    result = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (result != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_ERROR, "[speaker] SYS_Bind ADEC->AO failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    /* AO channel may already be enabled by audio_init(); treat EXIST as OK */
    result = HI_MPI_AO_EnableChn(ao_dev, ao_chn);
    if (result != HI_SUCCESS && result != (HI_S32)0xa0158009 /* HI_ERR_AO_EXIST */) {
        LOGGER(LOGGER_LEVEL_ERROR, "[speaker] AO_EnableChn failed: 0x%x", result);
        return LOCALSDK_ERROR;
    }

    return LOCALSDK_OK;
}

static int speaker_feed_pcm(void *data, int size)
{
    AUDIO_FRAME_S stFrame;
    HI_S32 result;

    if (!data || size <= 0) return LOCALSDK_ERROR;

    memset(&stFrame, 0, sizeof(stFrame));
    stFrame.enBitwidth     = AUDIO_BIT_WIDTH_16;
    stFrame.enSoundmode    = AUDIO_SOUND_MODE_MONO;
    stFrame.u32Len         = size;
    stFrame.u64VirAddr[0]  = (HI_U8 *)data;
    stFrame.u64PhyAddr[0]  = 0;

    result = HI_MPI_AO_SendFrame(audio_get_ao_dev(), audio_get_ao_chn(), &stFrame, 1000);
    return (result == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static int speaker_feed_g711(void *data, int size)
{
    AUDIO_STREAM_S stStream;
    HI_S32 result;

    if (!data || size <= 0) return LOCALSDK_ERROR;

    memset(&stStream, 0, sizeof(stStream));
    stStream.pStream        = (HI_U8 *)data;
    stStream.u32Len         = size;
    stStream.u64TimeStamp   = 0;

    result = HI_MPI_ADEC_SendStream(g_adecChn, &stStream, HI_TRUE);
    return (result == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static int speaker_clean_buf(void)
{
    HI_MPI_AO_ClearChnBuf(audio_get_ao_dev(), audio_get_ao_chn());
    return LOCALSDK_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool speaker_init(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (result &= (speaker_mpp_init() == LOCALSDK_OK)) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "speaker_mpp_init()");
        if (result &= (speaker_mpp_start() == LOCALSDK_OK)) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "speaker_mpp_start()");
        } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "speaker_mpp_start()");
    } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "speaker_mpp_init()");

    if (!result) {
        if (speaker_free()) LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "speaker_free()");
        else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "speaker_free()");
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

bool speaker_free(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", "true");
    return true;
}

bool speaker_play_media(char *filename, int type)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    LOGGER(LOGGER_LEVEL_INFO, "Filename: %s", filename);
    LOGGER(LOGGER_LEVEL_INFO, "Type: %d", type);

    if (speaker_status_media() != SPEAKER_MEDIA_STOPPED) {
        if (speaker_stop_media()) {
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "speaker_stop_media()");
            while (speaker_status_media() != SPEAKER_MEDIA_STOPPED) { usleep(100000); }
        }
    }
    playback_status = SPEAKER_MEDIA_PLAYING;

    FILE *media = fopen(filename, "rb");
    if (result &= (media != NULL)) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "fopen()");

        if (type != LOCALSDK_SPEAKER_G711_TYPE)
            fseek(media, 44, SEEK_SET); /* skip WAV header */

        if (result &= (speaker_clean_buf() == LOCALSDK_OK))
            LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "speaker_clean_buf()");
        else
            LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "speaker_clean_buf()");

        int error_counter = 0;
        int buffer_size = LOCALSDK_AUDIO_PCM_BUFFER_SIZE;
        if (type == LOCALSDK_SPEAKER_G711_TYPE)
            buffer_size = LOCALSDK_AUDIO_G711_BUFFER_SIZE;

        while (!feof(media)) {
            error_counter = 0;
            char *buffer = malloc(buffer_size);
            if (buffer != NULL) {
                size_t length = fread(buffer, 1, buffer_size, media);
                while (true) {
                    if (type == LOCALSDK_SPEAKER_G711_TYPE) {
                        if (speaker_feed_g711(buffer, length) == LOCALSDK_OK || error_counter >= 300) break;
                    } else {
                        if (speaker_feed_pcm(buffer, length) == LOCALSDK_OK || error_counter >= 300) break;
                    }
                    usleep(100000);
                    error_counter++;
                    if (speaker_status_media() == SPEAKER_MEDIA_STOPPED)  playback_status = SPEAKER_MEDIA_STOPPING;
                    if (speaker_status_media() == SPEAKER_MEDIA_STOPPING) break;
                }
                free(buffer);
            } else LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "malloc(buffer_size)");
            if (speaker_status_media() == SPEAKER_MEDIA_STOPPED)  playback_status = SPEAKER_MEDIA_STOPPING;
            if (speaker_status_media() == SPEAKER_MEDIA_STOPPING) break;
        }

        if (error_counter >= 300) {
            LOGGER(LOGGER_LEVEL_WARNING, "%s error!", "error_counter (>= 300)");
            result &= false;
        }

        /* no-op: finish_buf does nothing */
        fclose(media);
    } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "fopen()");

    playback_status = SPEAKER_MEDIA_STOPPED;
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

int speaker_status_media(void)
{
    return playback_status;
}

bool speaker_stop_media(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    if (result &= (speaker_status_media() != SPEAKER_MEDIA_STOPPED))
        playback_status = SPEAKER_MEDIA_STOPPING;

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

bool speaker_set_volume(int value)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    LOGGER(LOGGER_LEVEL_INFO, "Volume: %d", value);
    if (result &= (HI_MPI_AO_SetVolume(audio_get_ao_dev(), value) == HI_SUCCESS)) {
        LOGGER(LOGGER_LEVEL_DEBUG, "%s success.", "AO_SetVolume()");
        APP_CFG.speaker.volume = value;
    } else LOGGER(LOGGER_LEVEL_ERROR, "%s error!", "AO_SetVolume()");

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", (result ? "true" : "false"));
    return result;
}

int speaker_get_volume(void)
{
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    int value = APP_CFG.speaker.volume;
    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (volume = %d).", value);
    return value;
}
