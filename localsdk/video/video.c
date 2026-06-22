/**
 * @file video.c
 * @brief Video subsystem — VPSS + VENC pipeline (Phase 1 refactoring).
 *
 * Owns all video state and MPP calls.  The public API (video_init, video_free,
 * video_deinit, video_force_i_frame, video_get_jpeg) is declared in video.h.
 *
 * Initialization order (enforced by video_init):
 *   VB → SYS → SetVIVPSSMode → board.pfnBringupSensor → board.pfnInit
 *   → CreateGrp → SetChnAttr+WrapAttr → EnableChn → StartGrp → VI→VPSS bind
 *   → (per-channel) CreateVencChn → StartRecvFrame → VPSS→VENC bind
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

/* Hisilicon MPP */
#include "hi_comm_sys.h"
#include "hi_comm_video.h"
#include "hi_comm_venc.h"
#include "hi_comm_vpss.h"
#include "hi_comm_vi.h"
#include "mpi_sys.h"
#include "mpi_venc.h"
#include "mpi_vpss.h"
#include "mpi_vi.h"
#include "mpi_vb.h"
#include "hi_buffer.h"

/* Internal */
#include "video.h"                 /* owns all LOCALSDK_VIDEO_* types and constants */
#include "../platform/platform.h"  /* board_cfg_t, BOARD_* constants */
#include "../osd/osd.h"
#include "../../logger/logger.h"
#include "../../configs/configs.h"
#include "../../rtsp/rtsp.h"

/* ============================================================================
   INTERNAL LOGGING
   ============================================================================ */

static inline void sdk_log(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    int len = (int)strlen(buf);
    if (len > 0 && buf[len - 1] == '\n') buf[len - 1] = '\0';
    LOGGER(LOGGER_LEVEL_DEBUG, "%s", buf);
}

/* ============================================================================
   GLOBAL STATE
   ============================================================================ */

static const board_cfg_t *g_board_cfg = NULL;

/* VPSS / VENC handles */
static VPSS_GRP g_vpssGrp       = 0;
static VPSS_CHN g_vpssChn[4]    = {0, 1, 2, 3};
static VENC_CHN g_vencChn[2]    = {0, 1};
static int32_t  g_videoStarted[2] = {0, 0};

/* Wrap buffer parameters for main channel (chn0), computed before VB init. */
static HI_U32   g_mainWrapBufLine = 0;
static HI_U32   g_mainWrapBufSize = 0;

/* Callbacks */
static int32_t (*g_encCb[2])(LOCALSDK_H26X_FRAME_INFO *frameInfo) = {NULL, NULL};
static int32_t (*g_yuvCb[2])(LOCALSDK_H26X_FRAME_INFO *frameInfo) = {NULL, NULL};
static int (*g_algoRegisterCb)(void)   = NULL;
static int (*g_algoUnregisterCb)(void) = NULL;

/* Run threads */
static pthread_t       g_videoRunThread[2]  = {0, 0};
static volatile int    g_videoRunning[2]    = {0, 0};

/* Brightness lock (used if brightness control is re-enabled) */
static pthread_mutex_t g_videoBrightLock;

/* Video parameter storage (LOCALSDK_VIDEO_OPTIONS per channel, 32 words each) */
static uint32_t g_videoParams[256];

/* ============================================================================
   CONSTANTS
   ============================================================================ */

#define SDK_SYS_ALIGN_WIDTH   64
#define SDK_INVALID_CHANNEL   0xffffffff

/* ============================================================================
   INTERNAL HELPERS
   ============================================================================ */

static int sdk_video_any_started(void) {
    return g_videoStarted[0] || g_videoStarted[1];
}

static LOCALSDK_VIDEO_OPTIONS *sdk_video_get_options(int chn) {
    if (chn < 0 || chn > 1) return NULL;
    return (LOCALSDK_VIDEO_OPTIONS *)&g_videoParams[chn * 32];
}

static uint32_t sdk_align_up(uint32_t value, uint32_t align) {
    return (value + (align - 1)) & ~(align - 1);
}

static uint32_t sdk_calc_yuv420_blk_size(uint32_t width, uint32_t height) {
    uint32_t w = sdk_align_up(width,  SDK_SYS_ALIGN_WIDTH);
    uint32_t h = sdk_align_up(height, SDK_SYS_ALIGN_WIDTH);
    return (w * h * 3) / 2;
}

static int sdk_video_resolution_to_size(int resolution, SIZE_S *size) {
    if (!size) return LOCALSDK_ERROR;
    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            size->u32Width = 1920; size->u32Height = 1080; return LOCALSDK_OK;
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
            size->u32Width = 640;  size->u32Height = 360;  return LOCALSDK_OK;
        default: return LOCALSDK_ERROR;
    }
}

static PAYLOAD_TYPE_E sdk_video_payload_type(uint32_t payload, int jpeg) {
    if (jpeg) return PT_JPEG;
    switch (payload) {
        case LOCALSDK_VIDEO_PAYLOAD_H265: return PT_H265;
        default:                          return PT_H264;
    }
}

/* Resolution utility functions (static: only used inside this module) */
static int video_get_pic_size(int resolution, LOCALSDK_PICTURE_SIZE *size) {
    if (!size) return LOCALSDK_ERROR;
    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
            size->width = 640; size->height = 360; return LOCALSDK_OK;
        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            size->width = 1920; size->height = 1080; return LOCALSDK_OK;
        default: return LOCALSDK_ERROR;
    }
}

static int video_validate_resolution(int resolution, int *out) {
    if (!out) return LOCALSDK_ERROR;
    switch (resolution) {
        case LOCALSDK_VIDEO_RESOLUTION_640x360:
        case LOCALSDK_VIDEO_RESOLUTION_1920x1080:
            *out = resolution; return LOCALSDK_OK;
        default: return LOCALSDK_ERROR;
    }
}

/* ============================================================================
   VPSS / VENC BIND HELPERS
   ============================================================================ */

static int sdk_video_bind_vpss_venc(int chn, VENC_CHN vencChn) {
    MPP_CHN_S src, dst;
    memset(&src, 0, sizeof(src)); memset(&dst, 0, sizeof(dst));
    src.enModId = HI_ID_VPSS; src.s32DevId = g_vpssGrp; src.s32ChnId = g_vpssChn[chn];
    dst.enModId = HI_ID_VENC; dst.s32DevId = 0;          dst.s32ChnId = vencChn;
    return (HI_MPI_SYS_Bind(&src, &dst) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

static int sdk_video_unbind_vpss_venc(int chn, VENC_CHN vencChn) {
    MPP_CHN_S src, dst;
    memset(&src, 0, sizeof(src)); memset(&dst, 0, sizeof(dst));
    src.enModId = HI_ID_VPSS; src.s32DevId = g_vpssGrp; src.s32ChnId = g_vpssChn[chn];
    dst.enModId = HI_ID_VENC; dst.s32DevId = 0;          dst.s32ChnId = vencChn;
    return (HI_MPI_SYS_UnBind(&src, &dst) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

int sdk_video_bind_vi_vpss(void) {
    MPP_CHN_S src, dst;
    VI_DEV viDev = 0; VI_PIPE viPipe = 0; VI_CHN viChn = 0;
    memset(&src, 0, sizeof(src)); memset(&dst, 0, sizeof(dst));
    if (g_board_cfg->pfnGetVi) g_board_cfg->pfnGetVi(&viDev, &viPipe, &viChn);
    src.enModId = HI_ID_VI;   src.s32DevId = viPipe;    src.s32ChnId = viChn;
    dst.enModId = HI_ID_VPSS; dst.s32DevId = g_vpssGrp; dst.s32ChnId = 0;
    return (HI_MPI_SYS_Bind(&src, &dst) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

int sdk_video_unbind_vi_vpss(void) {
    MPP_CHN_S src, dst;
    VI_DEV viDev = 0; VI_PIPE viPipe = 0; VI_CHN viChn = 0;
    memset(&src, 0, sizeof(src)); memset(&dst, 0, sizeof(dst));
    if (g_board_cfg && g_board_cfg->pfnGetVi)
        g_board_cfg->pfnGetVi(&viDev, &viPipe, &viChn);
    src.enModId = HI_ID_VI;   src.s32DevId = viPipe;    src.s32ChnId = viChn;
    dst.enModId = HI_ID_VPSS; dst.s32DevId = g_vpssGrp; dst.s32ChnId = 0;
    return (HI_MPI_SYS_UnBind(&src, &dst) == HI_SUCCESS) ? LOCALSDK_OK : LOCALSDK_ERROR;
}

/* ============================================================================
   CHANNEL LIFECYCLE
   ============================================================================ */

static void sdk_video_shutdown_channel(int chn) {
    if (chn < 0 || chn > 1) return;
    sdk_video_unbind_vpss_venc(chn, g_vencChn[chn]);
    HI_MPI_VENC_StopRecvFrame(g_vencChn[chn]);
    HI_MPI_VENC_DestroyChn(g_vencChn[chn]);
    g_videoStarted[chn] = 0;
}

static int sdk_video_create_venc_channel(VENC_CHN vencChn,
                                          LOCALSDK_VIDEO_OPTIONS *options, int jpeg) {
    VENC_CHN_ATTR_S attr;
    SIZE_S stSize;
    HI_S32 result;

    if (!options || sdk_video_resolution_to_size(options->resolution, &stSize) != LOCALSDK_OK)
        return LOCALSDK_ERROR;

    memset(&attr, 0, sizeof(attr));
    PAYLOAD_TYPE_E enType = sdk_video_payload_type(options->payload, jpeg);

    attr.stVencAttr.enType         = enType;
    attr.stVencAttr.u32MaxPicWidth  = stSize.u32Width;
    attr.stVencAttr.u32MaxPicHeight = stSize.u32Height;
    attr.stVencAttr.u32PicWidth     = stSize.u32Width;
    attr.stVencAttr.u32PicHeight    = stSize.u32Height;
    attr.stVencAttr.u32Profile      = 0;
    attr.stVencAttr.bByFrame        = HI_TRUE;
    attr.stGopAttr.enGopMode              = VENC_GOPMODE_NORMALP;
    attr.stGopAttr.stNormalP.s32IPQpDelta = 2;

    if (enType == PT_H264) {
        /* Quarter-frame stream buffer + shared recon: matches original firmware
           MMZ footprint (~328 KB) and frees ~5 MB for IVP buffers. */
        attr.stVencAttr.u32BufSize                   = stSize.u32Width * stSize.u32Height / 4;
        attr.stVencAttr.stAttrH264e.bRcnRefShareBuf  = HI_TRUE;

        if (options->rcmode == LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE) {
            attr.stRcAttr.enRcMode                       = VENC_RC_MODE_H264VBR;
            attr.stRcAttr.stH264Vbr.u32Gop              = options->gop ? options->gop : options->fps;
            attr.stRcAttr.stH264Vbr.u32StatTime         = 1;
            attr.stRcAttr.stH264Vbr.u32SrcFrameRate     = options->fps;
            attr.stRcAttr.stH264Vbr.fr32DstFrameRate    = options->fps;
            attr.stRcAttr.stH264Vbr.u32MaxBitRate       = options->bitrate;
        } else if (options->rcmode == LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY) {
            attr.stRcAttr.enRcMode                       = VENC_RC_MODE_H264FIXQP;
            attr.stRcAttr.stH264FixQp.u32Gop            = options->gop ? options->gop : options->fps;
            attr.stRcAttr.stH264FixQp.u32SrcFrameRate   = options->fps;
            attr.stRcAttr.stH264FixQp.fr32DstFrameRate  = options->fps;
            attr.stRcAttr.stH264FixQp.u32IQp = 20;
            attr.stRcAttr.stH264FixQp.u32PQp = 23;
        } else {
            attr.stRcAttr.enRcMode                       = VENC_RC_MODE_H264CBR;
            attr.stRcAttr.stH264Cbr.u32Gop              = options->gop ? options->gop : options->fps;
            attr.stRcAttr.stH264Cbr.u32StatTime         = 1;
            attr.stRcAttr.stH264Cbr.u32SrcFrameRate     = options->fps;
            attr.stRcAttr.stH264Cbr.fr32DstFrameRate    = options->fps;
            attr.stRcAttr.stH264Cbr.u32BitRate          = options->bitrate;
        }
    } else if (enType == PT_H265) {
        attr.stVencAttr.u32BufSize                   = stSize.u32Width * stSize.u32Height / 2;
        attr.stVencAttr.stAttrH265e.bRcnRefShareBuf  = HI_TRUE;

        if (options->rcmode == LOCALSDK_VIDEO_RCMODE_VARIABLE_BITRATE) {
            attr.stRcAttr.enRcMode                       = VENC_RC_MODE_H265VBR;
            attr.stRcAttr.stH265Vbr.u32Gop              = options->gop ? options->gop : options->fps;
            attr.stRcAttr.stH265Vbr.u32StatTime         = 1;
            attr.stRcAttr.stH265Vbr.u32SrcFrameRate     = options->fps;
            attr.stRcAttr.stH265Vbr.fr32DstFrameRate    = options->fps;
            attr.stRcAttr.stH265Vbr.u32MaxBitRate       = options->bitrate;
        } else if (options->rcmode == LOCALSDK_VIDEO_RCMODE_CONSTANT_QUALITY) {
            attr.stRcAttr.enRcMode                       = VENC_RC_MODE_H265FIXQP;
            attr.stRcAttr.stH265FixQp.u32Gop            = options->gop ? options->gop : options->fps;
            attr.stRcAttr.stH265FixQp.u32SrcFrameRate   = options->fps;
            attr.stRcAttr.stH265FixQp.fr32DstFrameRate  = options->fps;
            attr.stRcAttr.stH265FixQp.u32IQp = 20;
            attr.stRcAttr.stH265FixQp.u32PQp = 23;
        } else {
            attr.stRcAttr.enRcMode                       = VENC_RC_MODE_H265CBR;
            attr.stRcAttr.stH265Cbr.u32Gop              = options->gop ? options->gop : options->fps;
            attr.stRcAttr.stH265Cbr.u32StatTime         = 1;
            attr.stRcAttr.stH265Cbr.u32SrcFrameRate     = options->fps;
            attr.stRcAttr.stH265Cbr.fr32DstFrameRate    = options->fps;
            attr.stRcAttr.stH265Cbr.u32BitRate          = options->bitrate;
        }
    } else {
        /* JPEG snapshot */
        attr.stVencAttr.u32BufSize =
            (((stSize.u32Width  + 15) >> 4) << 4) *
            (((stSize.u32Height + 15) >> 4) << 4);
        attr.stVencAttr.stAttrJpege.bSupportDCF   = HI_FALSE;
        attr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
    }

    result = HI_MPI_VENC_CreateChn(vencChn, &attr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] VENC_CreateChn(%d) failed: 0x%x\n", vencChn, result);
        return LOCALSDK_ERROR;
    }

    if (!jpeg) {
        VENC_RECV_PIC_PARAM_S recv = { .s32RecvPicNum = -1 };
        result = HI_MPI_VENC_StartRecvFrame(vencChn, &recv);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video] StartRecvFrame(%d) failed: 0x%x\n", vencChn, result);
            HI_MPI_VENC_DestroyChn(vencChn);
            return LOCALSDK_ERROR;
        }
    }
    return LOCALSDK_OK;
}

/* ============================================================================
   VIDEO INIT / CREATE / START / STOP
   ============================================================================ */

static int video_sys_init(void) {
    VB_CONFIG_S stVbConf;
    VI_VPSS_MODE_S stVIVPSSMode;
    HI_S32 result;

    /* Pre-compute VPSS wrap buffer size.  VPSS_GetWrapBufferSize is a pure
       calculation, safe before SYS init. */
    if (g_board_cfg->vb_main_wrap_lines > 0) {
        g_mainWrapBufLine = g_board_cfg->vb_main_wrap_lines;
        g_mainWrapBufSize = VPSS_GetWrapBufferSize(
            BOARD_WIDTH, BOARD_HEIGHT, g_mainWrapBufLine,
            PIXEL_FORMAT_YVU_SEMIPLANAR_420,
            DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
        sdk_log("[sdk][video] wrap: line=%u size=%u\n",
                g_mainWrapBufLine, g_mainWrapBufSize);
    }

    /* VB pools.
       Pool 0: main YUV420 1080p — wrap buffer (~1.2 MB) or full frame (~3.1 MB).
       Pool 1: sub  YUV420 640x360. */
    memset(&stVbConf, 0, sizeof(stVbConf));
    stVbConf.u32MaxPoolCnt = 2;
    stVbConf.astCommPool[0].u64BlkSize = (g_mainWrapBufSize > 0)
        ? g_mainWrapBufSize
        : sdk_calc_yuv420_blk_size(BOARD_WIDTH, BOARD_HEIGHT);
    stVbConf.astCommPool[0].u32BlkCnt  = g_board_cfg->vb_main_blk_cnt;
    stVbConf.astCommPool[1].u64BlkSize = sdk_calc_yuv420_blk_size(BOARD_SUB_WIDTH, BOARD_SUB_HEIGHT);
    stVbConf.astCommPool[1].u32BlkCnt  = g_board_cfg->vb_sub_blk_cnt;

    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    result = HI_MPI_VB_SetConfig(&stVbConf);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] VB_SetConfig failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    result = HI_MPI_VB_Init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] VB_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    result = HI_MPI_SYS_Init();
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] SYS_Init failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* VI_ONLINE_VPSS_ONLINE: direct hardware pipeline, required for wrap + 3DNR. */
    memset(&stVIVPSSMode, 0, sizeof(stVIVPSSMode));
    HI_MPI_SYS_GetVIVPSSMode(&stVIVPSSMode);
    stVIVPSSMode.aenMode[0] = BOARD_VIVPSS_MODE;
    result = HI_MPI_SYS_SetVIVPSSMode(&stVIVPSSMode);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] SetVIVPSSMode failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    return LOCALSDK_OK;
}

static int video_vpss_init(void) {
    VPSS_GRP_ATTR_S grpAttr;
    HI_S32 result;

    memset(&grpAttr, 0, sizeof(grpAttr));
    grpAttr.u32MaxW                     = BOARD_WIDTH;
    grpAttr.u32MaxH                     = BOARD_HEIGHT;
    grpAttr.enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    grpAttr.enDynamicRange              = DYNAMIC_RANGE_SDR8;
    grpAttr.stFrameRate.s32SrcFrameRate = -1;
    grpAttr.stFrameRate.s32DstFrameRate = -1;
    /* 3DNR — requires wrap (vb_main_wrap_lines=416) for the NR reference frame.
       COMPRESS_MODE_FRAME: reference frame compressed, required by hi3516ev200. */
    grpAttr.bNrEn                        = HI_TRUE;
    grpAttr.stNrAttr.enNrType            = VPSS_NR_TYPE_VIDEO;
    grpAttr.stNrAttr.enCompressMode      = COMPRESS_MODE_FRAME;
    grpAttr.stNrAttr.enNrMotionMode      = NR_MOTION_MODE_NORMAL;

    result = HI_MPI_VPSS_CreateGrp(g_vpssGrp, &grpAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] VPSS_CreateGrp failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }
    /* StartGrp and VI→VPSS bind deferred to video_create_channel() after
       SetChnBufWrapAttr, as required by the SDK. */
    return LOCALSDK_OK;
}

static int video_create_channel(int chn, LOCALSDK_VIDEO_OPTIONS *options) {
    VPSS_CHN_ATTR_S chnAttr;
    SIZE_S stSize;
    HI_S32 result;

    if (chn < 0 || chn > 3 || !options) return LOCALSDK_ERROR;
    if (sdk_video_resolution_to_size(options->resolution, &stSize) != LOCALSDK_OK)
        return LOCALSDK_ERROR;

    sdk_log("[sdk][video] Creating VPSS chn %d (%ux%u)\n",
            chn, stSize.u32Width, stSize.u32Height);

    memset(&chnAttr, 0, sizeof(chnAttr));
    chnAttr.enChnMode               = VPSS_CHN_MODE_USER;
    chnAttr.u32Width                = stSize.u32Width;
    chnAttr.u32Height               = stSize.u32Height;
    chnAttr.enVideoFormat           = VIDEO_FORMAT_LINEAR;
    chnAttr.enPixelFormat           = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    chnAttr.enDynamicRange          = DYNAMIC_RANGE_SDR8;
    /* COMPRESS_MODE_SEG required for chn0 wrap; others use NONE. */
    chnAttr.enCompressMode = (g_vpssChn[chn] == 0 && g_mainWrapBufLine > 0)
        ? COMPRESS_MODE_SEG : COMPRESS_MODE_NONE;
    chnAttr.stFrameRate.s32SrcFrameRate = -1;
    chnAttr.stFrameRate.s32DstFrameRate = -1;
    /* Secondary channel: depth=2 so IVP can GetChnFrame alongside the VENC bind. */
    chnAttr.u32Depth = (g_vpssChn[chn] == 1) ? 2 : 0;
    /* Orientation corrected at sensor level; VPSS always FALSE (chn0+wrap requires it). */
    chnAttr.bMirror = HI_FALSE;
    chnAttr.bFlip   = HI_FALSE;

    result = HI_MPI_VPSS_SetChnAttr(g_vpssGrp, g_vpssChn[chn], &chnAttr);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] SetChnAttr failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    /* Wrap mode on chn0 only.  Must be called after SetChnAttr, before EnableChn. */
    if (g_vpssChn[chn] == 0 && g_mainWrapBufLine > 0) {
        VPSS_CHN_BUF_WRAP_S wrapAttr = {
            .bEnable           = HI_TRUE,
            .u32BufLine        = g_mainWrapBufLine,
            .u32WrapBufferSize = g_mainWrapBufSize,
        };
        result = HI_MPI_VPSS_SetChnBufWrapAttr(g_vpssGrp, g_vpssChn[chn], &wrapAttr);
        if (result != HI_SUCCESS)
            sdk_log("[sdk][video] SetChnBufWrapAttr failed: 0x%x\n", result);
        else
            sdk_log("[sdk][video] wrap chn0: line=%u size=%u\n",
                    g_mainWrapBufLine, g_mainWrapBufSize);
    }

    result = HI_MPI_VPSS_EnableChn(g_vpssGrp, g_vpssChn[chn]);
    if (result != HI_SUCCESS) {
        sdk_log("[sdk][video] EnableChn failed: 0x%x\n", result);
        return LOCALSDK_ERROR;
    }

    memcpy(&g_videoParams[chn * 32], options, sizeof(LOCALSDK_VIDEO_OPTIONS));

    /* StartGrp + VI→VPSS bind after all channels configured (chn1 is last). */
    if (chn == 1) {
        result = HI_MPI_VPSS_StartGrp(g_vpssGrp);
        if (result != HI_SUCCESS) {
            sdk_log("[sdk][video] StartGrp failed: 0x%x\n", result);
            return LOCALSDK_ERROR;
        }
        result = sdk_video_bind_vi_vpss();
        if (result != LOCALSDK_OK) {
            sdk_log("[sdk][video] VI→VPSS bind failed\n");
            HI_MPI_VPSS_StopGrp(g_vpssGrp);
            HI_MPI_VPSS_DestroyGrp(g_vpssGrp);
            return LOCALSDK_ERROR;
        }
        sdk_log("[sdk][video] VPSS started, VI→VPSS bound\n");
    }

    sdk_log("[sdk][video] Channel %d created\n", chn);
    return LOCALSDK_OK;
}

static int video_start_channel(int chn) {
    LOCALSDK_VIDEO_OPTIONS *options = sdk_video_get_options(chn);
    HI_S32 result;

    if (chn < 0 || chn > 3 || !options) return LOCALSDK_ERROR;

    result = sdk_video_create_venc_channel(g_vencChn[chn], options, 0);
    if (result != LOCALSDK_OK) return LOCALSDK_ERROR;

    result = sdk_video_bind_vpss_venc(chn, g_vencChn[chn]);
    if (result != LOCALSDK_OK) {
        HI_MPI_VENC_StopRecvFrame(g_vencChn[chn]);
        HI_MPI_VENC_DestroyChn(g_vencChn[chn]);
        sdk_log("[sdk][video] VPSS→VENC bind failed for chn %d\n", chn);
        return LOCALSDK_ERROR;
    }

    if (g_algoRegisterCb && chn == LOCALSDK_VIDEO_SECONDARY_CHANNEL)
        g_algoRegisterCb();

    HI_MPI_VENC_RequestIDR(g_vencChn[chn], HI_TRUE);
    g_videoStarted[chn] = 1;
    return LOCALSDK_OK;
}

/* ============================================================================
   VENC CAPTURE THREAD
   ============================================================================ */

typedef struct { int chn; } VideoRunArg;
static VideoRunArg g_videoRunArgs[2] = {{0}, {1}};

static void *sdk_video_run_thread(void *arg) {
    VideoRunArg *a = (VideoRunArg *)arg;
    int chn = a->chn;
    VENC_CHN vencChn = g_vencChn[chn];
    int vencFd;
    fd_set read_fds;
    struct timeval tv;

    vencFd = HI_MPI_VENC_GetFd(vencChn);
    if (vencFd < 0) { g_videoRunning[chn] = 0; return NULL; }

    while (g_videoRunning[chn]) {
        FD_ZERO(&read_fds);
        FD_SET(vencFd, &read_fds);
        tv.tv_sec = 1; tv.tv_usec = 0;

        int sel = select(vencFd + 1, &read_fds, NULL, NULL, &tv);
        if (sel < 0) break;
        if (sel == 0 || !FD_ISSET(vencFd, &read_fds)) continue;

        VENC_CHN_STATUS_S stStat;
        if (HI_MPI_VENC_QueryStatus(vencChn, &stStat) != HI_SUCCESS) continue;
        if (stStat.u32CurPacks == 0) continue;

        VENC_STREAM_S stStream;
        memset(&stStream, 0, sizeof(stStream));
        stStream.pstPack = malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
        if (!stStream.pstPack) continue;
        stStream.u32PackCount = stStat.u32CurPacks;

        if (HI_MPI_VENC_GetStream(vencChn, &stStream, HI_FALSE) != HI_SUCCESS) {
            free(stStream.pstPack); continue;
        }

        if (g_encCb[chn]) {
            for (uint32_t i = 0; i < stStream.u32PackCount; i++) {
                LOCALSDK_H26X_FRAME_INFO fi = {
                    .data      = (signed char *)stStream.pstPack[i].pu8Addr,
                    .size      = stStream.pstPack[i].u32Len,
                    .timestamp = (uint32_t)stStream.pstPack[i].u64PTS,
                    .offset    = (uint16_t)stStream.pstPack[i].u32Offset,
                    .frame_end = (uint16_t)stStream.pstPack[i].bFrameEnd,
                    .type      = stStream.pstPack[i].DataType.enH264EType,
                };
                g_encCb[chn](&fi);
            }
        }

        HI_MPI_VENC_ReleaseStream(vencChn, &stStream);
        free(stStream.pstPack);
    }

    g_videoRunning[chn] = 0;
    return NULL;
}

/* ============================================================================
   RTSP CALLBACKS (was in old video.c wrapper)
   ============================================================================ */

static int h26x_capture_callback(int chn, LOCALSDK_H26X_FRAME_INFO *frameInfo) {
    if (frameInfo && frameInfo->size && rtsp_is_enabled(chn))
        rtsp_media_frame(chn, frameInfo->data, frameInfo->size,
                         frameInfo->timestamp, frameInfo->type);
    return LOCALSDK_OK;
}

static int h26x_capture_primary_channel(LOCALSDK_H26X_FRAME_INFO *frameInfo) {
    return h26x_capture_callback(LOCALSDK_VIDEO_PRIMARY_CHANNEL, frameInfo);
}

static int h26x_capture_secondary_channel(LOCALSDK_H26X_FRAME_INFO *frameInfo) {
    return h26x_capture_callback(LOCALSDK_VIDEO_SECONDARY_CHANNEL, frameInfo);
}

/* ============================================================================
   PUBLIC API
   ============================================================================ */

bool video_init(void) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    g_board_cfg = platform_get_board_cfg();
    pthread_mutex_init(&g_videoBrightLock, NULL);

    /* SYS + VB init */
    if (!(result = (video_sys_init() == LOCALSDK_OK))) {
        LOGGER(LOGGER_LEVEL_ERROR, "video_sys_init() error!");
        return false;
    }

    /* Sensor bring-up (MIPI + VI + ISP + fps + orientation) */
    if (!g_board_cfg->pfnBringupSensor) {
        LOGGER(LOGGER_LEVEL_ERROR, "board has no pfnBringupSensor");
        return false;
    }
    if (!(result = (g_board_cfg->pfnBringupSensor(
                (HI_BOOL)APP_CFG.video.mirror,
                (HI_BOOL)APP_CFG.video.flip) == HI_SUCCESS))) {
        LOGGER(LOGGER_LEVEL_ERROR, "pfnBringupSensor() error!");
        return false;
    }

    /* Board init: scene ISP params + GPIO initial state */
    if (g_board_cfg->pfnInit && g_board_cfg->pfnInit() != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "pfnInit() returned error (non-fatal)");
    }

    /* VPSS group */
    if (!(result = (video_vpss_init() == LOCALSDK_OK))) {
        LOGGER(LOGGER_LEVEL_ERROR, "video_vpss_init() error!");
        return false;
    }

    /* Primary channel (1920x1080) */
    LOCALSDK_VIDEO_OPTIONS primary = {
        .bitrate      = APP_CFG.video.primary_bitrate,
        .fps          = LOCALSDK_VIDEO_FRAMERATE,
        .resolution   = LOCALSDK_VIDEO_RESOLUTION_1920x1080,
        .flip         = APP_CFG.video.flip,
        .mirror       = APP_CFG.video.mirror,
        .channel_type = 0,
        .video        = true,
        .osd          = osd_is_enabled(),
        .payload      = APP_CFG.video.primary_type,
        .rcmode       = APP_CFG.video.primary_rcmode,
        .gop          = APP_CFG.video.gop * LOCALSDK_VIDEO_FRAMERATE,
        .screen_size  = BOARD_WIDTH * BOARD_HEIGHT,
        .frame_size   = LOCALSDK_VIDEO_PRIMARY_FRAMESIZE,
        .jpeg         = false,
    };
    if (!(result = (video_create_channel(LOCALSDK_VIDEO_PRIMARY_CHANNEL, &primary) == LOCALSDK_OK))) {
        LOGGER(LOGGER_LEVEL_ERROR, "video_create_channel(primary) error!");
        goto fail;
    }
    g_encCb[LOCALSDK_VIDEO_PRIMARY_CHANNEL] = h26x_capture_primary_channel;
    if (!(result = (video_start_channel(LOCALSDK_VIDEO_PRIMARY_CHANNEL) == LOCALSDK_OK))) {
        LOGGER(LOGGER_LEVEL_ERROR, "video_start_channel(primary) error!");
        goto fail;
    }
    g_videoRunning[LOCALSDK_VIDEO_PRIMARY_CHANNEL] = 1;
    if (pthread_create(&g_videoRunThread[LOCALSDK_VIDEO_PRIMARY_CHANNEL], NULL,
                       sdk_video_run_thread,
                       &g_videoRunArgs[LOCALSDK_VIDEO_PRIMARY_CHANNEL]) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "pthread_create(primary) error!");
        result = false; goto fail;
    }
    LOGGER(LOGGER_LEVEL_DEBUG, "primary channel started.");

    if (!osd_postinit()) {
        LOGGER(LOGGER_LEVEL_WARNING, "osd_postinit() error!");
    }

    /* Secondary channel (640x360) */
    LOCALSDK_VIDEO_OPTIONS secondary = {
        .bitrate      = APP_CFG.video.secondary_bitrate,
        .fps          = LOCALSDK_VIDEO_FRAMERATE,
        .resolution   = LOCALSDK_VIDEO_RESOLUTION_640x360,
        .flip         = APP_CFG.video.flip,
        .mirror       = APP_CFG.video.mirror,
        .channel_type = 1,
        .video        = true,
        .osd          = false,
        .payload      = APP_CFG.video.secondary_type,
        .rcmode       = APP_CFG.video.secondary_rcmode,
        .gop          = APP_CFG.video.gop * LOCALSDK_VIDEO_FRAMERATE,
        .screen_size  = BOARD_SUB_WIDTH * BOARD_SUB_HEIGHT,
        .frame_size   = LOCALSDK_VIDEO_SECONDARY_FRAMESIZE,
        .jpeg         = true,
        .reserved     = 1,
    };
    if (!(result = (video_create_channel(LOCALSDK_VIDEO_SECONDARY_CHANNEL, &secondary) == LOCALSDK_OK))) {
        LOGGER(LOGGER_LEVEL_ERROR, "video_create_channel(secondary) error!");
        goto fail;
    }
    g_encCb[LOCALSDK_VIDEO_SECONDARY_CHANNEL] = h26x_capture_secondary_channel;
    if (!(result = (video_start_channel(LOCALSDK_VIDEO_SECONDARY_CHANNEL) == LOCALSDK_OK))) {
        LOGGER(LOGGER_LEVEL_ERROR, "video_start_channel(secondary) error!");
        goto fail;
    }
    g_videoRunning[LOCALSDK_VIDEO_SECONDARY_CHANNEL] = 1;
    if (pthread_create(&g_videoRunThread[LOCALSDK_VIDEO_SECONDARY_CHANNEL], NULL,
                       sdk_video_run_thread,
                       &g_videoRunArgs[LOCALSDK_VIDEO_SECONDARY_CHANNEL]) != 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "pthread_create(secondary) error!");
        result = false; goto fail;
    }
    LOGGER(LOGGER_LEVEL_DEBUG, "secondary channel started.");

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = true).");
    return true;

fail:
    video_free();
    return false;
}

bool video_free(void) {
    LOGGER(LOGGER_LEVEL_DEBUG, "Function is called...");
    bool result = true;

    for (int chn = 1; chn >= 0; chn--) {
        if (g_videoRunning[chn]) {
            g_videoRunning[chn] = 0;
            if (g_videoRunThread[chn]) {
                pthread_join(g_videoRunThread[chn], NULL);
                g_videoRunThread[chn] = 0;
            }
        }
        sdk_video_shutdown_channel(chn);
    }

    LOGGER(LOGGER_LEVEL_DEBUG, "Function completed (result = %s).", result ? "true" : "false");
    return result;
}

void video_deinit(void) {
    sdk_video_unbind_vi_vpss();
    HI_MPI_VPSS_StopGrp(g_vpssGrp);
    HI_MPI_VPSS_DestroyGrp(g_vpssGrp);
    pthread_mutex_destroy(&g_videoBrightLock);
    memset(g_videoParams, 0, sizeof(g_videoParams));
    g_mainWrapBufLine = 0;
    g_mainWrapBufSize = 0;
}

int video_force_i_frame(int chn) {
    if (chn < 0 || chn > 3) return LOCALSDK_ERROR;
    return (HI_MPI_VENC_RequestIDR(g_vencChn[chn], HI_TRUE) == HI_SUCCESS)
        ? LOCALSDK_OK : LOCALSDK_ERROR;
}

int video_get_jpeg(int chn, char *file) {
    FILE *fp;
    VENC_STREAM_S stStream;
    VENC_CHN_STATUS_S stStat;
    VENC_RECV_PIC_PARAM_S recv;
    HI_S32 result;
    int32_t vencFd;
    fd_set read_fds;
    struct timeval timeout;
    SIZE_S stSize;
    VENC_CHN snapChn = 3;

    if (chn < 0 || chn > 3 || !file) return LOCALSDK_ERROR;

    stSize.u32Width  = (chn == LOCALSDK_VIDEO_PRIMARY_CHANNEL) ? 1920 : 640;
    stSize.u32Height = (chn == LOCALSDK_VIDEO_PRIMARY_CHANNEL) ? 1080 : 360;

    fp = fopen(file, "wb");
    if (!fp) return LOCALSDK_ERROR;

    VENC_CHN_ATTR_S jpegAttr;
    memset(&jpegAttr, 0, sizeof(jpegAttr));
    jpegAttr.stVencAttr.enType          = PT_JPEG;
    jpegAttr.stVencAttr.u32MaxPicWidth  = stSize.u32Width;
    jpegAttr.stVencAttr.u32MaxPicHeight = stSize.u32Height;
    jpegAttr.stVencAttr.u32PicWidth     = stSize.u32Width;
    jpegAttr.stVencAttr.u32PicHeight    = stSize.u32Height;
    jpegAttr.stVencAttr.u32BufSize      =
        (((stSize.u32Width  + 15) >> 4) << 4) *
        (((stSize.u32Height + 15) >> 4) << 4);
    jpegAttr.stVencAttr.bByFrame                        = HI_TRUE;
    jpegAttr.stVencAttr.stAttrJpege.bSupportDCF         = HI_FALSE;
    jpegAttr.stVencAttr.stAttrJpege.enReceiveMode       = VENC_PIC_RECEIVE_SINGLE;
    jpegAttr.stGopAttr.enGopMode                        = VENC_GOPMODE_NORMALP;
    jpegAttr.stGopAttr.stNormalP.s32IPQpDelta           = 2;

    result = HI_MPI_VENC_CreateChn(snapChn, &jpegAttr);
    if (result != HI_SUCCESS) { fclose(fp); return LOCALSDK_ERROR; }

    if (sdk_video_bind_vpss_venc(chn, snapChn) != LOCALSDK_OK) {
        HI_MPI_VENC_DestroyChn(snapChn); fclose(fp); return LOCALSDK_ERROR;
    }

    memset(&recv, 0, sizeof(recv));
    recv.s32RecvPicNum = 1;
    result = HI_MPI_VENC_StartRecvFrame(snapChn, &recv);
    if (result != HI_SUCCESS) {
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn); fclose(fp); return LOCALSDK_ERROR;
    }

    vencFd = HI_MPI_VENC_GetFd(snapChn);
    if (vencFd < 0) {
        HI_MPI_VENC_StopRecvFrame(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn); fclose(fp); return LOCALSDK_ERROR;
    }

    FD_ZERO(&read_fds); FD_SET(vencFd, &read_fds);
    timeout.tv_sec = 2; timeout.tv_usec = 0;
    result = select(vencFd + 1, &read_fds, NULL, NULL, &timeout);
    if (result <= 0 || !FD_ISSET(vencFd, &read_fds)) {
        HI_MPI_VENC_StopRecvFrame(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn); fclose(fp); return LOCALSDK_ERROR;
    }

    memset(&stStat, 0, sizeof(stStat));
    if (HI_MPI_VENC_QueryStatus(snapChn, &stStat) != HI_SUCCESS || stStat.u32CurPacks == 0) {
        HI_MPI_VENC_StopRecvFrame(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn); fclose(fp); return LOCALSDK_ERROR;
    }

    memset(&stStream, 0, sizeof(stStream));
    stStream.pstPack = malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
    if (!stStream.pstPack) {
        HI_MPI_VENC_StopRecvFrame(snapChn);
        sdk_video_unbind_vpss_venc(chn, snapChn);
        HI_MPI_VENC_DestroyChn(snapChn); fclose(fp); return LOCALSDK_ERROR;
    }
    stStream.u32PackCount = stStat.u32CurPacks;

    if (HI_MPI_VENC_GetStream(snapChn, &stStream, -1) == HI_SUCCESS) {
        for (uint32_t i = 0; i < stStream.u32PackCount; i++)
            fwrite(stStream.pstPack[i].pu8Addr, 1, stStream.pstPack[i].u32Len, fp);
        HI_MPI_VENC_ReleaseStream(snapChn, &stStream);
    }

    free(stStream.pstPack);
    HI_MPI_VENC_StopRecvFrame(snapChn);
    sdk_video_unbind_vpss_venc(chn, snapChn);
    HI_MPI_VENC_DestroyChn(snapChn);
    fclose(fp);
    return LOCALSDK_OK;
}

int video_set_encode_callback(int chn, int (*cb)(LOCALSDK_H26X_FRAME_INFO *)) {
    if (chn < 0 || chn > 3) return LOCALSDK_ERROR;
    g_encCb[chn] = cb;
    return LOCALSDK_OK;
}

int video_set_algo_register_callback(int (*cb)(void)) {
    g_algoRegisterCb = cb;
    return LOCALSDK_OK;
}

int video_set_kbps(int chn, int kbps) {
    LOCALSDK_VIDEO_OPTIONS *opts = sdk_video_get_options(chn);
    if (!opts) return LOCALSDK_ERROR;
    opts->bitrate = (uint32_t)kbps;
    return LOCALSDK_OK;
}

/* Exported for init.c (SYS teardown needs these handles) */
VPSS_GRP video_get_vpss_grp(void) { return g_vpssGrp; }
