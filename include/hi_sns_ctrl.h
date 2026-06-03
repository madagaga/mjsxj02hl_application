/******************************************************************************
  Copyright (C), 2016, Hisilicon Tech. Co., Ltd.
  File Name     : hi_sns_ctrl.h
  Description   : ISP sensor object interface (3516e/3518ev300 compatible)
******************************************************************************/

#ifndef __HI_SNS_CTRL_H__
#define __HI_SNS_CTRL_H__

#include "hi_type.h"
#include "hi_comm_isp.h"
#include "hi_comm_3a.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

typedef struct hiISP_SNS_STATE_S {
    HI_BOOL     bInit;
    HI_BOOL     bSyncInit;
    HI_U8       u8ImgMode;
    HI_U8       u8Hdr;
    WDR_MODE_E  enWDRMode;
    ISP_SNS_REGS_INFO_S astRegsInfo[2];
    HI_U32      au32FL[2];
    HI_U32      u32FLStd;
    HI_U32      au32WDRIntTime[4];
} ISP_SNS_STATE_S;

typedef enum hiISP_SNS_MIRRORFLIP_TYPE_E {
    ISP_SNS_NORMAL      = 0,
    ISP_SNS_MIRROR      = 1,
    ISP_SNS_FLIP        = 2,
    ISP_SNS_MIRROR_FLIP = 3,
    ISP_SNS_BUTT
} ISP_SNS_MIRRORFLIP_TYPE_E;

typedef struct hiISP_SNS_OBJ_S {
    HI_S32  (*pfnRegisterCallback)(VI_PIPE ViPipe, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib);
    HI_S32  (*pfnUnRegisterCallback)(VI_PIPE ViPipe, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib);
    HI_S32  (*pfnSetBusInfo)(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo);
    HI_VOID (*pfnStandby)(VI_PIPE ViPipe);
    HI_VOID (*pfnRestart)(VI_PIPE ViPipe);
    HI_VOID (*pfnMirrorFlip)(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip);
    HI_S32  (*pfnWriteReg)(VI_PIPE ViPipe, HI_S32 s32Addr, HI_S32 s32Data);
    HI_S32  (*pfnReadReg)(VI_PIPE ViPipe, HI_S32 s32Addr);
    HI_S32  (*pfnSetInit)(VI_PIPE ViPipe, ISP_INIT_ATTR_S *pstInitAttr);
} ISP_SNS_OBJ_S;

/* SOI F22 sensor object — exported by libsns_f22.so */
extern ISP_SNS_OBJ_S stSnsSoiSensorObj;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __HI_SNS_CTRL_H__ */
