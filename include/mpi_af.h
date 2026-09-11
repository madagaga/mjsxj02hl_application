/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Description: Function of mpi_af.h
 * Author: ISP SW
 * Create: 2012/12/19
 */
#ifndef __MPI_AF_H__
#define __MPI_AF_H__

#include "hi_comm_isp.h"
#include "hi_comm_3a.h"
#include "hi_af_comm.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* End of #ifdef __cplusplus */

/* The interface of af lib register to isp. */
HI_S32 HI_MPI_AF_Register(ISP_DEV IspDev, ALG_LIB_S *pstAfLib);
HI_S32 HI_MPI_AF_UnRegister(ISP_DEV IspDev, ALG_LIB_S *pstAfLib);

#if 0
/* The callback function of sensor register to af lib. */
HI_S32 hi_af_sensor_register_cb(ALG_LIB_S *pstAfLib, SENSOR_ID SensorId,
    CMOS_ISP_AF_DEFAULT_S *pstSnsDft, SENSOR_AF_EXP_FUNC_S *pstSnsExp);
#endif

/* The new awb lib is compatible with the old mpi interface. */
HI_S32 HI_MPI_ISP_SetFocusType(ISP_DEV IspDev, ISP_OP_TYPE_E enFocusType);    //not support yet
HI_S32 HI_MPI_ISP_GetFocusType(ISP_DEV IspDev, ISP_OP_TYPE_E *penFocusType);  //not support yet

HI_S32 HI_MPI_ISP_SetAFAttr(ISP_DEV IspDev, const ISP_AF_ATTR_S *pstAFAttr);    //not support yet
HI_S32 HI_MPI_ISP_GetAFAttr(ISP_DEV IspDev, ISP_AF_ATTR_S *pstAFAttr);          //not support yet

HI_S32 HI_MPI_ISP_SetMFAttr(ISP_DEV IspDev, const ISP_MF_ATTR_S *pstMFAttr);    //not support yet
HI_S32 HI_MPI_ISP_GetMFAttr(ISP_DEV IspDev, ISP_MF_ATTR_S *pstMFAttr);         //not support yet

HI_S32 HI_MPI_ISP_ManualFocusMove(ISP_DEV IspDev, HI_S32 s32MoveSteps);    //not support yet


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif

