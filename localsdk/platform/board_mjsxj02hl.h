#ifndef _BOARD_MJSXJ02HL_H_
#define _BOARD_MJSXJ02HL_H_

/* Xiaomi MJSXJ02HL board — compile-time sizing constants for localsdk.
 * These freeze the board+sensor couple: the BOARD_* values that depend on the
 * sensor are relayed from its #defines (no re-literal), the rest are board
 * tuning choices. localsdk reads BOARD_* directly (compile-time, single board:
 * no dynamic firmware). */

#include "../sensor/jxf/sensor_jxf22_defs.h"  /* JXF22_* */
#include "hi_type.h"
#include "hi_comm_vi.h"                        /* VI_ONLINE_VPSS_ONLINE */

/* --- relayed from the fitted sensor (JXF22) --- */
#define BOARD_WIDTH        JXF22_WIDTH       /* main channel / sensor active */
#define BOARD_HEIGHT       JXF22_HEIGHT
#define BOARD_FPS_NATIVE   JXF22_FPS_NATIVE  /* sensor input rate to VIPROC */
#define BOARD_FULL_LINES   JXF22_FULL_LINES  /* sensor VMAX @native (wrap calc) */

/* --- board tuning choices --- */
#define BOARD_TARGET_FPS   20                /* VMAX=1701; max IntTime ~50ms */
#define BOARD_WRAP_LINES   416               /* VPSS chn0 ring buffer lines */
#define BOARD_VIVPSS_MODE  VI_ONLINE_VPSS_ONLINE
#define BOARD_SUB_WIDTH    640               /* secondary channel */
#define BOARD_SUB_HEIGHT   360

#endif /* _BOARD_MJSXJ02HL_H_ */
