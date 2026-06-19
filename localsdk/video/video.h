#ifndef _LOCALSDK_VIDEO_H_
#define _LOCALSDK_VIDEO_H_

#include <stdbool.h>
#include "hi_comm_vpss.h"
#include "../localsdk.h"  /* LOCALSDK_H26X_FRAME_INFO, LOCALSDK_VIDEO_* constants */

/* Lifecycle */
bool video_init(void);
bool video_free(void);
void video_deinit(void);

/* Runtime controls */
int  video_force_i_frame(int chn);
int  video_get_jpeg(int chn, char *file);
int  video_set_encode_callback(int chn, int (*cb)(LOCALSDK_H26X_FRAME_INFO *));
int  video_set_algo_register_callback(int (*cb)(void));
int  video_set_kbps(int chn, int kbps);

/* Exported for init.c / localsdk_destory() teardown */
VPSS_GRP video_get_vpss_grp(void);

#endif
