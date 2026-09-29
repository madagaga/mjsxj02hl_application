#ifndef _SENSOR_JXF22_DEFS_H_
#define _SENSOR_JXF22_DEFS_H_

/* SOI JXF22 intrinsic characteristics — the literals live here ONCE.
 * A board that fits this sensor relays these via its BOARD_* defines.
 * Leaf header (no includes) so it can be pulled by board headers without
 * dragging the MPP types into the localsdk include graph. */

#define JXF22_WIDTH        1920
#define JXF22_HEIGHT       1080
#define JXF22_FPS_NATIVE   30      /* sensor native frame rate */
#define JXF22_FULL_LINES   1134    /* VMAX @30fps (VMAX_1080P30_LINEAR) */

#endif /* _SENSOR_JXF22_DEFS_H_ */
