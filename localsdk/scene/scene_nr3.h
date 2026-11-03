#ifndef _SCENE_NR3_H_
#define _SCENE_NR3_H_

#include "hi_type.h"

/* Dynamic VPSS 3DNR (NRX V3) driven by the scene INI [static_3dnr] section,
 * as libsceneauto's HI_SCENE_SetDynamic3DNR / HI_SCENE_Set3DNR. */

#define NR3_MAX_ROWS   12    /* ISO steps kept per profile (INI has 11) */
#define NR3_RAW_VALUES 253   /* numbers in one 3DnrParam_N row */

typedef struct { HI_U16 IES0, IES1, IES2, IES3, IEDZ; } nr3_iey_t;

typedef struct {
    HI_U16 SPN6, SBN6, PBR6, JMODE, SFR6[4], SBR6[2];
    HI_U16 SFN0, SFN1, SFN2, STH1, STH2, STHd1, STHd2, SFR;
    HI_U16 SFS1, SFT1, SBR1, SFS2, SFT2, SBR2, SFS4, SFT4, SBR4;
    HI_U16 SRT0, SRT1, kMode, DeRate, DeIdx;
} nr3_sfy_t;

typedef struct {
    HI_U16 bRef, TED, STR0, STR1, SDZ0, SDZ1, TSS0, TSS1, TSI0, TSI1;
    HI_U16 TFS0, TFS1, TDZ0, TDZ1, TDX0, TDX1, TFR0[6], TFR1[6], TFRS;
} nr3_tfy_t;

typedef struct {
    HI_U16 MAI00, MAI01, MAI02, MAI10, MAI11, MAI12, MABR0, MABR1;
    HI_U16 AdvMATH, AdvTH, MATH0, MATH1, MATHd0, MATHd1, MATE0, MATE1, MABW0, MABW1, MATW;
} nr3_mdy_t;

typedef struct { HI_U16 SFC, TFC, TPC, TRC; } nr3_nrc_t;

typedef struct {
    nr3_iey_t IEy[5];
    nr3_sfy_t SFy[5];
    nr3_mdy_t MDy[2];
    nr3_tfy_t TFy[3];
    nr3_nrc_t NRc;
} nr3_row_t;

typedef struct {
    HI_U32    count;                   /* 3DNRCount */
    HI_U32    iso[NR3_MAX_ROWS];       /* IsoThresh */
    nr3_row_t row[NR3_MAX_ROWS];       /* 3DnrParam_N, valid when row_ok[N] */
    HI_U8     row_ok[NR3_MAX_ROWS];
} scene_nr3_t;

/* INI parsing: call nr3_parse_begin() before each ini_parse(), feed every
 * [static_3dnr] key through nr3_parse_key() (value may be modified), and call
 * nr3_parse_end() after the parse to build the rows. */
void nr3_parse_begin(void);
void nr3_parse_key(scene_nr3_t *n, const char *name, char *value);
void nr3_parse_end(scene_nr3_t *n);

/* Program the VPSS group NR parameters for the given ISO. Returns 0 on success. */
int nr3_apply(const scene_nr3_t *n, HI_U32 iso, int debug);

#endif /* _SCENE_NR3_H_ */
