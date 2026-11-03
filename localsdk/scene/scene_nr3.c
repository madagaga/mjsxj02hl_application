/*
 * scene_nr3.c - dynamic VPSS 3DNR (NRX V3) for the scene engine.
 *
 * Behaviour reproduced from libsceneauto.so (RE, see re_notes/libsceneauto_spec.md):
 *  - [static_3dnr] holds 3DNRCount rows "3DnrParam_N", one per ISO threshold
 *    ("IsoThresh"), each a fixed-layout text table of 253 numbers;
 *  - HI_SCENE_SetDynamic3DNR picks the row by ISO (GetLevelLtoH), blends the
 *    neighbouring rows with SCENE_Interpulate on the MapISO() scale and writes
 *    the result through HI_MPI_VPSS_Get/SetGrpNRXParam (HI_SCENE_Set3DNR).
 *
 * The INI numbers are read in the order of the original's sscanf, ignoring the
 * "-label" words, the "(n)" notes and the '|' / ':' separators, so the table
 * can be laid out freely.
 */

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "hi_type.h"
#include "hi_comm_vpss.h"
#include "mpi_vpss.h"

#include "logger.h"
#include "scene_nr3.h"

/* Group handed to HI_MPI_VPSS_*GrpNRXParam: the single video group (video.c). */
#define NR3_VPSS_GRP 0

static HI_U16 s_raw[NR3_MAX_ROWS][NR3_RAW_VALUES + 8];
static int    s_fill[NR3_MAX_ROWS];
static int    s_warned;

/* Order of the numbers in a 3DnrParam_N row (= argument order of the original
   loader's sscanf). Returns the number of targets, NR3_RAW_VALUES expected. */
#define T(x) (*t++ = &(x))
static int nr3_targets(nr3_row_t *r, HI_U16 **t0)
{
    HI_U16 **t = t0;
    int i;

    for (i = 0; i < 4; i++) { T(r->SFy[i].SFS1); T(r->SFy[i].SFT1); T(r->SFy[i].SBR1); }
    for (i = 0; i < 4; i++) { T(r->SFy[i].SFS2); T(r->SFy[i].SFT2); T(r->SFy[i].SBR2); }
    for (i = 0; i < 4; i++) { T(r->SFy[i].SFS4); T(r->SFy[i].SFT4); T(r->SFy[i].SBR4); }
    T(r->SFy[0].SRT0); T(r->SFy[0].SRT1); T(r->SFy[2].kMode); T(r->SFy[3].kMode);
    T(r->SFy[0].DeRate); T(r->SFy[0].DeIdx);
    T(r->SFy[4].SFS1); T(r->SFy[4].SFS2); T(r->SFy[4].SFS4);
    for (i = 0; i < 5; i++) { T(r->IEy[i].IES0); T(r->IEy[i].IES1); T(r->IEy[i].IES2); T(r->IEy[i].IES3); }
    for (i = 0; i < 5; i++) T(r->IEy[i].IEDZ);
    for (i = 0; i < 5; i++) { T(r->SFy[i].SPN6); T(r->SFy[i].SBN6); T(r->SFy[i].PBR6); T(r->SFy[i].JMODE); }
    for (i = 0; i < 5; i++) { T(r->SFy[i].SFR6[0]); T(r->SFy[i].SFR6[1]); T(r->SFy[i].SFR6[2]); T(r->SFy[i].SFR6[3]); }
    for (i = 0; i < 5; i++) { T(r->SFy[i].SBR6[0]); T(r->SFy[i].SBR6[1]); }
    for (i = 0; i < 5; i++) { T(r->SFy[i].SFN0); T(r->SFy[i].SFN1); T(r->SFy[i].SFN2); }
    for (i = 0; i < 5; i++) { T(r->SFy[i].STH1); T(r->SFy[i].STH2); }
    for (i = 0; i < 5; i++) { T(r->SFy[i].STHd1); T(r->SFy[i].STHd2); }
    for (i = 0; i < 5; i++) T(r->SFy[i].SFR);

    T(r->TFy[0].bRef); T(r->TFy[1].bRef);
    T(r->TFy[1].TED); T(r->TFy[2].TED); T(r->MDy[1].MATH1);
    T(r->MDy[1].MATHd1);
    T(r->TFy[0].STR0); T(r->TFy[1].STR0); T(r->TFy[1].STR1); T(r->TFy[2].STR0); T(r->MDy[1].MATE1);
    T(r->TFy[0].SDZ0); T(r->TFy[1].SDZ0); T(r->TFy[1].SDZ1); T(r->TFy[2].SDZ0); T(r->MDy[1].MABW1);
    T(r->TFy[0].TSS0); T(r->TFy[1].TSS0); T(r->TFy[1].TSS1); T(r->TFy[2].TSS0);
    T(r->TFy[0].TSI0); T(r->TFy[1].TSI0); T(r->TFy[1].TSI1); T(r->TFy[2].TSI0);
    T(r->TFy[0].TFS0); T(r->TFy[1].TFS0); T(r->TFy[1].TFS1); T(r->TFy[2].TFS0);
    T(r->TFy[0].TDZ0); T(r->TFy[1].TDZ0); T(r->TFy[1].TDZ1); T(r->TFy[2].TDZ0);
    T(r->TFy[0].TDX0); T(r->TFy[1].TDX0); T(r->TFy[1].TDX1); T(r->TFy[2].TDX0);
    T(r->TFy[0].TFRS);
    for (i = 0; i < 3; i++) { T(r->TFy[i].TFR0[0]); T(r->TFy[i].TFR0[1]); T(r->TFy[i].TFR0[2]); }
    T(r->NRc.SFC);
    for (i = 0; i < 3; i++) { T(r->TFy[i].TFR0[3]); T(r->TFy[i].TFR0[4]); T(r->TFy[i].TFR0[5]); }
    T(r->NRc.TFC);
    T(r->TFy[1].TFR1[0]); T(r->TFy[1].TFR1[1]); T(r->TFy[1].TFR1[2]);
    T(r->NRc.TPC);
    T(r->TFy[1].TFR1[3]); T(r->TFy[1].TFR1[4]); T(r->TFy[1].TFR1[5]);
    T(r->NRc.TRC);

    for (i = 0; i < 2; i++) { T(r->MDy[i].MAI00); T(r->MDy[i].MAI01); T(r->MDy[i].MAI02); }
    T(r->MDy[0].MAI10); T(r->MDy[0].MAI11); T(r->MDy[0].MAI12);
    T(r->MDy[0].MABR0); T(r->MDy[0].MABR1); T(r->MDy[1].MABR0);
    T(r->MDy[0].AdvMATH);
    T(r->MDy[0].AdvTH);
    T(r->MDy[0].MATH0); T(r->MDy[0].MATH1); T(r->MDy[1].MATH0);
    T(r->MDy[0].MATHd0); T(r->MDy[0].MATHd1); T(r->MDy[1].MATHd0);
    T(r->MDy[0].MATE0); T(r->MDy[0].MATE1); T(r->MDy[1].MATE0);
    T(r->MDy[0].MABW0); T(r->MDy[0].MABW1); T(r->MDy[1].MABW0);
    T(r->MDy[0].MATW); T(r->MDy[1].MATW);
    return (int)(t - t0);
}
#undef T

/* ------------------------------------------------------------------------- */

void nr3_parse_begin(void)
{
    memset(s_raw, 0, sizeof(s_raw));
    memset(s_fill, 0, sizeof(s_fill));
}

/* Appends the numbers of one (continuation) line of a row to its raw buffer. */
static void nr3_add_numbers(int idx, const char *line)
{
    const char *s = line;
    while (*s) {
        while (*s == ' ' || *s == '\t' || *s == ':' || *s == '|' || *s == '\\')
            s++;
        if (!*s) break;
        const char *e = s;
        while (*e && *e != ' ' && *e != '\t' && *e != ':' && *e != '|' && *e != '\\')
            e++;
        size_t len = (size_t)(e - s);
        int is_num = 1, has_digit = 0;
        for (size_t k = 0; k < len; k++) {
            if (isdigit((unsigned char)s[k])) has_digit = 1;
            else if (!(k == 0 && s[k] == '-' && len > 1)) { is_num = 0; break; }
        }
        if (is_num && has_digit) {
            if (s_fill[idx] < NR3_RAW_VALUES + 8) {
                long v = strtol(s, NULL, 10);
                s_raw[idx][s_fill[idx]] = (HI_U16)(v < 0 ? 0 : v);
            }
            s_fill[idx]++;     /* counted even past the end so an overflow is detected */
        }
        s = e;
    }
}

void nr3_parse_key(scene_nr3_t *n, const char *name, char *value)
{
    int idx;
    if (strcasecmp(name, "3DNRCount") == 0) {
        n->count = (HI_U32)atoi(value);
    } else if (strcasecmp(name, "IsoThresh") == 0) {
        int k = 0;
        for (char *tok = strtok(value, ","); tok && k < NR3_MAX_ROWS; tok = strtok(NULL, ","))
            n->iso[k++] = (HI_U32)strtoul(tok, NULL, 10);
    } else if (sscanf(name, "3DnrParam_%d", &idx) == 1 && idx >= 0 && idx < NR3_MAX_ROWS) {
        nr3_add_numbers(idx, value);
    }
}

void nr3_parse_end(scene_nr3_t *n)
{
    HI_U32 rows = n->count < NR3_MAX_ROWS ? n->count : NR3_MAX_ROWS;
    for (HI_U32 i = 0; i < rows; i++) {
        HI_U16 *t[NR3_RAW_VALUES + 8];
        n->row_ok[i] = 0;
        if (s_fill[i] != NR3_RAW_VALUES) {
            if (s_fill[i] > 0)
                LOGGER(LOGGER_LEVEL_WARNING, "[scene] 3DnrParam_%u: %d values (expected %d), row ignored",
                       (unsigned)i, s_fill[i], NR3_RAW_VALUES);
            continue;
        }
        memset(&n->row[i], 0, sizeof(n->row[i]));
        if (nr3_targets(&n->row[i], t) != NR3_RAW_VALUES)
            continue;
        for (int k = 0; k < NR3_RAW_VALUES; k++)
            *t[k] = s_raw[i][k];
        n->row_ok[i] = 1;
    }
}

/* ------------------------------------------------------------------------- */

/* MapISO of the original: ~6 steps per octave, linear below ISO 72. */
static HI_U32 map_iso(HI_U32 iso)
{
    if (iso < 72) return iso;
    HI_U32 i = (iso >= 200) + (iso >= 400) + (iso >= 800) + (iso >= 1600) + (iso >= 3200) + (iso >= 6400);
    for (int s = 5; s <= 14; s++)
        i += (iso >= (HI_U32)(400u << s));
    HI_U32 j = (iso > (112u << i)) + (iso > (125u << i)) + (iso > (141u << i)) +
               (iso > (158u << i)) + (iso > (178u << i));
    return i * 6 + j + (iso >= 80) + (iso >= 90) + (iso >= 100) - 3;
}

static HI_U32 interp(HI_U32 x, HI_U32 xl, HI_U32 yl, HI_U32 xr, HI_U32 yr)
{
    if (x <= xl) return yl;
    if (x >= xr) return yr;
    HI_U32 k = xr - xl;
    return ((xr - x) * yl + (x - xl) * yr + (k >> 1)) / k;
}

#define I(f) ((HI_U16)interp(x, xl, a->f, xr, b->f))

/* Blends rows a (lower) and b (upper) like HI_SCENE_SetDynamic3DNR: only the
   fields the original interpolates; everything else stays from row a. */
static void nr3_blend(nr3_row_t *out, const nr3_row_t *a, const nr3_row_t *b, HI_U32 iso,
                      HI_U32 iso_l, HI_U32 iso_r)
{
    HI_U32 x = map_iso(iso), xl = map_iso(iso_l), xr = map_iso(iso_r);
    *out = *a;
    for (int i = 0; i < 5; i++) {
        const nr3_iey_t *ia = &a->IEy[i], *ib = &b->IEy[i];
        out->IEy[i].IES0 = (HI_U16)interp(x, xl, ia->IES0, xr, ib->IES0);
        out->IEy[i].IES1 = (HI_U16)interp(x, xl, ia->IES1, xr, ib->IES1);
        out->IEy[i].IES2 = (HI_U16)interp(x, xl, ia->IES2, xr, ib->IES2);
        out->IEy[i].IES3 = (HI_U16)interp(x, xl, ia->IES3, xr, ib->IES3);
        out->IEy[i].IEDZ = (HI_U16)interp(x, xl, ia->IEDZ, xr, ib->IEDZ);
    }
    for (int i = 0; i < 4; i++) {
        const nr3_sfy_t *sa = &a->SFy[i], *sb = &b->SFy[i];
        out->SFy[i].SFS1 = (HI_U16)interp(x, xl, sa->SFS1, xr, sb->SFS1);
        out->SFy[i].SFT1 = (HI_U16)interp(x, xl, sa->SFT1, xr, sb->SFT1);
        out->SFy[i].SBR1 = (HI_U16)interp(x, xl, sa->SBR1, xr, sb->SBR1);
        out->SFy[i].SFS2 = (HI_U16)interp(x, xl, sa->SFS2, xr, sb->SFS2);
        out->SFy[i].SFT2 = (HI_U16)interp(x, xl, sa->SFT2, xr, sb->SFT2);
        out->SFy[i].SBR2 = (HI_U16)interp(x, xl, sa->SBR2, xr, sb->SBR2);
        out->SFy[i].SFS4 = (HI_U16)interp(x, xl, sa->SFS4, xr, sb->SFS4);
        out->SFy[i].SFT4 = (HI_U16)interp(x, xl, sa->SFT4, xr, sb->SFT4);
        out->SFy[i].SBR4 = (HI_U16)interp(x, xl, sa->SBR4, xr, sb->SBR4);
    }
    out->SFy[4].SFS1 = (HI_U16)interp(x, xl, a->SFy[4].SFS1, xr, b->SFy[4].SFS1);
    out->SFy[4].SFS2 = (HI_U16)interp(x, xl, a->SFy[4].SFS2, xr, b->SFy[4].SFS2);
    out->SFy[4].SFS4 = (HI_U16)interp(x, xl, a->SFy[4].SFS4, xr, b->SFy[4].SFS4);
    out->MDy[0].AdvMATH = (HI_U16)interp(x, xl, a->MDy[0].AdvMATH, xr, b->MDy[0].AdvMATH);
    out->MDy[0].AdvTH   = (HI_U16)interp(x, xl, a->MDy[0].AdvTH,   xr, b->MDy[0].AdvTH);
    out->NRc.SFC = I(NRc.SFC);
    out->NRc.TFC = I(NRc.TFC);
    out->NRc.TPC = I(NRc.TPC);
    out->NRc.TRC = I(NRc.TRC);
}
#undef I

/* Row -> VPSS_NRX_V3_S, field set and quirks of HI_SCENE_Set3DNR (the original
   loop over TFy writes TFy[1].SDZ1 from TFy[i].SDZ1, so the last value, i = 2,
   wins; kept as is). */
static void nr3_to_vpss(VPSS_NRX_V3_S *x, const nr3_row_t *r)
{
    int i, j;
    for (i = 0; i < 5; i++) {
        x->IEy[i].IES0 = r->IEy[i].IES0; x->IEy[i].IES1 = r->IEy[i].IES1;
        x->IEy[i].IES2 = r->IEy[i].IES2; x->IEy[i].IES3 = r->IEy[i].IES3;
        x->IEy[i].IEDZ = r->IEy[i].IEDZ;
    }
    for (i = 0; i < 5; i++) {
        const nr3_sfy_t *s = &r->SFy[i];
        x->SFy[i].SPN6 = s->SPN6; x->SFy[i].SBN6 = s->SBN6; x->SFy[i].PBR6 = s->PBR6;
        x->SFy[i].JMODE = s->JMODE;
        for (j = 0; j < 4; j++) x->SFy[i].SFR6[j] = (HI_U8)s->SFR6[j];
        x->SFy[i].SBR6[0] = (HI_U8)s->SBR6[0]; x->SFy[i].SBR6[1] = (HI_U8)s->SBR6[1];
        x->SFy[i].SFN0 = s->SFN0; x->SFy[i].SFN1 = s->SFN1; x->SFy[i].SFN2 = s->SFN2;
        x->SFy[i].STH1 = s->STH1; x->SFy[i].STH2 = s->STH2;
        x->SFy[i].STHd1 = s->STHd1; x->SFy[i].STHd2 = s->STHd2;
        x->SFy[i].SFR = s->SFR;
    }
    for (i = 0; i < 4; i++) {
        const nr3_sfy_t *s = &r->SFy[i];
        x->SFy[i].SFS1 = s->SFS1; x->SFy[i].SFT1 = s->SFT1; x->SFy[i].SBR1 = s->SBR1;
        x->SFy[i].SFS2 = s->SFS2; x->SFy[i].SFT2 = s->SFT2; x->SFy[i].SBR2 = s->SBR2;
        x->SFy[i].SFS4 = s->SFS4; x->SFy[i].SFT4 = s->SFT4; x->SFy[i].SBR4 = s->SBR4;
    }
    x->SFy[0].SRT0 = r->SFy[0].SRT0; x->SFy[0].SRT1 = r->SFy[0].SRT1;
    x->SFy[2].kMode = r->SFy[2].kMode; x->SFy[3].kMode = r->SFy[3].kMode;
    x->SFy[0].DeIdx = r->SFy[0].DeIdx; x->SFy[0].DeRate = (HI_U8)r->SFy[0].DeRate;
    x->SFy[4].SFS1 = (HI_U8)r->SFy[4].SFS1; x->SFy[4].SFS2 = (HI_U8)r->SFy[4].SFS2;
    x->SFy[4].SFS4 = (HI_U8)r->SFy[4].SFS4;

    for (i = 0; i < 3; i++) {
        const nr3_tfy_t *t = &r->TFy[i];
        if (i != 2) x->TFy[i].bRef = t->bRef;
        if (i != 0) x->TFy[i].TED = t->TED;
        x->TFy[i].STR0 = t->STR0;  x->TFy[1].STR1 = r->TFy[1].STR1;
        x->TFy[i].SDZ0 = t->SDZ0;  x->TFy[1].SDZ1 = t->SDZ1;
        x->TFy[i].TSS0 = t->TSS0;  x->TFy[1].TSS1 = r->TFy[1].TSS1;
        x->TFy[i].TSI0 = t->TSI0;  x->TFy[1].TSI1 = r->TFy[1].TSI1;
        x->TFy[i].TFS0 = t->TFS0;  x->TFy[1].TFS1 = r->TFy[1].TFS1;
        x->TFy[i].TDZ0 = t->TDZ0;  x->TFy[1].TDZ1 = r->TFy[1].TDZ1;
        x->TFy[i].TDX0 = t->TDX0;  x->TFy[1].TDX1 = r->TFy[1].TDX1;
        for (j = 0; j < 6; j++) x->TFy[i].TFR0[j] = (HI_U8)t->TFR0[j];
        for (j = 0; j < 6; j++) x->TFy[1].TFR1[j] = (HI_U8)r->TFy[1].TFR1[j];
    }
    x->TFy[0].TFRS = r->TFy[0].TFRS;

    for (i = 0; i < 2; i++) {
        const nr3_mdy_t *m = &r->MDy[i];
        x->MDy[i].MAI00 = m->MAI00; x->MDy[i].MAI01 = m->MAI01; x->MDy[i].MAI02 = m->MAI02;
        x->MDy[i].MAI10 = m->MAI10; x->MDy[i].MAI11 = m->MAI11; x->MDy[i].MAI12 = m->MAI12;
        x->MDy[i].MABR0 = (HI_U8)m->MABR0; x->MDy[0].MABR1 = (HI_U8)r->MDy[0].MABR1;
        x->MDy[0].AdvMATH = r->MDy[0].AdvMATH; x->MDy[0].AdvTH = r->MDy[0].AdvTH;
        x->MDy[i].MATH0 = m->MATH0; x->MDy[i].MATH1 = m->MATH1;
        x->MDy[i].MATHd0 = m->MATHd0; x->MDy[i].MATHd1 = m->MATHd1;
        x->MDy[i].MATE0 = m->MATE0; x->MDy[i].MATE1 = (HI_U8)m->MATE1;
        x->MDy[i].MABW0 = (HI_U8)m->MABW0; x->MDy[i].MABW1 = (HI_U8)m->MABW1;
        x->MDy[i].MATW = m->MATW;
    }
    x->NRc.SFC = (HI_U8)r->NRc.SFC; x->NRc.TFC = r->NRc.TFC;
    x->NRc.TPC = r->NRc.TPC; x->NRc.TRC = (HI_U8)r->NRc.TRC;
}

static void nr3_dump(HI_U32 iso, const nr3_row_t *row)
{
    nr3_row_t tmp = *row;
    HI_U16 *t[NR3_RAW_VALUES + 8];
    int n = nr3_targets(&tmp, t);
    char line[256];
    for (int k = 0; k < n; k += 32) {
        size_t len = 0;
        line[0] = '\0';
        for (int m = k; m < n && m < k + 32; m++)
            len += (size_t)snprintf(line + len, sizeof(line) - len, "%u,", (unsigned)*t[m]);
        LOGGER(LOGGER_LEVEL_INFO, "[scene] 3DNR iso=%u [%d..]: %s", (unsigned)iso, k, line);
    }
}

int nr3_apply(const scene_nr3_t *n, HI_U32 iso, int debug)
{
    if (n->count == 0)
        return 0;
    HI_U32 cnt = n->count < NR3_MAX_ROWS ? n->count : NR3_MAX_ROWS;

    /* GetLevelLtoH: first threshold >= iso, clamped to the last row. */
    HI_U32 l = 0;
    while (l < cnt && iso > n->iso[l]) l++;
    if (l >= cnt) l = cnt - 1;

    nr3_row_t cur;
    if (l == 0) {
        if (!n->row_ok[0]) return -1;
        cur = n->row[0];
    } else {
        if (!n->row_ok[l - 1] || !n->row_ok[l]) return -1;
        nr3_blend(&cur, &n->row[l - 1], &n->row[l], iso, n->iso[l - 1], n->iso[l]);
    }

    VPSS_GRP_NRX_PARAM_S nrx;
    memset(&nrx, 0, sizeof(nrx));
    nrx.enNRVer = VPSS_NR_V3;
    nrx.stNRXParam_V3.enOptMode = OPERATION_MODE_MANUAL;
    HI_S32 ret = HI_MPI_VPSS_GetGrpNRXParam(NR3_VPSS_GRP, &nrx);
    if (ret != HI_SUCCESS) {
        /* Expected until the VPSS group is created (scene init runs earlier):
           report only the first failure of a series. */
        if (!s_warned)
            LOGGER(LOGGER_LEVEL_WARNING, "[scene] 3DNR GetGrpNRXParam failed 0x%x (retrying)", (unsigned)ret);
        s_warned = 1;
        return -1;
    }
    s_warned = 0;
    nr3_to_vpss(&nrx.stNRXParam_V3.stNRXManual.stNRXParam, &cur);
    nrx.enNRVer = VPSS_NR_V3;
    nrx.stNRXParam_V3.enOptMode = OPERATION_MODE_MANUAL;
    ret = HI_MPI_VPSS_SetGrpNRXParam(NR3_VPSS_GRP, &nrx);
    if (ret != HI_SUCCESS) {
        LOGGER(LOGGER_LEVEL_WARNING, "[scene] 3DNR SetGrpNRXParam failed 0x%x", (unsigned)ret);
        return -1;
    }
    if (debug)
        nr3_dump(iso, &cur);
    return 0;
}
