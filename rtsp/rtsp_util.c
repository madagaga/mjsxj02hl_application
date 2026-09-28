#include <stdio.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "./rtsp_util.h"

/* ============================================================================
   MD5 (RFC 1321), only used for RTSP Digest authentication
   ============================================================================ */

typedef struct {
    uint32_t state[4];
    uint64_t length;
    uint8_t  block[64];
    size_t   used;
} md5_ctx_t;

static const uint32_t md5_k[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391
};

static const uint8_t md5_r[64] = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21
};

static void md5_transform(md5_ctx_t *ctx, const uint8_t *block) {
    uint32_t w[16];
    for (int i = 0; i < 16; i++) {
        w[i] = (uint32_t)block[i * 4] | ((uint32_t)block[i * 4 + 1] << 8) |
               ((uint32_t)block[i * 4 + 2] << 16) | ((uint32_t)block[i * 4 + 3] << 24);
    }
    uint32_t a = ctx->state[0], b = ctx->state[1], c = ctx->state[2], d = ctx->state[3];
    for (int i = 0; i < 64; i++) {
        uint32_t f, g;
        if (i < 16)      { f = (b & c) | (~b & d); g = i; }
        else if (i < 32) { f = (d & b) | (~d & c); g = (5 * i + 1) & 15; }
        else if (i < 48) { f = b ^ c ^ d;          g = (3 * i + 5) & 15; }
        else             { f = c ^ (b | ~d);       g = (7 * i) & 15; }
        uint32_t t = d;
        d = c;
        c = b;
        uint32_t x = a + f + md5_k[i] + w[g];
        b = b + ((x << md5_r[i]) | (x >> (32 - md5_r[i])));
        a = t;
    }
    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
}

static void md5_update(md5_ctx_t *ctx, const uint8_t *data, size_t size) {
    ctx->length += size;
    while (size > 0) {
        size_t n = 64 - ctx->used;
        if (n > size) n = size;
        memcpy(ctx->block + ctx->used, data, n);
        ctx->used += n;
        data += n;
        size -= n;
        if (ctx->used == 64) {
            md5_transform(ctx, ctx->block);
            ctx->used = 0;
        }
    }
}

void rtsp_md5_hex(const void *data, size_t size, char out[33]) {
    md5_ctx_t ctx = { { 0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476 }, 0, { 0 }, 0 };
    md5_update(&ctx, (const uint8_t *)data, size);

    uint64_t bits = ctx.length * 8;
    uint8_t pad = 0x80;
    md5_update(&ctx, &pad, 1);
    pad = 0;
    while (ctx.used != 56) md5_update(&ctx, &pad, 1);
    uint8_t len[8];
    for (int i = 0; i < 8; i++) len[i] = (uint8_t)(bits >> (8 * i));
    md5_update(&ctx, len, 8);

    for (int i = 0; i < 16; i++) {
        snprintf(out + i * 2, 3, "%02x", (unsigned)((ctx.state[i / 4] >> (8 * (i % 4))) & 0xff));
    }
    out[32] = '\0';
}

/* ============================================================================
   Base64 (SDP sprop parameter sets)
   ============================================================================ */

size_t rtsp_base64(const uint8_t *data, size_t size, char *out, size_t out_size) {
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t needed = ((size + 2) / 3) * 4;
    if (out_size < needed + 1) return 0;

    size_t o = 0;
    for (size_t i = 0; i < size; i += 3) {
        uint32_t v = (uint32_t)data[i] << 16;
        if (i + 1 < size) v |= (uint32_t)data[i + 1] << 8;
        if (i + 2 < size) v |= data[i + 2];
        out[o++] = table[(v >> 18) & 0x3f];
        out[o++] = table[(v >> 12) & 0x3f];
        out[o++] = (i + 1 < size) ? table[(v >> 6) & 0x3f] : '=';
        out[o++] = (i + 2 < size) ? table[v & 0x3f] : '=';
    }
    out[o] = '\0';
    return o;
}

/* ============================================================================
   Random / time
   ============================================================================ */

static pthread_mutex_t random_lock = PTHREAD_MUTEX_INITIALIZER;
static uint32_t random_state = 0;

uint32_t rtsp_random(void) {
    pthread_mutex_lock(&random_lock);
    if (random_state == 0) {
        FILE *f = fopen("/dev/urandom", "rb");
        if (f) {
            if (fread(&random_state, sizeof(random_state), 1, f) != 1) random_state = 0;
            fclose(f);
        }
        if (random_state == 0) random_state = (uint32_t)rtsp_now_ms() | 1;
    }
    // xorshift32: enough for SSRC, sequence numbers, session ids and nonces
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    uint32_t value = random_state;
    pthread_mutex_unlock(&random_lock);
    return value;
}

uint64_t rtsp_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}
