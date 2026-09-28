#ifndef _RTSP_UTIL_H_
#define _RTSP_UTIL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// MD5 digest of a byte string, written as 32 lowercase hex chars + NUL
void rtsp_md5_hex(const void *data, size_t size, char out[33]);

// Base64 encode; returns the encoded length (without NUL) or 0 if out is too small
size_t rtsp_base64(const uint8_t *data, size_t size, char *out, size_t out_size);

// Pseudo-random 32-bit value (seeded from /dev/urandom on first use)
uint32_t rtsp_random(void);

// Monotonic clock in milliseconds
uint64_t rtsp_now_ms(void);

#endif
