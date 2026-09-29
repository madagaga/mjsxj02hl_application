#ifndef _RTSP_STREAM_H_
#define _RTSP_STREAM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#include "./rtsp_server.h"

// NAL (or audio frame) slots per stream. 20 fps x ~4 NALs x 2 s is ~160;
// a full slot table evicts the oldest entry like a full byte buffer does.
#define RTSP_STREAM_MAX_NALS   512

#define RTSP_NAL_AU_START      0x01  // first NAL of an access unit (frame)
#define RTSP_NAL_AU_END        0x02  // last NAL of an access unit: RTP marker
#define RTSP_NAL_KEY           0x04  // AU start of a key frame: a client may join here

#define RTSP_PARAM_MAX         128   // max size of a cached VPS/SPS/PPS

typedef struct {
    uint32_t offset;   // in buf
    uint32_t size;     // NAL without start code
    uint64_t pts;      // microseconds, shared by all NALs of an AU
    uint8_t  flags;
} rtsp_nal_t;

typedef struct {
    uint8_t  data[RTSP_PARAM_MAX];
    uint32_t size;
} rtsp_param_t;

/* A fixed-size ring of NAL units written by one producer (encoder thread) and
   read in place by the network thread. Both sides hold `lock`. Entries are
   addressed by a monotonic sequence number; evicted entries simply fall
   behind `first_seq`, which tells a slow reader it has been lapped. */
typedef struct {
    pthread_mutex_t lock;
    rtsp_codec_t codec;
    uint32_t clock_rate;

    uint8_t *buf;
    uint32_t buf_size;
    uint32_t head;              // next write offset in buf

    rtsp_nal_t nal[RTSP_STREAM_MAX_NALS];
    uint32_t first_seq;         // oldest entry still stored
    uint32_t next_seq;          // next entry to be written
    uint32_t key_seq;           // latest RTSP_NAL_KEY entry
    bool     has_key;
    uint32_t au_seq;            // AU start of the AU being written
    bool     au_open;           // an AU is being written (no frame end seen yet)

    rtsp_param_t vps, sps, pps; // latest parameter sets, for the SDP

    uint64_t last_pts;          // latest pushed pts
    bool     clock_set;         // pts -> wallclock anchor, for RTCP SR
    uint64_t clock_pts;
    uint64_t clock_wall_us;     // CLOCK_REALTIME at clock_pts
} rtsp_stream_t;

// The ring is an anonymous mapping: pages are only backed once written
bool rtsp_stream_init(rtsp_stream_t *s, rtsp_codec_t codec, uint32_t buf_size);
void rtsp_stream_free(rtsp_stream_t *s);

// Drops every entry and hands the ring pages back to the kernel (no reader
// left). Cached parameter sets are kept for the next SDP.
void rtsp_stream_reset(rtsp_stream_t *s);

// Producer side. Video: an Annex-B buffer holding one or more NALs;
// frame_end marks the last buffer of a frame. Audio: one frame.
// With store = false only the parameter sets are cached (no reader).
bool rtsp_stream_push(rtsp_stream_t *s, const uint8_t *data, size_t size, uint64_t pts, bool frame_end, bool store);

// Reader side, `lock` held: true if seq is still stored
static inline bool rtsp_stream_has(const rtsp_stream_t *s, uint32_t seq) {
    return (uint32_t)(seq - s->first_seq) < (uint32_t)(s->next_seq - s->first_seq);
}

static inline const rtsp_nal_t *rtsp_stream_nal(const rtsp_stream_t *s, uint32_t seq) {
    return &s->nal[seq % RTSP_STREAM_MAX_NALS];
}

// Reader side, `lock` held: first entry with pts >= pts (next_seq if none)
uint32_t rtsp_stream_find_pts(const rtsp_stream_t *s, uint64_t pts);

// NTP-style wallclock (microseconds since 1970) of a pts, `lock` held
uint64_t rtsp_stream_wallclock(const rtsp_stream_t *s, uint64_t pts);

// Clock rate of a codec (90 kHz video, 8 kHz PCMA)
uint32_t rtsp_codec_clock_rate(rtsp_codec_t codec);

#endif
