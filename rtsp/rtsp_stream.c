#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>

#include "./rtsp_stream.h"

uint32_t rtsp_codec_clock_rate(rtsp_codec_t codec) {
    switch (codec) {
        case RTSP_CODEC_H264:
        case RTSP_CODEC_H265: return 90000;
        case RTSP_CODEC_PCMA: return 8000;
        default: return 0;
    }
}

bool rtsp_stream_init(rtsp_stream_t *s, rtsp_codec_t codec, uint32_t buf_size) {
    memset(s, 0, sizeof(*s));
    s->codec = codec;
    s->clock_rate = rtsp_codec_clock_rate(codec);
    s->buf_size = buf_size;
    void *buf = mmap(NULL, buf_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (buf == MAP_FAILED) return false;
    s->buf = buf;
    pthread_mutex_init(&s->lock, NULL);
    return true;
}

void rtsp_stream_free(rtsp_stream_t *s) {
    if (s->buf) {
        pthread_mutex_destroy(&s->lock);
        munmap(s->buf, s->buf_size);
    }
    memset(s, 0, sizeof(*s));
}

void rtsp_stream_reset(rtsp_stream_t *s) {
    if (!s->buf) return;
    pthread_mutex_lock(&s->lock);
    s->first_seq = s->next_seq;
    s->has_key = false;
    s->head = 0;
    madvise(s->buf, s->buf_size, MADV_DONTNEED);
    pthread_mutex_unlock(&s->lock);
}

/* ============================================================================
   NAL classification
   ============================================================================ */

typedef enum { NAL_OTHER, NAL_VPS, NAL_SPS, NAL_PPS, NAL_IRAP } nal_kind_t;

static nal_kind_t nal_kind(rtsp_codec_t codec, const uint8_t *nal, size_t size) {
    if (size < 1) return NAL_OTHER;
    if (codec == RTSP_CODEC_H264) {
        switch (nal[0] & 0x1f) {
            case 5: return NAL_IRAP;
            case 7: return NAL_SPS;
            case 8: return NAL_PPS;
            default: return NAL_OTHER;
        }
    }
    if (codec == RTSP_CODEC_H265) {
        uint8_t type = (nal[0] >> 1) & 0x3f;
        if (type >= 16 && type <= 21) return NAL_IRAP;  // BLA/IDR/CRA
        switch (type) {
            case 32: return NAL_VPS;
            case 33: return NAL_SPS;
            case 34: return NAL_PPS;
            default: return NAL_OTHER;
        }
    }
    return NAL_OTHER;
}

// Next Annex-B start code at or after pos; returns its offset (size if none)
static size_t find_start_code(const uint8_t *data, size_t size, size_t pos, size_t *sc_len) {
    for (size_t i = pos; i + 3 <= size; i++) {
        if (data[i] == 0 && data[i + 1] == 0) {
            if (data[i + 2] == 1) { *sc_len = 3; return i; }
            if (i + 4 <= size && data[i + 2] == 0 && data[i + 3] == 1) { *sc_len = 4; return i; }
        }
    }
    *sc_len = 0;
    return size;
}

/* ============================================================================
   Ring storage
   ============================================================================ */

static void evict_oldest(rtsp_stream_t *s) {
    if (s->first_seq == s->next_seq) return;
    if (s->has_key && s->key_seq == s->first_seq) s->has_key = false;
    s->first_seq++;
}

// Reserves `size` contiguous bytes, evicting whatever overlaps them
static bool reserve(rtsp_stream_t *s, uint32_t size, uint32_t *offset) {
    if (size > s->buf_size / 2) return false;  // would starve the ring
    uint32_t start = s->head;
    if (start + size > s->buf_size) start = 0;  // never split an entry: wrap
    uint32_t end = start + size;

    // Entries are stored in order, so the oldest ones are the only candidates
    while (s->first_seq != s->next_seq) {
        const rtsp_nal_t *old = rtsp_stream_nal(s, s->first_seq);
        uint32_t old_end = old->offset + old->size;
        bool overlap = (old->offset < end) && (start < old_end);
        // Wrapping to 0 also discards the unused tail behind the old head
        bool skipped = (start < s->head) && (old->offset >= s->head);
        if (!overlap && !skipped) break;
        evict_oldest(s);
    }
    if ((uint32_t)(s->next_seq - s->first_seq) >= RTSP_STREAM_MAX_NALS) evict_oldest(s);

    s->head = end;
    *offset = start;
    return true;
}

static void cache_param(rtsp_param_t *param, const uint8_t *nal, size_t size) {
    if (size > RTSP_PARAM_MAX) return;
    memcpy(param->data, nal, size);
    param->size = (uint32_t)size;
}

static bool store(rtsp_stream_t *s, const uint8_t *data, size_t size, uint64_t pts, uint8_t flags) {
    uint32_t offset;
    if (!reserve(s, (uint32_t)size, &offset)) return false;
    memcpy(s->buf + offset, data, size);

    rtsp_nal_t *nal = &s->nal[s->next_seq % RTSP_STREAM_MAX_NALS];
    nal->offset = offset;
    nal->size = (uint32_t)size;
    nal->pts = pts;
    nal->flags = flags;
    if (flags & RTSP_NAL_KEY) {
        s->key_seq = s->next_seq;
        s->has_key = true;
    }
    s->next_seq++;
    return true;
}

static void anchor_clock(rtsp_stream_t *s, uint64_t pts) {
    s->last_pts = pts;
    if (s->clock_set) return;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    s->clock_pts = pts;
    s->clock_wall_us = (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
    s->clock_set = true;
}

static bool push_nal(rtsp_stream_t *s, const uint8_t *nal, size_t size, uint64_t pts, bool au_end, bool keep) {
    uint8_t flags = 0;
    bool au_start = !s->au_open;
    nal_kind_t kind = nal_kind(s->codec, nal, size);

    switch (kind) {
        case NAL_VPS: cache_param(&s->vps, nal, size); break;
        case NAL_SPS: cache_param(&s->sps, nal, size); break;
        case NAL_PPS: cache_param(&s->pps, nal, size); break;
        default: break;
    }
    if (!keep) {
        s->au_open = !au_end;  // AU boundaries stay right when storing resumes
        return true;
    }

    if (au_start) flags |= RTSP_NAL_AU_START;
    if (au_end) flags |= RTSP_NAL_AU_END;
    // A key frame is joinable at its AU start (VPS/SPS first on HiSilicon)
    if (au_start && kind != NAL_OTHER) flags |= RTSP_NAL_KEY;

    uint32_t seq = s->next_seq;
    if (!store(s, nal, size, pts, flags)) return false;
    if (au_start) s->au_seq = seq;

    // IRAP found after other NALs (e.g. SEI first): make its AU joinable
    if (kind == NAL_IRAP && !au_start && rtsp_stream_has(s, s->au_seq)) {
        s->nal[s->au_seq % RTSP_STREAM_MAX_NALS].flags |= RTSP_NAL_KEY;
        s->key_seq = s->au_seq;
        s->has_key = true;
    }

    s->au_open = !au_end;
    return true;
}

bool rtsp_stream_push(rtsp_stream_t *s, const uint8_t *data, size_t size, uint64_t pts, bool frame_end, bool keep) {
    if (!s->buf || !data || size == 0) return false;
    bool result = true;

    pthread_mutex_lock(&s->lock);
    anchor_clock(s, pts);

    if (s->codec == RTSP_CODEC_PCMA) {
        // Every audio frame is its own joinable AU
        if (keep) result = store(s, data, size, pts, RTSP_NAL_AU_START | RTSP_NAL_AU_END | RTSP_NAL_KEY);
    } else {
        // Split on start codes; a buffer without any is taken as one NAL
        size_t sc_len;
        size_t pos = find_start_code(data, size, 0, &sc_len);
        if (pos == size) {
            result = push_nal(s, data, size, pts, frame_end, keep);
        } else {
            while (pos < size) {
                size_t nal_start = pos + sc_len;
                size_t next = find_start_code(data, size, nal_start, &sc_len);
                bool last = (next == size);
                if (next > nal_start) {
                    result &= push_nal(s, data + nal_start, next - nal_start, pts, last && frame_end, keep);
                } else if (last && frame_end && s->au_open) {
                    s->au_open = false;  // empty trailing NAL: still close the AU
                }
                pos = next;
            }
        }
        if (!result) {
            // A dropped NAL breaks its AU: readers must not join or resume on it
            s->au_open = !frame_end;
        }
    }

    pthread_mutex_unlock(&s->lock);
    return result;
}

uint32_t rtsp_stream_find_pts(const rtsp_stream_t *s, uint64_t pts) {
    for (uint32_t seq = s->first_seq; seq != s->next_seq; seq++) {
        if (rtsp_stream_nal(s, seq)->pts >= pts) return seq;
    }
    return s->next_seq;
}

uint64_t rtsp_stream_wallclock(const rtsp_stream_t *s, uint64_t pts) {
    if (!s->clock_set) return 0;
    return s->clock_wall_us + (pts - s->clock_pts);
}
