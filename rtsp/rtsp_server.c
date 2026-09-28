#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include "./rtsp_server.h"
#include "./rtsp_stream.h"
#include "./rtsp_util.h"
#include "./../logger/logger.h"

/* ============================================================================
   Limits
   ============================================================================ */

#define RTSP_RX_SIZE          2048   // one RTSP request (+ interleaved RTCP)
#define RTSP_TX_SIZE          4096   // one reply, or the rest of a partly sent packet
#define RTSP_MAX_PAYLOAD      1400   // RTP payload, fits a WiFi MTU
#define RTSP_HEADER_MAX       20     // interleave (4) + RTP (12) + FU header (3)
#define RTSP_BATCH            32     // RTP packets per writev
#define RTSP_TIMEOUT_S        60     // session timeout advertised to clients
#define RTSP_STALL_MS         30000  // TCP client not draining anything: dropped
#define RTSP_SR_INTERVAL_MS   5000
#define RTSP_THREAD_STACK     (64 * 1024)
#define RTSP_REALM            "mjsxj02hl"
#define RTSP_TRACK_VIDEO      0
#define RTSP_TRACK_AUDIO      1
#define RTSP_TRACKS           2

/* ============================================================================
   State
   ============================================================================ */

typedef struct {
    bool     used;
    char     name[64];
    uint32_t framerate;
    rtsp_stream_t stream[RTSP_TRACKS];   // codec NONE when absent
} session_t;

typedef struct {
    bool     setup;
    uint8_t  channel;                    // TCP interleaved RTP channel (RTCP = +1)
    struct sockaddr_in rtp_addr;         // UDP peer
    struct sockaddr_in rtcp_addr;
    uint32_t ssrc;
    uint16_t seq;
    uint32_t ts_base;
    uint32_t cur;                        // next ring entry to send
    uint32_t frag;                       // bytes of `cur` already packetized
    bool     wait_key;                   // skip until a key frame AU start
    uint32_t packets;
    uint32_t octets;
    uint64_t last_pts;
    bool     sent;
} track_t;

typedef struct {
    bool     used;
    int      fd;
    struct sockaddr_in peer;
    int      session;                    // -1 until DESCRIBE/SETUP
    bool     tcp;                        // interleaved transport
    bool     playing;
    bool     closing;                    // close once tx is flushed
    bool     blocked;                    // last write hit EAGAIN: wait for POLLOUT
    uint32_t session_id;
    char     nonce[33];
    uint8_t  rx[RTSP_RX_SIZE];
    size_t   rx_len;
    size_t   rx_skip;                    // interleaved bytes still to discard
    uint8_t  tx[RTSP_TX_SIZE];
    size_t   tx_off;
    size_t   tx_len;
    track_t  track[RTSP_TRACKS];
    uint64_t activity_ms;
    uint64_t progress_ms;
    uint64_t sr_ms;
} client_t;

static session_t g_sessions[RTSP_SERVER_MAX_SESSIONS];
static client_t  g_clients[RTSP_SERVER_MAX_CLIENTS];
static rtsp_server_config_t g_config;
static char      g_username[64];
static char      g_password[64];
static bool      g_auth = false;

static int       g_listen_fd = -1;
static int       g_rtp_fd = -1;          // UDP, shared by every client
static int       g_rtcp_fd = -1;
static uint16_t  g_rtp_port = 0;
static int       g_wake_fd[2] = { -1, -1 };
static volatile int g_wake_pending = 0;
static volatile int g_playing = 0;       // clients currently playing
static volatile int g_connected = 0;     // clients connected: rings only fill then
static volatile bool g_running = false;
static pthread_t g_thread;

/* ============================================================================
   Helpers
   ============================================================================ */

static void set_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void wake(void) {
    if (g_playing <= 0) return;
    if (__sync_lock_test_and_set(&g_wake_pending, 1)) return;
    char c = 1;
    if (write(g_wake_fd[1], &c, 1) < 0) { /* pipe full: already woken */ }
}

static uint32_t rtp_ts(const track_t *t, const rtsp_stream_t *s, uint64_t pts) {
    return t->ts_base + (uint32_t)((pts * s->clock_rate) / 1000000);
}

static bool tx_append(client_t *c, const void *data, size_t size) {
    if (c->tx_off > 0) {  // compact
        memmove(c->tx, c->tx + c->tx_off, c->tx_len - c->tx_off);
        c->tx_len -= c->tx_off;
        c->tx_off = 0;
    }
    if (c->tx_len + size > RTSP_TX_SIZE) return false;
    memcpy(c->tx + c->tx_len, data, size);
    c->tx_len += size;
    return true;
}

// Sends pending tx bytes; false on a fatal socket error
static bool tx_flush(client_t *c) {
    while (c->tx_off < c->tx_len) {
        ssize_t n = send(c->fd, c->tx + c->tx_off, c->tx_len - c->tx_off, MSG_NOSIGNAL);
        if (n > 0) {
            c->tx_off += (size_t)n;
            c->progress_ms = rtsp_now_ms();
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            c->blocked = true;
            return true;
        }
        return false;
    }
    c->tx_off = c->tx_len = 0;
    return true;
}

static void client_close(client_t *c) {
    if (!c->used) return;
    LOGGER(LOGGER_LEVEL_INFO, "RTSP client %s:%u disconnected.", inet_ntoa(c->peer.sin_addr), ntohs(c->peer.sin_port));
    if (c->playing) __sync_fetch_and_sub(&g_playing, 1);
    close(c->fd);
    memset(c, 0, sizeof(*c));
    c->fd = -1;

    // Last client gone: give the ring memory back until someone connects
    if (__sync_sub_and_fetch(&g_connected, 1) == 0) {
        for (int i = 0; i < RTSP_SERVER_MAX_SESSIONS; i++) {
            for (int k = 0; k < RTSP_TRACKS; k++) rtsp_stream_reset(&g_sessions[i].stream[k]);
        }
    }
}

static void client_play(client_t *c, bool play) {
    if (c->playing == play) return;
    c->playing = play;
    if (play) __sync_fetch_and_add(&g_playing, 1);
    else __sync_fetch_and_sub(&g_playing, 1);
}

/* ============================================================================
   RTP packetization (built in place from the ring, payload never copied)
   ============================================================================ */

typedef struct {
    uint8_t  header[RTSP_HEADER_MAX];
    uint32_t header_size;                // including the 4-byte interleave prefix
    const uint8_t *payload;
    uint32_t payload_size;
    uint32_t next_cur;                   // cursor after this packet
    uint32_t next_frag;
    uint64_t pts;
} packet_t;

// Builds the packet at the track cursor (track_resync() already stepped over
// what a key frame wait skips); false when the cursor is caught up
static bool packet_build(const track_t *t, const rtsp_stream_t *s, packet_t *p) {
    if (t->wait_key) return false;
    if (t->cur != s->next_seq) {
        const rtsp_nal_t *nal = rtsp_stream_nal(s, t->cur);
        const uint8_t *data = s->buf + nal->offset;
        uint32_t size = nal->size;
        uint8_t *h = p->header + 4 + 12;  // after interleave + RTP header
        uint32_t extra = 0;               // FU bytes
        bool last;

        p->next_cur = t->cur;
        p->pts = nal->pts;

        if (s->codec == RTSP_CODEC_PCMA || size <= RTSP_MAX_PAYLOAD) {
            p->payload = data;
            p->payload_size = size;
            last = true;
        } else if (s->codec == RTSP_CODEC_H264) {
            // FU-A: the NAL header byte is carried in the FU indicator/header
            uint32_t start = t->frag ? t->frag : 1;
            uint32_t chunk = size - start;
            if (chunk > RTSP_MAX_PAYLOAD - 2) chunk = RTSP_MAX_PAYLOAD - 2;
            last = (start + chunk == size);
            h[0] = (uint8_t)((data[0] & 0xe0) | 28);
            h[1] = (uint8_t)((t->frag == 0 ? 0x80 : 0) | (last ? 0x40 : 0) | (data[0] & 0x1f));
            extra = 2;
            p->payload = data + start;
            p->payload_size = chunk;
            p->next_frag = start + chunk;
        } else {
            // H.265 FU (type 49): 2-byte payload header + 1-byte FU header
            uint32_t start = t->frag ? t->frag : 2;
            uint32_t chunk = size - start;
            if (chunk > RTSP_MAX_PAYLOAD - 3) chunk = RTSP_MAX_PAYLOAD - 3;
            last = (start + chunk == size);
            h[0] = (uint8_t)((data[0] & 0x81) | (49 << 1));
            h[1] = data[1];
            h[2] = (uint8_t)((t->frag == 0 ? 0x80 : 0) | (last ? 0x40 : 0) | ((data[0] >> 1) & 0x3f));
            extra = 3;
            p->payload = data + start;
            p->payload_size = chunk;
            p->next_frag = start + chunk;
        }

        if (last) {
            p->next_cur = t->cur + 1;
            p->next_frag = 0;
        }

        bool marker = last && (nal->flags & RTSP_NAL_AU_END) && s->codec != RTSP_CODEC_PCMA;
        uint8_t pt = (s->codec == RTSP_CODEC_PCMA) ? 8 : 96;
        uint32_t ts = rtp_ts(t, s, nal->pts);
        uint32_t rtp_size = 12 + extra + p->payload_size;

        uint8_t *r = p->header;
        r[0] = '$';
        r[1] = t->channel;
        r[2] = (uint8_t)(rtp_size >> 8);
        r[3] = (uint8_t)rtp_size;
        r[4] = 0x80;
        r[5] = (uint8_t)((marker ? 0x80 : 0) | pt);
        r[6] = (uint8_t)(t->seq >> 8);
        r[7] = (uint8_t)t->seq;
        r[8] = (uint8_t)(ts >> 24);
        r[9] = (uint8_t)(ts >> 16);
        r[10] = (uint8_t)(ts >> 8);
        r[11] = (uint8_t)ts;
        r[12] = (uint8_t)(t->ssrc >> 24);
        r[13] = (uint8_t)(t->ssrc >> 16);
        r[14] = (uint8_t)(t->ssrc >> 8);
        r[15] = (uint8_t)t->ssrc;
        p->header_size = 4 + 12 + extra;
        return true;
    }
    return false;
}

static void packet_commit(track_t *t, const packet_t *p) {
    t->cur = p->next_cur;
    t->frag = p->next_frag;
    t->seq++;
    t->packets++;
    t->octets += (p->header_size - 16) + p->payload_size;
    t->last_pts = p->pts;
    t->sent = true;
}

// Lapped by the producer: resume on the latest buffered key frame. Then,
// while waiting for a key frame, step over the entries before it.
static void track_resync(track_t *t, const rtsp_stream_t *s) {
    if (t->cur != s->next_seq && !rtsp_stream_has(s, t->cur)) {
        if (s->has_key) {
            t->cur = s->key_seq;
            t->wait_key = false;
        } else {
            t->cur = s->next_seq;
            t->wait_key = true;
        }
        t->frag = 0;
    }
    while (t->wait_key && t->cur != s->next_seq) {
        if ((rtsp_stream_nal(s, t->cur)->flags & RTSP_NAL_KEY) && t->frag == 0) {
            t->wait_key = false;
            break;
        }
        t->cur++;
        t->frag = 0;
    }
}

// TCP: false on a fatal socket error
static bool pump_tcp(client_t *c, track_t *t, rtsp_stream_t *s) {
    packet_t pkt[RTSP_BATCH];
    struct iovec iov[RTSP_BATCH * 2];
    bool result = true;

    pthread_mutex_lock(&s->lock);
    track_resync(t, s);

    for (;;) {
        int count = 0;
        size_t total = 0;
        track_t probe = *t;  // cursor as it will be once the batch is sent
        while (count < RTSP_BATCH && packet_build(&probe, s, &pkt[count])) {
            // A FU fragment continues the same NAL: next_cur stays, frag moves
            iov[count * 2].iov_base = pkt[count].header;
            iov[count * 2].iov_len = pkt[count].header_size;
            iov[count * 2 + 1].iov_base = (void *)pkt[count].payload;
            iov[count * 2 + 1].iov_len = pkt[count].payload_size;
            total += pkt[count].header_size + pkt[count].payload_size;
            probe.cur = pkt[count].next_cur;
            probe.frag = pkt[count].next_frag;
            probe.seq++;
            count++;
        }
        if (count == 0) break;

        ssize_t n = writev(c->fd, iov, count * 2);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                c->blocked = true;
            } else {
                result = false;
            }
            break;
        }
        c->progress_ms = rtsp_now_ms();

        size_t sent = (size_t)n;
        for (int i = 0; i < count; i++) {
            size_t size = pkt[i].header_size + pkt[i].payload_size;
            if (sent >= size) {
                sent -= size;
                packet_commit(t, &pkt[i]);
                continue;
            }
            if (sent > 0) {
                // Keep TCP framing intact: the rest of this packet goes
                // through tx, so the ring may overwrite its payload freely
                if (sent < pkt[i].header_size) {
                    tx_append(c, pkt[i].header + sent, pkt[i].header_size - sent);
                    tx_append(c, pkt[i].payload, pkt[i].payload_size);
                } else {
                    size_t off = sent - pkt[i].header_size;
                    tx_append(c, pkt[i].payload + off, pkt[i].payload_size - off);
                }
                packet_commit(t, &pkt[i]);
            }
            c->blocked = true;
            break;
        }
        if (c->blocked || (size_t)n < total) break;
    }

    pthread_mutex_unlock(&s->lock);
    return result;
}

// UDP: a packet the socket refuses is dropped and video resyncs
static void pump_udp(track_t *t, rtsp_stream_t *s) {
    packet_t pkt;
    pthread_mutex_lock(&s->lock);
    track_resync(t, s);
    while (packet_build(t, s, &pkt)) {
        struct iovec iov[2] = {
            { pkt.header + 4, pkt.header_size - 4 },  // no interleave prefix
            { (void *)pkt.payload, pkt.payload_size }
        };
        struct msghdr msg;
        memset(&msg, 0, sizeof(msg));
        msg.msg_name = &t->rtp_addr;
        msg.msg_namelen = sizeof(t->rtp_addr);
        msg.msg_iov = iov;
        msg.msg_iovlen = 2;
        ssize_t n = sendmsg(g_rtp_fd, &msg, MSG_NOSIGNAL);
        packet_commit(t, &pkt);
        if (n < 0) {
            if (s->codec != RTSP_CODEC_PCMA) t->wait_key = true;
            break;
        }
    }
    pthread_mutex_unlock(&s->lock);
}

static void pump(client_t *c) {
    if (!c->playing || c->closing || c->tx_len > c->tx_off) return;
    session_t *sess = &g_sessions[c->session];
    // Audio first: small and latency sensitive
    for (int i = RTSP_TRACKS - 1; i >= 0; i--) {
        track_t *t = &c->track[i];
        if (!t->setup) continue;
        if (c->tcp) {
            if (c->blocked) return;
            if (!pump_tcp(c, t, &sess->stream[i])) {
                c->closing = true;
                c->tx_off = c->tx_len = 0;
                return;
            }
        } else {
            pump_udp(t, &sess->stream[i]);
        }
    }
}

/* ============================================================================
   RTCP sender reports
   ============================================================================ */

static void send_sr(client_t *c) {
    session_t *sess = &g_sessions[c->session];
    for (int i = 0; i < RTSP_TRACKS; i++) {
        track_t *t = &c->track[i];
        rtsp_stream_t *s = &sess->stream[i];
        if (!t->setup || !t->sent) continue;

        pthread_mutex_lock(&s->lock);
        uint64_t wall = rtsp_stream_wallclock(s, t->last_pts);
        uint32_t ts = rtp_ts(t, s, t->last_pts);
        pthread_mutex_unlock(&s->lock);
        if (wall == 0) continue;

        uint32_t ntp_sec = (uint32_t)(wall / 1000000 + 2208988800ULL);
        uint32_t ntp_frac = (uint32_t)(((wall % 1000000) << 32) / 1000000);
        uint32_t words[7] = {
            htonl(0x80c80006),  // V=2, PT=200 (SR), length 6
            htonl(t->ssrc), htonl(ntp_sec), htonl(ntp_frac),
            htonl(ts), htonl(t->packets), htonl(t->octets)
        };

        if (c->tcp) {
            uint8_t prefix[4] = { '$', (uint8_t)(t->channel + 1), 0, sizeof(words) };
            if (tx_append(c, prefix, sizeof(prefix))) tx_append(c, words, sizeof(words));
        } else {
            sendto(g_rtcp_fd, words, sizeof(words), MSG_NOSIGNAL,
                   (struct sockaddr *)&t->rtcp_addr, sizeof(t->rtcp_addr));
        }
    }
}

/* ============================================================================
   RTSP protocol
   ============================================================================ */

typedef struct {
    char method[16];
    char url[256];
    int  cseq;
    const char *transport;
    const char *session;
    const char *authorization;
    size_t content_length;
} request_t;

static void header_value(char *line, const char *name, const char **value) {
    size_t len = strlen(name);
    if (strncasecmp(line, name, len) == 0 && line[len] == ':') {
        char *v = line + len + 1;
        while (*v == ' ' || *v == '\t') v++;
        *value = v;
    }
}

// Parses the request in place (header lines are NUL terminated)
static bool request_parse(char *text, request_t *req) {
    memset(req, 0, sizeof(*req));
    char *save = NULL;
    char *line = strtok_r(text, "\r\n", &save);
    if (!line || sscanf(line, "%15s %255s", req->method, req->url) != 2) return false;

    while ((line = strtok_r(NULL, "\r\n", &save))) {
        const char *v = NULL;
        header_value(line, "CSeq", &v);
        if (v) { req->cseq = atoi(v); continue; }
        header_value(line, "Content-Length", &v);
        if (v) { req->content_length = (size_t)strtoul(v, NULL, 10); continue; }
        header_value(line, "Transport", &req->transport);
        header_value(line, "Session", &req->session);
        header_value(line, "Authorization", &req->authorization);
    }
    return true;
}

// Splits the URL path: rtsp://host[:port]/<name>[/<track>]
static void url_path(const char *url, char *name, size_t name_size, char *track, size_t track_size) {
    const char *p = strstr(url, "://");
    p = p ? strchr(p + 3, '/') : url;
    name[0] = track[0] = '\0';
    if (!p) return;
    while (*p == '/') p++;
    size_t n = strcspn(p, "/?");
    snprintf(name, name_size, "%.*s", (int)n, p);
    p += n;
    while (*p == '/') p++;
    n = strcspn(p, "/?");
    snprintf(track, track_size, "%.*s", (int)n, p);
}

static int session_find(const char *name) {
    for (int i = 0; i < RTSP_SERVER_MAX_SESSIONS; i++) {
        if (g_sessions[i].used && strcmp(g_sessions[i].name, name) == 0) return i;
    }
    return -1;
}

static void reply(client_t *c, int cseq, const char *status, const char *headers, const char *body) {
    char buf[RTSP_TX_SIZE];
    size_t body_len = body ? strlen(body) : 0;
    int n = snprintf(buf, sizeof(buf),
                     "RTSP/1.0 %s\r\nCSeq: %d\r\nServer: mjsxj02hl\r\n%s", status, cseq, headers ? headers : "");
    if (c->session_id && n > 0 && (size_t)n < sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - n, "Session: %08X;timeout=%d\r\n", c->session_id, RTSP_TIMEOUT_S);
    }
    if (body_len && n > 0 && (size_t)n < sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - n, "Content-Length: %u\r\n", (unsigned)body_len);
    }
    if (n > 0 && (size_t)n < sizeof(buf)) n += snprintf(buf + n, sizeof(buf) - n, "\r\n%s", body ? body : "");
    if (n <= 0 || (size_t)n >= sizeof(buf) || !tx_append(c, buf, (size_t)n)) {
        LOGGER(LOGGER_LEVEL_WARNING, "RTSP reply does not fit (%s).", status);
        c->closing = true;
    }
}

// Value of a Digest parameter: key="value" or key=value
static bool digest_param(const char *auth, const char *key, char *out, size_t out_size) {
    size_t len = strlen(key);
    for (const char *p = auth; (p = strstr(p, key)); p += len) {
        if (p != auth && p[-1] != ' ' && p[-1] != ',') continue;
        const char *v = p + len;
        while (*v == ' ') v++;
        if (*v != '=') continue;
        v++;
        while (*v == ' ') v++;
        size_t n;
        if (*v == '"') { v++; n = strcspn(v, "\""); }
        else n = strcspn(v, ", \r\n");
        if (n >= out_size) return false;
        memcpy(out, v, n);
        out[n] = '\0';
        return true;
    }
    return false;
}

static bool authorized(client_t *c, const request_t *req) {
    if (!g_auth) return true;
    if (req->authorization && strncasecmp(req->authorization, "Digest ", 7) == 0 && c->nonce[0]) {
        char user[64], realm[64], nonce[64], uri[256], response[64];
        const char *a = req->authorization + 7;
        if (digest_param(a, "username", user, sizeof(user)) &&
            digest_param(a, "realm", realm, sizeof(realm)) &&
            digest_param(a, "nonce", nonce, sizeof(nonce)) &&
            digest_param(a, "uri", uri, sizeof(uri)) &&
            digest_param(a, "response", response, sizeof(response)) &&
            strcmp(user, g_username) == 0 && strcmp(nonce, c->nonce) == 0) {
            char buf[512], ha1[33], ha2[33], expected[33];
            snprintf(buf, sizeof(buf), "%s:%s:%s", g_username, realm, g_password);
            rtsp_md5_hex(buf, strlen(buf), ha1);
            snprintf(buf, sizeof(buf), "%s:%s", req->method, uri);
            rtsp_md5_hex(buf, strlen(buf), ha2);
            snprintf(buf, sizeof(buf), "%s:%s:%s", ha1, nonce, ha2);
            rtsp_md5_hex(buf, strlen(buf), expected);
            if (strcasecmp(expected, response) == 0) return true;
        }
    }

    if (!c->nonce[0]) {
        snprintf(c->nonce, sizeof(c->nonce), "%08x%08x%08x%08x",
                 rtsp_random(), rtsp_random(), rtsp_random(), rtsp_random());
    }
    char headers[160];
    snprintf(headers, sizeof(headers), "WWW-Authenticate: Digest realm=\"%s\", nonce=\"%s\"\r\n", RTSP_REALM, c->nonce);
    reply(c, req->cseq, "401 Unauthorized", headers, NULL);
    return false;
}

static void local_ip(client_t *c, char *out, size_t out_size) {
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    if (getsockname(c->fd, (struct sockaddr *)&addr, &len) == 0) snprintf(out, out_size, "%s", inet_ntoa(addr.sin_addr));
    else snprintf(out, out_size, "0.0.0.0");
}

static size_t sdp_param(char *out, size_t out_size, const char *key, const rtsp_param_t *param) {
    char b64[RTSP_PARAM_MAX * 2];
    if (!param->size || !rtsp_base64(param->data, param->size, b64, sizeof(b64))) return 0;
    int n = snprintf(out, out_size, "%s=%s", key, b64);
    return (n > 0 && (size_t)n < out_size) ? (size_t)n : 0;
}

static void sdp_build(client_t *c, session_t *sess, char *sdp, size_t size) {
    char ip[32];
    local_ip(c, ip, sizeof(ip));
    int n = snprintf(sdp, size,
                     "v=0\r\no=- %u 1 IN IP4 %s\r\ns=%s\r\nc=IN IP4 0.0.0.0\r\nt=0 0\r\n"
                     "a=control:*\r\na=range:npt=0-\r\n",
                     rtsp_random(), ip, sess->name);

    rtsp_stream_t *v = &sess->stream[RTSP_TRACK_VIDEO];
    if (v->codec != RTSP_CODEC_NONE) {
        char fmtp[640] = "";
        size_t f = 0;
        pthread_mutex_lock(&v->lock);
        if (v->codec == RTSP_CODEC_H264) {
            f += snprintf(fmtp + f, sizeof(fmtp) - f, "packetization-mode=1");
            if (v->sps.size >= 4 && v->pps.size) {
                f += snprintf(fmtp + f, sizeof(fmtp) - f, ";profile-level-id=%02X%02X%02X;",
                              v->sps.data[1], v->sps.data[2], v->sps.data[3]);
                f += sdp_param(fmtp + f, sizeof(fmtp) - f, "sprop-parameter-sets", &v->sps);
                f += snprintf(fmtp + f, sizeof(fmtp) - f, ",");
                char b64[RTSP_PARAM_MAX * 2];
                if (rtsp_base64(v->pps.data, v->pps.size, b64, sizeof(b64))) {
                    f += snprintf(fmtp + f, sizeof(fmtp) - f, "%s", b64);
                }
            }
        } else if (v->vps.size && v->sps.size && v->pps.size) {
            f += sdp_param(fmtp + f, sizeof(fmtp) - f, "sprop-vps", &v->vps);
            f += snprintf(fmtp + f, sizeof(fmtp) - f, ";");
            f += sdp_param(fmtp + f, sizeof(fmtp) - f, "sprop-sps", &v->sps);
            f += snprintf(fmtp + f, sizeof(fmtp) - f, ";");
            f += sdp_param(fmtp + f, sizeof(fmtp) - f, "sprop-pps", &v->pps);
        }
        pthread_mutex_unlock(&v->lock);
        n += snprintf(sdp + n, size - n,
                      "m=video 0 RTP/AVP 96\r\na=rtpmap:96 %s/90000\r\n%s%s%s"
                      "a=framerate:%u\r\na=control:track%d\r\n",
                      v->codec == RTSP_CODEC_H264 ? "H264" : "H265",
                      fmtp[0] ? "a=fmtp:96 " : "", fmtp, fmtp[0] ? "\r\n" : "",
                      sess->framerate, RTSP_TRACK_VIDEO);
    }
    if (sess->stream[RTSP_TRACK_AUDIO].codec == RTSP_CODEC_PCMA) {
        snprintf(sdp + n, size - n, "m=audio 0 RTP/AVP 8\r\na=rtpmap:8 PCMA/8000/1\r\na=control:track%d\r\n",
                 RTSP_TRACK_AUDIO);
    }
}

static void handle_describe(client_t *c, const request_t *req) {
    char name[64], track[32];
    url_path(req->url, name, sizeof(name), track, sizeof(track));
    int s = session_find(name);
    if (s < 0) { reply(c, req->cseq, "404 Not Found", NULL, NULL); return; }
    if (c->session >= 0 && c->session != s) { reply(c, req->cseq, "455 Method Not Valid in This State", NULL, NULL); return; }
    c->session = s;

    char sdp[1536], headers[320];
    sdp_build(c, &g_sessions[s], sdp, sizeof(sdp));
    size_t url_len = strlen(req->url);
    snprintf(headers, sizeof(headers), "Content-Base: %s%s\r\nContent-Type: application/sdp\r\n",
             req->url, (url_len && req->url[url_len - 1] == '/') ? "" : "/");
    reply(c, req->cseq, "200 OK", headers, sdp);
}

static void handle_setup(client_t *c, const request_t *req) {
    char name[64], track_name[32];
    url_path(req->url, name, sizeof(name), track_name, sizeof(track_name));
    int s = session_find(name);
    if (s < 0) { reply(c, req->cseq, "404 Not Found", NULL, NULL); return; }
    if (c->session >= 0 && c->session != s) { reply(c, req->cseq, "459 Aggregate Operation Not Allowed", NULL, NULL); return; }

    int index = (strcmp(track_name, "track1") == 0) ? RTSP_TRACK_AUDIO : RTSP_TRACK_VIDEO;
    rtsp_stream_t *stream = &g_sessions[s].stream[index];
    if (stream->codec == RTSP_CODEC_NONE || (track_name[0] && strncmp(track_name, "track", 5) != 0)) {
        reply(c, req->cseq, "404 Not Found", NULL, NULL);
        return;
    }

    const char *tr = req->transport ? req->transport : "";
    bool tcp = strstr(tr, "RTP/AVP/TCP") != NULL;
    bool any_setup = c->track[0].setup || c->track[1].setup;
    if (strstr(tr, "multicast") || (any_setup && tcp != c->tcp)) {
        reply(c, req->cseq, "461 Unsupported Transport", NULL, NULL);
        return;
    }

    track_t *t = &c->track[index];
    memset(t, 0, sizeof(*t));
    t->ssrc = rtsp_random();
    t->seq = (uint16_t)rtsp_random();
    t->ts_base = rtsp_random();

    char headers[256];
    if (tcp) {
        unsigned a = index * 2, b = index * 2 + 1;
        const char *p = strstr(tr, "interleaved=");
        if (p) sscanf(p + 12, "%u-%u", &a, &b);
        t->channel = (uint8_t)a;
        snprintf(headers, sizeof(headers), "Transport: RTP/AVP/TCP;unicast;interleaved=%u-%u;ssrc=%08X\r\n",
                 a, a + 1, t->ssrc);
    } else {
        unsigned a = 0, b = 0;
        const char *p = strstr(tr, "client_port=");
        if (!p || sscanf(p + 12, "%u-%u", &a, &b) < 1 || a == 0 || g_rtp_fd < 0) {
            reply(c, req->cseq, "461 Unsupported Transport", NULL, NULL);
            return;
        }
        if (b == 0) b = a + 1;
        t->rtp_addr = c->peer;
        t->rtp_addr.sin_port = htons((uint16_t)a);
        t->rtcp_addr = c->peer;
        t->rtcp_addr.sin_port = htons((uint16_t)b);
        snprintf(headers, sizeof(headers),
                 "Transport: RTP/AVP;unicast;client_port=%u-%u;server_port=%u-%u;ssrc=%08X\r\n",
                 a, b, g_rtp_port, g_rtp_port + 1, t->ssrc);
    }

    t->setup = true;
    c->tcp = tcp;
    c->session = s;
    if (!c->session_id) c->session_id = rtsp_random() | 1;
    reply(c, req->cseq, "200 OK", headers, NULL);
}

static void handle_play(client_t *c, const request_t *req) {
    if (c->session < 0 || (!c->track[0].setup && !c->track[1].setup)) {
        reply(c, req->cseq, "455 Method Not Valid in This State", NULL, NULL);
        return;
    }
    session_t *sess = &g_sessions[c->session];
    bool need_key = false;

    if (!c->playing) {
        // Video starts on the latest buffered key frame; audio at the same pts
        uint64_t start_pts = 0;
        bool have_pts = false;
        track_t *tv = &c->track[RTSP_TRACK_VIDEO];
        rtsp_stream_t *sv = &sess->stream[RTSP_TRACK_VIDEO];
        if (tv->setup) {
            pthread_mutex_lock(&sv->lock);
            tv->frag = 0;
            if (sv->has_key) {
                tv->cur = sv->key_seq;
                tv->wait_key = false;
                start_pts = rtsp_stream_nal(sv, tv->cur)->pts;
            } else {
                tv->cur = sv->next_seq;
                tv->wait_key = true;
                start_pts = sv->last_pts;
                need_key = true;
            }
            have_pts = true;
            pthread_mutex_unlock(&sv->lock);
        }
        track_t *ta = &c->track[RTSP_TRACK_AUDIO];
        rtsp_stream_t *sa = &sess->stream[RTSP_TRACK_AUDIO];
        if (ta->setup) {
            pthread_mutex_lock(&sa->lock);
            ta->frag = 0;
            ta->wait_key = false;
            ta->cur = have_pts ? rtsp_stream_find_pts(sa, start_pts) : sa->next_seq;
            if (!have_pts) start_pts = sa->last_pts;
            pthread_mutex_unlock(&sa->lock);
        }

        char headers[640];
        size_t n = snprintf(headers, sizeof(headers), "Range: npt=0.000-\r\nRTP-Info: ");
        size_t url_len = strlen(req->url);
        const char *sep = (url_len && req->url[url_len - 1] == '/') ? "" : "/";
        bool first = true;
        for (int i = 0; i < RTSP_TRACKS; i++) {
            track_t *t = &c->track[i];
            if (!t->setup) continue;
            n += snprintf(headers + n, sizeof(headers) - n, "%surl=%s%strack%d;seq=%u;rtptime=%u",
                          first ? "" : ",", req->url, sep, i, t->seq, rtp_ts(t, &sess->stream[i], start_pts));
            first = false;
        }
        snprintf(headers + n, sizeof(headers) - n, "\r\n");
        reply(c, req->cseq, "200 OK", headers, NULL);
        client_play(c, true);
        c->sr_ms = rtsp_now_ms();
        LOGGER(LOGGER_LEVEL_INFO, "RTSP client %s:%u playing \"%s\" over %s.",
               inet_ntoa(c->peer.sin_addr), ntohs(c->peer.sin_port), sess->name, c->tcp ? "TCP" : "UDP");
    } else {
        reply(c, req->cseq, "200 OK", NULL, NULL);
    }

    if (need_key && g_config.key_frame_needed) g_config.key_frame_needed(c->session);
}

static void handle_request(client_t *c, char *text) {
    request_t req;
    if (!request_parse(text, &req)) {
        reply(c, 0, "400 Bad Request", NULL, NULL);
        c->closing = true;
        return;
    }
    c->activity_ms = rtsp_now_ms();

    if (req.session && c->session_id && strtoul(req.session, NULL, 16) != c->session_id) {
        reply(c, req.cseq, "454 Session Not Found", NULL, NULL);
        return;
    }

    if (strcmp(req.method, "OPTIONS") == 0) {
        reply(c, req.cseq, "200 OK",
              "Public: OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER, SET_PARAMETER\r\n", NULL);
    } else if (strcmp(req.method, "GET_PARAMETER") == 0 || strcmp(req.method, "SET_PARAMETER") == 0) {
        reply(c, req.cseq, "200 OK", NULL, NULL);  // keep-alive
    } else if (!authorized(c, &req)) {
        return;
    } else if (strcmp(req.method, "DESCRIBE") == 0) {
        handle_describe(c, &req);
    } else if (strcmp(req.method, "SETUP") == 0) {
        handle_setup(c, &req);
    } else if (strcmp(req.method, "PLAY") == 0) {
        handle_play(c, &req);
    } else if (strcmp(req.method, "PAUSE") == 0) {
        client_play(c, false);
        reply(c, req.cseq, "200 OK", NULL, NULL);
    } else if (strcmp(req.method, "TEARDOWN") == 0) {
        client_play(c, false);
        reply(c, req.cseq, "200 OK", NULL, NULL);
        c->closing = true;
    } else {
        reply(c, req.cseq, "501 Not Implemented", NULL, NULL);
    }
}

// Consumes rx: RTSP requests and interleaved packets (RTCP receiver reports)
static void handle_input(client_t *c) {
    size_t pos = 0;
    while (pos < c->rx_len && !c->closing) {
        if (c->rx_skip) {
            size_t n = c->rx_len - pos < c->rx_skip ? c->rx_len - pos : c->rx_skip;
            pos += n;
            c->rx_skip -= n;
            continue;
        }
        if (c->rx[pos] == '$') {
            if (c->rx_len - pos < 4) break;
            c->rx_skip = 4 + (((size_t)c->rx[pos + 2] << 8) | c->rx[pos + 3]);
            c->activity_ms = rtsp_now_ms();
            continue;
        }
        char *start = (char *)c->rx + pos;
        char *end = memmem(start, c->rx_len - pos, "\r\n\r\n", 4);
        if (!end) break;
        size_t head = (size_t)(end - start) + 4;
        start[head - 2] = '\0';  // terminate the header block
        request_t peek;
        char copy[RTSP_RX_SIZE];
        memcpy(copy, start, head - 1);
        request_parse(copy, &peek);
        if (c->rx_len - pos < head + peek.content_length) {
            if (head + peek.content_length > RTSP_RX_SIZE) { c->closing = true; break; }
            start[head - 2] = '\r';  // incomplete body: wait for more
            break;
        }
        handle_request(c, start);
        pos += head + peek.content_length;
    }

    if (pos > 0) {
        memmove(c->rx, c->rx + pos, c->rx_len - pos);
        c->rx_len -= pos;
    }
    if (c->rx_len == RTSP_RX_SIZE) {
        LOGGER(LOGGER_LEVEL_WARNING, "RTSP request too large, closing client.");
        c->closing = true;
    }
}

/* ============================================================================
   Sockets and main loop
   ============================================================================ */

static void accept_client(void) {
    struct sockaddr_in peer;
    socklen_t len = sizeof(peer);
    int fd = accept(g_listen_fd, (struct sockaddr *)&peer, &len);
    if (fd < 0) return;

    client_t *c = NULL;
    for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) {
        if (!g_clients[i].used) { c = &g_clients[i]; break; }
    }
    if (!c) {
        static const char busy[] = "RTSP/1.0 503 Service Unavailable\r\n\r\n";
        if (send(fd, busy, sizeof(busy) - 1, MSG_NOSIGNAL) < 0) { /* closing anyway */ }
        close(fd);
        LOGGER(LOGGER_LEVEL_WARNING, "RTSP client %s refused: %d clients max.", inet_ntoa(peer.sin_addr), RTSP_SERVER_MAX_CLIENTS);
        return;
    }

    set_nonblock(fd);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    int sndbuf = 64 * 1024;  // latency stays low, the ring does the buffering
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    memset(c, 0, sizeof(*c));
    __sync_fetch_and_add(&g_connected, 1);
    c->used = true;
    c->fd = fd;
    c->peer = peer;
    c->session = -1;
    c->activity_ms = c->progress_ms = rtsp_now_ms();
    LOGGER(LOGGER_LEVEL_INFO, "RTSP client %s:%u connected.", inet_ntoa(peer.sin_addr), ntohs(peer.sin_port));
}

static void read_client(client_t *c) {
    ssize_t n = recv(c->fd, c->rx + c->rx_len, RTSP_RX_SIZE - c->rx_len, 0);
    if (n > 0) {
        c->rx_len += (size_t)n;
        handle_input(c);
    } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        client_close(c);
    }
}

static void read_rtcp(void) {
    uint8_t buf[512];
    struct sockaddr_in from;
    socklen_t len = sizeof(from);
    while (recvfrom(g_rtcp_fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &len) > 0) {
        for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) {
            client_t *c = &g_clients[i];
            if (c->used && !c->tcp && c->peer.sin_addr.s_addr == from.sin_addr.s_addr) c->activity_ms = rtsp_now_ms();
        }
        len = sizeof(from);
    }
}

static void timers(void) {
    uint64_t now = rtsp_now_ms();
    for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) {
        client_t *c = &g_clients[i];
        if (!c->used) continue;
        // UDP players must send keep-alives (RTSP or RTCP); any idle
        // non-playing connection would otherwise hold a client slot
        if ((!c->tcp || !c->playing) && now - c->activity_ms > RTSP_TIMEOUT_S * 1000ULL) {
            LOGGER(LOGGER_LEVEL_INFO, "RTSP client %s timed out.", inet_ntoa(c->peer.sin_addr));
            client_close(c);
            continue;
        }
        if (c->tcp && c->blocked && now - c->progress_ms > RTSP_STALL_MS) {
            LOGGER(LOGGER_LEVEL_INFO, "RTSP client %s stalled, closing.", inet_ntoa(c->peer.sin_addr));
            client_close(c);
            continue;
        }
        if (c->playing && now - c->sr_ms >= RTSP_SR_INTERVAL_MS && c->tx_len == c->tx_off && !c->blocked) {
            c->sr_ms = now;
            send_sr(c);
        }
    }
}

static void *server_thread(void *arg) {
    (void)arg;
    struct pollfd fds[3 + RTSP_SERVER_MAX_CLIENTS];
    int map[3 + RTSP_SERVER_MAX_CLIENTS];

    while (g_running) {
        int n = 0;
        fds[n].fd = g_listen_fd; fds[n].events = POLLIN; map[n++] = -1;
        fds[n].fd = g_wake_fd[0]; fds[n].events = POLLIN; map[n++] = -2;
        if (g_rtcp_fd >= 0) { fds[n].fd = g_rtcp_fd; fds[n].events = POLLIN; map[n++] = -3; }
        for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) {
            client_t *c = &g_clients[i];
            if (!c->used) continue;
            fds[n].fd = c->fd;
            fds[n].events = POLLIN | (c->blocked ? POLLOUT : 0);
            map[n++] = i;
        }

        if (poll(fds, n, 500) < 0 && errno != EINTR) break;

        for (int k = 0; k < n; k++) {
            if (!fds[k].revents) continue;
            if (map[k] == -1) accept_client();
            else if (map[k] == -2) {
                char drain[64];
                while (read(g_wake_fd[0], drain, sizeof(drain)) > 0) {}
                __sync_lock_release(&g_wake_pending);
            } else if (map[k] == -3) read_rtcp();
            else {
                client_t *c = &g_clients[map[k]];
                if (!c->used) continue;
                if (fds[k].revents & POLLOUT) c->blocked = false;
                if (fds[k].revents & (POLLIN | POLLHUP | POLLERR)) read_client(c);
            }
        }

        for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) {
            client_t *c = &g_clients[i];
            if (!c->used) continue;
            if (!c->blocked && !tx_flush(c)) { client_close(c); continue; }
            if (c->tx_len == c->tx_off && c->closing) { client_close(c); continue; }
            pump(c);
            if (c->used && !c->blocked && c->tx_len > c->tx_off && !tx_flush(c)) client_close(c);
        }
        timers();
    }
    return NULL;
}

static int udp_socket(uint16_t port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(fd); return -1; }
    set_nonblock(fd);
    return fd;
}

// One even/odd UDP port pair for every client's RTP/RTCP
static void open_udp(void) {
    for (uint16_t port = 6970; port < 7070; port += 2) {
        int rtp = udp_socket(port);
        if (rtp < 0) continue;
        int rtcp = udp_socket(port + 1);
        if (rtcp < 0) { close(rtp); continue; }
        g_rtp_fd = rtp;
        g_rtcp_fd = rtcp;
        g_rtp_port = port;
        return;
    }
    LOGGER(LOGGER_LEVEL_WARNING, "No UDP port pair for RTP, TCP transport only.");
}

static void close_fd(int *fd) {
    if (*fd >= 0) close(*fd);
    *fd = -1;
}

/* ============================================================================
   Public API
   ============================================================================ */

bool rtsp_server_start(const rtsp_server_config_t *config, const rtsp_session_config_t *sessions, int count) {
    if (g_running || !config || !sessions || count <= 0 || count > RTSP_SERVER_MAX_SESSIONS) return false;

    g_config = *config;
    snprintf(g_username, sizeof(g_username), "%s", config->username ? config->username : "");
    snprintf(g_password, sizeof(g_password), "%s", config->password ? config->password : "");
    g_auth = g_username[0] || g_password[0];

    memset(g_sessions, 0, sizeof(g_sessions));
    for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) { memset(&g_clients[i], 0, sizeof(g_clients[i])); g_clients[i].fd = -1; }

    for (int i = 0; i < count; i++) {
        session_t *s = &g_sessions[i];
        s->used = true;
        snprintf(s->name, sizeof(s->name), "%s", sessions[i].name);
        s->framerate = sessions[i].framerate;
        if (sessions[i].video != RTSP_CODEC_NONE &&
            !rtsp_stream_init(&s->stream[RTSP_TRACK_VIDEO], sessions[i].video, sessions[i].video_buffer)) return false;
        if (sessions[i].audio != RTSP_CODEC_NONE &&
            !rtsp_stream_init(&s->stream[RTSP_TRACK_AUDIO], sessions[i].audio, sessions[i].audio_buffer)) return false;
        LOGGER(LOGGER_LEVEL_INFO, "RTSP session \"%s\": video %u KB, audio %u KB.", s->name,
               sessions[i].video != RTSP_CODEC_NONE ? sessions[i].video_buffer / 1024 : 0,
               sessions[i].audio != RTSP_CODEC_NONE ? sessions[i].audio_buffer / 1024 : 0);
    }

    if (pipe(g_wake_fd) < 0) return false;
    set_nonblock(g_wake_fd[0]);
    set_nonblock(g_wake_fd[1]);

    g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen_fd < 0) return false;
    int one = 1;
    setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(config->port);
    if (bind(g_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(g_listen_fd, 4) < 0) {
        LOGGER(LOGGER_LEVEL_ERROR, "RTSP bind/listen on port %u failed: %s", config->port, strerror(errno));
        close_fd(&g_listen_fd);
        return false;
    }
    set_nonblock(g_listen_fd);
    open_udp();

    g_running = true;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, RTSP_THREAD_STACK);
    int rc = pthread_create(&g_thread, &attr, server_thread, NULL);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        g_running = false;
        close_fd(&g_listen_fd);
        return false;
    }
    LOGGER(LOGGER_LEVEL_INFO, "RTSP server listening on port %u (UDP RTP %u, auth %s).",
           config->port, g_rtp_port, g_auth ? "digest" : "none");
    return true;
}

void rtsp_server_stop(void) {
    if (!g_running) return;
    g_running = false;
    pthread_join(g_thread, NULL);
    for (int i = 0; i < RTSP_SERVER_MAX_CLIENTS; i++) client_close(&g_clients[i]);
    close_fd(&g_listen_fd);
    close_fd(&g_rtp_fd);
    close_fd(&g_rtcp_fd);
    close_fd(&g_wake_fd[0]);
    close_fd(&g_wake_fd[1]);
    // Stream rings are kept: encoder threads may still be pushing into them
}

bool rtsp_server_push_video(int session, const uint8_t *data, size_t size, uint64_t pts, bool frame_end) {
    if (!g_running || session < 0 || session >= RTSP_SERVER_MAX_SESSIONS) return false;
    rtsp_stream_t *s = &g_sessions[session].stream[RTSP_TRACK_VIDEO];
    if (s->codec == RTSP_CODEC_NONE) return false;
    bool result = rtsp_stream_push(s, data, size, pts, frame_end, g_connected > 0);
    if (frame_end) wake();
    return result;
}

bool rtsp_server_push_audio(int session, const uint8_t *data, size_t size, uint64_t pts) {
    if (!g_running || session < 0 || session >= RTSP_SERVER_MAX_SESSIONS) return false;
    rtsp_stream_t *s = &g_sessions[session].stream[RTSP_TRACK_AUDIO];
    if (s->codec == RTSP_CODEC_NONE) return false;
    bool result = rtsp_stream_push(s, data, size, pts, true, g_connected > 0);
    wake();
    return result;
}
