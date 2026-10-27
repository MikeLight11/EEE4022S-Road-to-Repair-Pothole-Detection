/*
 * stream.c - tiny HTTP server for the livestream.
 *
 * One thread accepts connections and each client gets its own detached
 * thread (only a handful of phones/laptops will ever connect). Clients
 * always receive the newest frame, so a slow client skips frames rather
 * than falling behind or slowing the camera loop.
 */
#include "stream.h"

#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define STATUS_MAX 1536

static struct {
    pthread_mutex_t m;
    pthread_cond_t c;
    uint8_t *jpeg;
    size_t len, cap;
    uint64_t seq;           /* bumped on every publish */
    int wanted;             /* clients currently waiting for frames */
    char status[STATUS_MAX];
    int listen_fd;
    volatile int running;
    pthread_t accept_thread;
    stream_api_fn api;      /* /api/... handler */
    void *api_ctx;
    char file_dir[256];     /* served at /files/<name> */
} S = {
    .m = PTHREAD_MUTEX_INITIALIZER,
    .c = PTHREAD_COND_INITIALIZER,
    .status = "{}",
    .listen_fd = -1,
};

static const char PAGE[] =
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Pothole Camera</title><style>"
    "body{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif}"
    "#v{display:block;width:100%;max-width:1280px;margin:0 auto;background:#000}"
    "#s{padding:8px 16px;text-align:center;font-variant-numeric:tabular-nums}"
    "a{color:#8cf}"
    "#cal{max-width:720px;margin:12px auto;padding:12px 16px;border:1px solid #333;border-radius:8px}"
    "#cal h2{font-size:15px;margin:0 0 6px}#cal p{margin:6px 0;color:#bbb}"
    "button{font:inherit;padding:8px 14px;margin:4px 6px 4px 0;border-radius:6px;border:1px solid #555;"
    "background:#2a2a2a;color:#eee}button.go{background:#1f6f3a;border-color:#2f9a55}"
    "button:disabled{opacity:.5}"
    "table{border-collapse:collapse;margin:6px 0;font-variant-numeric:tabular-nums}"
    "td{padding:2px 12px 2px 0}td:first-child{color:#999}"
    "#ov{width:100%;margin-top:6px;border-radius:4px}.bad{color:#f87}"
    "</style></head><body>"
    "<img id=v src=/stream alt='camera stream'>"
    "<div id=s>connecting...</div>"
    "<div style='text-align:center'><a href=/snapshot.jpg download=snapshot.jpg>save snapshot</a></div>"
    "<div id=cal hidden><h2>Mount calibration</h2><div id=cb></div></div>"
    "<script>"
    "const $=id=>document.getElementById(id);"
    "async function t(){try{const r=await fetch('/status');const j=await r.json();"
    "$('s').textContent="
    "`camera ${j.camera_fps.toFixed(1)} fps · ${j.width}x${j.height} · "
    "stream ${j.stream_width}x${j.stream_height} · skipped ${j.skipped} · "
    "${j.viewers} viewer(s) · up ${j.uptime_s} s`"
    "+(j.detecting?` · detection ${j.det_fps.toFixed(1)} fps, inference ${j.infer_ms.toFixed(1)} ms, "
    "${j.frame_ms.toFixed(1)} ms/frame · ${j.tracks} tracking, ${j.potholes} saved`:' · detection off')}"
    "catch(e){$('s').textContent='no status'}}"
    "t();setInterval(t,1000);"
    /* Calibration panel: polls /api/calibrate/status and renders each state. */
    "let last='';"
    "async function api(a){const r=await fetch('/api/calibrate/'+a,{method:'POST'});return r.json()}"
    "function row(k,v){return `<tr><td>${k}</td><td>${v}</td></tr>`}"
    "async function c(){let j;try{j=await api('status')}catch(e){return}"
    "if(j.error){$('cal').hidden=true;return}$('cal').hidden=false;"
    "const key=j.state+j.captured+j.elapsed_s+j.message;if(key===last)return;last=key;let h='';"
    "if(j.state==='idle'||j.state==='failed'){"
    "h=`<p>Lay the A3 checkerboard (${j.board}) flat on the road where the camera can see all of it, "
    "and keep the car still. Current: height ${(j.current_h_mm/10).toFixed(1)} cm, "
    "tilt ${j.current_tilt_deg.toFixed(2)}°.</p>`"
    "+(j.state==='failed'?`<p class=bad>Failed: ${j.message||'unknown error'}</p>`:'')"
    "+`<button class=go onclick=\"api('start')\">${j.state==='failed'?'Try again':'Calibrate'}</button>`}"
    "else if(j.state==='capturing'){h=`<p>Capturing ${j.captured} / ${j.total} frames… keep still.</p>"
    "<button onclick=\"api('cancel')\">Cancel</button>`}"
    "else if(j.state==='processing'){h=`<p>Estimating height and tilt… ${j.elapsed_s} s "
    "(about 1–2 minutes on the Pi).</p>`}"
    "else if(j.state==='done'){h=`<table>`"
    "+row('Height',`<b>${(j.height_m*100).toFixed(1)} cm</b> (was ${(j.current_h_mm/10).toFixed(1)})`)"
    "+row('Tilt',`<b>${j.tilt_deg.toFixed(2)}°</b> (was ${j.current_tilt_deg.toFixed(2)})`)"
    "+row('Roll',`${j.roll_deg.toFixed(2)}° (level the mount if this is large)`)"
    "+row('Lens to board centre',`${j.board_distance_m.toFixed(2)} m along the ground`)"
    "+row('Frames used',`${j.frames_used} of ${j.total} (reprojection ${j.reproj_px.toFixed(2)} px)`)"
    "+`</table><p>Check with a tape measure: the lens should be ${(j.height_m*100).toFixed(1)} cm above the road "
    "and the board centre ${j.board_distance_m.toFixed(2)} m from the point below the lens.</p>"
    "<button class=go onclick=\"api('apply')\">Apply and restart</button>"
    "<button onclick=\"api('cancel')\">Discard</button>"
    "<a href=/files/overlay.jpg target=_blank><img id=ov src='/files/overlay.jpg?${Date.now()}'></a>`}"
    "else if(j.state==='applied'){h='<p>Saved to pothole.conf. Restarting with the new values…</p>';"
    "setTimeout(()=>location.reload(),8000)}"
    "$('cb').innerHTML=h}"
    "c();setInterval(c,1000)"
    "</script></body></html>";

static int send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static int send_response(int fd, const char *status, const char *type, const void *body, size_t len)
{
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
                     "HTTP/1.0 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
                     "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                     status, type, len);
    if (send_all(fd, hdr, (size_t)n))
        return -1;
    return send_all(fd, body, len);
}

/* Waits for a frame newer than *last and copies it into *buf.
 * Returns its length, or 0 if none arrived within timeout_ms or the server is stopping. */
static size_t wait_frame(uint64_t *last, uint8_t **buf, size_t *cap, int timeout_ms)
{
    struct timespec dl;
    clock_gettime(CLOCK_REALTIME, &dl);
    dl.tv_sec += timeout_ms / 1000;
    dl.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (dl.tv_nsec >= 1000000000L) {
        dl.tv_sec++;
        dl.tv_nsec -= 1000000000L;
    }

    size_t len = 0;
    pthread_mutex_lock(&S.m);
    while (S.running && S.seq == *last)
        if (pthread_cond_timedwait(&S.c, &S.m, &dl) == ETIMEDOUT)
            break;
    if (S.running && S.seq != *last && S.len) {
        if (*cap < S.len) {
            uint8_t *nb = realloc(*buf, S.len);
            if (nb) {
                *buf = nb;
                *cap = S.len;
            }
        }
        if (*cap >= S.len) {
            memcpy(*buf, S.jpeg, S.len);
            len = S.len;
            *last = S.seq;
        }
    }
    pthread_mutex_unlock(&S.m);
    return len;
}

static void set_wanted(int delta)
{
    pthread_mutex_lock(&S.m);
    S.wanted += delta;
    pthread_mutex_unlock(&S.m);
}

static void serve_stream(int fd)
{
    static const char hdr[] =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    if (send_all(fd, hdr, sizeof(hdr) - 1))
        return;

    uint8_t *buf = NULL;
    size_t cap = 0;
    uint64_t last = 0;
    set_wanted(1);
    while (S.running) {
        size_t len = wait_frame(&last, &buf, &cap, 1000);
        if (!len)
            continue;
        char part[128];
        int n = snprintf(part, sizeof(part),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", len);
        if (send_all(fd, part, (size_t)n) || send_all(fd, buf, len) || send_all(fd, "\r\n", 2))
            break; /* client went away */
    }
    set_wanted(-1);
    free(buf);
}

static void serve_snapshot(int fd)
{
    uint8_t *buf = NULL;
    size_t cap = 0;
    uint64_t last;

    /* Ask for a fresh frame: the main loop only encodes while someone wants one. */
    pthread_mutex_lock(&S.m);
    last = S.seq;
    pthread_mutex_unlock(&S.m);
    set_wanted(1);
    size_t len = wait_frame(&last, &buf, &cap, 3000);
    set_wanted(-1);

    if (len)
        send_response(fd, "200 OK", "image/jpeg", buf, len);
    else
        send_response(fd, "503 Service Unavailable", "text/plain", "no frame\n", 9);
    free(buf);
}

static void serve_status(int fd)
{
    char body[STATUS_MAX];
    pthread_mutex_lock(&S.m);
    memcpy(body, S.status, sizeof(body));
    pthread_mutex_unlock(&S.m);
    send_response(fd, "200 OK", "application/json", body, strlen(body));
}

static void serve_api(int fd, const char *path)
{
    char reply[2048];
    stream_api_fn fn;
    void *ctx;
    pthread_mutex_lock(&S.m);
    fn = S.api;
    ctx = S.api_ctx;
    pthread_mutex_unlock(&S.m);
    if (!fn) {
        static const char na[] = "{\"error\":\"not available\"}";
        send_response(fd, "200 OK", "application/json", na, sizeof(na) - 1);
        return;
    }
    reply[0] = '\0';
    fn(path, reply, sizeof(reply), ctx);
    send_response(fd, "200 OK", "application/json", reply, strlen(reply));
}

/* A plain file name (no directories) from file_dir. */
static void serve_file(int fd, const char *name)
{
    if (!S.file_dir[0] || !name[0] || strspn(name, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-") !=
                                         strlen(name) || name[0] == '.') {
        send_response(fd, "404 Not Found", "text/plain", "not found\n", 10);
        return;
    }
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", S.file_dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) {
        send_response(fd, "404 Not Found", "text/plain", "not found\n", 10);
        return;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = len > 0 && len < (32L << 20) ? malloc((size_t)len) : NULL;
    if (buf && fread(buf, 1, (size_t)len, f) == (size_t)len) {
        const char *ext = strrchr(name, '.');
        const char *type = ext && !strcmp(ext, ".jpg") ? "image/jpeg" : "text/plain; charset=utf-8";
        send_response(fd, "200 OK", type, buf, (size_t)len);
    } else {
        send_response(fd, "500 Internal Server Error", "text/plain", "read error\n", 11);
    }
    free(buf);
    fclose(f);
}

static void *client_thread(void *arg)
{
    int fd = (int)(intptr_t)arg;
    char req[2048];
    size_t got = 0;

    /* Read until the end of the request headers; only the request line is used. */
    while (got < sizeof(req) - 1) {
        ssize_t n = recv(fd, req + got, sizeof(req) - 1 - got, 0);
        if (n <= 0)
            break;
        got += (size_t)n;
        req[got] = '\0';
        if (strstr(req, "\r\n\r\n"))
            break;
    }
    req[got] = '\0';

    char method[8] = "", path[256] = "";
    if (sscanf(req, "%7s %255s", method, path) == 2 && (!strcmp(method, "GET") || !strcmp(method, "POST"))) {
        char *q = strchr(path, '?');
        if (q)
            *q = '\0';
        if (strcmp(path, "/") == 0)
            send_response(fd, "200 OK", "text/html; charset=utf-8", PAGE, sizeof(PAGE) - 1);
        else if (strcmp(path, "/stream") == 0)
            serve_stream(fd);
        else if (strcmp(path, "/snapshot.jpg") == 0)
            serve_snapshot(fd);
        else if (strcmp(path, "/status") == 0)
            serve_status(fd);
        else if (strncmp(path, "/api/", 5) == 0)
            serve_api(fd, path + 5);
        else if (strncmp(path, "/files/", 7) == 0)
            serve_file(fd, path + 7);
        else
            send_response(fd, "404 Not Found", "text/plain", "not found\n", 10);
    } else if (got) {
        send_response(fd, "400 Bad Request", "text/plain", "bad request\n", 12);
    }
    close(fd);
    return NULL;
}

static void *accept_loop(void *arg)
{
    (void)arg;
    while (S.running) {
        int fd = accept(S.listen_fd, NULL, NULL);
        if (fd < 0) {
            if (errno == EINTR || errno == ECONNABORTED)
                continue;
            break; /* listen socket closed by stream_stop() */
        }
        /* Drop clients that stop reading (e.g. phone left the hotspot) instead of blocking forever. */
        struct timeval tv = {.tv_sec = 5};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        pthread_t t;
        if (pthread_create(&t, NULL, client_thread, (void *)(intptr_t)fd) == 0)
            pthread_detach(t);
        else
            close(fd);
    }
    return NULL;
}

int stream_start(int port)
{
    int fd = socket(AF_INET6, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("stream: socket");
        return -1;
    }
    int one = 1, zero = 0;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &zero, sizeof(zero)); /* accept IPv4 too */

    struct sockaddr_in6 addr = {.sin6_family = AF_INET6, .sin6_port = htons(port), .sin6_addr = in6addr_any};
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) || listen(fd, 8)) {
        fprintf(stderr, "stream: cannot listen on port %d: %s\n", port, strerror(errno));
        close(fd);
        return -1;
    }
    S.listen_fd = fd;
    S.running = 1;
    if (pthread_create(&S.accept_thread, NULL, accept_loop, NULL)) {
        S.running = 0;
        close(fd);
        S.listen_fd = -1;
        return -1;
    }
    fprintf(stderr, "stream: listening on port %d\n", port);
    return 0;
}

void stream_stop(void)
{
    if (!S.running)
        return;
    pthread_mutex_lock(&S.m);
    S.running = 0;
    pthread_cond_broadcast(&S.c);
    pthread_mutex_unlock(&S.m);
    shutdown(S.listen_fd, SHUT_RDWR);
    close(S.listen_fd);
    pthread_join(S.accept_thread, NULL);
    S.listen_fd = -1;
}

int stream_wanted(void)
{
    pthread_mutex_lock(&S.m);
    int w = S.wanted;
    pthread_mutex_unlock(&S.m);
    return w;
}

void stream_publish(const uint8_t *jpeg, size_t len)
{
    pthread_mutex_lock(&S.m);
    if (S.cap < len) {
        uint8_t *nb = realloc(S.jpeg, len);
        if (!nb) {
            pthread_mutex_unlock(&S.m);
            return;
        }
        S.jpeg = nb;
        S.cap = len;
    }
    memcpy(S.jpeg, jpeg, len);
    S.len = len;
    S.seq++;
    pthread_cond_broadcast(&S.c);
    pthread_mutex_unlock(&S.m);
}

void stream_set_api(stream_api_fn fn, void *ctx)
{
    pthread_mutex_lock(&S.m);
    S.api = fn;
    S.api_ctx = ctx;
    pthread_mutex_unlock(&S.m);
}

void stream_set_file_dir(const char *dir)
{
    snprintf(S.file_dir, sizeof(S.file_dir), "%s", dir);
}

void stream_set_status(const char *json)
{
    pthread_mutex_lock(&S.m);
    snprintf(S.status, sizeof(S.status), "%s", json);
    pthread_mutex_unlock(&S.m);
}
