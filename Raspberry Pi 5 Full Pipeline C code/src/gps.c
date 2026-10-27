/*
 * gps.c - GNSS fixes from NMEA on a background thread.
 */
#include "gps.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "serial.h"

#define RING 1024              /* fixes kept: ~100 s at 10 Hz, far longer than any track */
#define EARTH_R 6371008.8      /* mean Earth radius, m */
#define SILENT_RESTART_S 15    /* no NMEA for this long: reopen and switch the GNSS on again */
#define MAX_SV 128             /* satellites tracked for the signal figure, all constellations */

struct gps {
    char nmea_dev[128], at_dev[128];
    uint64_t delay_ns;
    pthread_t thread;
    pthread_mutex_t m;
    volatile int stop;
    gps_fix ring[RING];
    int head, len;             /* ring[head] is the newest */
    struct {
        char talker;           /* 'P' GPS, 'L' GLONASS, 'B'/'D' BeiDou, 'A' Galileo */
        int prn, snr;
        uint64_t t;
    } sv[MAX_SV];
    int nsv;
};

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* ------------------------------------------------------------------- NMEA */

static int checksum_ok(const char *s)
{
    if (s[0] != '$')
        return 0;
    const char *star = strchr(s, '*');
    if (!star || !star[1] || !star[2])
        return 0;
    unsigned sum = 0;
    for (const char *p = s + 1; p < star; p++)
        sum ^= (unsigned char)*p;
    unsigned want;
    return sscanf(star + 1, "%2x", &want) == 1 && want == sum;
}

/* Splits the sentence (copied into buf) at commas; fields stop at '*'. */
static int split(const char *s, char *buf, size_t cap, char **f, int maxf)
{
    snprintf(buf, cap, "%s", s);
    char *star = strchr(buf, '*');
    if (star)
        *star = '\0';
    int n = 0;
    char *p = buf;
    while (n < maxf) {
        f[n++] = p;
        char *c = strchr(p, ',');
        if (!c)
            break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

/* ddmm.mmmm + hemisphere -> signed degrees */
static int nmea_deg(const char *v, const char *hemi, int deg_digits, double *out)
{
    if (!*v || !*hemi || strlen(v) < (size_t)deg_digits + 2)
        return -1;
    char d[4] = {0};
    memcpy(d, v, (size_t)deg_digits);
    double x = atof(d) + atof(v + deg_digits) / 60.0;
    *out = (*hemi == 'S' || *hemi == 'W') ? -x : x;
    return 0;
}

int gps_parse_nmea(const char *s, gps_fix *out, int *gga_sats, double *gga_hdop)
{
    if (!checksum_ok(s) || strlen(s) < 7)
        return 0;
    char buf[128], *f[20];
    int n = split(s, buf, sizeof(buf), f, 20);
    const char *type = f[0] + 3;   /* $GPRMC, $GNRMC, $GLRMC ... */

    /* GNS ($GNGNS) counts satellites of every constellation used; GGA from a
     * $GP talker counts only GPS ones (0 on a GLONASS/BeiDou fix), so GNS wins. */
    if (!strcmp(type, "GNS") && n >= 9) {
        if (*f[2] && *f[7]) {
            *gga_sats = atoi(f[7]);
            *gga_hdop = *f[8] ? atof(f[8]) : -1;
        }
        return 2;
    }
    if (!strcmp(type, "GGA") && n >= 9 && !strncmp(f[0], "$GP", 3) && *gga_sats > 0)
        return 2;   /* already counted by GNS */
    if (!strcmp(type, "GGA") && n >= 9) {
        if (atoi(f[6]) > 0) {      /* fix quality */
            *gga_sats = *f[7] ? atoi(f[7]) : -1;
            *gga_hdop = *f[8] ? atof(f[8]) : -1;
        } else {
            *gga_sats = 0;
            *gga_hdop = -1;
        }
        return 2;
    }
    if (strcmp(type, "RMC") || n < 10 || strcmp(f[2], "A"))   /* A = valid, V = no fix */
        return 0;
    gps_fix x = {.sats = *gga_sats, .hdop = *gga_hdop};
    if (nmea_deg(f[3], f[4], 2, &x.lat) || nmea_deg(f[5], f[6], 3, &x.lon))
        return 0;
    x.speed_mps = atof(f[7]) * 0.514444;
    x.course_deg = *f[8] ? atof(f[8]) : -1;
    int hh, mi, ss, dd, mo, yy;
    if (sscanf(f[1], "%2d%2d%2d", &hh, &mi, &ss) != 3 || sscanf(f[9], "%2d%2d%2d", &dd, &mo, &yy) != 3)
        return 0;
    /* % 100u: each field is 2 digits (%2d above); tells the compiler too */
    snprintf(x.utc, sizeof(x.utc), "%04u-%02u-%02uT%02u:%02u:%02u+00:00", 2000u + (unsigned)yy % 100u,
             (unsigned)mo % 100u, (unsigned)dd % 100u, (unsigned)hh % 100u, (unsigned)mi % 100u,
             (unsigned)ss % 100u);
    *out = x;
    return 1;
}

int gps_parse_gsv(const char *s, int *prn, int *snr, int max)
{
    if (!checksum_ok(s) || strlen(s) < 7 || strncmp(s + 3, "GSV", 3))
        return 0;
    char buf[128], *f[24];
    int n = split(s, buf, sizeof(buf), f, 24), k = 0;
    /* $xxGSV,total,num,in_view, then (prn, elevation, azimuth, snr) x up to 4 [, signal id] */
    for (int i = 4; i + 3 < n && k < max; i += 4) {
        if (!*f[i])
            continue;
        prn[k] = atoi(f[i]);
        snr[k] = *f[i + 3] ? atoi(f[i + 3]) : 0;
        k++;
    }
    return k;
}

static void note_gsv(gps *g, const char *line, uint64_t t)
{
    int prn[4], snr[4], n = gps_parse_gsv(line, prn, snr, 4);
    char talker = line[2];
    pthread_mutex_lock(&g->m);
    for (int i = 0; i < n; i++) {
        int j = 0;
        while (j < g->nsv && !(g->sv[j].talker == talker && g->sv[j].prn == prn[i]))
            j++;
        if (j == g->nsv) {
            if (g->nsv == MAX_SV) {   /* full: reuse the stalest */
                j = 0;
                for (int q = 1; q < g->nsv; q++)
                    if (g->sv[q].t < g->sv[j].t)
                        j = q;
            } else {
                g->nsv++;
            }
        }
        g->sv[j].talker = talker;
        g->sv[j].prn = prn[i];
        g->sv[j].snr = snr[i];
        g->sv[j].t = t;
    }
    pthread_mutex_unlock(&g->m);
}

void gps_signal(gps *g, int *heard, double *top4_db)
{
    *heard = 0;
    *top4_db = 0;
    if (!g)
        return;
    int top[4] = {0};
    uint64_t now = now_ns();
    pthread_mutex_lock(&g->m);
    for (int i = 0; i < g->nsv; i++) {
        if (now - g->sv[i].t > 5000000000ull || g->sv[i].snr <= 0)
            continue;
        (*heard)++;
        int v = g->sv[i].snr;
        for (int k = 0; k < 4; k++)   /* keep the four largest */
            if (v > top[k]) {
                int tmp = top[k];
                top[k] = v;
                v = tmp;
            }
    }
    pthread_mutex_unlock(&g->m);
    int n = *heard < 4 ? *heard : 4;
    for (int k = 0; k < n; k++)
        *top4_db += top[k];
    if (n)
        *top4_db /= n;
}

void gps_shift(double *lat, double *lon, double course_deg, double d_m)
{
    double psi = course_deg * M_PI / 180.0, phi = *lat * M_PI / 180.0;
    *lat += d_m * cos(psi) / EARTH_R * 180.0 / M_PI;
    *lon += d_m * sin(psi) / (EARTH_R * cos(phi)) * 180.0 / M_PI;
}

/* ----------------------------------------------------------------- thread */

/* SIM7600: GNSS on (AT+CGPS=1). It answers ERROR if it's already on, so check first. */
static void gnss_on(gps *g)
{
    int fd = serial_open(g->at_dev, 115200);
    if (fd < 0)
        return;
    char r[256];
    if (at_sync(fd, 5) == 0) {
        at_command(fd, "ATE0", NULL, 0, 1000);
        if (at_command(fd, "AT+CGPS?", r, sizeof(r), 2000) == 0 && strstr(r, "+CGPS: 1"))
            fprintf(stderr, "gps: GNSS already on (%s)\n", g->at_dev);
        else if (at_command(fd, "AT+CGPS=1", r, sizeof(r), 5000) == 0)
            fprintf(stderr, "gps: GNSS switched on (%s)\n", g->at_dev);
        else
            fprintf(stderr, "gps: AT+CGPS=1 failed on %s\n", g->at_dev);
    }
    close(fd);   /* not held: sim7600_test and the uploader's checks can use the AT port */
}

static void *worker(void *arg)
{
    gps *g = arg;
    int gga_sats = -1, warned = 0;
    double gga_hdop = -1;
    while (!g->stop) {
        if (g->at_dev[0])
            gnss_on(g);
        int fd = serial_open(g->nmea_dev, 115200);
        if (fd < 0) {
            if (!warned++)
                fprintf(stderr, "gps: waiting for %s (modem booting or not connected)\n", g->nmea_dev);
            sleep(2);
            continue;
        }
        fprintf(stderr, "gps: reading NMEA from %s\n", g->nmea_dev);
        warned = 0;
        char buf[2048], line[256];
        size_t len = 0;
        uint64_t last_line = now_ns();
        int have_fix = 0;
        while (!g->stop) {
            int n = serial_read_line(fd, buf, sizeof(buf), &len, line, sizeof(line), 500);
            uint64_t t = now_ns();
            if (n < 0)
                break;
            if (n == 0) {
                if (t - last_line > SILENT_RESTART_S * 1000000000ull) {
                    fprintf(stderr, "gps: no NMEA for %d s, restarting\n", SILENT_RESTART_S);
                    break;
                }
                continue;
            }
            last_line = t;
            if (strstr(line, "GSV,")) {
                note_gsv(g, line, t);
                continue;
            }
            gps_fix x;
            if (gps_parse_nmea(line, &x, &gga_sats, &gga_hdop) != 1)
                continue;
            x.mono_ns = t - g->delay_ns;
            if (!have_fix++)
                fprintf(stderr, "gps: first fix %.6f, %.6f (%s, %d sats)\n", x.lat, x.lon, x.utc, x.sats);
            pthread_mutex_lock(&g->m);
            g->head = (g->head + 1) % RING;
            g->ring[g->head] = x;
            if (g->len < RING)
                g->len++;
            pthread_mutex_unlock(&g->m);
        }
        close(fd);
        if (!g->stop)
            sleep(1);
    }
    return NULL;
}

gps *gps_start(const char *nmea_dev, const char *at_dev, double delay_s)
{
    gps *g = calloc(1, sizeof(*g));
    if (!g)
        return NULL;
    snprintf(g->nmea_dev, sizeof(g->nmea_dev), "%s", nmea_dev);
    snprintf(g->at_dev, sizeof(g->at_dev), "%s", at_dev ? at_dev : "");
    g->delay_ns = (uint64_t)(delay_s * 1e9);
    g->head = RING - 1;
    pthread_mutex_init(&g->m, NULL);
    if (pthread_create(&g->thread, NULL, worker, g)) {
        free(g);
        return NULL;
    }
    return g;
}

void gps_stop(gps *g)
{
    if (!g)
        return;
    g->stop = 1;
    pthread_join(g->thread, NULL);
    pthread_mutex_destroy(&g->m);
    free(g);
}

int gps_nearest(gps *g, uint64_t ts_ns, double max_gap_s, gps_fix *out)
{
    if (!g)
        return -1;
    uint64_t best = UINT64_MAX;
    pthread_mutex_lock(&g->m);
    for (int i = 0; i < g->len; i++) {
        const gps_fix *x = &g->ring[(g->head - i + RING) % RING];
        uint64_t d = x->mono_ns > ts_ns ? x->mono_ns - ts_ns : ts_ns - x->mono_ns;
        if (d < best) {
            best = d;
            *out = *x;
        }
        if (x->mono_ns < ts_ns)   /* older still only gets further away */
            break;
    }
    pthread_mutex_unlock(&g->m);
    return best <= (uint64_t)(max_gap_s * 1e9) ? 0 : -1;
}

int gps_latest(gps *g, gps_fix *out, double *rate_hz)
{
    *rate_hz = 0;
    if (!g)
        return -1;
    uint64_t now = now_ns();
    int rc = -1, n = 0;
    pthread_mutex_lock(&g->m);
    if (g->len) {
        *out = g->ring[g->head];
        rc = now - out->mono_ns < 3000000000ull ? 0 : -1;
        for (; n < g->len && now - g->ring[(g->head - n + RING) % RING].mono_ns < 10000000000ull; n++)
            ;
    }
    pthread_mutex_unlock(&g->m);
    *rate_hz = n / 10.0;
    return rc;
}
