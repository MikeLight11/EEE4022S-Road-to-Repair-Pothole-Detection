/*
 * sim7600_test.c - bench checks for the SIM7600X 4G HAT, one step at a time.
 *
 *   sim7600_test ports              which /dev/ttyUSB* is which (USB IDs from sysfs)
 *   sim7600_test info               SIM, signal, registration, APN, USB mode, GNSS state,
 *                                   default route and a request to the upload server
 *   sim7600_test power              pulse the HAT's PWRKEY (BCM 6) to switch the module on
 *   sim7600_test rndis APN          one-off: set the APN and switch the USB mode to RNDIS
 *                                   (the module reboots and appears as network interface usb0)
 *   sim7600_test dial               start the RNDIS data call by hand, if usb0 has no address
 *   sim7600_test at "AT+CGACT?"     send one AT command and print the whole reply
 *   sim7600_test gps [SECONDS]      switch the GNSS on and print fixes (default 120 s): time
 *                                   to first fix, satellites, measured rate
 *   sim7600_test gps-rate 1|10      try setting the NMEA rate (AT+CGPSNMEARATE)
 *   sim7600_test send DIR/ID.json   upload one record from the pipeline, as build/pothole would
 *   sim7600_test send-test [JPG]    upload a made-up 300 x 500 mm pothole at the current GPS fix
 *                                   (default image hailo/test_frame.jpg); delete it on the web app after
 *
 * Ports, URL and key come from uplink.conf (as build/pothole); --uplink FILE to change.
 */
#include <curl/curl.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "gps.h"
#include "serial.h"
#include "uploader.h"

static char g_nmea[128] = "/dev/ttyUSB1", g_at[128] = "/dev/ttyUSB2", g_url[400] = "", g_key[256] = "";

static void conf_string(const char *path, const char *key, char *out, size_t cap)
{
    FILE *f = fopen(path, "r");
    char line[512], k[32], v[400];
    if (!f)
        return;
    while (fgets(line, sizeof(line), f)) {
        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        if (sscanf(line, " %31[a-z0-9_] = %399s", k, v) == 2 && !strcmp(k, key))
            snprintf(out, cap, "%s", v);
    }
    fclose(f);
}

static int read_sys(const char *path, char *out, size_t cap)
{
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;
    int ok = fgets(out, (int)cap, f) != NULL;
    fclose(f);
    if (!ok)
        return -1;
    out[strcspn(out, "\r\n")] = '\0';
    return 0;
}

/* --------------------------------------------------------------- commands */

static int cmd_ports(void)
{
    DIR *d = opendir("/sys/class/tty");
    struct dirent *e;
    int found = 0;
    while (d && (e = readdir(d))) {
        if (strncmp(e->d_name, "ttyUSB", 6))
            continue;
        /* device = the usb-serial port, .. = its USB interface, ../.. = the USB device */
        char p[512], vid[16] = "", pid[16] = "", ifn[16] = "";
        snprintf(p, sizeof(p), "/sys/class/tty/%s/device/../bInterfaceNumber", e->d_name);
        read_sys(p, ifn, sizeof(ifn));
        snprintf(p, sizeof(p), "/sys/class/tty/%s/device/../../idVendor", e->d_name);
        read_sys(p, vid, sizeof(vid));
        snprintf(p, sizeof(p), "/sys/class/tty/%s/device/../../idProduct", e->d_name);
        read_sys(p, pid, sizeof(pid));
        int i = (int)strtol(ifn, NULL, 16);
        /* SIMCom's interface layout: 9001 = diag, NMEA, AT, modem, audio (+ wwan);
         * 9011 (RNDIS) puts the two RNDIS interfaces first. */
        static const char *const roles[] = {"diag", "NMEA (gps_dev)", "AT (at_dev)", "modem (PPP)", "audio"};
        int k = !strcmp(pid, "9011") ? i - 2 : i;
        const char *role = !strcmp(vid, "1e0e") && k >= 0 && k < 5 ? roles[k] : "";
        printf("/dev/%-8s usb %s:%s interface %s  %s\n", e->d_name, vid, pid, ifn, role);
        found++;
    }
    if (d)
        closedir(d);
    if (!found)
        printf("no /dev/ttyUSB* ports: is the HAT's USB port cabled to the Pi and the module on?\n"
               "  (its NET LED blinks once it's running; 'sim7600_test power' switches it on)\n");
    else
        printf("uplink.conf now: gps_dev = %s, at_dev = %s\n", g_nmea, g_at);
    return found ? 0 : 1;
}

static int open_at(void)
{
    int fd = serial_open(g_at, 115200);
    if (fd < 0) {
        fprintf(stderr, "can't open %s: %s (run 'sim7600_test ports')\n", g_at, strerror(errno));
        return -1;
    }
    if (at_sync(fd, 10)) {
        fprintf(stderr, "%s doesn't answer AT: wrong port? ModemManager holding it? "
                        "(sudo systemctl disable --now ModemManager)\n", g_at);
        close(fd);
        return -1;
    }
    at_command(fd, "ATE0", NULL, 0, 1000);
    at_command(fd, "AT+CMEE=2", NULL, 0, 1000);
    return fd;
}

/* Prints the reply lines without the echo / blank lines / OK. */
static void show(int fd, const char *label, const char *cmd, int timeout_ms)
{
    char r[1024];
    int rc = at_command(fd, cmd, r, sizeof(r), timeout_ms);
    printf("%-14s", label);
    int first = 1;
    for (char *l = strtok(r, "\r\n"); l; l = strtok(NULL, "\r\n")) {
        if (!strcmp(l, "OK") || !strcmp(l, cmd))
            continue;
        printf("%s%s", first ? "" : " | ", l);
        first = 0;
    }
    printf("%s\n", rc ? (first ? "(no reply / ERROR)" : "") : "");
}

static void route_info(void)
{
    FILE *f = fopen("/proc/net/route", "r");
    char line[256], ifc[32];
    unsigned long dst, gw;
    unsigned flags, metric;
    int printed = 0;
    if (f) {
        while (fgets(line, sizeof(line), f))
            if (sscanf(line, "%31s %lx %lx %x %*d %*d %u", ifc, &dst, &gw, &flags, &metric) == 5 && dst == 0) {
                printf("%-14s%s (metric %u)%s\n", printed ? "" : "default route", ifc, metric,
                       !strncmp(ifc, "usb", 3) || !strncmp(ifc, "wwan", 4) ? "  <- the 4G HAT" : "");
                printed = 1;
            }
        fclose(f);
    }
    if (!printed)
        printf("default route none: no internet (RNDIS not set up / data call not up?)\n");
}

static size_t discard(char *p, size_t s, size_t n, void *u)
{
    (void)p;
    (void)u;
    return s * n;
}

static void server_check(void)
{
    if (!g_url[0]) {
        printf("server        no upload_url in uplink.conf\n");
        return;
    }
    char base[400];
    snprintf(base, sizeof(base), "%s", g_url);
    char *path = strstr(base, "://");
    if (path && (path = strchr(path + 3, '/')))
        path[1] = '\0';
    CURL *c = curl_easy_init();
    curl_easy_setopt(c, CURLOPT_URL, base);
    curl_easy_setopt(c, CURLOPT_NOBODY, 1L);
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, discard);
    curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 20L);
    curl_easy_setopt(c, CURLOPT_TIMEOUT, 30L);
    CURLcode rc = curl_easy_perform(c);
    long st = 0;
    double t = 0;
    char *ip = NULL;
    curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &st);
    curl_easy_getinfo(c, CURLINFO_TOTAL_TIME, &t);
    curl_easy_getinfo(c, CURLINFO_LOCAL_IP, &ip);
    if (rc == CURLE_OK)
        printf("server        %s -> HTTP %ld in %.2f s, from local address %s\n", base, st, t, ip ? ip : "?");
    else
        printf("server        %s unreachable: %s\n", base, curl_easy_strerror(rc));
    curl_easy_cleanup(c);
}

static int cmd_info(void)
{
    int fd = open_at();
    if (fd >= 0) {
        show(fd, "module", "ATI", 2000);
        show(fd, "firmware", "AT+CGMR", 2000);
        show(fd, "SIM", "AT+CPIN?", 3000);
        show(fd, "signal", "AT+CSQ", 2000);   /* rssi 0-31 (>= 10 usable, 99 unknown) */
        show(fd, "operator", "AT+COPS?", 5000);
        show(fd, "LTE reg", "AT+CEREG?", 2000);   /* ,1 home / ,5 roaming = registered */
        show(fd, "network", "AT+CPSI?", 2000);
        show(fd, "APN", "AT+CGDCONT?", 2000);
        show(fd, "USB mode", "AT+CUSBPIDSWITCH?", 2000);   /* 9011 = RNDIS (network interface usb0) */
        show(fd, "GNSS", "AT+CGPS?", 2000);
        show(fd, "NMEA rate", "AT+CGPSNMEARATE?", 2000);
        close(fd);
    }
    route_info();
    server_check();
    return fd < 0;
}

static int cmd_power(void)
{
    if (access(g_at, F_OK) == 0) {
        printf("%s exists: the module is already on (another pulse would switch it off)\n", g_at);
        return 0;
    }
    printf("pulsing PWRKEY (BCM 6) high for 2 s...\n");
    if (system("pinctrl set 6 op dh && sleep 2 && pinctrl set 6 op dl")) {
        fprintf(stderr, "pinctrl failed (sudo? Pi OS's pinctrl tool installed?)\n");
        return 1;
    }
    for (int i = 0; i < 40 && access(g_at, F_OK); i++)
        sleep(1);
    printf(access(g_at, F_OK) ? "%s still missing after 40 s: check the HAT's USB cable\n" : "%s is up\n", g_at);
    return access(g_at, F_OK) != 0;
}

static int cmd_rndis(const char *apn)
{
    int fd = open_at();
    if (fd < 0)
        return 1;
    char cmd[160], r[512];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    if (at_command(fd, cmd, r, sizeof(r), 5000)) {
        fprintf(stderr, "%s failed: %s\n", cmd, r);
        close(fd);
        return 1;
    }
    printf("APN set to %s\n", apn);
    at_command(fd, "AT+CUSBPIDSWITCH?", r, sizeof(r), 2000);
    if (strstr(r, "9011")) {
        printf("already in RNDIS mode\n");
        close(fd);
        return 0;
    }
    printf("switching to RNDIS (AT+CUSBPIDSWITCH=9011,1,1); the module reboots...\n");
    at_command(fd, "AT+CUSBPIDSWITCH=9011,1,1", r, sizeof(r), 10000);
    close(fd);
    sleep(5);
    for (int i = 0; i < 60 && access("/sys/class/net/usb0", F_OK); i++)
        sleep(1);
    if (access("/sys/class/net/usb0", F_OK)) {
        printf("usb0 hasn't appeared after 60 s: check 'ip link' and 'sim7600_test ports'\n");
        return 1;
    }
    printf("usb0 is up: NetworkManager gives it an address; check with 'sim7600_test info'\n"
           "(the port numbers may have moved: run 'sim7600_test ports')\n");
    return 0;
}

static int cmd_dial(void)
{
    int fd = open_at();
    if (fd < 0)
        return 1;
    char r[512];
    int rc = at_command(fd, "AT$QCRMCALL=1,1", r, sizeof(r), 30000);
    printf("AT$QCRMCALL=1,1 -> %s\n", r);
    close(fd);
    return rc != 0;
}

static int cmd_gps(int secs)
{
    int fd = open_at();
    if (fd < 0)
        return 1;
    char r[256];
    if (at_command(fd, "AT+CGPS?", r, sizeof(r), 2000) || !strstr(r, "+CGPS: 1"))
        printf("AT+CGPS=1 -> %s\n", at_command(fd, "AT+CGPS=1", r, sizeof(r), 5000) ? "ERROR" : "OK");
    close(fd);
    int nf = serial_open(g_nmea, 115200);
    if (nf < 0) {
        fprintf(stderr, "can't open %s: %s\n", g_nmea, strerror(errno));
        return 1;
    }
    printf("reading %s for %d s (the GNSS antenna needs sky view; a cold start takes ~30-60 s)\n", g_nmea, secs);
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    char buf[2048], line[256];
    size_t len = 0;
    int sats = -1, fixes = 0, sentences = 0;
    double hdop = -1, first = -1, last_print = -10;
    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &t);
        double el = (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
        if (el > secs)
            break;
        int n = serial_read_line(nf, buf, sizeof(buf), &len, line, sizeof(line), 500);
        if (n < 0) {
            fprintf(stderr, "%s went away\n", g_nmea);
            break;
        }
        if (n == 0)
            continue;
        sentences++;
        gps_fix x;
        if (gps_parse_nmea(line, &x, &sats, &hdop) != 1) {
            if (el - last_print >= 5 && fixes == 0) {
                printf("%5.0f s  no fix yet (%d sentences, last: %.60s)\n", el, sentences, line);
                last_print = el;
            }
            continue;
        }
        if (fixes++ == 0) {
            first = el;
            printf("%5.1f s  FIRST FIX\n", el);
        }
        if (el - last_print >= 1) {
            printf("%5.1f s  %.6f, %.6f  %5.1f km/h  course %5.1f  %2d sats  hdop %.1f  %s  (%.1f Hz)\n", el, x.lat,
                   x.lon, x.speed_mps * 3.6, x.course_deg, x.sats, x.hdop, x.utc,
                   el > first ? (fixes - 1) / (el - first) : 0.0);
            last_print = el;
        }
    }
    close(nf);
    if (fixes)
        printf("first fix after %.0f s, %d fixes, %.1f Hz\n", first, fixes,
               secs > first ? (fixes - 1) / (secs - first) : 0.0);
    else
        printf("no fix in %d s (%d NMEA sentences: %s)\n", secs, sentences,
               sentences ? "receiver working, needs sky view" : "wrong gps_dev?");
    return fixes == 0;
}

static int cmd_gps_rate(int hz)
{
    int fd = open_at();
    if (fd < 0)
        return 1;
    char cmd[64], r[256];
    at_command(fd, "AT+CGPS=0", NULL, 0, 5000);   /* the rate can only change while it's off */
    sleep(1);
    snprintf(cmd, sizeof(cmd), "AT+CGPSNMEARATE=%d", hz == 10 ? 1 : 0);
    int rc = at_command(fd, cmd, r, sizeof(r), 3000);
    printf("%s -> %s\n", cmd, rc ? "ERROR (not supported on this firmware?)" : "OK");
    at_command(fd, "AT+CGPS=1", NULL, 0, 5000);
    close(fd);
    printf("check the real rate with 'sim7600_test gps 60'\n");
    return rc != 0;
}

static int cmd_send(const char *path)
{
    char dir[400], name[200];
    const char *slash = strrchr(path, '/');
    snprintf(dir, sizeof(dir), "%.*s", slash ? (int)(slash - path) : 1, slash ? path : ".");
    snprintf(name, sizeof(name), "%s", slash ? slash + 1 : path);
    if (!g_url[0] || !g_key[0]) {
        fprintf(stderr, "set upload_url and api_key in uplink.conf\n");
        return 1;
    }
    up_config uc = {dir, g_url, g_key, 1};
    char msg[512];
    int st = uploader_send_one(&uc, name, msg, sizeof(msg));
    printf("%s\n", msg);
    return !(st == 200 || st == 201);
}

static int copy_file(const char *from, const char *to)
{
    FILE *a = fopen(from, "rb"), *b = a ? fopen(to, "wb") : NULL;
    char buf[65536];
    size_t n;
    int rc = a && b ? 0 : -1;
    while (!rc && (n = fread(buf, 1, sizeof(buf), a)) > 0)
        rc = fwrite(buf, 1, n, b) == n ? 0 : -1;
    if (a)
        fclose(a);
    if (b)
        fclose(b);
    return rc;
}

static int cmd_send_test(const char *jpg)
{
    /* Current position from the NMEA port. */
    int fd = open_at();
    if (fd >= 0) {
        at_command(fd, "AT+CGPS=1", NULL, 0, 5000);
        close(fd);
    }
    int nf = serial_open(g_nmea, 115200);
    if (nf < 0) {
        fprintf(stderr, "can't open %s\n", g_nmea);
        return 1;
    }
    printf("waiting up to 120 s for a GPS fix...\n");
    char buf[2048], line[256];
    size_t len = 0;
    int sats = -1, got = 0;
    double hdop = -1;
    gps_fix x;
    time_t until = time(NULL) + 120;
    while (!got && time(NULL) < until) {
        int n = serial_read_line(nf, buf, sizeof(buf), &len, line, sizeof(line), 1000);
        if (n < 0)
            break;
        got = n > 0 && gps_parse_nmea(line, &x, &sats, &hdop) == 1;
    }
    close(nf);
    if (!got) {
        fprintf(stderr, "no GPS fix: take the antenna outside / to a window\n");
        return 1;
    }
    printf("fix %.6f, %.6f (%d sats) at %s\n", x.lat, x.lon, x.sats, x.utc);

    const char *dir = "detections_test";
    mkdir(dir, 0755);
    char id[64], path[512], json[1024];
    snprintf(id, sizeof(id), "test-%ld", (long)time(NULL));
    snprintf(path, sizeof(path), "%s/%s.jpg", dir, id);
    if (copy_file(jpg, path)) {
        fprintf(stderr, "can't copy %s\n", jpg);
        return 1;
    }
    snprintf(json, sizeof(json),
             "{\"id\":\"%s\",\"label\":\"Pothole\",\"valid\":true,\"width_m\":0.300,\"length_m\":0.500,"
             "\"image\":\"%s.jpg\",\n \"gps\":{\"lat\":%.7f,\"lon\":%.7f,\"utc\":\"%s\"}}\n",
             id, id, x.lat, x.lon, x.utc);
    snprintf(path, sizeof(path), "%s/%s.json", dir, id);
    FILE *f = fopen(path, "w");
    if (!f || fputs(json, f) < 0) {
        fprintf(stderr, "can't write %s\n", path);
        return 1;
    }
    fclose(f);
    printf("sending %s (moves to %s/sent/ when the server accepts it)\n", path, dir);
    return cmd_send(path);
}

int main(int argc, char **argv)
{
    const char *uplink = "uplink.conf";
    if (argc > 2 && !strcmp(argv[1], "--uplink")) {
        uplink = argv[2];
        argc -= 2;
        argv += 2;
    }
    conf_string(uplink, "gps_dev", g_nmea, sizeof(g_nmea));
    conf_string(uplink, "at_dev", g_at, sizeof(g_at));
    conf_string(uplink, "upload_url", g_url, sizeof(g_url));
    conf_string(uplink, "api_key", g_key, sizeof(g_key));
    curl_global_init(CURL_GLOBAL_DEFAULT);
    setvbuf(stdout, NULL, _IOLBF, 0);

    const char *c = argc > 1 ? argv[1] : "";
    if (!strcmp(c, "ports"))
        return cmd_ports();
    if (!strcmp(c, "info"))
        return cmd_info();
    if (!strcmp(c, "power"))
        return cmd_power();
    if (!strcmp(c, "rndis") && argc > 2)
        return cmd_rndis(argv[2]);
    if (!strcmp(c, "dial"))
        return cmd_dial();
    if (!strcmp(c, "at") && argc > 2) {   /* one raw command, full reply */
        int fd = open_at();
        if (fd < 0)
            return 1;
        char r[2048];
        int rc = at_command(fd, argv[2], r, sizeof(r), 30000);
        printf("%s\n", r);
        close(fd);
        return rc != 0;
    }
    if (!strcmp(c, "gps"))
        return cmd_gps(argc > 2 ? atoi(argv[2]) : 120);
    if (!strcmp(c, "gps-rate") && argc > 2)
        return cmd_gps_rate(atoi(argv[2]));
    if (!strcmp(c, "send") && argc > 2)
        return cmd_send(argv[2]);
    if (!strcmp(c, "send-test"))
        return cmd_send_test(argc > 2 ? argv[2] : "hailo/test_frame.jpg");
    fprintf(stderr,
            "usage: %s [--uplink FILE] ports | info | power | rndis APN | dial | gps [SECONDS] |\n"
            "       gps-rate 1|10 | send DIR/ID.json | send-test [JPG]\n",
            argv[0]);
    return 2;
}
