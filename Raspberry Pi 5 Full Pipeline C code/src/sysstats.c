/*
 * sysstats.c - RAM, CPU, temperature and clock readings from /proc and /sys.
 */
#include "sysstats.h"

#include <glob.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static long read_long(const char *path, int base)
{
    char buf[64];
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;
    long v = fgets(buf, sizeof(buf), f) ? strtol(buf, NULL, base) : -1;
    fclose(f);
    return v;
}

/* Value of "key:" in a /proc file of "key: value kB" lines, in MB. */
static long kv_mb(const char *path, const char *key)
{
    char line[256];
    size_t klen = strlen(key);
    long v = -1;
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;
    while (fgets(line, sizeof(line), f))
        if (!strncmp(line, key, klen) && line[klen] == ':') {
            v = strtol(line + klen + 1, NULL, 10) / 1024;
            break;
        }
    fclose(f);
    return v;
}

static int cpu_jiffies(unsigned long long *total, unsigned long long *idle)
{
    unsigned long long v[8] = {0};
    FILE *f = fopen("/proc/stat", "r");
    if (!f)
        return -1;
    int n = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                   &v[6], &v[7]);
    fclose(f);
    if (n < 5)
        return -1;
    *total = 0;
    for (int i = 0; i < 8; i++)
        *total += v[i];
    *idle = v[3] + v[4];   /* idle + iowait */
    return 0;
}

static unsigned long long proc_jiffies(void)
{
    char buf[1024];
    unsigned long long ut = 0, st = 0;
    FILE *f = fopen("/proc/self/stat", "r");
    if (!f)
        return 0;
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';
    const char *p = strrchr(buf, ')');   /* the command name may contain spaces */
    /* fields 3-13 skipped, then utime (14) and stime (15) */
    if (p)
        sscanf(p + 2, "%*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu", &ut, &st);
    return ut + st;
}

/* Pi firmware exposes the under-voltage/throttle bits in sysfs; the path varies by model. */
static const char *throttled_path(void)
{
    static char path[256];
    static int looked;
    if (!looked) {
        looked = 1;
        glob_t g;
        if (glob("/sys/devices/platform/*/*firmware/get_throttled", 0, NULL, &g) == 0 ||
            glob("/sys/devices/platform/*/*/*firmware/get_throttled", 0, NULL, &g) == 0) {
            snprintf(path, sizeof(path), "%s", g.gl_pathv[0]);
            globfree(&g);
        }
    }
    return path[0] ? path : NULL;
}

void sys_stats_sample(sys_stats *s)
{
    static unsigned long long prev_total, prev_idle, prev_proc;

    memset(s, 0, sizeof(*s));
    s->mem_total_mb = kv_mb("/proc/meminfo", "MemTotal");
    s->mem_avail_mb = kv_mb("/proc/meminfo", "MemAvailable");
    s->sys_used_mb = s->mem_total_mb - s->mem_avail_mb;
    s->cma_used_mb = kv_mb("/proc/meminfo", "CmaTotal") - kv_mb("/proc/meminfo", "CmaFree");
    s->swap_used_mb = kv_mb("/proc/meminfo", "SwapTotal") - kv_mb("/proc/meminfo", "SwapFree");
    s->proc_rss_mb = kv_mb("/proc/self/status", "VmRSS");
    s->proc_peak_mb = kv_mb("/proc/self/status", "VmHWM");

    unsigned long long total, idle, proc = proc_jiffies();
    if (cpu_jiffies(&total, &idle) == 0) {
        if (prev_total && total > prev_total) {
            double dt = (double)(total - prev_total);
            s->cpu_total_pct = 100.0 * (dt - (double)(idle - prev_idle)) / dt;
            /* /proc/stat counts jiffies across all cores, the same units as the process times */
            s->proc_cpu_pct = 100.0 * (double)(proc - prev_proc) / dt;
        }
        prev_total = total;
        prev_idle = idle;
        prev_proc = proc;
    }

    long t = read_long("/sys/class/thermal/thermal_zone0/temp", 10);
    s->temp_c = t >= 0 ? t / 1000.0 : -1;
    long khz = read_long("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", 10);
    s->arm_mhz = khz >= 0 ? (int)(khz / 1000) : -1;
    const char *tp = throttled_path();
    s->throttled = tp ? read_long(tp, 16) : -1;

    FILE *f = fopen("/proc/uptime", "r");
    if (f) {
        if (fscanf(f, "%lf", &s->uptime_s) != 1)
            s->uptime_s = 0;
        fclose(f);
    }
}
