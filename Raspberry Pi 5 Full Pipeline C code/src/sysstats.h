/*
 * sysstats.h - RAM, CPU, temperature and clock readings from /proc and /sys.
 */
#ifndef SYSSTATS_H
#define SYSSTATS_H

typedef struct {
    long mem_total_mb;
    long sys_used_mb;       /* MemTotal - MemAvailable: what the whole system needs */
    long mem_avail_mb;
    long cma_used_mb;       /* contiguous memory: camera and Hailo DMA buffers */
    long swap_used_mb;      /* > 0 means RAM ran short */
    long proc_rss_mb;       /* this process, resident */
    long proc_peak_mb;      /* this process, peak resident (VmHWM) */
    double cpu_total_pct;   /* all cores, since the previous sample */
    double proc_cpu_pct;    /* this process, as a share of all cores */
    double temp_c;          /* SoC */
    int arm_mhz;
    long throttled;         /* firmware get_throttled bits, -1 if unavailable */
    double uptime_s;
} sys_stats;

/* Fills s; CPU percentages are over the interval since the previous call. */
void sys_stats_sample(sys_stats *s);

#endif /* SYSSTATS_H */
