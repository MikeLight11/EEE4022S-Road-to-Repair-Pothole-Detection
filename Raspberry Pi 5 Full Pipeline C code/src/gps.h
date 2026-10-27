/*
 * gps.h - GNSS fixes from NMEA on a background thread, kept for matching to frames.
 *
 * Test hardware: the SIM7600X 4G HAT, whose GNSS is started with AT+CGPS=1 on
 * its AT port and then streams NMEA on its NMEA port (both USB serial ports).
 * The final build's ATGM336H streams NMEA on a UART by itself: leave at_dev
 * NULL for that.
 *
 * Each RMC fix is stamped with CLOCK_MONOTONIC (the camera's clock) at arrival,
 * minus delay_s, the receiver's epoch-to-Pi latency (0.2 s measured for the
 * ATGM336H in the report). A frame is located with the fix nearest its
 * timestamp, as in the report's "Locating the Pothole".
 */
#ifndef GPS_H
#define GPS_H

#include <stdint.h>

typedef struct {
    uint64_t mono_ns;       /* receiver epoch on CLOCK_MONOTONIC (arrival - delay) */
    double lat, lon;        /* degrees */
    double speed_mps;
    double course_deg;      /* course over ground, clockwise from true north */
    int sats;               /* satellites used (from the latest GGA), -1 unknown */
    double hdop;            /* -1 unknown */
    char utc[32];           /* ISO 8601, e.g. 2026-10-24T14:03:27+00:00 */
} gps_fix;

typedef struct gps gps;

/* nmea_dev: where NMEA arrives. at_dev: AT port used to switch the GNSS on
 * (SIM7600), or NULL. Opening is retried in the background, so the modem may
 * still be booting. */
gps *gps_start(const char *nmea_dev, const char *at_dev, double delay_s);
void gps_stop(gps *g);

/* The fix nearest ts_ns (CLOCK_MONOTONIC), if one is within max_gap_s.
 * Returns 0 and fills out, or -1. */
int gps_nearest(gps *g, uint64_t ts_ns, double max_gap_s, gps_fix *out);

/* For the status line: latest fix (0 if one arrived in the last 3 s, else -1),
 * and the measured fix rate over the last ~10 s. */
int gps_latest(gps *g, gps_fix *out, double *rate_hz);

/* Parses one NMEA sentence (checksum verified). Returns 1 for a valid RMC fix
 * (written to out, sats/hdop from the last GGA seen via *gga_sats, *gga_hdop),
 * 2 for a GGA (updates those), 0 for anything else or a bad checksum. */
int gps_parse_nmea(const char *s, gps_fix *out, int *gga_sats, double *gga_hdop);

/* Satellites in one GSV sentence: their numbers and signal strengths (C/N0,
 * dB-Hz; 0 = in view but not heard). Returns how many, or 0 if not a GSV. */
int gps_parse_gsv(const char *s, int *prn, int *snr, int max);

/* Signal for the status line: satellites heard (C/N0 > 0) in the last 5 s and
 * the mean C/N0 of the strongest four (the usual receiver-health figure: about
 * 35-45 dB-Hz with open sky, below ~25 too weak to keep a fix). */
void gps_signal(gps *g, int *heard, double *top4_db);

/* Moves (lat, lon) d_m metres along course_deg (flat-earth, as the report's Eq. gps_shift). */
void gps_shift(double *lat, double *lon, double course_deg, double d_m);

#endif /* GPS_H */
