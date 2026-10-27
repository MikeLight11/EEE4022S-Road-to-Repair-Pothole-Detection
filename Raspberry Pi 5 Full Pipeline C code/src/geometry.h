/*
 * geometry.h - pothole size on the road from its mask outline.
 *
 * C port of seg_geometry.py / vidar_geometry.py (Pothole Detection), with the
 * OpenCV pieces they used (findContours, contourArea, minAreaRect,
 * undistortPoints) implemented here.
 *
 * Ground coordinates, in metres: X lateral (positive right of the optical axis),
 * Y forward horizontal distance from the camera's foot point.
 */
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdint.h>

typedef struct {
    double f_px, cx, cy;    /* intrinsics, pixels at width x height */
    double h_mm;            /* camera height above the road */
    double tilt_rad;        /* pitch below horizontal */
    int width, height;      /* resolution f_px/cx/cy were measured at */
    double dist[5];         /* k1 k2 p1 p2 k3, used if has_dist */
    int has_dist;
} geo_calib;

/* OV5647 on the grille mount (calibration.py CAL), 1920x1080, no undistortion. */
geo_calib geo_calib_ov5647(void);

/* Overrides c from a "key = value" file (h_mm, tilt_deg, f_px, cx, cy, width,
 * height, k1, k2, p1, p2, k3; '#' starts a comment; start_conf and min_hits are
 * tracking settings, skipped here). Returns 1 if the file was
 * read, 0 if it doesn't exist, -1 on a bad line. */
int geo_calib_load(const char *path, geo_calib *c);

/* Rows [ymin, ymax) seeing the road from max_dist_mm in to the frame bottom
 * (roi_row_bounds with max_distance). */
void geo_roi_rows(const geo_calib *c, int img_h, double max_dist_mm, int *ymin, int *ymax);

typedef struct {
    float x, y;
} geo_pt;

/* One or more closed outlines: contour k is pts[start[k] .. start[k + 1]). */
typedef struct {
    geo_pt *pts;
    int n, cap;
    int *start;
    int ncont, ccap;
} geo_poly;

void geo_poly_clear(geo_poly *p);
void geo_poly_free(geo_poly *p);
int geo_poly_copy(geo_poly *dst, const geo_poly *src);

/* Outer outlines of the 8-connected components of a w x h bitmap (as
 * findContours RETR_EXTERNAL + CHAIN_APPROX_SIMPLE), in bitmap pixels. */
int geo_trace(const uint8_t *bits, int w, int h, geo_poly *out);

/* Sum of the outlines' areas (contourArea of each). */
double geo_poly_area(const geo_poly *p);

/* True if any point lies within margin of the image border (mask cut off). */
int geo_touches_border(const geo_poly *p, int img_w, int img_h, float margin);

typedef struct {
    int valid;              /* at least 3 points projected onto the road */
    double width_m;         /* max X - min X */
    double length_m;        /* max Y - min Y */
    double area_m2;         /* area of the ground outline */
    double major_m, minor_m, angle_deg;   /* minimum-area rectangle; angle of the long side from straight ahead */
    int n_points, n_rejected;
} geo_meas;

/* measure_polygon(): projects every outline point (image pixels at the
 * calibration's resolution) onto the road, drops points on/above the horizon
 * or beyond max_forward_m (0 = no limit), and measures what's left. If ground
 * is non-NULL it receives the kept ground points (metres). */
int geo_measure(const geo_calib *c, const geo_poly *px, double max_forward_m, geo_meas *m, geo_poly *ground);

#endif /* GEOMETRY_H */
