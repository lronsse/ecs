#ifndef CORNER_DETECT_H
#define CORNER_DETECT_H

#include <stddef.h>

// A point in polar or Cartesian form
typedef struct {
    double theta;   // angle in radians
    double r;       // radius (for polar)
    double x, y;    // cartesian coords
} Point;

// Convert a single polar point to Cartesian (fills x,y)
void polar_to_cartesian(const Point *p, Point *out);

// Convert arrays of ranges+thetas → x,y arrays
void polar_to_cartesian_array(
    const double ranges[],
    double x[], double y[],
    size_t n,
    double start_angle,
    double angle_increment
);

// Filter polar_points by radius.
// Returns a newly malloc'd array of Points (polar form).
// Important: free() the return value
Point *apply_radial_mask(const Point *polar_points, size_t n,
                         double min_r, double max_r,
                         size_t *out_n);

// Compute the angle between vectors (p1→p2) and (p2→p3) in radians.
double calculate_angle_between_polar_vectors(const Point *p1,
                                             const Point *p2,
                                             const Point *p3);

// Detect corners by sliding window in polar_points[0..n-1].
// angle_lower < angle < angle_upper (radians),
// skip windows of size `span`, and discard if distance to neighbors > dist_thresh.
// Returns malloc'd array of Points (polar form).
Point *detect_corners_polar(const Point *polar_points, size_t n,
                            double angle_lower, double angle_upper,
                            size_t span, double dist_thresh,
                            size_t *out_n);

// Collapse nearby detected corners by angle: group within angle_thresh of 90°,
// choose the one closest to angle_target. Returns malloc'd array.
Point *reduce_corner_blobs_by_angle(const Point *corners, size_t n,
                                    double angle_target,
                                    double angle_threshold,
                                    size_t *out_n);

// Remove corners closer than min_dist (in same units as x,y) by merging pairs.
Point *remove_close_corners_by_distance(Point *corners, size_t n,
                                        double min_dist,
                                        size_t *out_n);

#endif // CORNER_DETECT_H
