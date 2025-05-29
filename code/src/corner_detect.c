#include "corner_detect.h"
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>

// Convert polar to cartesian
void polar_to_cartesian(const Point *p, Point *out) {
    out->theta = p->theta;
    out->r     = p->r;
    out->x     = p->r * cos(p->theta);
    out->y     = p->r * sin(p->theta);
}

void polar_to_cartesian_array(
    const double ranges[],
    double x[], double y[],
    size_t n,
    double start_angle,
    double angle_increment
) {
    for (size_t i = 0; i < n; i++) {
        double theta = start_angle + i * angle_increment;
        x[i] = ranges[i] * cos(theta);
        y[i] = ranges[i] * sin(theta);
    }
}


// filter points by radial value
Point *apply_radial_mask(const Point *polar_points, size_t n,
                         double min_r, double max_r,
                         size_t *out_n)
{
    Point *tmp = malloc(n * sizeof(Point));
    size_t cnt = 0;
    for (size_t i = 0; i < n; i++) {
        double r = polar_points[i].r;
        if (r >= min_r && r <= max_r) {
            tmp[cnt++] = polar_points[i];
        }
    }
    Point *res = malloc(cnt * sizeof(Point));
    memcpy(res, tmp, cnt * sizeof(Point));
    free(tmp);
    *out_n = cnt;
    return res;
}

// Euclidean norm
static double norm2(double x, double y) {
    return sqrt(x*x + y*y);
}

// Dot product
static double dot2(double ax, double ay, double bx, double by) {
    return ax*bx + ay*by;
}

// Calculate angle between polar vectors
double calculate_angle_between_polar_vectors(const Point *p1,
                                             const Point *p2,
                                             const Point *p3)
{
    Point c1, c2, c3;
    polar_to_cartesian(p1, &c1);
    polar_to_cartesian(p2, &c2);
    polar_to_cartesian(p3, &c3);

    double v1x = c2.x - c1.x;
    double v1y = c2.y - c1.y;
    double v2x = c3.x - c2.x;
    double v2y = c3.y - c2.y;

    double n1 = norm2(v1x, v1y), n2 = norm2(v2x, v2y);
    if (n1 < 1e-9 || n2 < 1e-9) return 0.0;

    double ux = v1x/n1, uy = v1y/n1;
    double vx = v2x/n2, vy = v2y/n2;
    double cos_t = dot2(ux, uy, vx, vy);
    if      (cos_t >  1.0) cos_t =  1.0;
    else if (cos_t < -1.0) cos_t = -1.0;
    return acos(cos_t);
}


Point *detect_corners_polar(const Point *polar_points, size_t n,
                            double angle_lower, double angle_upper,
                            size_t span, double dist_thresh,
                            size_t *out_n)
{
    Point *corners = malloc(n * sizeof(Point));
    size_t cnt = 0;
    int prev_corner = 0;

    for (size_t i = span; i + span < n; i++) {
        // distances to neighbors
        Point c_prev, c_cur, c_next;
        polar_to_cartesian(&polar_points[i-span], &c_prev);
        polar_to_cartesian(&polar_points[i],      &c_cur);
        polar_to_cartesian(&polar_points[i+span], &c_next);
        double d1 = norm2(c_cur.x - c_prev.x, c_cur.y - c_prev.y);
        double d2 = norm2(c_next.x - c_cur.x, c_next.y - c_cur.y);
        if (d1 > dist_thresh || d2 > dist_thresh) {
            // prev_corner is NOT changed here, consistent with Python version
            continue;
        }
        double ang = calculate_angle_between_polar_vectors(
                         &polar_points[i-span],
                         &polar_points[i],
                         &polar_points[i+span]);
        if (ang > angle_lower && ang < angle_upper) {
            if (prev_corner) {
                corners[cnt++] = polar_points[i];
            }
            prev_corner = 1;
        } else {
            prev_corner = 0;
        }
    }

    *out_n = cnt;
    // Optimize memory allocation: shrink the array to the actual number of corners.
    if (cnt == 0) {
        // If no corners were found, free the initial allocation and return NULL.
        free(corners);
        return NULL;
    }
    
    Point *resized_corners = realloc(corners, cnt * sizeof(Point));
    if (resized_corners == NULL) {
        // Realloc failed (unlikely when shrinking, but handle defensively).
        // Return the original oversized 'corners' array to avoid data loss.
        // *out_n is already set to cnt.
        return corners;
    }
    
    // Realloc succeeded, return the correctly sized array.
    return resized_corners;
}

// Compare by theta
static int cmp_theta(const void *a, const void *b) {
    double ta = ((const Point*)a)->theta;
    double tb = ((const Point*)b)->theta;
    if (ta < tb) return -1;
    if (ta > tb) return  1;
    return 0;
}

Point *reduce_corner_blobs_by_angle(const Point *corners, size_t n,
                                    double angle_target,
                                    double angle_threshold,
                                    size_t *out_n)
{
    if (n == 0) {
        *out_n = 0;
        return NULL;
    }
    // copy & sort
    Point *sorted = malloc(n * sizeof(Point));
    memcpy(sorted, corners, n * sizeof(Point));
    qsort(sorted, n, sizeof(Point), cmp_theta);

    Point *result = malloc(n * sizeof(Point));
    size_t res_cnt = 0;

    // group consecutive within threshold
    size_t start = 0;
    for (size_t i = 1; i <= n; i++) {
        if (i == n || fabs(sorted[i].theta - sorted[i-1].theta) > angle_threshold) {
            // group [start..i-1]
            // pick one closest to angle_target
            double best_diff = DBL_MAX;
            size_t best_j = start;
            for (size_t j = start; j < i; j++) {
                double diff = fabs(sorted[j].theta - angle_target);
                if (diff < best_diff) {
                    best_diff = diff;
                    best_j = j;
                }
            }
            result[res_cnt++] = sorted[best_j];
            start = i;
        }
    }

    free(sorted);
    *out_n = res_cnt;
    return result;
}


Point *remove_close_corners_by_distance(Point *corners, size_t n,
                                        double min_dist,
                                        size_t *out_n)
{
    if (n <= 1) {
        *out_n = n;
        return corners;  // no change
    }
    // convert to Cartesian
    Point *cart = malloc(n * sizeof(Point));
    for (size_t i = 0; i < n; i++) {
        polar_to_cartesian(&corners[i], &cart[i]);
    }
    int *keep = malloc(n * sizeof(int));
    for (size_t i = 0; i < n; i++) keep[i] = 1;

    for (size_t i = 0; i < n; i++) {
        if (!keep[i]) continue;
        for (size_t j = i+1; j < n; j++) {
            if (!keep[j]) continue;
            double d = norm2(cart[i].x - cart[j].x,
                             cart[i].y - cart[j].y);
            if (d < min_dist) {
                // merge: average polar coords
                corners[i].theta = 0.5*(corners[i].theta + corners[j].theta);
                corners[i].r     = 0.5*(corners[i].r     + corners[j].r);
                keep[j] = 0;
            }
        }
    }

    // compact
    Point *res = malloc(n * sizeof(Point));
    size_t cnt = 0;
    for (size_t i = 0; i < n; i++) {
        if (keep[i]) res[cnt++] = corners[i];
    }

    free(cart);
    free(keep);
    *out_n = cnt;
    return res;
}
