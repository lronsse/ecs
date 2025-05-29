#include "csv_reader.h"
#include "corner_detect.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 

static const char *CSV_FILE = "lidarData.csv"; // lidar scan filename

#define min_radius 280
#define max_radius 3000
#define angle_range_degrees 30
#define span 6
#define match_threshold_degrees 0.5
#define min_corner_distance_mm 50
#define memory_length 5

#define MAX_ROWS        10000
#define HEADER_LINES    3
#define COL_INDEX       1 // Which scan column we analyse

// Radial mask limits (in meters)
#define MIN_RANGE 0.1
#define MAX_RANGE 30.0

// Corner detection parameters
#define ANGLE_LOWER     (30.0  * M_PI / 180.0)    // 30°
#define ANGLE_UPPER     (150.0 * M_PI / 180.0)    // 150°
#define SPAN            3
#define DIST_THRESH     0.05                     // meters

// Corner blob reduction
#define ANGLE_TARGET    (M_PI / 2.0)             // 90°
#define ANGLE_THRESH    (5.0  * M_PI / 180.0)    // ±5°

// define pi value if not defined by math.h
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


int main(void) {
    printf("Reading scan from %s\n", CSV_FILE);


    double *ranges = malloc(sizeof(double) * MAX_ROWS);
    if (!ranges) {perror("malloc ranges");return EXIT_FAILURE;}

    // Read data from csv
    int n_points = read_csv_column(
        CSV_FILE,
        HEADER_LINES,
        COL_INDEX,
        ranges,
        MAX_ROWS
    );
    if (n_points < 0) {
        fprintf(stderr, "Error reading CSV: %d\n", n_points);
        free(ranges);
        return EXIT_FAILURE;
    }

    // Build polar array for corner detection
    Point *polar = malloc(sizeof *polar * n_points);
    if (!polar) { perror("malloc polar"); free(ranges); return EXIT_FAILURE; }
    double start_rad = ANGLE_LOWER * M_PI/180.0;
    double end_rad   = ANGLE_UPPER   * M_PI/180.0;
    double angle_inc = (end_rad - start_rad) / (n_points - 1);
    for (int i = 0; i < n_points; i++) {
        polar[i].r     = ranges[i];
        polar[i].theta = start_rad + i * angle_inc;
    }

    // Export cartesian coordinates for python visualisation
    double *x = malloc(sizeof *x * n_points);
    double *y = malloc(sizeof *y * n_points);
    if (!x || !y) { perror("malloc xy"); free(ranges); free(polar); return EXIT_FAILURE; }
    polar_to_cartesian_array(
        ranges, x, y, n_points,
        start_rad, angle_inc
    );
    FILE* fout1 = fopen("lidarDataCartesian.csv", "w");
    if (!fout1) {
        perror("Failed to open outputCartesian.csv");
        return EXIT_FAILURE;
    }
    fprintf(fout1, "x,y\n");
    for (int i = 0; i < n_points; i++) {
        fprintf(fout1, "%f,%f\n", x[i], y[i]);
    }
    fclose(fout1);



    // Radial mask
    size_t n_masked;
    Point *masked = apply_radial_mask(
        polar, (size_t)n_points,
        MIN_RANGE, MAX_RANGE,
        &n_masked
    );

    // Detect corners
    size_t n_raw;
    Point *raw_corners = detect_corners_polar(
        masked, n_masked,
        ANGLE_LOWER, ANGLE_UPPER,
        SPAN, DIST_THRESH,
        &n_raw
    );

    // Reduce blobs by angle
    size_t n_reduced;
    Point *reduced = reduce_corner_blobs_by_angle(
        raw_corners, n_raw,
        ANGLE_TARGET, ANGLE_THRESH,
        &n_reduced
    );

    // Remove close-by duplicates
    size_t n_final;
    Point *final_corners = remove_close_corners_by_distance(
        reduced, n_reduced,
        DIST_THRESH,    // reuse dist threshold as min corner separation
        &n_final
    );

    // Output final corners (in Cartesian form)
    printf("Detected %zu corners:\n", n_final);
    for (size_t i = 0; i < n_final; i++) {
        Point c;
        polar_to_cartesian(&final_corners[i], &c);
        printf("  corner %2zu: θ=%7.3f°,  r=%7.3f m  →  x=%7.3f, y=%7.3f\n",
               i,
               final_corners[i].theta * 180.0/M_PI,
               final_corners[i].r,
               c.x, c.y);
    }

    // export cartesian coordinates of corners for visualisation
    FILE *fout2 = fopen("outputCorners.csv", "w");
    if (!fout2) {
        perror("Failed to open outputCorners.csv");
        // Clean up and exit or continue as appropriate
    } else {
        // CSV header
        fprintf(fout2, "x,y\n");
        // Loop over your final corner array:
        for (size_t i = 0; i < n_final; i++) {
            // Convert each polar corner to Cartesian
            Point c;
            polar_to_cartesian(&final_corners[i], &c);
            // Write x and y
            fprintf(fout2, "%f,%f\n", c.x, c.y);
        }
        fclose(fout2);
    }

    free(ranges);
    free(polar);
    free(masked);
    free(raw_corners);
    free(reduced);
    free(final_corners);
    free(x);
    free(y);

    //system("python3 ../plotting.py");
    return EXIT_SUCCESS;
}