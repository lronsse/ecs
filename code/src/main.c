#include "csv_reader.h"
#include "corner_detect.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 

// define pi value if not defined by math.h
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)

// Configuration values from config.json (or derived)
// Assuming CSV 'ranges' data is in millimeters (mm) as per problem description.

// Radial mask limits (in mm)
#define CONFIG_MIN_RANGE_MM 280.0       // from json: min_radius
#define CONFIG_MAX_RANGE_MM 3000.0      // from json: max_radius

// Corner detection parameters
#define CONFIG_ANGLE_TARGET_DEGREES 90.0    // from json: angle_target_degrees
#define CONFIG_ANGLE_RANGE_DEGREES 30.0     // from json: angle_range_degrees
#define CONFIG_ANGLE_LOWER_RAD DEG2RAD(CONFIG_ANGLE_TARGET_DEGREES - CONFIG_ANGLE_RANGE_DEGREES) // 60°
#define CONFIG_ANGLE_UPPER_RAD DEG2RAD(CONFIG_ANGLE_TARGET_DEGREES + CONFIG_ANGLE_RANGE_DEGREES) // 120°

#define CONFIG_DETECT_SPAN 6                // from json: span
#define CONFIG_DETECT_DIST_THRESH_MM (CONFIG_DETECT_SPAN * 30.0) // 180mm, as per spec (span * 30mm)

// Corner blob reduction parameters
// ANGLE_TARGET_DEGREES (90.0) is used here, which is from json: angle_target_degrees
#define CONFIG_BLOB_ANGLE_TARGET_RAD DEG2RAD(CONFIG_ANGLE_TARGET_DEGREES) 
#define CONFIG_BLOB_ANGLE_THRESH_RAD DEG2RAD(0.5) // from json: angle_grouping_threshold_degrees

// Final corner filtering parameter
#define CONFIG_FINAL_MIN_CORNER_DIST_MM 50.0 // from json: min_corner_distance_mm

// Lidar sensor characteristics
#define LIDAR_SCAN_START_DEGREES 0.0
#define LIDAR_SCAN_END_DEGREES 240.0

static const char *CSV_FILE = "../src/LidarData.csv"; // lidar scan filename (path relative to build dir)

#define memory_length 5 // Unrelated to this task, kept.

#define MAX_ROWS        10000
#define HEADER_LINES    3
#define COL_INDEX       1 // Which scan column we analyse

// Old defines related to configurable parameters have been removed.
// New CONFIG_ prefixed defines are used instead.
// The M_PI guard was moved to the top in the previous step.


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
    // Calculate angles based on Lidar's actual scanning range
    double start_rad = DEG2RAD(LIDAR_SCAN_START_DEGREES);
    double end_rad   = DEG2RAD(LIDAR_SCAN_END_DEGREES);
    double angle_inc = 0.0;
    if (n_points > 1) {
        angle_inc = (end_rad - start_rad) / (n_points - 1);
    } else {
        // Handle n_points = 0 or 1 case, though typically n_points will be large.
        // If n_points = 1, angle_inc is not really used in the loop but set to 0 to be safe.
        // If n_points = 0, loop won't run.
        angle_inc = 0.0; 
    }

    for (int i = 0; i < n_points; i++) {
        polar[i].r     = ranges[i];
        polar[i].theta = start_rad + i * angle_inc;
    }

    // First Radial mask applied to all lidar points
    size_t n_masked;
    Point *masked = apply_radial_mask(
        polar, (size_t)n_points,
        CONFIG_MIN_RANGE_MM, CONFIG_MAX_RANGE_MM,
        &n_masked
    );
    // 'polar' array is processed into 'masked'. Free 'polar'.
    // 'ranges' is still needed if other processing depends on raw ranges, otherwise it could be freed too.
    // For now, only freeing polar as its direct data is in 'masked'.
    free(polar); 
    polar = NULL; // Avoid dangling pointer

    // Export cartesian coordinates of *masked* Lidar points for Python visualisation
    FILE* fout1 = fopen("lidarDataCartesian.csv", "w");
    if (!fout1) {
        perror("Failed to open lidarDataCartesian.csv"); // Corrected file name in error
        // Consider freeing already allocated memory before exit
        free(ranges);
        if (masked) free(masked); // masked might be NULL if n_masked is 0
        return EXIT_FAILURE;
    }
    fprintf(fout1, "x,y\n"); // CSV Header
    for (size_t i = 0; i < n_masked; i++) {
        Point cart_point;
        polar_to_cartesian(&masked[i], &cart_point); // Convert each masked polar point
        fprintf(fout1, "%f,%f\n", cart_point.x, cart_point.y);
    }
    fclose(fout1);
    // The x and y arrays are no longer needed for this CSV.

    // Detect corners
    size_t n_raw;
    Point *raw_corners = detect_corners_polar(
        masked, n_masked,
        CONFIG_ANGLE_LOWER_RAD, CONFIG_ANGLE_UPPER_RAD,
        CONFIG_DETECT_SPAN, CONFIG_DETECT_DIST_THRESH_MM,
        &n_raw
    );

    // Add second call to apply_radial_mask on raw_corners.
    // apply_radial_mask always returns a new block of memory or NULL if count is 0.
    // The original raw_corners (from detect_corners_polar) must be freed.
    Point* masked_raw_corners = apply_radial_mask(
        raw_corners, n_raw, // Pass current raw_corners and its count
        CONFIG_MIN_RANGE_MM, CONFIG_MAX_RANGE_MM,
        &n_raw // n_raw is updated with the count of points after masking
    );
    free(raw_corners); // Free the original array returned by detect_corners_polar
    raw_corners = masked_raw_corners; // Update raw_corners to point to the new masked array

    // Reduce blobs by angle using the (potentially new) raw_corners and n_raw
    size_t n_reduced;
    Point *reduced = reduce_corner_blobs_by_angle(
        raw_corners, n_raw, // Use the updated raw_corners and n_raw
        CONFIG_BLOB_ANGLE_TARGET_RAD, CONFIG_BLOB_ANGLE_THRESH_RAD,
        &n_reduced
    );

    // Remove close-by duplicates
    size_t n_final;
    Point *final_corners = remove_close_corners_by_distance(
        reduced, n_reduced,
        CONFIG_FINAL_MIN_CORNER_DIST_MM,
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
    free(polar); // polar is already freed and set to NULL above. If block not entered, it's freed here.
                 // Defensive free: if polar is NULL, free(NULL) is a no-op.
    free(masked);
    free(raw_corners);
    free(reduced);
    free(final_corners);
    // free(x) and free(y) are removed as x and y arrays are no longer used.

    //system("python3 ../plotting.py");
    return EXIT_SUCCESS;
}