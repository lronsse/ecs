#include "csv_reader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

int read_csv_column(const char *filename,
                    int header_lines,
                    int col_index,
                    double *out_array,
                    size_t max_rows)
{
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("read_csv_column: open failed");
        return -1;
    }

    char linebuf[8192];
    size_t row = 0;

    // Skip header lines
    for (int h = 0; h < header_lines; h++) {
        if (!fgets(linebuf, sizeof(linebuf), fp)) {
            fprintf(stderr, "read_csv_column: unexpected EOF in header\n");
            fclose(fp);
            return -2;
        }
    }

    // Read data lines
    while (row < max_rows && fgets(linebuf, sizeof(linebuf), fp)) {
        // Use strtok_r for thread safety
        char *saveptr = NULL;
        char *token = strtok_r(linebuf, ",", &saveptr);
        int col = 0;

        // Walk to the desired column
        while (token && col < col_index) {
            token = strtok_r(NULL, ",", &saveptr);
            col++;
        }

        if (!token) {
            // line had too few columns—skip or treat as error
            continue;  
        }

        // Convert to double
        errno = 0;
        char *endptr = NULL;
        double val = strtod(token, &endptr);
        if (errno != 0 || endptr == token) {
            // conversion error or no digits
            fprintf(stderr, "read_csv_column: parse error on line %zu\n", row + header_lines + 1);
            continue;
        }

        // Store result
        out_array[row++] = val;
    }

    fclose(fp);
    if (row == 0) return -3;  // no data read

    return (int)row;
}