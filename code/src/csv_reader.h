#ifndef CSV_READER_H
#define CSV_READER_H

#include <stddef.h>
int read_csv_column(const char *filename,
                    int header_lines,
                    int col_index,
                    double *out_array,
                    size_t max_rows);

#endif 