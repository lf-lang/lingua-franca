/**
 * Implementation of the Matrix abstract data type for use in some benchmarks.
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stddef.h>

/*
 * A matrix containing double or integer entries.
 */
typedef struct matrix_t {
    int size_x;
    int size_y;
    union {
        double* data_d;
        int* data_i;
    };
} matrix_t;

/*
 * Allocate a new double matrix containing only zeros.
 * @param size_x The number of rows in the matrix.
 * @param size_y The number of columns in the matrix.
 * @return The new matrix.
 */
matrix_t mat_new_d(size_t size_x, size_t size_y);

/*
 * Allocate a new integer matrix containing only zeros.
 * @param size_x The number of rows in the matrix.
 * @param size_y The number of columns in the matrix.
 * @return The new matrix.
 */
matrix_t mat_new_i(size_t size_x, size_t size_y);

/*
 * Deallocate the given double matrix.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_d(matrix_t matrix);

/*
 * Deallocate the given integer matrix.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_i(matrix_t matrix);

/*
 * Return a pointer to entry (i, j) of the given double matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 * @return A pointer to the requested matrix entry.
 */
double* mat_at_d(matrix_t matrix, size_t i, size_t j);

/*
 * Return a pointer to entry (i, j) of the given integer matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 * @return A pointer to the requested matrix entry.
 */
int* mat_at_i(matrix_t matrix, size_t i, size_t j);

#endif
