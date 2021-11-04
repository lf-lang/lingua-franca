/**
 * Implementation of the matrix data type for use in some benchmarks.
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stddef.h>
#include <assert.h>

/**
 * Supported types for this matrix implementation.
 */
typedef enum {
    MAT_DOUBLE,
    MAT_INT
} _matrix_types;

/*
 * A matrix containing double or integer entries.
 */
typedef struct matrix_t {
    int size_x;
    int size_y;
    _matrix_types type;
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
 * Will raise an assert error if the matrix is not a double type.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_d(matrix_t matrix);

/*
 * Deallocate the given integer matrix.
 * Will raise an assert error if the matrix is not an integer type.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_i(matrix_t matrix);

/*
 * Return a pointer to entry (i, j) of the given double matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 * @return A pointer to the requested matrix entry, or NULL if 'matrix'
 *  is of the wrong type.
 */
static inline double* mat_at_d(matrix_t matrix, size_t i, size_t j) {
    assert(i < matrix.size_x);
    assert(j < matrix.size_y);
    assert(matrix.type == MAT_DOUBLE);
    return &(matrix.data_d[i * matrix.size_y + j]);
}

/*
 * Return a pointer to entry (i, j) of the given integer matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 * @return A pointer to the requested matrix entry, or NULL if 'matrix'
 *  is of the wrong type.
 */
static inline int* mat_at_i(matrix_t matrix, size_t i, size_t j) {
    assert(i < matrix.size_x);
    assert(j < matrix.size_y);
    assert(matrix.type == MAT_INT);
    return &(matrix.data_i[i * matrix.size_y + j]);
}

#endif
