#include "matrix.h"

#include <stdlib.h>
#include <assert.h>

/*
 * Allocate a new double matrix containing only zeros.
 * @param size_x The number of rows in the matrix.
 * @param size_y The number of columns in the matrix.
 * @return The new matrix.
 */
matrix_t mat_new_d(size_t size_x, size_t size_y) {
    double* data_d = (double*) calloc(size_x * size_y, sizeof(double));
    if (data_d == NULL) exit(EXIT_FAILURE);
    return (matrix_t) { .size_x = size_x, .size_y = size_y, .type = MAT_DOUBLE, .data_d = data_d };
}

/*
 * Allocate a new integer matrix containing only zeros.
 * @param size_x The number of rows in the matrix.
 * @param size_y The number of columns in the matrix.
 * @return The new matrix.
 */
matrix_t mat_new_i(size_t size_x, size_t size_y) {
    int* data_i = (int*) calloc(size_x * size_y, sizeof(int));
    if (data_i == NULL) exit(EXIT_FAILURE);
    return (matrix_t) { .size_x = size_x, .size_y = size_y, .type = MAT_INT, .data_i = data_i };
}

/*
 * Deallocate the given double matrix.
 * Will raise an assert error if the matrix is not a double type.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_d(matrix_t matrix) {
    assert(matrix.type == MAT_DOUBLE);
    free(matrix.data_d);
}

/*
 * Deallocate the given integer matrix.
 * Will raise an assert error if the matrix is not a double type.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_i(matrix_t matrix) {
    assert(matrix.type == MAT_INT);
    free(matrix.data_i);
}

/*
 * Return a pointer to entry (i, j) of the given double matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 * @return A pointer to the requested matrix entry, or NULL if 'matrix'
 *  is of the wrong type.
 */
double* mat_at_d(matrix_t matrix, size_t i, size_t j) {
    assert(i < matrix.size_x);
    assert(j < matrix.size_y);
    if(matrix.type != MAT_DOUBLE) {
        return NULL;
    }
    return &(matrix.data_d[i * matrix.size_y + j]);
}

/*
 * Return a pointer to entry (i, j) of the given integer matrix. * 
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 * @return A pointer to the requested matrix entry, or NULL if 'matrix'
 *  is of the wrong type.
 */
int* mat_at_i(matrix_t matrix, size_t i, size_t j) {
    assert(i < matrix.size_x);
    assert(j < matrix.size_y);
    if(matrix.type != MAT_INT) {
        return NULL;
    }
    return &(matrix.data_i[i * matrix.size_y + j]);
}
