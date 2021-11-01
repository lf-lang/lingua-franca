#include "matrix.h"

#include <stdlib.h>
#include <assert.h>

/*
 * Allocate a new double matrix.
 * @param size_x The number of rows in the matrix.
 * @param size_y The number of columns in the matrix.
 */
matrix_t mat_new_d(size_t size_x, size_t size_y) {
    double* data_d = (double*) calloc(size_x * size_y, sizeof(double));
    if (data_d == NULL) exit(EXIT_FAILURE);
    matrix_t ret = { .size_x = size_x, .size_y = size_y, .data_d = data_d };
    return ret;
}

/*
 * Allocate a new integer matrix.
 * @param size_x The number of rows in the matrix.
 * @param size_y The number of columns in the matrix.
 */
matrix_t mat_new_i(size_t size_x, size_t size_y) {
    int* data_i = (int*) calloc(size_x * size_y, sizeof(int));
    if (data_i == NULL) exit(EXIT_FAILURE);
    matrix_t ret = { .size_x = size_x, .size_y = size_y, .data_i = data_i };
    return ret;
}

/*
 * Deallocate the given double matrix.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_d(matrix_t matrix) {
    free(matrix.data_d);
}

/*
 * Deallocate the given integer matrix.
 * @param matrix The matrix to deallocate.
 */
void mat_destroy_i(matrix_t matrix) {
    free(matrix.data_i);
}

/*
 * Return a pointer to entry (i, j) of the given double matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 */
double* mat_at_d(matrix_t matrix, size_t i, size_t j) {
    assert(i < matrix.size_x);
    assert(j < matrix.size_y);
    return &(matrix.data_d[i * matrix.size_y + j]);
}

/*
 * Return a pointer to entry (i, j) of the given integer matrix.
 * @param matrix The matrix to be accessed.
 * @param i The row to be accessed.
 * @param j The column to be accessed.
 */
int* mat_at_i(matrix_t matrix, size_t i, size_t j) {
    assert(i < matrix.size_x);
    assert(j < matrix.size_y);
    return &(matrix.data_i[i * matrix.size_y + j]);
}
