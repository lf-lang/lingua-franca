/**
 * Implementation of the Matrix abstract data type for use in some benchmarks.
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stddef.h>
#include <stdlib.h>

typedef struct Matrix {
    int n_cols;
    double* content;
} Matrix;

/*
 * Allocate a new Matrix.
 */
Matrix mat_new(size_t m, size_t n) {
    Matrix ret = {.n_cols = n, .content = (double*) malloc(m * n * sizeof(double))};
    return ret;
}

/*
 * Deallocate the given Matrix.
 */
void mat_destroy(Matrix matrix) {
    free(matrix.content);
}

/*
 * Return a pointer to entry (i, j) of the given Matrix.
 */
double* mat_at(Matrix matrix, size_t i, size_t j) {
    return (double*) matrix.content + i * matrix.n_cols + j;
}

/*
 * Allocate a new Matrix filled with zeros.
 */
Matrix mat_zeros(size_t m, size_t n) {
    Matrix ret = mat_new(m, n);
    for (size_t i = 0; i < m; ++i)
        for (size_t j = 0; j < n; ++j)
            *mat_at(ret, i, j) = 0;
    return ret;
}

#endif
