#ifndef ARRAY_H
#define ARRAY_H

#include "stdlib.h"

typedef struct int_array_t {
    int* data;
    size_t length;
} int_array_t;

int_array_t* int_array_constructor(size_t length) {
    int_array_t* val = (int_array_t*) malloc(sizeof(int_array_t));
    val->data = (int*) calloc(length, sizeof(int));
    val->length = length;
    return val;
}

void int_array_destructor(void* arr) {
    free(((int_array_t*) arr)->data);
    free(arr);
}

#endif