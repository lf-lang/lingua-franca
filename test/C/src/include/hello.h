/**
 * Simple struct for testing.
 * This provides a constructor and a copy constructor.
 * A destructor is not needed because the default destructor works
 * (under the assumption that the name field is a static string).
 * @author Hou Seng (Steven) Wong
 */

#ifndef HELLO_H
#define HELLO_H

#include "stdlib.h"

typedef struct hello_t {
    char* name;
    int value;
 } hello_t;

typedef int* int_pointer;

static hello_t* hello_constructor(char* name, int value) {
    hello_t* val = (hello_t*) malloc(sizeof(hello_t));
    val->name = name;
    val->value = value;
    return val;
}

static hello_t* hello_copy_constructor(hello_t v) {
    hello_t* val = (hello_t*) malloc(sizeof(hello_t));
    val->name = v.name;
    val->value = v.value;
    return val;
}

#endif
