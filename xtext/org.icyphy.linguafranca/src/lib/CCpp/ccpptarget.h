#ifndef CPP_TARGET_H
#define CPP_TARGET_H

#include <iostream>
#include "core/pqueue.c"
#include "core/reactor.c"

typedef struct {
    int value;
    bool is_present;
    int num_destinations;
} template_port_struct;

template <class T>
void SET(template_port_struct* out, T value)
{
    out->value = value;
    out->is_present = true;
}

#endif // CPP_TARGET_H