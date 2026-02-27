#include <stdio.h>

#include "../include/IntPrint/Print.h"
#include "../include/IntPrint/Check.h"
#include "../include/api/reaction_macros.h"

void sender(print_self_t* self, print_out_t* out) {
    lf_set(out, 42);
}

void receiver(check_self_t* self, check_in_t* in) {
    printf("Received: %d\n", in->value);
    if (in->value != self->expected) {
        printf("ERROR: Expected value to be %d.\n", self->expected);
        exit(1);
    }
}
