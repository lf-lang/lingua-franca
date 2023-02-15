#include "../include/Print.h"
#include "../include/Check.h"
#include "../include/api/set.h"

void send(Print_self_t* self, print_out_t* out) {
    lf_set(out, 42);
}

void receive(Check_self_t* self, check_in_t* in) {
    printf("Received: %d\n", in->value);
    if (in->value != self->expected) {
        printf("ERROR: Expected value to be %d.\n", self->expected);
        exit(1);
    }
}
