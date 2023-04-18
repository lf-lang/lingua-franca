#include <stdio.h>
#include "../include/CountHierarchy/CountHierarchy.h"

void increment(counthierarchy_self_t* self, timer_out_t* out) {
    printf("in increment, count=%d\n", self->count);
    self->count++;
}

void check_done(counthierarchy_self_t* self, timer_out_t* out) {
    printf("in done, count=%d\n", self->count);
    if (self->count > 10) {
        printf("%s", "requesting stop\n");
        lf_request_stop();
    }
}
