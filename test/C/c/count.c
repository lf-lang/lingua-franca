#include <stdio.h>
#include "../include/Count.h"

void increment(Count_self_t* self) {
    printf("in increment, count=%d\n", self->count);
    self->count++;
}

void check_done(Count_self_t* self) {
    printf("in done, count=%d\n", self->count);
    if (self->count > 10) {
        printf("%s", "requesting stop\n");
        lf_request_stop();
    }
}
