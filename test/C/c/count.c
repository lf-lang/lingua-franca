#include "../include/count.h"

void increment(count_self_t* self) {
    self->count++;
}

void done(count_self_t* self) {
    if (self->count > 10) lf_request_stop();
}
