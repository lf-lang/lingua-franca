#include "../include/MultiportToReactionNoInlining/MultiportToReactionNoInlining.h"

void check(multiporttoreactionnoinlining_self_t* self, source_out_t** out) {
    int sum = 0;
    for (int i = 0; i < 4; i++) {
        if (out[i]->is_present) sum += out[i]->value;
    }
    printf("Sum of received: %d.\n", sum);
    if (sum != self->s) {
        printf("ERROR: Expected %d.\n", self->s);
        exit(1);
    }
    self->s += 16;
}
