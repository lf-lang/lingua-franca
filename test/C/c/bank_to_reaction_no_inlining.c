#include "../include/BankToReactionNoInlining/BankToReactionNoInlining.h"

void check(banktoreactionnoinlining_self_t* self, count_out_t** out) {
    for (int i = 0; i < 2; i++) {
        lf_print("Received %d.", out[i]->value);
        if (self->count != out[i]->value) {
            lf_print_error_and_exit("Expected %d but got %d.", self->count, out[i]->value);
        }
    }
    self->count++;
}
