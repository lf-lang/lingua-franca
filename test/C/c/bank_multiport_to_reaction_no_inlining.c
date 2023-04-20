#include "../include/BankMultiportToReactionNoInlining/BankMultiportToReactionNoInlining.h"

void check(bankmultiporttoreactionnoinlining_self_t* self, doublecount_out_t*** out) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (out[i][j]->is_present) {
                lf_print("Received %d.", out[i][j]->value);
                if (self->count != out[i][j]->value) {
                    lf_print_error_and_exit("Expected %d.", self->count);
                }
                self->received = true;
            }
        }
    }
    self->count++;
}
