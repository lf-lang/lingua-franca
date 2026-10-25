#include "../include/ActionSource/R.h"

void handle(r_self_t* self, r_a_t* a) {
    lf_print_error_and_exit("A reaction with no triggers should not be invoked.");
}
