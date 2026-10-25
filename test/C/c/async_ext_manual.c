// Externally defined reactions that use lf_async_ext() and lf_async_resume().
#include "../include/AsyncExtManual/Blocker.h"

static int slow(int iterations) {
    int result = 0;
    for (int j = 0; j < iterations; j++) {
        result++;
        lf_sleep(MSEC(10));
    }
    return result;
}

void resume(blocker_self_t* self, blocker_awake_t* awake) {
    // Unblock the lf_async_ext() function.
    lf_async_resume(awake);
}

void block(blocker_self_t* self, blocker_awake_t* awake) {
    environment_t* env = self->base.environment;
    lf_print("Starting Blocker %d", self->id);
    int result = lf_async_ext(self, awake, slow, 50);
    lf_print("Blocker %d: Result: %d at time " PRINTF_TIME, self->id, result, lf_time_logical_elapsed(env));
    if (lf_time_logical_elapsed(env) < MSEC(500)) {
        lf_print_error_and_exit("Expected lf_async_ext() to return at a logical time of at least 500 ms.");
    }
    result += lf_async_ext(self, awake, slow, 20);
    lf_print("Blocker %d: Result: %d at time " PRINTF_TIME, self->id, result, lf_time_logical_elapsed(env));
    if (lf_time_logical_elapsed(env) < MSEC(700)) {
        lf_print_error_and_exit("Expected second lf_async_ext() to return at a logical time of at least 700 ms.");
    }
    self->result = result;
}

void check(blocker_self_t* self) {
    if (self->result != 70) {
        lf_print_error_and_exit("Expected result 70 from lf_async_ext(). Got %d.", self->result);
    }
}
