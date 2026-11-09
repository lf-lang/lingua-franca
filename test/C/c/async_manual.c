// Externally defined reactions that use lf_async_ext() and lf_async_resume().
#include "../include/AsyncExtManual/Blocker.h"

// To use reaction macros, the self pointer argument must be named "self".
#include "../include/api/reaction_macros.h"

static int slow(int iterations) {
    int result = 0;
    for (int j = 0; j < iterations; j++) {
        result++;
        lf_sleep(MSEC(500));
    }
    return result;
}

void resume(blocker_self_t* self, blocker_awake_t* awake) {
    // Unblock the lf_async_ext() function.
    lf_async_resume(awake);
}

void block(blocker_self_t* self, blocker_blocker_t* blocker, blocker_awake_t* awake) {
    environment_t* env = self->base.environment;
    lf_print("Starting Blocker %d", self->id);
    // These outputs must be delivered at this tag, before lf_async_ext() lets logical time advance.
    lf_set(blocker, 1);
    lf_print("Blocker blocking.");
    int result = lf_async(awake, slow, 1);
    lf_print("Blocker resuming %d: Result: %d at logical time " PRINTF_TIME, self->id, result,
             lf_time_logical_elapsed());
    if (lf_time_logical_elapsed() < MSEC(500)) {
        lf_print_error_and_exit("Expected lf_async() to return at a logical time of at least 500 ms.");
    }
    lf_set(blocker, 2);
    result += lf_async(awake, slow, 1);
    lf_print("Blocker %d: Result: %d at logical time " PRINTF_TIME, self->id, result,
             lf_time_logical_elapsed());
    if (lf_time_logical_elapsed() < SEC(1)) {
        lf_print_error_and_exit("Expected second lf_async() to return at a logical time of at least 1 s.");
    }
    lf_set(blocker, 3);
    self->result = result;
}

void check(blocker_self_t* self) {
    if (self->result != 2) {
        lf_print_error_and_exit("Expected result 2 from lf_async(). Got %d.", self->result);
    }
}
