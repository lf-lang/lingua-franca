// An externally defined reaction that uses lf_async_ext() with a logical action.
#include "../include/AsyncExtLogical/Blocker.h"

static int slow(int milliseconds) {
    lf_sleep(MSEC(milliseconds));
    return milliseconds;
}

void block(blocker_self_t* self, blocker_done_t* done) {
    environment_t* env = self->base.environment;
    interval_t start = lf_time_logical_elapsed(env);
    lf_print("Calling lf_async_ext() at " PRINTF_TIME " (physical " PRINTF_TIME ")", start,
             lf_time_physical_elapsed());
    self->early = lf_async_ext(self, done, slow, 50);
    interval_t first = lf_time_logical_elapsed(env);
    lf_print("First resume at " PRINTF_TIME " (physical " PRINTF_TIME ")", first, lf_time_physical_elapsed());
    if (first != MSEC(200)) {
        lf_print_error_and_exit(
            "Expected the first lf_async_ext() to resume at logical time 200 ms. Got " PRINTF_TIME ".", first);
    }
    self->late = lf_async_ext(self, done, slow, 350);
    interval_t second = lf_time_logical_elapsed(env);
    interval_t physical = lf_time_physical_elapsed();
    lf_print("Second resume at " PRINTF_TIME " (physical " PRINTF_TIME ")", second, physical);
    if (second != MSEC(400)) {
        lf_print_error_and_exit(
            "Expected the second lf_async_ext() to resume at logical time 400 ms. Got " PRINTF_TIME ".", second);
    }
    // Resuming at 400 ms logical while the function takes 350 ms from the 200 ms tag means the
    // function, not merely the delay, determined when the call returned.
    if (physical < MSEC(500)) {
        lf_print_error_and_exit(
            "Expected the slow function to finish after the second resume. Physical time " PRINTF_TIME ".",
            physical);
    }
}
