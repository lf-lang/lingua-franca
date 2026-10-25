// Externally defined reactions that use lf_await_ext() and lf_await_resume_ext().
#include "../include/AwaitExt/Requester.h"
#include "../include/AwaitExt/Forgotten.h"

void on_response(requester_self_t* self, requester_response_t* response) {
    environment_t* env = self->base.environment;
    self->responses++;
    lf_print("Response %d received at " PRINTF_TIME ".", response->value, lf_time_logical_elapsed(env));
    lf_await_resume_ext(self, response);
}

void wait_for_response(requester_self_t* self, requester_response_t* response) {
    environment_t* env = self->base.environment;
    tag_t before = lf_tag(env);
    lf_await_ext(self, response);
    tag_t after = lf_tag(env);
    lf_print("Resumed at " PRINTF_TIME " (microstep %u) with response %d.", lf_time_logical_elapsed(env),
             after.microstep, response->value);
    if (after.time - before.time != MSEC(100) || after.microstep != 0) {
        lf_print_error_and_exit("Expected lf_await_ext() to resume exactly 100 ms after " PRINTF_TIME
                                ". Resumed at " PRINTF_TIME " (microstep %u).",
                                before.time - lf_time_start(), after.time - lf_time_start(), after.microstep);
    }
    // The response reaction ran before this reaction resumed, and the response is present.
    if (self->responses != 1 || !response->is_present || response->value != 1) {
        lf_print_error_and_exit("Expected response 1 to be present before resuming. responses=%d.",
                                self->responses);
    }
    self->completed = true;
}

void late_wait(requester_self_t* self, requester_response_t* response) {
    environment_t* env = self->base.environment;
    lf_print("Late reaction awaiting at " PRINTF_TIME ".", lf_time_logical_elapsed(env));
    lf_await_ext(self, response);
    self->late_resumed_at = lf_time_logical_elapsed(env);
    lf_print("Late reaction resumed at " PRINTF_TIME ".", self->late_resumed_at);
    // The same lf_await_resume_ext() that resumed the startup reaction resumed this one.
    if (self->late_resumed_at != MSEC(100)) {
        lf_print_error_and_exit("Expected the late reaction to resume at 100 ms. Resumed at " PRINTF_TIME ".",
                                self->late_resumed_at);
    }
}

void check(requester_self_t* self) {
    if (!self->completed) {
        lf_print_error_and_exit("Expected the startup reaction to complete before shutdown.");
    }
    if (self->responses != 1) {
        lf_print_error_and_exit("Expected 1 response. Got %d.", self->responses);
    }
    if (self->late_resumed_at != MSEC(100)) {
        lf_print_error_and_exit("Expected the late reaction to have resumed at 100 ms.");
    }
}

void forgotten(forgotten_self_t* self, forgotten_silent_t* silent) {
    environment_t* env = self->base.environment;
    lf_print("Forgotten: suspending; nothing will resume this reaction.");
    lf_await_ext(self, silent);
    // The program has stopped, so this must be the stop tag.
    if (lf_time_logical_elapsed(env) != SEC(1) || lf_tag(env).microstep != 0) {
        lf_print_error_and_exit("Expected lf_await_ext() to return at the stop tag (1 s). Returned at " PRINTF_TIME
                                " (microstep %u).",
                                lf_time_logical_elapsed(env), lf_tag(env).microstep);
    }
    lf_print("Forgotten: completed at the stop tag.");
}
