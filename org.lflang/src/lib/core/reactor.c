/* Runtime infrastructure for the non-threaded version of the C target of Lingua Franca. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/** Runtime infrastructure for the non-threaded version of the C target
 *  of Lingua Franca.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#include "reactor_common.c"
#include "platform.h"
#include <signal.h> // To trap ctrl-c and invoke termination().
//#include <assert.h>

/**
 * Schedule the specified trigger at current_tag.time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_token(void* action, interval_t extra_delay, lf_token_t* token) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    return __schedule(trigger, extra_delay, token);
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, int length) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    lf_token_t* token = create_token(trigger->element_size);
    token->value = value;
    token->length = length;
    return schedule_token(action, extra_delay, token);
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_copy(void* action, interval_t offset, void* value, int length) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    if (value == NULL) {
        return schedule_token(action, offset, NULL);
    }
    if (trigger == NULL || trigger->token == NULL || trigger->token->element_size <= 0) {
        error_print("schedule: Invalid trigger or element size.");
        return -1;
    }
    DEBUG_PRINT("schedule_copy: Allocating memory for payload (token value): %p.", trigger);
    // Initialize token with an array size of length and a reference count of 0.
    lf_token_t* token = __initialize_token(trigger->token, length);
    // Copy the value into the newly allocated memory.
    memcpy(token->value, value, token->element_size * length);
    // The schedule function will increment the reference count.
    return schedule_token(action, offset, token);
}

/**
 * Advance logical time to the lesser of the specified time or the
 * timeout time, if a timeout time has been given. If the -fast command-line option
 * was not given, then wait until physical time matches or exceeds the start time of
 * execution plus the current_tag.time plus the specified logical time.  If this is not
 * interrupted, then advance current_tag.time by the specified logical_delay.
 * Return 0 if time advanced to the time of the event and -1 if the wait
 * was interrupted or if the timeout time was reached.
 */ 
int wait_until(instant_t logical_time_ns) {
    int return_value = 0;
    if (!fast) {
        LOG_PRINT("Waiting for elapsed logical time %lld.", logical_time_ns - start_time);
        interval_t ns_to_wait = logical_time_ns - get_physical_time();
    
        if (ns_to_wait <= 0) {
            return return_value;
        }
    
        // timespec is seconds and nanoseconds.
        struct timespec wait_time = {(time_t)ns_to_wait / BILLION, (long)ns_to_wait % BILLION};
        DEBUG_PRINT("Waiting %lld seconds, %lld nanoseconds.", ns_to_wait / BILLION, ns_to_wait % BILLION);
        struct timespec remaining_time;
        // FIXME: If the wait time is less than the time resolution, don't sleep.
        return_value = lf_nanosleep(_LF_CLOCK, &wait_time, &remaining_time);
    }
    return return_value;
}

void print_snapshot() {
    if(LOG_LEVEL > 3) {
        DEBUG_PRINT(">>> START Snapshot\n");
        pqueue_dump(reaction_q, reaction_q->prt);
        DEBUG_PRINT(">>> END Snapshot\n");
    }
}

/**
 * Put the specified reaction on the reaction queue.
 * This version does not acquire a mutex lock.
 * @param reaction The reaction.
 */
void _lf_enqueue_reaction(reaction_t* reaction) {
    // Do not enqueue this reaction twice.
    if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
        DEBUG_PRINT("Enqueing downstream reaction %s.", reaction->name);
        pqueue_insert(reaction_q, reaction);
    }
}

/**
 * Execute all the reactions in the reaction queue at the current tag.
 * 
 * @return Returns 1 if the execution should continue and 0 if the execution
 *  should stop.
 */
int _lf_do_step() {
    // Invoke reactions.
    while(pqueue_size(reaction_q) > 0) {
        print_snapshot();
        reaction_t* reaction = (reaction_t*)pqueue_pop(reaction_q);
        
        LOG_PRINT("Invoking reaction %s at elapsed logical tag (%lld, %d).",
        		reaction->name,
                current_tag.time - start_time, current_tag.microstep);

        bool violation = false;

        // If the reaction has a deadline, compare to current physical time
        // and invoke the deadline violation reaction instead of the reaction function
        // if a violation has occurred. Note that the violation reaction will be invoked
        // at most once per logical time value. If the violation reaction triggers the
        // same reaction at the current time value, even if at a future superdense time,
        // then the reaction will be invoked and the violation reaction will not be invoked again.
        if (reaction->deadline > 0LL) {
            // Get the current physical time.
            instant_t physical_time = get_physical_time();
            // Check for deadline violation.
            // There are currently two distinct deadline mechanisms:
            // local deadlines are defined with the reaction;
            // container deadlines are defined in the container.
            // They can have different deadlines, so we have to check both.
            // Handle the local deadline first.
            if (reaction->deadline > 0LL && physical_time > current_tag.time + reaction->deadline) {
                LOG_PRINT("Deadline violation. Invoking deadline handler.");
                // Deadline violation has occurred.
                violation = true;
                // Invoke the local handler, if there is one.
                reaction_function_t handler = reaction->deadline_violation_handler;
                if (handler != NULL) {
                    (*handler)(reaction->self);
                    // If the reaction produced outputs, put the resulting
                    // triggered reactions into the queue.
                    schedule_output_reactions(reaction, 0);
                }
            }
        }
        
        if (!violation) {
            // Invoke the reaction function.
            tracepoint_reaction_starts(reaction, 0); // 0 indicates unthreaded.
            reaction->function(reaction->self);
            tracepoint_reaction_ends(reaction, 0);

            // If the reaction produced outputs, put the resulting triggered
            // reactions into the queue.
            schedule_output_reactions(reaction, 0);
        }
    }
    
    // No more reactions should be blocked at this point.
    //assert(pqueue_size(blocked_q) == 0);

    if (compare_tags(current_tag, stop_tag) >= 0) {
        return 0;
    }

    return 1;
}

// Wait until physical time matches or exceeds the time of the least tag
// on the event queue. If there is no event in the queue, return 0.
// After this wait, advance current_tag.time to match
// this tag. Then pop the next event(s) from the
// event queue that all have the same tag, and extract from those events
// the reactions that are to be invoked at this logical time.
// Sort those reactions by index (determined by a topological sort)
// and then execute the reactions in order. Each reaction may produce
// outputs, which places additional reactions into the index-ordered
// priority queue. All of those will also be executed in order of indices.
// If the -timeout option has been given on the command line, then return
// 0 when the logical time duration matches the specified duration.
// Also return 0 if there are no more events in the queue and
// the keepalive command-line option has not been given.
// Otherwise, return 1.
int next() {
    event_t* event = (event_t*)pqueue_peek(event_q);
    //pqueue_dump(event_q, event_q->prt);
    // If there is no next event and -keepalive has been specified
    // on the command line, then we will wait the maximum time possible.
    // FIXME: is LLONG_MAX different from FOREVER?
    tag_t next_tag = { .time = LLONG_MAX, .microstep = UINT_MAX};
    if (event == NULL) {
        // No event in the queue.
        if (!keepalive_specified) { // FIXME: validator should issue a warning for unthreaded implementation
                                    // schedule is not thread-safe
            _lf_set_stop_tag((tag_t){.time=current_tag.time,.microstep=current_tag.microstep+1});
        }
    } else {
        next_tag.time = event->time;
        // Deduce the microstep
        if (next_tag.time == current_tag.time) {
            next_tag.microstep = get_microstep() + 1;
        } else {
            next_tag.microstep = 0;
        }
    }
    
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        // Cannot process events after the stop tag.
        next_tag = stop_tag;
    }

    LOG_PRINT("Next event (elapsed) time is %lld.", next_tag.time - start_time);
    // Wait until physical time >= event.time.
    // The wait_until function will advance current_tag.time.
    if (wait_until(next_tag.time) != 0) {
        DEBUG_PRINT("***** wait_until was interrupted.");
        // Sleep was interrupted.
        // FIXME: It is unclear what would cause this to occur in this unthreaded
        // runtime since schedule() is not thread safe here and should not
        // be called asynchronously. Perhaps in some runtime such as for a
        // PRET machine this will be supported, so here we handle this as
        // if an asynchronous call to schedule has occurred. In that case,
        // we should return 1 to let the runtime loop around to see what
        // is on the event queue.
        return 1;
    }

    // At this point, finally, we have an event to process.
    // Advance current time to match that of the first event on the queue.
    _lf_advance_logical_time(next_tag.time);

    if (compare_tags(current_tag, stop_tag) >= 0) {        
        __trigger_shutdown_reactions();
    }

    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    __start_time_step();
    
    // Pop all events from event_q with timestamp equal to current_tag.time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    __pop_events();

    return _lf_do_step();
}

/**
 * Stop execution at the conclusion of the next microstep.
 */
void request_stop() {
	tag_t new_stop_tag;
	new_stop_tag.time = current_tag.time;
	new_stop_tag.microstep = current_tag.microstep + 1;
	_lf_set_stop_tag(new_stop_tag);
}

/**
 * Do nothing. This implementation is not multithreaded.
 */
void _lf_notify_workers() {
}

/**
 * Return false.
 * @param reaction The reaction.
 */
bool _lf_is_blocked_by_executing_reaction() {
    return false;
}


int main(int argc, char* argv[]) {
    // Invoke the function that optionally provides default command-line options.
    __set_default_command_line_options();

    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {

        if (atexit(termination) != 0) {
            warning_print("Failed to register termination function!");
        }
        // The above handles only "normal" termination (via a call to exit).
        // As a consequence, we need to also trap ctrl-C, which issues a SIGINT,
        // and cause it to call exit.
        signal(SIGINT, exit);

        initialize(); // Sets start_time.
        current_tag = (tag_t){.time = start_time, .microstep = 0u};
        _lf_execution_started = true;
        __trigger_startup_reactions();
        __initialize_timers(); 
        // If the stop_tag is (0,0), also insert the shutdown
        // reactions. This can only happen if the timeout time
        // was set to 0.
        if (compare_tags(current_tag, stop_tag) >= 0) {
            __trigger_shutdown_reactions(); // __trigger_shutdown_reactions();
        }
        // Handle reactions triggered at time (T,m).
        if (_lf_do_step()) {
            while (next() != 0);
        }
        return 0;
    } else {
        return -1;
    }
}
