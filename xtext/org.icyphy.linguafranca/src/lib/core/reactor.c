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
 */

#include "reactor_common.c"
//#include <assert.h>

/**
 * Schedule the specified trigger at current_time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_token(void* action, interval_t extra_delay, token_t* token) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    return __schedule(trigger, extra_delay, token);
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, int length) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    token_t* token = create_token(trigger->element_size);
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
        fprintf(stderr, "ERROR: schedule: Invalid trigger or element size.\n");
        return -1;
    }
    // printf("DEBUG: schedule_copy: Allocating memory for payload (token value): %p\n", trigger);
    // Initialize token with an array size of length and a reference count of 0.
    token_t* token = __initialize_token(trigger->token, length);
    // Copy the value into the newly allocated memory.
    memcpy(token->value, value, token->element_size * length);
    // The schedule function will increment the reference count.
    return schedule_token(action, offset, token);
}

/**
 * Advance logical time to the lesser of the specified time or the
 * timeout time, if a timeout time has been given. If the -fast command-line option
 * was not given, then wait until physical time matches or exceeds the start time of
 * execution plus the current_time plus the specified logical time.  If this is not
 * interrupted, then advance current_time by the specified logical_delay. 
 * Return 0 if time advanced to the time of the event and -1 if the wait
 * was interrupted or if the timeout time was reached.
 */ 
int wait_until(instant_t logical_time_ns) {
    int return_value = 0;
    if (stop_time > 0LL && logical_time_ns > stop_time) {
        logical_time_ns = stop_time;
        current_microstep = 0;
        // Indicate on return that the time of the event was not reached.
        // We still wait for time to elapse in case asynchronous events come in.
        return_value = -1;
    }
    if (!fast) {
        // printf("DEBUG: Waiting for logical time %lld.\n", logical_time_ns);
    
        // Get the current physical time.
        struct timespec current_physical_time;
        clock_gettime(CLOCK_REALTIME, &current_physical_time);
    
        long long ns_to_wait = logical_time_ns
                - (current_physical_time.tv_sec * BILLION
                + current_physical_time.tv_nsec);
    
        if (ns_to_wait <= 0) {
            // Advance current time.
            _lf_advance_logical_time(logical_time_ns);
            return return_value;
        }
    
        // timespec is seconds and nanoseconds.
        struct timespec wait_time = {(time_t)ns_to_wait / BILLION, (long)ns_to_wait % BILLION};
        // printf("DEBUG: Waiting %lld seconds, %lld nanoseconds.\n", ns_to_wait / BILLION, ns_to_wait % BILLION);
        struct timespec remaining_time;
        // FIXME: If the wait time is less than the time resolution, don't sleep.
        if (nanosleep(&wait_time, &remaining_time) != 0) {
            // Sleep was interrupted.
            // May have been an asynchronous call to schedule(), or
            // it may have been a control-C to stop the process.
            // Set current time to match physical time, but not less than
            // current logical time nor more than next time in the event queue.
            clock_gettime(CLOCK_REALTIME, &current_physical_time);
            long long current_physical_time_ns 
                    = current_physical_time.tv_sec * BILLION
                    + current_physical_time.tv_nsec;
            if (current_physical_time_ns > current_time) {
                if (current_physical_time_ns < logical_time_ns) {
                    // Advance current time.
                    _lf_advance_logical_time(current_physical_time_ns);
                    return -1;
                }
            } else {
                // Current physical time does not exceed current logical
                // time, so do not advance current time.
                return -1;
            }
        }
    }
    // Advance current time.
    _lf_advance_logical_time(logical_time_ns);
    return return_value;
}

void print_snapshot() {
    printf(">>> START Snapshot\n");
    pqueue_dump(reaction_q, stdout, reaction_q->prt);
    printf(">>> END Snapshot\n");
}

/**
 * Put the specified reaction on the reaction queue.
 * This version does not acquire a mutex lock.
 * @param reaction The reaction.
 */
void _lf_enqueue_reaction(reaction_t* reaction) {
    // Do not enqueue this reaction twice.
    if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
        // printf("DEBUG: Enqueing downstream reaction %p.\n", reaction);
        pqueue_insert(reaction_q, reaction);
    }
}

int __do_step() {
        // Invoke reactions.
    while(pqueue_size(reaction_q) > 0) {
        //print_snapshot();
        reaction_t* reaction = (reaction_t*)pqueue_pop(reaction_q);
        
        // printf("DEBUG: Popped from reaction_q reaction with deadline: %lld\n", reaction->deadline);
        // printf("DEBUG: Address of reaction: %p\n", reaction);

        bool violation = false;
        // If the tardiness amount for the reaction is larger than zero,
        // an input trigger to this reaction has been triggered at a later
        // logical time than originally anticipated. In this case, a special
        // tardy reaction will be invoked.             
        // FIXME: Note that the tardy reaction will be invoked
        // at most once per logical time value. If the tardy reaction triggers the
        // same reaction at the current time value, even if at a future superdense time,
        // then the reaction will be invoked and the tardy reaction will not be invoked again.
        // However, inputs ports to a federate reactor are network port types so this possibly should
        // be disallowed.
        // @note The tardy handler and the deadline handler are not mutually exclusive.
        //  In other words, both can be invoked for a reaction if it is triggered late
        //  in logical time (tardy) and also misses the constraint on physical time (deadline).
        // @note The tardy handler currently can only be invoked in a special circumstance in
        //  the federated execution. Since federated execution uses the threaded runtime, this
        //  condition currently will not occur here (in the unthreaded runtime). However, tardiness
        //  handling is replicated here for future compatibility.
        // if (reaction->tardiness > 0LL) {
        //     // There is a violation
        //     violation = true;
        //     reaction_function_t handler = reaction->tardy_handler;
        //     if (handler != NULL) {
        //         (*handler)(reaction->self);

        //         // If the reaction produced outputs, put the resulting
        //         // triggered reactions into the queue or execute them directly if possible.
        //         schedule_output_reactions(reaction);
        //     }
        // }

        // If the reaction has a deadline, compare to current physical time
        // and invoke the deadline violation reaction instead of the reaction function
        // if a violation has occurred. Note that the violation reaction will be invoked
        // at most once per logical time value. If the violation reaction triggers the
        // same reaction at the current time value, even if at a future superdense time,
        // then the reaction will be invoked and the violation reaction will not be invoked again.
        if (reaction->deadline > 0LL) {
            // Get the current physical time.
            struct timespec current_physical_time;
            clock_gettime(CLOCK_REALTIME, &current_physical_time);
            // Convert to instant_t.
            instant_t physical_time = 
                    current_physical_time.tv_sec * BILLION
                    + current_physical_time.tv_nsec;
            // Check for deadline violation.
            // There are currently two distinct deadline mechanisms:
            // local deadlines are defined with the reaction;
            // container deadlines are defined in the container.
            // They can have different deadlines, so we have to check both.
            // Handle the local deadline first.
            if (reaction->deadline > 0LL && physical_time > current_time + reaction->deadline) {
                printf("Deadline violation.\n");
                // Deadline violation has occurred.
                violation = true;
                // Invoke the local handler, if there is one.
                reaction_function_t handler = reaction->deadline_violation_handler;
                if (handler != NULL) {
                    (*handler)(reaction->self);
                    // If the reaction produced outputs, put the resulting
                    // triggered reactions into the queue.
                    schedule_output_reactions(reaction);
                }
            }
        }
        
        if (!violation) {
            // Invoke the reaction function.
            reaction->function(reaction->self);

            // If the reaction produced outputs, put the resulting triggered
            // reactions into the queue.
            schedule_output_reactions(reaction);
        }
    }
    
    // No more reactions should be blocked at this point.
    //assert(pqueue_size(blocked_q) == 0);

    if (stop_time > 0LL && current_time >= stop_time) {
        stop_requested = true;
        return 0;
    }
    return 1;
}

// Wait until physical time matches or exceeds the time of the least tag
// on the event queue. If there is no event in the queue, return 0.
// After this wait, advance current_time to match
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
    //pqueue_dump(event_q, stdout, event_q->prt);
    // If there is no next event and -keepalive has been specified
    // on the command line, then we will wait the maximum time possible.
    instant_t next_time = LLONG_MAX;
    if (event == NULL) {
        // No event in the queue.
        if (!keepalive_specified) {
            return 0;
        }
    } else {
        next_time = event->time;
    }
    //printf("DEBUG: Next event (elapsed) time is %lld.\n", next_time - start_time);
    // Wait until physical time >= event.time.
    // The wait_until function will advance current_time.
    if (wait_until(next_time) < 0) {
        // Sleep was interrupted or the timeout time has been reached.
        // Time has not advanced to the time of the event.
        // There may be a new earlier event on the queue.
        event_t* new_event = (event_t*)pqueue_peek(event_q);
        printf("HERE\n");
        if (new_event == event) {
            // There is no new event. If the timeout time has been reached,
            // or if the maximum time has been reached (unlikely), then return.
            if (new_event == NULL || (stop_time > 0LL && current_time >= stop_time)) {
                stop_requested = true;
                return 0;
            }
            printf("DEBUG: Setting current (elapsed) time to %lld.\n", next_time - start_time);
        } else {
            // Handle the new event.
            event = new_event;
            next_time = event->time;
            printf("DEBUG: New event at (elapsed) time %lld.\n", next_time - start_time);
        }
    }

    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    __start_time_step();
    
    // Pop all events from event_q with timestamp equal to current_time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    __pop_events();
    
    return __do_step();
}

// Stop execution at the conclusion of the current logical time.
void stop() {
    stop_requested = true;
}

// Print elapsed logical and physical times.
void wrapup() {
    // Invoke any code generated wrapup. If this returns true,
    // then actions have been scheduled at the next microstep.
    // Invoke next() one more time to react to those actions.
    if (__wrapup()) {
        next();
    }
}

int main(int argc, char* argv[]) {
    // Invoke the function that optionally provides default command-line options.
    __set_default_command_line_options();

    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {
        initialize();
        _lf_execution_started = true;
        __trigger_startup_reactions();
        __initialize_timers();
        // Handle reactions triggered at time (T,0).
        __do_step();
        while (next() != 0 && !stop_requested);
        wrapup();
        termination();
        return 0;
    } else {
        printf("DEBUG: invoking termination.\n");
        termination();
        return -1;
    }
}
