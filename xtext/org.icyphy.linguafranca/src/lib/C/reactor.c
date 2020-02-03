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

// Schedule the specified trigger at current_time plus the
// offset declared in the trigger plus the extra_delay.
// If the offset of the trigger and the extra_delay are both zero,
// then schedule the trigger to occur one microstep later in superdense time.
// The value is required to be a pointer returned by malloc
// because it will be freed after having been delivered to
// all relevant destinations unless it is NULL, in which case
// it will be ignored.
// NOTE: There is multithreading support in this implementation, so
// asynchronous calls to this function should not be made. The calls
// should only be made within reactions.
// If you need asynchronous calls, then use reactor_threaded.c.
handle_t schedule(trigger_t* trigger, interval_t extra_delay, void* value) {
    return __schedule(trigger, extra_delay, value);
}

// Advance logical time to the lesser of the specified time or the
// timeout time, if a timeout time has been given. If the -fast command-line option
// was not given, then wait until physical time matches or exceeds the start time of
// execution plus the current_time plus the specified logical time.  If this is not
// interrupted, then advance current_time by the specified logical_delay. 
// Return 0 if time advanced to the time of the event and -1 if the wait
// was interrupted or if the timeout time was reached.
int wait_until(instant_t logical_time_ns) {
    int return_value = 0;
    if (stop_time > 0LL && logical_time_ns > stop_time) {
        logical_time_ns = stop_time;
        // Indicate on return that the time of the event was not reached.
        // We still wait for time to elapse in case asynchronous events come in.
        return_value = -1;
    }
    if (!fast) {
        // printf("-------- Waiting for logical time %lld.\n", logical_time_ns);
    
        // Get the current physical time.
        struct timespec current_physical_time;
        clock_gettime(CLOCK_REALTIME, &current_physical_time);
    
        long long ns_to_wait = logical_time_ns
                - (current_physical_time.tv_sec * BILLION
                + current_physical_time.tv_nsec);
    
        if (ns_to_wait <= 0) {
            // Advance current time.
            current_time = logical_time_ns;
            return return_value;
        }
    
        // timespec is seconds and nanoseconds.
        struct timespec wait_time = {(time_t)ns_to_wait / BILLION, (long)ns_to_wait % BILLION};
        // printf("-------- Waiting %lld seconds, %lld nanoseconds.\n", ns_to_wait / BILLION, ns_to_wait % BILLION);
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
                    current_time = current_physical_time_ns;
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
    current_time = logical_time_ns;
    return return_value;
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
    event_t* event = pqueue_peek(event_q);
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
    // Wait until physical time >= event.time.
    // The wait_until function will advance current_time.
    if (wait_until(next_time) < 0) {
        // Sleep was interrupted or the timeout time has been reached.
        // Time has not advanced to the time of the event.
        // There may be a new earlier event on the queue.
        event_t* new_event = pqueue_peek(event_q);
        if (new_event == event) {
            // There is no new event. If the timeout time has been reached,
            // or if the maximum time has been reached (unlikely), then return.
            if ((stop_time > 0LL && current_time >= stop_time) || new_event == NULL) {
            	stop_requested = true;
                return 0;
            }
        } else {
        	// Handle the new event.
        	event = new_event;
        	next_time = event->time;
        }
    }
    
    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    __start_time_step();
    
    // Pop all events from event_q with timestamp equal to current_time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    do {
        event = pqueue_pop(event_q);
        for (int i = 0; i < event->trigger->number_of_reactions; i++) {
            // printf("Pushed on reaction_q reaction with level: %lld\n", event->trigger->reactions[i]->index);
            pqueue_insert(reaction_q, event->trigger->reactions[i]);
        }
        if (!(event->trigger->is_physical) && event->trigger->period > 0) {
            // Reschedule the trigger.
            // Note that the delay here may be negative because the __schedule
            // function will add the trigger->offset, which we don't want at this point.
            // NULL argument indicates that there is no value.
            __schedule(event->trigger, event->trigger->period - event->trigger->offset, NULL);
        }
        // Copy the value pointer into the trigger struct so that the
        // reactions can access it.
        event->trigger->value = event->value;
        
        // If the value is non-null, record the event to free the value
        // at the end of the current logical time. Otherwise, recycle the event.
        // In either case, so that sorting doesn't cost anything,
        // give all recycled events the same zero time stamp.
        event->time = 0LL;
        if (event->value == NULL) {
       		pqueue_insert(recycle_q, event);
       	} else {
       		pqueue_insert(free_q, event);
       	}
		// Peek at the next event in the event queue.
        event = pqueue_peek(event_q);
    } while(event != NULL && event->time == current_time);

    // Invoke reactions.
    while(pqueue_size(reaction_q) > 0) {
        reaction_t* reaction = pqueue_pop(reaction_q);
        // printf("Popped from reaction_q reaction with deadline: %lld\n", reaction->deadline);
        // printf("Address of reaction: %p\n", reaction);

        // If the reaction has a deadline, compare to current physical time
        // and invoke the deadline violation reaction instead of the reaction function
        // if a violation has occurred. Note that the violation reaction will be invoked
        // at most once per logical time value. If the violation reaction triggers the
        // same reaction at the current time value, even if at a future superdense time,
        // then the reaction will be invoked and the violation reaction will not be invoked again.
        bool violation = false;
        if (reaction->local_deadline > 0LL) {
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
            if (reaction->local_deadline > 0LL && physical_time > current_time + reaction->local_deadline) {
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
    // Free any values that need to be freed and recycle the event
    // carrying them.
    event_t* free_event = pqueue_pop(free_q);
    while (free_event != NULL) {
        if (free_event->value != NULL) {
        	// This is where an action payload gets freed.
    	    free(free_event->value);
    	}
    	if (free_event->trigger != NULL) {
    	    // Make sure the trigger is not pointing to freed memory.
    	    free_event->trigger->value = NULL;
    	}
    	pqueue_insert(recycle_q, free_event);
    	free_event = pqueue_pop(free_q);
    }
    
    if (stop_time > 0LL && current_time >= stop_time) {
    	stop_requested = true;
        return 0;
    }
    return 1;
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
        __start_timers();
        while (next() != 0 && !stop_requested);
        wrapup();
        termination();
    	return 0;
    } else {
        termination();
    	return -1;
    }
}
