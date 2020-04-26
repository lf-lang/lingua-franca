/* Runtime infrastructure for the C target of Lingua Franca. */

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

/** Runtime infrastructure for the C target of Lingua Franca.
 *  This file contains resources that are shared by the threaded and
 *  non-threaded versions of the C runtime.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Mehrdad Niknami <mniknami@berkeley.edu>}
 */
#include "reactor.h"

/** Global constants. */
bool False = false;
bool True = true;

/** 
 * Indicator of whether to wait for physical time to match logical time.
 * By default, execution will wait. The command-line argument -fast will
 * eliminate the wait and allow logical time to exceed physical time.
 */ 
bool fast = false;

/** By default, execution is not threaded. */
int number_of_threads = 0; // FIXME: should be unsigned

/**
 * Current time in nanoseconds since January 1, 1970.
 * This is not in scope for reactors.
 */
instant_t current_time = 0LL;

/** Logical time at the start of execution. */
interval_t start_time = 0LL;

/** Physical time at the start of the execution. */
instant_t physical_start_time = 0LL;

/**
 * Indicator that the execution should stop after the completion of the
 * current logical time. This can be set to true by calling the `stop()`
 * function in a reaction.
 */
bool stop_requested = false;

/** 
 * The logical time to elapse during execution, or -1 if no timeout time has
 * been given. When the logical equal to start_time + duration has been
 * reached, execution will terminate.
 */
instant_t duration = -1LL;

/**
 * Stop time (start_time + duration), or 0 if no timeout time has been given.
 */
instant_t stop_time = 0LL;

/** Indicator of whether the keepalive command-line option was given. */
bool keepalive_specified = false;

/////////////////////////////
// The following functions are in scope for all reactors:

/** Return the elapsed logical time in nanoseconds since the start of execution. */
interval_t get_elapsed_logical_time() {
    return current_time - start_time;
}

/** Return the current logical time in nanoseconds since January 1, 1970. */
instant_t get_logical_time() {
    // FIXME: Does this need acquire the mutex?
    return current_time;
}

/** Return the current physical time in nanoseconds since January 1, 1970. */
instant_t get_physical_time() {
    struct timespec physicalTime;
    clock_gettime(CLOCK_REALTIME, &physicalTime);
    return physicalTime.tv_sec * BILLION + physicalTime.tv_nsec;
}

/** Return the elapsed physical time in nanoseconds. */
instant_t get_elapsed_physical_time() {
    struct timespec physicalTime;
    clock_gettime(CLOCK_REALTIME, &physicalTime);
    return physicalTime.tv_sec * BILLION + physicalTime.tv_nsec - physical_start_time;
}

/** Print a non-negative time value in nanoseconds with commas separating thousands
 *  followed by a carriage return. Ideally, this would use the locale to
 *  use periods if appropriate, but I haven't found a sufficiently portable
 *  way to do that.
 *  @param time A time value.
 */
void print_time(instant_t time) {
    if (time < 1000 || time < 0) {
        printf("%lld", time);
        return;
    }
    print_time(time/1000LL);
    int to_print = time%1000;
    printf (",%03d", to_print);
}

/////////////////////////////
// The following is not in scope for reactors:

/** Priority queues. */
pqueue_t* event_q;     // For sorting by time.
// pqueue_t* blocked_q;   // To store reactions that are blocked by other reactions.

pqueue_t* reaction_q;  // For sorting by deadline.
pqueue_t* recycle_q;   // For recycling malloc'd events.

handle_t __handle = 1;

// ********** Priority Queue Support Start

/**
 * Return whether the first and second argument are given in reverse order.
 */
static int in_reverse_order(pqueue_pri_t this, pqueue_pri_t that) {
    return (this > that);
}

/**
 * Return whether or not the given events have matching triggers.
 */
static int event_matches(void* next, void* curr) {
    return (((event_t*)next)->trigger == ((event_t*)curr)->trigger);
}

/**
 * Return whether or not the given reaction_t pointers 
 * point to the same struct.
 */
static int reaction_matches(void* next, void* curr) {
    return (next == curr);
}

/**
 * Report a priority equal to the time of the given event.
 * Used for sorting pointers to event_t structs in the event queue.
 */
static pqueue_pri_t get_event_time(void *a) {
    return (pqueue_pri_t)(((event_t*) a)->time);
}

/**
 * Report a priority equal to the index of the given reaction.
 * Used for sorting pointers to reaction_t structs in the 
 * blocked and executing queues.
 */
static pqueue_pri_t get_reaction_index(void *a) {
    return ((reaction_t*) a)->index;
}

/**
 * Return the given event's position in the queue.
 */
static size_t get_event_position(void *a) {
    return ((event_t*) a)->pos;
}

/**
 * Return the given reaction's position in the queue.
 */
static size_t get_reaction_position(void *a) {
    return ((reaction_t*) a)->pos;
}

/**
 * Set the given event's position in the queue.
 */
static void set_event_position(void *a, size_t pos) {
    ((event_t*) a)->pos = pos;
}

/**
 * Return the given reaction's position in the queue.
 */
static void set_reaction_position(void *a, size_t pos) {
    ((reaction_t*) a)->pos = pos;
}

/**
 * Print some information about the given reaction.
 */
static void print_reaction(FILE *out, void *reaction) {
	reaction_t *r = reaction;
    fprintf(out, "chain_id:%llu, index: %llu, reaction: %p\n", 
        r->chain_id, r->index, r);
}

/**
 * Print some information about the given event.
 */
static void print_event(FILE *out, void *event) {
	event_t *e = event;
    fprintf(out, "time: %lld, trigger: %p, token: %p\n",
			e->time, e->trigger, e->token);
}

// ********** Priority Queue Support End

/**
 * Counter used to issue a warning if memory is
 * allocated for message payloads and never freed.
 */
static int __count_payload_allocations;

/**
 * Counter used to issue a warning if memory is
 * allocated for tokens and never freed. Note that
 * every trigger will have one token allocated for
 * it. That token is not counted because it is not
 * expected to be freed.
 */
static int __count_token_allocations;

/**
 * Tokens always have the same size in memory so they are easily recycled.
 * When a token is freed, this pointer will be updated to point to it.
 * Freed tokens are chained using their next_free field.
 */
token_t* __token_recycling_bin = NULL;

/** Count of the number of tokens in the recycling bin. */
int __token_recycling_bin_size = 0;

/**
 * To allow a system to recover from burst of activity, the token recycling
 * bin has a limited size. When it becomes full, token are freed using free().
 */
#define __TOKEN_RECYCLING_BIN_SIZE_LIMIT 512

/** Possible return values for __done_using. */
typedef enum token_freed {
    NOT_FREED,     // Nothing was freed.
    VALUE_FREED,   // The value (payload) was freed.
    TOKEN_FREED    // The value and the token were freed.
} token_freed;

/**
 * Decrement the reference count of the specified token.
 * If the reference count hits 0, free the memory for the value
 * carried by the token, and, if the token is not also the template
 * token of its trigger, free the token.
 * @param token Pointer to a token.
 * @return NOT_FREED if nothing was freed, VALUE_FREED if the value
 *  was freed, and TOKEN_FREED if both the value and the token were
 *  freed.
 */
token_freed __done_using(token_t* token) {
    token_freed result = NOT_FREED;
    if (token == NULL) return result;
    if (token->ref_count == 0) {
        fprintf(stderr, "WARNING: Token being freed that has already been freed: %p\n", token);
    } else {
        token->ref_count--;
    }
    // printf("DEBUG: __done_using: ref_count = %d.\n", token->ref_count);
    if (token->ref_count == 0) {
        if (token->value != NULL) {
            // Count frees to issue a warning if this is never freed.
            __count_payload_allocations--;
            // printf("DEBUG: __done_using: Freeing allocated memory for payload (token value): %p\n", token->value);
            free(token->value);
            token->value = NULL;
            result = VALUE_FREED;
        }
        // Tokens that are created at the start of execution and associated with
        // output ports or actions are pointed to by those actions and output
        // ports and should not be freed. They are expected to be reused instead.
        if (token->ok_to_free) {
            // Need to free the token_t struct also.
            if (__token_recycling_bin_size < __TOKEN_RECYCLING_BIN_SIZE_LIMIT) {
                // Recycle instead of freeing.
                token->next_free = __token_recycling_bin;
                __token_recycling_bin = token;
                __token_recycling_bin_size++;
            } else {
                // Recycling bin is full.
                free(token);
            }
            __count_token_allocations--;
            // printf("DEBUG: __done_using: Freeing allocated memory for token: %p\n", token);
            result = TOKEN_FREED;
        }
    }
    return result;
}

/**
 * Create a new token_t struct and initialize it for assignment to a trigger.
 * The value pointer will be NULL and the length will be 0.
 * This function is for tokens that are not expected to be freed, and
 * reactors are not expected to use it. It is used by the code generator
 * to initialize actions with tokens.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A new or recycled token_t struct.
 */
token_t* __create_token(size_t element_size) {
    token_t* token;
    // Check the recycling bin.
    if (__token_recycling_bin != NULL) {
        token = __token_recycling_bin;
        __token_recycling_bin = token->next_free;
        __token_recycling_bin_size--;
        // printf("DEBUG: __create_token: Retrieved token from the recycling bin: %p\n", token);
    } else {
        token = (token_t*)malloc(sizeof(token_t));
        // printf("DEBUG: __create_token: Allocated memory for token: %p\n", token);
    }
    token->value = NULL;
    token->length = 0;
    token->element_size = element_size;
    token->ref_count = 0;
    token->ok_to_free = false;
    token->next_free = NULL;
    return token;
}

/**
 * Create a new token and initialize it.
 * The value pointer will be NULL and the length will be 0.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A new or recycled token_t struct.
 */
token_t* create_token(size_t element_size) {
    // printf("DEBUG: create_token: element_size: %zu\n", element_size);
    __count_token_allocations++;
    token_t* result = __create_token(element_size);
    result->ok_to_free = true;
    return result;
}

/**
 * If the specified token is available (its reference count is 0),
 * then initialize it with the specified value, length (number of array
 * elements or 1 if it is not an array), and reference count.
 * Otherwise, create a new token and initialize it.
 * The new token will point to the same trigger and element_size
 * as the specified token. Return either the specified token, or,
 * if a new one was created, the new token.
 *
 * @param token The token to populate, if it is available or null to create a new token.
 * @param value The value to be carried by the token or NULL for none.
 * @param element_size The size of memory for the token payload.
 * @param length The length of the array, or 1 if it is not an array.
 * @param ref_count The initial reference count.
 * @return Either the specified token or a new one carrying the value.
 */
token_t* __initialize_token(token_t* token, void* value, size_t element_size, int length, int ref_count) {
    // assert(token != NULL);

    // If necessary, allocate memory for a new token_t struct.
    // This assumes that the token_t* in the self struct has been initialized to NULL.
    token_t* result = token;
    // printf("DEBUG: initializing a token %p with ref_count %d.\n", token, token->ref_count);
    if (token == NULL || token->ref_count > 0) {
        // The specified token is not available.
        result = create_token(element_size);
    }
    result->value = value;
    result->length = length;
    // printf("DEBUG: setting ref_count to %d.\n", ref_count);
    result->ref_count = ref_count;
    return result;
}

/**
 * Pop all events from event_q with timestamp equal to current_time, extract all
 * the reactions triggered by these events, and stick them into the reaction
 * queue.
 */
void __pop_events() {
    event_t* event;
    do {
        event = pqueue_pop(event_q);

        token_t* token = event->token;

        // Push the corresponding reactions onto the reaction queue.
        for (int i = 0; i < event->trigger->number_of_reactions; i++) {
            // printf("Pushed on reaction_q: %p\n", event->trigger->reactions[i]);
            // printf("Pushed reaction args: %p\n", event->trigger->reactions[i]->args);
            pqueue_insert(reaction_q, event->trigger->reactions[i]);
        }
        // If the trigger is a periodic clock, create a new event for its next execution.
        if (event->trigger->is_timer && event->trigger->period > 0LL) {
            // Reschedule the trigger.
            // Note that the delay here may be negative because the __schedule
            // function will add the trigger->offset, which we don't want at this point.
            // NULL argument indicates that there is no value.
            __schedule(event->trigger, event->trigger->period - event->trigger->offset, NULL);
        }

        // Copy the token pointer into the trigger struct so that the
        // reactions can access it. This overwrites the previous template token,
        // for which we decrement the reference count.
        if (event->trigger->token != event->token && event->trigger->token != NULL) {
            // Mark the previous one ok_to_free so we don't get a memory leak.
            event->trigger->token->ok_to_free = true;
            // Free the token if its reference count is zero. Since __done_using
            // decrements the reference count, first increment it here.
            event->trigger->token->ref_count++;
            __done_using(event->trigger->token);
        }
        event->trigger->token = token;
        // Prevent this token from being freed. It is the new template.
        // This might be null if there are no reactions to the action.
        if (token != NULL) token->ok_to_free = false;

        // Mark the trigger present.
        event->trigger->is_present = true;

        // Recycle the event.
        // So that sorting doesn't cost anything,
        // give all recycled events the same zero time stamp.
        event->time = 0LL;
        // Also remove pointers that will be replaced.
        event->token = NULL;
        pqueue_insert(recycle_q, event);

        // Peek at the next event in the event queue.
        event = pqueue_peek(event_q);
    } while(event != NULL && event->time == current_time);

}

/**
 * Schedule the specified trigger at current_time plus the offset of the
 * specified trigger plus the delay. See schedule_token() in reactor.h for details.
 * This is the internal implementation shared by both the threaded
 * and non-threaded versions.
 *
 * The value is required to be either
 * NULL or a pointer to a token wrapping the payload. The token carries
 * a reference count, and when the reference count decrements to 0,
 * the will be freed. Hence, it is essential that the payload be in
 * memory allocated using malloc.
 *
 * There are three conditions under which this function will not
 * actually put an event on the event queue and decrement the reference count
 * of the token (if there is one), which could result in the payload being
 * freed. In all three cases, this function returns 0. Otherwise,
 * it returns a handle to the scheduled trigger, which is an integer
 * greater than 0.
 *
 * The first condition is that a stop has been requested and the trigger
 * offset plus the extra delay is greater than zero.
 * The second condition is that the trigger offset plus the extra delay
 * is greater that the requested stop time (timeout).
 * The third condition is that the trigger argument is null.
 *
 * @param trigger The trigger to be invoked at a later logical time.
 * @param extra_delay The logical time delay, which gets added to the
 *  trigger's minimum delay, if it has one.
 * @param token The token wrapping the payload or NULL for no payload.
 * @return A handle to the event, or 0 if no new event was scheduled, or -1 for error.
 */
handle_t __schedule(trigger_t* trigger, interval_t extra_delay, token_t* token) {

    // printf("DEBUG: __schedule: scheduling trigger %p with delay %lld and token %p.\n", trigger, extra_delay, token);
    if (token != NULL) {
        // printf("DEBUG: __schedule: payload at %p.\n", token->value);
    }

	// The trigger argument could be null, meaning that nothing is triggered.
    // Doing this after incrementing the reference count ensures that the
    // payload will be freed, if there is one.
	if (trigger == NULL) {
	    __done_using(token);
	    return 0;
	}

    // Increment the reference count of the token.
	if (token != NULL) {
	    token->ref_count++;
	}

    // Compute the tag (the logical timestamp for the future event).
	// We first do this assuming it is logical action and then, if it is a
	// physical action, modify it if physical time exceeds the result.
    interval_t delay = trigger->offset + extra_delay;
    interval_t tag = current_time + delay;
    // printf("DEBUG: __schedule: current_time = %lld.\n", current_time);
    // printf("DEBUG: __schedule: total logical delay = %lld.\n", delay);
    interval_t min_inter_arrival = trigger->period;

    // Get an event_t struct to put on the event queue.
    // Recycle event_t structs, if possible.    
    event_t* e = pqueue_pop(recycle_q);
    if (e == NULL) {
        e = malloc(sizeof(struct event_t));
    }
    
    // Set the payload.
    e->token = token;

    // Make sure the event points to this trigger so when it is
    // dequeued, it will trigger this trigger.
    e->trigger = trigger;

    // If the trigger is physical, then we need to check whether
    // physical time is larger than the tag and, if so, update the tag.
    if (trigger->is_physical) {
        // Get the current physical time.
        instant_t physical_time = get_physical_time();

        if (physical_time > tag) {
            // printf("DEBUG: Physical time %lld is larger than the tag by %lld. Using physical time.\n", physical_time, physical_time - tag);
            // FIXME: In some circumstances (like Ptides), this is an
            // error condition because it introduces nondeterminism.
            // Do we want another kind of action, say a ptides action,
            // that is physical but flags or handles this error here
            // in some way?
            tag = physical_time;
        }
    }

    // Check to see whether the event is early based on the minimum interarrival time.
    // This check is not needed if this action has had no event.
    if (trigger->scheduled != NEVER) {
        instant_t earliest_time = trigger->scheduled + min_inter_arrival;

        // If the event is early, see which policy applies.
        event_t* existing = NULL;
        if (earliest_time > tag) {
            if (trigger->policy == DROP) {
                // Recycle the new event and the token.
                if (existing == NULL || existing->token != token) __done_using(token);
                e->token = NULL;
                pqueue_insert(recycle_q, e);
                return(0);
            }
            if (trigger->policy == DEFER) {
                // Adjust the tag.
                tag = earliest_time;
            }
        }
    }
    if (tag < current_time) {
        fprintf(stderr, "WARNING: Attempting to schedule an event earlier than current logical time by %lld nsec!\n"
                "Revising request to the current time %lld.\n", current_time - tag, current_time);
        tag = current_time;
    }

    // Set the tag of the event.
    e->time = tag;

    // Do not schedule events if a stop has been requested
    // and the event is strictly in the future (current microsteps are
    // allowed), or if the event time is past the requested stop time.
    // printf("DEBUG: Comparing event with elapsed time %lld against stop time %lld.\n", e->time - start_time, stop_time - start_time);
    if ((stop_requested && e->time != current_time)
            || (stop_time > 0LL && e->time > stop_time)) {
        // printf("DEBUG: __schedule: event time is past the timeout. Discarding event.\n");
        __done_using(token);
        e->token = NULL;
        pqueue_insert(recycle_q, e);
        return(0);
    }

    // Handle duplicate events for logical actions (events with the same tag).
    // This replaces the previous payload with the new one.
    if (!trigger->is_physical) {
        event_t* existing = pqueue_find_equal_same_priority(event_q, e);
        if (existing != NULL) {
            // Free the previous payload.
            if (existing->token != token) __done_using(existing->token);
            existing->token = token;
            // Recycle the new event.
            e->token = NULL;
            pqueue_insert(recycle_q, e);
            return(0);
        }
    }

    // Record the tag for the next check of MIT.
    trigger->scheduled = tag;

    // Queue the event.
    // NOTE: There is no need for an explicit microstep because
    // when this is called, all events at the current tag
    // (time and microstep) have been pulled from the queue,
    // and any new events added at this tag will go into the reaction_q
    // rather than the event_q, so anything put in the event_q with this
    // same time will automatically be executed at the next microstep.
    // printf("DEBUG: Inserting event in the event queue with elapsed time %lld.\n", e->time - start_time);
    pqueue_insert(event_q, e);
    // FIXME: make a record of handle and implement unschedule.
    // NOTE: Rather than wrapping around to get a negative number,
    // we reset the handle on the assumption that much earlier
    // handles are irrelevant.
    int return_value = __handle++;
    if (__handle < 0) __handle = 1;
    return return_value;
}

/**
 * For the specified reaction, if it has produced outputs, insert the
 * resulting triggered reactions into the reaction queue.
 */
void schedule_output_reactions(reaction_t* reaction) {
    // If the reaction produced outputs, put the resulting triggered
    // reactions into the blocking queue.
    for(int i=0; i < reaction->num_outputs; i++) {
        if (*(reaction->output_produced[i])) {
            trigger_t** triggerArray = (reaction->triggers)[i];
            for (int j=0; j < reaction->triggered_sizes[i]; j++) {
                trigger_t* trigger = triggerArray[j];
                if (trigger != NULL) {
                    for (int k=0; k < trigger->number_of_reactions; k++) {
                        reaction_t* reaction = trigger->reactions[k];
                        if (reaction != NULL) {
                            // Do not enqueue this reaction twice.
                            if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
                                pqueue_insert(reaction_q, reaction);
                            }
                        }
                    }
                }
            }
        }
	}
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with no payload (no value conveyed).
 * See schedule_token(), which this uses, for details.
 * @param trigger Pointer to a trigger object (typically an action on a self struct).
 * @param offset The time offset over and above that in the action.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule(trigger_t* trigger, interval_t offset) {
    return schedule_token(trigger, offset, NULL);
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value. If the value is non-null,
 * then it will be copied into newly allocated memory under the assumption
 * that its size given in the trigger's token object's element_size field
 * multiplied by the specified length.
 * See schedule_token(), which this uses, for details.
 * @param trigger Pointer to a trigger object (typically an action on a self struct).
 * @param offset The time offset over and above that in the action.
 * @param value A pointer to the value to copy.
 * @param length The length, if an array, 1 if a scalar, and 0 if value is NULL.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_copy(trigger_t* trigger, interval_t offset, void* value, int length) {
    if (value == NULL) {
        return schedule_token(trigger, offset, NULL);
    }
    if (trigger == NULL || trigger->token == NULL || trigger->token->element_size <= 0) {
        fprintf(stderr, "ERROR: schedule: Invalid trigger or element size.\n");
        return -1;
    }
    int element_size = trigger->token->element_size;
    void* container = malloc(element_size * length);
    __count_payload_allocations++;
    // printf("DEBUG: __schedule_with_copy_impl: Allocating memory for payload (token value): %p\n", container);
    memcpy(container, value, element_size * length);
    // Initialize token with an array size of length and a reference count of 0.
    token_t* token = __initialize_token(trigger->token, container, trigger->element_size, length, 0);
    // The schedule function will increment the reference count.
    return schedule_token(trigger, offset, token);
}

/**
 * Schedule the specified action at a later logical time that depends
 * on whether the action is logical or physical and what its parameter
 * values are.
 *
 * logical action: A logical action has an offset (default is zero)
 * and a minimum interarrival time (MIT), which also defaults to zero.
 * The logical time at which this scheduled event will trigger is
 * the current time plus the offset plus the delay argument given to
 * this function. If, however, that time is not greater than a prior
 * triggering of this logical action by at least the MIT, then the
 * one of two things can happen depending on the policy specified
 * for the action. If the action's policy is DROP (default), then the
 * action is simply dropped and the memory pointed to by value argument
 * is freed. If the policy is DEFER, then the time will be increased
 * to equal the time of the most recent triggering plus the MIT.
 *
 * For the above, "current time" means the logical time of the
 * reaction that is calling this function, or if this function is
 * being called from outside a reaction (asynchronously), then the
 * most recent logical time relevant to the reactor that owns the
 * action.
 *
 * physical action: A physical action has all the same parameters
 * as a logical action, but instead of "current time" being a logical
 * time, current time is defined as the larger of the current logical
 * time (as above) and the time of the physical clock on the currently
 * executing platform.
 *
 * The value is required to be either NULL or a pointer to
 * memory dynamically allocated using malloc (it will be freed when
 * its reference count decrements to zero).
 *
 * There are three conditions under which this function will not
 * actually put an event on the event queue and decrement the reference count
 * of the token (if there is one), which could result in the payload being
 * freed. In all three cases, this function returns 0. Otherwise,
 * it returns a handle to the scheduled trigger, which is an integer
 * greater than 0.
 *
 * The first condition is that a stop has been requested and the time offset
 * of this event is greater than zero.
 * The second condition is that the logical time of the event
 * is greater that the requested stop time (timeout).
 * The third condition is that the trigger argument is null.
 *
 * @param action The action or timer to be triggered.
 * @param extra_delay Extra offset of the event release.
 * @param value Dynamically allocated memory containing the value to send.
 * @param length The length of the array, if it is an array, or 1 for a
 *  scalar and 0 for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_value(trigger_t* trigger, interval_t extra_delay, void* value, int length) {
    token_t* token = create_token(trigger->element_size);
    token->value = value;
    token->length = length;
    return schedule_token(trigger, extra_delay, token);
}

/**
 * Schedule the specified action with an integer value at a later logical
 * time that depends on whether the action is logical or physical and
 * what its parameter values are. See schedule_value().
 */
handle_t schedule_int(trigger_t* trigger, interval_t extra_delay, int value) {
    if (trigger->element_size != sizeof(int)) {
        fprintf(stderr, "Action type is not an integer.");
        return -1;
    }
    int* container = malloc(sizeof(int));
    *container = value;
    return schedule_value(trigger, extra_delay, container, 1);
}

/**
 * Library function for allocating memory for an array to be sent on an output.
 * This turns over "ownership" of the allocated memory to the output, so
 * the allocated memory will be freed downstream.
 * @param token The token to use as a template (or if it is free, to use).
 * @param length The length of the array.
 * @param num_destinations The number of destinations (for initializing the reference count).
 * @return A pointer to the new or reused token.
 */
void* __set_new_array_impl(token_t* token, int length, int num_destinations) {
    // First, initialize the token, reusing the one given if possible.
    // FIXME: This token gets lost!!!!
    token_t* new_token = __initialize_token(token, malloc(token->element_size * length), token->element_size, length, num_destinations);
    // printf("DEBUG: __set_new_array_impl: Allocated memory for payload %p\n", new_token->value);
    // Count allocations to issue a warning if this is never freed.
    __count_payload_allocations++;
    return new_token;
}

/**
 * Return a writable copy of the specified token.
 * If the reference count is 1, this returns the original token rather than a copy.
 * The reference count will still be 1.
 * Otherwise, this returns a new token with a reference count of 0.
 * To ensure that the allocated memory is not leaked, this new token must be
 * either passed to an output using set_token() or scheduled with a action
 * using schedule_token().
 */
token_t* writable_copy(token_t* token) {
    // printf("DEBUG: writable_copy: Requesting writable copy of token %p with reference count %d.\n", token, token->ref_count);
    if (token->ref_count == 1) {
        // printf("DEBUG: writable_copy: Avoided copy because reference count is %d.\n", token->ref_count);
        return token;
   } else {
        // printf("DEBUG: writable_copy: Copying array because reference count is greater than 1. It is %d.\n", token->ref_count);
        size_t size = token->element_size * token->length;
        void* copy = malloc(size);
        // printf("DEBUG: Allocating memory for writable copy %p.\n", copy);
        memcpy(copy, token->value, size);
        // Count allocations to issue a warning if this is never freed.
        __count_payload_allocations++;
        // Create a new, dynamically allocated token.
        token_t* result = create_token(token->element_size);
        result->length = token->length;
        result->value = copy;
        return result;
    }
}

/**
 * Print a usage message.
 */
void usage(int argc, char* argv[]) {
    printf("\nCommand-line arguments: \n\n");
    printf("  -f, --fast [true | false]\n");
    printf("   Whether to wait for physical time to match logical time.\n\n");
    printf("  -o, --timeout <duration> <units>\n");
    printf("   Stop after the specified amount of logical time, where units are one of\n");
    printf("   nsec, usec, msec, sec, minute, hour, day, week, or the plurals of those.\n\n");
    printf("  -k, --keepalive\n");
    printf("   Whether continue execution even when there are no events to process.\n\n");
    printf("  -t, --threads <n>\n");
    printf("   Executed in <n> threads if possible (optional feature).\n\n");

    printf("Command given:\n");
    for (int i = 0; i < argc; i++) {
        printf("%s ", argv[i]);
    }
    printf("\n\n");
}

// Some options given in the target directive are provided here as
// default command-line options.
int default_argc = 0;
char** default_argv = NULL;

/**
 * Process the command-line arguments. If the command line arguments are not
 * understood, then print a usage message and return 0. Otherwise, return 1.
 * @return 1 if the arguments processed successfully, 0 otherwise.
 */
int process_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--fast") == 0) {
            if (argc < i + 2) {
                printf("Error: --fast needs a boolean.\n");
                usage(argc, argv);
                return 0;
            }
            i++;
            char* fast_spec = argv[i];
            if (strcmp(fast_spec, "true") == 0) {
                fast = true;
            } else if (strcmp(fast_spec, "false") == 0) {
                fast = false;
            } else {
                printf("Error: Invalid value for --fast: %s\n", fast_spec);
            }
       } else if (strcmp(argv[i], "-o") == 0
               || strcmp(argv[i], "--timeout") == 0
               || strcmp(argv[i], "-timeout") == 0) {
           // Tolerate -timeout for legacy uses.
           if (argc < i + 3) {
               printf("Error: --timeout needs time and units.\n");
               usage(argc, argv);
               return 0;
           }
           i++;
           char* time_spec = argv[i++];
           char* units = argv[i];
           duration = atoll(time_spec);
           // A parse error returns 0LL, so check to see whether that is what is meant.
           if (duration == 0LL && strncmp(time_spec, "0", 1) != 0) {
        	   // Parse error.
        	   printf("Error: invalid time value: %s", time_spec);
        	   usage(argc, argv);
        	   return 0;
           }
           if (strncmp(units, "sec", 3) == 0) {
        	   duration = SEC(duration);
           } else if (strncmp(units, "msec", 4) == 0) {
        	   duration = MSEC(duration);
           } else if (strncmp(units, "usec", 4) == 0) {
        	   duration = USEC(duration);
           } else if (strncmp(units, "nsec", 4) == 0) {
        	   duration = NSEC(duration);
           } else if (strncmp(units, "min", 3) == 0) {
        	   duration = MINUTE(duration);
           } else if (strncmp(units, "hour", 4) == 0) {
        	   duration = HOUR(duration);
           } else if (strncmp(units, "day", 3) == 0) {
        	   duration = DAY(duration);
           } else if (strncmp(units, "week", 4) == 0) {
        	   duration = WEEK(duration);
           } else {
        	   // Invalid units.
        	   printf("Error: invalid time units: %s", units);
        	   usage(argc, argv);
        	   return 0;
           }
       } else if (strcmp(argv[i], "-k") == 0 || strcmp(argv[i], "--keepalive") == 0) {
    	   if (argc < i + 2) {
    		   printf("Error: --keepalive needs a boolean.\n");
    		   usage(argc, argv);
    		   return 0;
    	   }
    	   i++;
    	   char* keep_spec = argv[i];
    	   if (strcmp(keep_spec, "true") == 0) {
    		   keepalive_specified = true;
    	   } else if (strcmp(keep_spec, "false") == 0) {
    		   keepalive_specified = false;
    	   } else {
    		   printf("Error: Invalid value for --keepalive: %s\n", keep_spec);
    	   }
       } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--threads") == 0) {
    	   if (argc < i + 2) {
    		   printf("Error: --threads needs an integer argument.\n");
    		   usage(argc, argv);
    		   return 0;
    	   }
    	   i++;
    	   char* threads_spec = argv[i++];
    	   number_of_threads = atoi(threads_spec);
    	   if (number_of_threads <= 0) {
    		   printf("Error: Invalid value for --threads: %s\n", threads_spec);
    	   }
       } else {
    	   printf("Error: Unrecognized command-line argument: %s\n", argv[i]);
    	   usage(argc, argv);
    	   return 0;
       }
    }
    return 1;
}

/**
 * Initialize the priority queues and set logical time to match
 * physical time. This also prints a message reporting the start time.
 */
void initialize() {
    __count_payload_allocations = 0;
    __count_token_allocations = 0;
#if _WIN32 || WIN32
    HMODULE ntdll = GetModuleHandleA("ntdll.dll");
    if (ntdll) {
        NtDelayExecution = (NtDelayExecution_t *)GetProcAddress(ntdll, "NtDelayExecution");
        NtQueryPerformanceCounter = (NtQueryPerformanceCounter_t *)GetProcAddress(ntdll, "NtQueryPerformanceCounter");
        NtQuerySystemTime = (NtQuerySystemTime_t *)GetProcAddress(ntdll, "NtQuerySystemTime");
    }
#endif

    // Initialize our priority queues.

    // Reaction queue ordered first by deadline, then by level.
    // The index of the reaction holds the deadline in the 48 most significant bits,
    // the level in the 16 least significant bits.
    reaction_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);    

    event_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, in_reverse_order, get_event_time,
            get_event_position, set_event_position, event_matches, print_event);
	// NOTE: The recycle queue does not need to be sorted. But here it is.
    recycle_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, in_reverse_order, get_event_time,
            get_event_position, set_event_position, event_matches, print_event);

    // Initialize the trigger table.
    __initialize_trigger_objects();

    // Initialize logical time to match physical time.
    struct timespec actualStartTime;
    clock_gettime(CLOCK_REALTIME, &actualStartTime);
    physical_start_time = actualStartTime.tv_sec * BILLION + actualStartTime.tv_nsec;

    printf("---- Start execution at time %s---- plus %ld nanoseconds.\n",
            ctime(&actualStartTime.tv_sec), actualStartTime.tv_nsec);
    current_time = physical_start_time;
    start_time = current_time;
    
    if (duration >= 0LL) {
        // A duration has been specified. Calculate the stop time.
        stop_time = current_time + duration;
    }
}

// Check that memory allocated by set_new, set_new_array, or writable_copy
// has been freed and print a warning message if not.
void termination() {
    // Invoke the code generated termination function.
    __termination();

    // If the event queue still has events on it, report that.
    if (pqueue_size(event_q) > 0) {
        printf("---- There are %zu unprocessed future events on the event queue.\n", pqueue_size(event_q));
        event_t* event = pqueue_peek(event_q);
        interval_t event_time = event->time - start_time;
        printf("---- The first future event has timestamp %lld after start time.\n", event_time);
    }
    // Issue a warning if a memory leak has been detected.
    if (__count_payload_allocations > 0) {
        printf("**** WARNING: Memory allocated for messages has not been freed.\n");
        printf("**** Number of unfreed messages: %d.\n", __count_payload_allocations);
    }
    if (__count_token_allocations > 0) {
        printf("**** WARNING: Memory allocated for tokens has not been freed!\n");
        printf("**** Number of unfreed tokens: %d.\n", __count_token_allocations);
    }
    // Print elapsed times.
    interval_t elapsed_logical_time = current_time - physical_start_time;
    printf("---- Elapsed logical time (in nsec): ");
    print_time(elapsed_logical_time);
    printf("\n");

    struct timespec physicalEndTime;
    clock_gettime(CLOCK_REALTIME, &physicalEndTime);
    interval_t elapsed_physical_time
        = (physicalEndTime.tv_sec * BILLION + physicalEndTime.tv_nsec)
        - physical_start_time;
    printf("---- Elapsed physical time (in nsec): ");
    print_time(elapsed_physical_time);
    printf("\n");
}

// ********** Start Windows Support
// Windows is not POSIX, so we include here compatibility definitions.
#if _WIN32 || WIN32
NtDelayExecution_t *NtDelayExecution = NULL;
NtQueryPerformanceCounter_t *NtQueryPerformanceCounter = NULL;
NtQuerySystemTime_t *NtQuerySystemTime = NULL;
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    int result = -1;
    int days_from_1601_to_1970 = 134774 /* there were no leap seconds during this time, so life is easy */;
    long long timestamp, counts, counts_per_sec;
    switch (clk_id) {
    case CLOCK_REALTIME:
        NtQuerySystemTime((PLARGE_INTEGER)&timestamp);
        timestamp -= days_from_1601_to_1970 * 24LL * 60 * 60 * 1000 * 1000 * 10;
        tp->tv_sec = (time_t)(timestamp / (BILLION / 100));
        tp->tv_nsec = (long)((timestamp % (BILLION / 100)) * 100);
        result = 0;
        break;
    case CLOCK_MONOTONIC:
        if ((*NtQueryPerformanceCounter)((PLARGE_INTEGER)&counts, (PLARGE_INTEGER)&counts_per_sec) == 0) {
            tp->tv_sec = counts / counts_per_sec;
            tp->tv_nsec = (long)((counts % counts_per_sec) * BILLION / counts_per_sec);
            result = 0;
        } else {
            errno = EINVAL;
            result = -1;
        }
        break;
    default:
        errno = EINVAL;
        result = -1;
        break;
    }
    return result;
}
int nanosleep(const struct timespec *req, struct timespec *rem) {
    unsigned char alertable = rem ? 1 : 0;
    long long duration = -(req->tv_sec * (BILLION / 100) + req->tv_nsec / 100);
    NTSTATUS status = (*NtDelayExecution)(alertable, (PLARGE_INTEGER)&duration);
    int result = status == 0 ? 0 : -1;
    if (alertable) {
        if (status < 0) {
            errno = EINVAL;
        } else if (status > 0 && clock_gettime(CLOCK_MONOTONIC, rem) == 0) {
            errno = EINTR;
        }
    }
    return result;
}
#endif
// ********** End Windows Support

// Patmos does not have an epoch, so it does not have clock_gettime
// clock() looks like not working, use the hardware counter of Patmos
#ifdef __PATMOS__
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    // TODO: use all 64 bits of the timer
    int timestamp = TIMER_US_LOW;
// printf("Time %d\n", timestamp);
    tp->tv_sec = timestamp/1000000;
    tp->tv_nsec = (timestamp%1000000) * 1000;
// printf("clock_gettime: %lld %ld\n", tp->tv_sec, tp->tv_nsec);
    return 0;
}

int nanosleep(const struct timespec *req, struct timespec *rem) {

    // We could use our deadline device here
    int timestamp = TIMER_US_LOW;
// printf("nanosleep: %lld %ld\n", req->tv_sec, req->tv_nsec);
    
    timestamp += req->tv_sec * 1000000 + req->tv_nsec / 1000;
// printf("sleep to %d\n", timestamp);
    while (timestamp - TIMER_US_LOW > 0) {
        ;
// printf("time %d\n", TIMER_US_LOW);
    }
    if (rem != 0) {
        rem->tv_sec = 0;
        rem->tv_nsec = 0;

    }
    return 0;
}
#endif
