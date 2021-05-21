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

/**
 * Runtime infrastructure for the C target of Lingua Franca.
 * This file contains resources that are shared by the threaded and
 * non-threaded versions of the C runtime.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Mehrdad Niknami <mniknami@berkeley.edu>}
 *  @author{Soroush Bateni <soroush@utdallas.edu}
 */
#include "reactor.h"
#include "tag.c"
#include "pqueue.c"
#include "util.c"

/** 
 * Indicator of whether to wait for physical time to match logical time.
 * By default, execution will wait. The command-line argument -fast will
 * eliminate the wait and allow logical time to exceed physical time.
 */ 
bool fast = false;

/**
 * The number of worker threads for threaded execution.
 * By default, execution is not threaded and this variable will have value 0,
 * meaning that the execution is not threaded.
 */
unsigned int _lf_number_of_threads;

/** 
 * The logical time to elapse during execution, or -1 if no timeout time has
 * been given. When the logical equal to start_time + duration has been
 * reached, execution will terminate.
 */
instant_t duration = -1LL;

/**
 * Indicates whether or not the execution
 * has started.
 */
bool _lf_execution_started = false;

/**
 * The tag at which the Lingua Franca program should stop.
 * It will be initially set to timeout if it is set. However,
 * starvation or calling request_stop() can also alter the stop_tag by moving it
 * earlier.
 * 
 * FIXME: This variable might need to be volatile
 */
tag_t stop_tag = FOREVER_TAG;

/** Indicator of whether the keepalive command-line option was given. */
bool keepalive_specified = false;

// Define the array of pointers to the _is_present fields of all the
// self structs that need to be reinitialized at the start of each time step.
// NOTE: This may have to be resized for a mutation.
bool** __is_present_fields = NULL;
int __is_present_fields_size = 0;

// Define the array of pointers to the intended_tag fields of all
// ports and actions that need to be reinitialized at the start
// of each time step.
tag_t** __intended_tag_fields = NULL;
int __intended_tag_fields_size = 0;

// Define the array of pointers to the token fields of all the
// actions and inputs that need to have their reference counts
// decremented at the start of each time step.
// NOTE: This may have to be resized for a mutation.
token_present_t* __tokens_with_ref_count = NULL;
// Dynamically created list of tokens that are copies made
// as a result of mutable inputs. These need to also have
// __done_using() called on them at the start of the next time step.
lf_token_t* _lf_more_tokens_with_ref_count = NULL;
int __tokens_with_ref_count_size = 0;

/**
 * Global STP offset uniformly applied to advancement of each
 * time step in federated execution. This can be retrieved in
 * user code by calling get_stp_offset() and adjusted by
 * calling set_stp_offset(interval_t offset).
 */
interval_t _lf_global_time_STP_offset = 0LL;

#ifdef FEDERATED
/**
 * Reset absent fields on network input ports.
 * @note Defined in federate.c
 */
void reset_status_fields_on_input_port_triggers();

/**
 * Enqueue network control reactions.
 */
void enqueue_network_control_reactions(pqueue_t* reaction_q);

/**
 * Determine the status of the port at the current logical time.
 * If successful, return true. If the status cannot be determined
 * at this moment, return false.
 * 
 * @param portID the ID of the port to determine status for
 */
port_status_t determine_port_status_if_possible(int portID);
#endif

/**
 * Set the stop tag.
 * 
 * This function will always choose the minimum
 * of the provided tag and stop_tag
 * 
 * @note In threaded programs, the mutex must be locked before
 *  calling this function.
 */
void _lf_set_stop_tag(tag_t tag) {
    if (compare_tags(tag, stop_tag) < 0) {
        stop_tag = tag;
    }
}

/////////////////////////////
// The following functions are in scope for all reactors:

/**
 * Return the global STP offset on advancement of logical
 * time for federated execution.
 */
interval_t get_stp_offset() {
    return _lf_global_time_STP_offset;
}

/**
 * Set the global STP offset on advancement of logical
 * time for federated execution.
 * 
 * @param offset A positive time value to be applied
 *  as the STP offset.
 */
void set_stp_offset(interval_t offset) {
    if (offset > 0LL) {
        _lf_global_time_STP_offset = offset;
    }
}

/**
 * Print a non-negative time value in nanoseconds with commas separating thousands
 * into the specified buffer. Ideally, this would use the locale to
 * use periods if appropriate, but I haven't found a sufficiently portable
 * way to do that.
 * @param buffer A buffer long enough to contain a string like "9,223,372,036,854,775,807".
 * @param time A time value.
 */
void readable_time(char* buffer, instant_t time) {
    // If the number is negative or below 1000, just print it and return.
    if (time < 1000LL) {
        sprintf(buffer, "%lld", time);
        return;
    }
    int count = 0;
    instant_t clauses[7];
    while (time > 0LL) {
        clauses[count++] = time;
        time = time/1000LL;
    }
    // Highest order clause should not be filled with zeros.
    instant_t to_print = clauses[--count] % 1000LL;
    sprintf(buffer, "%lld", to_print);
    if (to_print >= 100LL) {
        buffer += 3;
    } else if (to_print >= 10LL) {
        buffer += 2;
    } else {
        buffer += 1;
    }
    while (count-- > 1) {
        to_print = clauses[count] % 1000LL;
        sprintf(buffer, ",%03lld,", to_print);
        buffer += 4;
    }
    sprintf(buffer, ",%03lld", clauses[0] % 1000LL);
}

/////////////////////////////
// The following is not in scope for reactors:

/** Priority queues. */
pqueue_t* event_q;     // For sorting by time.

pqueue_t* reaction_q;  // For sorting by deadline.
pqueue_t* recycle_q;   // For recycling malloc'd events.
pqueue_t* next_q;      // For temporarily storing the next event lined 
                       // up in superdense time.

handle_t __handle = 1;

// ********** Priority Queue Support Start

/**
 * Return whether the first and second argument are given in reverse order.
 */
static int in_reverse_order(pqueue_pri_t thiz, pqueue_pri_t that) {
    return (thiz > that);
}

/**
 * Return whether the first and second argument are given in reverse order.
 */
static int in_no_particular_order(pqueue_pri_t thiz, pqueue_pri_t that) {
    return false;
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
 * 
 * DEBUG function only.
 */
static void print_reaction(void *reaction) {
	reaction_t *r = (reaction_t*)reaction;
    DEBUG_PRINT("%s: chain_id:%llu, index: %llx, reaction: %p",
    		r->name, r->chain_id, r->index, r);
}

/**
 * Print some information about the given event.
 * 
 * DEBUG function only.
 */
static void print_event(void *event) {
	event_t *e = (event_t*)event;
    DEBUG_PRINT("time: %lld, trigger: %p, token: %p",
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
lf_token_t* __token_recycling_bin = NULL;

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
token_freed __done_using(lf_token_t* token) {
    token_freed result = NOT_FREED;
    if (token == NULL) return result;
    if (token->ref_count == 0) {
        warning_print("Token being freed that has already been freed: %p", token);
        return NOT_FREED;
    }
    token->ref_count--;
    DEBUG_PRINT("__done_using: ref_count = %d.", token->ref_count);
    if (token->ref_count == 0) {
        if (token->value != NULL) {
            // Count frees to issue a warning if this is never freed.
            // Do not free the value field if it is garbage collected.
            __count_payload_allocations--;
            if(OK_TO_FREE != token_only) {
                DEBUG_PRINT("__done_using: Freeing allocated memory for payload (token value): %p", token->value);
                free(token->value);
            }
            token->value = NULL;
            result = VALUE_FREED;
        }
        // Tokens that are created at the start of execution and associated with
        // output ports or actions are pointed to by those actions and output
        // ports and should not be freed. They are expected to be reused instead.
        if (token->ok_to_free) {
            // Need to free the lf_token_t struct also.
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
            DEBUG_PRINT("__done_using: Freeing allocated memory for token: %p", token);
            result = TOKEN_FREED;
        }
    }
    return result;
}

/**
 * Put the specified reaction on the reaction queue.
 * This version is just a template.
 * @param reaction The reaction.
 */
void _lf_enqueue_reaction(reaction_t* reaction);

/**
 * Use tables to reset is_present fields to false,
 * set intended_tag fields in federated execution
 * to the current_tag, and decrement reference
 * counts between time steps and at the end of execution.
 */
void __start_time_step() {
    LOG_PRINT("--------- Start time step at tag (%lld, %u).", current_tag.time - start_time, current_tag.microstep);
    for(int i = 0; i < __tokens_with_ref_count_size; i++) {
        if (*(__tokens_with_ref_count[i].status) == present) {
            if (__tokens_with_ref_count[i].reset_is_present) {
                *(__tokens_with_ref_count[i].status) = absent;
            }
            __done_using(*(__tokens_with_ref_count[i].token));
        }
    }
    // Also handle dynamically created tokens for mutable inputs.
    while (_lf_more_tokens_with_ref_count != NULL) {
        lf_token_t* next = _lf_more_tokens_with_ref_count->next_free;
        __done_using(_lf_more_tokens_with_ref_count);
        _lf_more_tokens_with_ref_count = next;
    }
    for(int i = 0; i < __is_present_fields_size; i++) {
        *__is_present_fields[i] = false;
#ifdef FEDERATED_DECENTRALIZED
        // FIXME: For now, an intended tag of (NEVER, 0)
        // indicates that it has never been set.
        *__intended_tag_fields[i] = (tag_t) {NEVER, 0};
#endif
    }
#ifdef FEDERATED
    // Reset absent fields on network ports because
    // their status is unknown
    reset_status_fields_on_input_port_triggers();
#endif
}

/**
 * Create a new lf_token_t struct and initialize it for assignment to a trigger.
 * The value pointer will be NULL and the length will be 0.
 * This function is for tokens that are not expected to be freed, and
 * reactors are not expected to use it. It is used by the code generator
 * to initialize actions with tokens.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A new or recycled lf_token_t struct.
 */
lf_token_t* __create_token(size_t element_size) {
    lf_token_t* token;
    // Check the recycling bin.
    if (__token_recycling_bin != NULL) {
        token = __token_recycling_bin;
        __token_recycling_bin = token->next_free;
        __token_recycling_bin_size--;
        DEBUG_PRINT("__create_token: Retrieved token from the recycling bin: %p", token);
    } else {
        token = (lf_token_t*)malloc(sizeof(lf_token_t));
        DEBUG_PRINT("__create_token: Allocated memory for token: %p", token);
    }
    token->value = NULL;
    token->length = 0;
    token->element_size = element_size;
    token->ref_count = 0;
    token->ok_to_free = no;
    token->next_free = NULL;
    return token;
}

/**
 * Create a new token and initialize it.
 * The value pointer will be NULL and the length will be 0.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A new or recycled lf_token_t struct.
 * 
 * @note For multithreaded applications, the caller must hold
 *  the mutex lock because it accesses global variables.
 */
lf_token_t* create_token(size_t element_size) {
    DEBUG_PRINT("create_token: element_size: %zu", element_size);
    __count_token_allocations++;
    lf_token_t* result = __create_token(element_size);
    result->ok_to_free = OK_TO_FREE;
    return result;
}

/**
 * Return a token for storing an array of the specified length
 * with the specified value containing the array.
 * If the specified token is available (its reference count is 0),
 * then reuse it. Otherwise, create a new token.
 * The element_size for elements of the array is specified by
 * the specified token.
 *
 * @param token The token to populate, if it is available (must not be NULL).
 * @param value The value of the array.
 * @param length The length of the array, or 1 if it is not an array.
 * @return Either the specified token or a new one, in each case with a value
 *  field pointing to newly allocated memory.
 */
lf_token_t* __initialize_token_with_value(lf_token_t* token, void* value, int length) {
    // assert(token != NULL);

    // If necessary, allocate memory for a new lf_token_t struct.
    // This assumes that the lf_token_t* in the self struct has been initialized to NULL.
    lf_token_t* result = token;
    DEBUG_PRINT("Initializing a token %p with ref_count %d.", token, token->ref_count);
    if (token == NULL || token->ref_count > 0) {
        // The specified token is not available.
        result = create_token(token->element_size);
    }
    result->value = value;
    result->length = length;
    return result;
}

/**
 * Return a token for storing an array of the specified length
 * with new memory allocated (using malloc) for storing that array.
 * If the specified token is available (its reference count is 0),
 * then reuse it. Otherwise, create a new token.
 * The element_size for elements of the array is specified by
 * the specified token. The caller should populate the value and
 * ref_count field of the returned token after this returns.
 *
 * @param token The token to populate, if it is available (must not be NULL).
 * @param length The length of the array, or 1 if it is not an array.
 * @return Either the specified token or a new one, in each case with a value
 *  field pointing to newly allocated memory.
 */
lf_token_t* __initialize_token(lf_token_t* token, int length) {
    // assert(token != NULL);

    // Allocate memory for storing the array.
    void* value = malloc(token->element_size * length);
    // Count allocations to issue a warning if this is never freed.
    __count_payload_allocations++;
    return __initialize_token_with_value(token, value, length);
}

/**
 * A helper function that returns true if the provided tag is after stop tag.
 * 
 * @param tag The tag to check against stop tag
 */
bool _lf_is_tag_after_stop_tag(tag_t tag) {
    return (compare_tags(tag, stop_tag) > 0);
}

/**
 * Pop all events from event_q with timestamp equal to current_tag.time, extract all
 * the reactions triggered by these events, and stick them into the reaction
 * queue.
 */
void __pop_events() {
    event_t* event = (event_t*)pqueue_peek(event_q);
    while(event != NULL && event->time == current_tag.time) {
        event = (event_t*)pqueue_pop(event_q);
        
        if (event->is_dummy) {
        	DEBUG_PRINT("Popped dummy event from the event queue.");
        	if (event->next != NULL) {
            	DEBUG_PRINT("Putting event from the event queue for the next microstep.");
        		pqueue_insert(next_q, event->next);
        	}
            _lf_recycle_event(event);
            // Peek at the next event in the event queue.
            event = (event_t*)pqueue_peek(event_q);
            continue;
        }

        lf_token_t *token = event->token;

        // Put the corresponding reactions onto the reaction queue.
        for (int i = 0; i < event->trigger->number_of_reactions; i++) {
            reaction_t *reaction = event->trigger->reactions[i];
            // Do not enqueue this reaction twice.
            if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
#ifdef FEDERATED_DECENTRALIZED
                // In federated execution, an intended tag that is not (NEVER, 0)
                // indicates that this particular event is triggered by a network message.
                // The intended tag is set in handle_timed_message in federate.c whenever
                // a timed message arrives from another federate.
                if (event->intended_tag.time != NEVER) {
                    // If the intended tag of the event is actually set,
                    // transfer the intended tag to the trigger so that
                    // the reaction can access the value.
                    event->trigger->intended_tag = event->intended_tag;
                    // And check if it is in the past compared to the current tag.
                    if (compare_tags(event->intended_tag,
                                    current_tag) < 0) {
                        // Mark the triggered reaction with a STP violation
                        reaction->is_STP_violated = true;
                        LOG_PRINT("Trigger %p has violated the reaction's STP offset. Intended tag: (%lld, %u). Current tag: (%lld, %u)",
                                    event->trigger,
                                    event->intended_tag.time - start_time, event->intended_tag.microstep,
                                    current_tag.time - start_time, current_tag.microstep);
                    }
                }
#endif
                DEBUG_PRINT("Enqueing reaction %s.", reaction->name);
                pqueue_insert(reaction_q, reaction);
            } else {
                DEBUG_PRINT("Reaction is already on the reaction_q: %s", reaction->name);
            }
        }

        // Mark the trigger present.
        event->trigger->status = present;

        // If the trigger is a periodic timer, create a new event for its next execution.
        if (event->trigger->is_timer && event->trigger->period > 0LL) {
            // Reschedule the trigger.
            // Note that the delay here may be negative because the __schedule
            // function will add the trigger->offset, which we don't want at this point.
            // NULL argument indicates that there is no value.
            __schedule(event->trigger,
                    event->trigger->period - event->trigger->offset, NULL);
        }

        // Copy the token pointer into the trigger struct so that the
        // reactions can access it. This overwrites the previous template token,
        // for which we decrement the reference count.
        if (event->trigger->token != event->token
                && event->trigger->token != NULL) {
            // Mark the previous one ok_to_free so we don't get a memory leak.
            event->trigger->token->ok_to_free = OK_TO_FREE;
            // Free the token if its reference count is zero. Since __done_using
            // decrements the reference count, first increment it here.
            event->trigger->token->ref_count++;
            __done_using(event->trigger->token);
        }
        event->trigger->token = token;
        // Prevent this token from being freed. It is the new template.
        // This might be null if there are no reactions to the action.
        if (token != NULL) {
            token->ok_to_free = no;
        }

        // Mark the trigger present.
        event->trigger->status = present;
        
        // If this event points to a next event, insert it into the next queue.
        if (event->next != NULL) {
            // Insert the next event into the next queue.
            pqueue_insert(next_q, event->next);
        }

        _lf_recycle_event(event);
        
        // Peek at the next event in the event queue.
        event = (event_t*)pqueue_peek(event_q);
    };

#ifdef FEDERATED
    // Insert network dependent reactions for network input ports into
    // the reaction queue
    enqueue_network_control_reactions(reaction_q);
#endif // FEDERATED

    DEBUG_PRINT("There are %d events deferred to the next microstep.", pqueue_size(next_q));

    // After populating the reaction queue, see if there are things on the
    // next queue to put back into the event queue.
    while(pqueue_peek(next_q) != NULL) {
        pqueue_insert(event_q, pqueue_pop(next_q));
    }
}

/**
 * Initialize the given timer.
 * If this timer has a zero offset, enqueue the reactions it triggers.
 * If this timer is to trigger reactions at a _future_ tag as well, 
 * schedule it accordingly. 
 */
void _lf_initialize_timer(trigger_t* timer) {
    interval_t delay = 0;
    if (timer->offset == 0) {
        for (int i = 0; i < timer->number_of_reactions; i++) {
            _lf_enqueue_reaction(timer->reactions[i]);
            tracepoint_schedule(timer, 0LL); // Trace even though schedule is not called.
        }
        if (timer->period == 0) {
            return;
        } else {
            // Schedule at t + period.
            delay = timer->period;
        }
    } else {
        // Schedule at t + offset.
        delay = timer->offset;
    }

    // Get an event_t struct to put on the event queue.
    // Recycle event_t structs, if possible.    
    event_t* e = _lf_get_new_event();
    e->trigger = timer;
    e->time = get_logical_time() + delay;
    // NOTE: No lock is being held. Assuming this only happens at startup.
    pqueue_insert(event_q, e);
    tracepoint_schedule(timer, delay); // Trace even though schedule is not called.
}

/**
 * Get a new event. If there is a recycled event available, use that.
 * If not, allocate a new one. In either case, all fields will be zero'ed out.
 */
event_t* _lf_get_new_event() {
    // Recycle event_t structs, if possible.    
    event_t* e = (event_t*)pqueue_pop(recycle_q);
    if (e == NULL) {
        e = (event_t*)calloc(1, sizeof(struct event_t));
#ifdef FEDERATED_DECENTRALIZED
        e->intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
#endif
    }
    return e;
}

/**
 * Recycle the given event.
 * Zero it out and pushed it onto the recycle queue.
 */
void _lf_recycle_event(event_t* e) {
    e->time = 0LL;
    e->trigger = NULL;
    e->pos = 0;
    e->token = NULL;
    e->is_dummy = false;
#ifdef FEDERATED_DECENTRALIZED
    e->intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
#endif
    e->next = NULL;
    pqueue_insert(recycle_q, e);
}

/**
 * Create dummy events to be used as spacers in the event queue.
 * @param trigger The eventual event to be triggered.
 * @param time The logical time of that event.
 * @param next The event to place after the dummy events.
 * @param offset The number of dummy events to insert.
 * @return A pointer to the first dummy event.
 */
event_t* _lf_create_dummy_events(trigger_t* trigger, instant_t time, event_t* next, microstep_t offset) {
    event_t* first_dummy = _lf_get_new_event();
    event_t* dummy = first_dummy;
    dummy->time = time;
    dummy->is_dummy = true;
    dummy->trigger = trigger;
    while (offset > 0) {
        if (offset == 1) {
            dummy->next = next;
            break;
        }
        dummy->next = _lf_get_new_event();
        dummy = dummy->next;
        dummy->time = time;
        dummy->is_dummy = true;
        dummy->trigger = trigger;
        offset--;
    }
    return first_dummy;
}

/**
 * Replace the token on the specified event with the specified
 * token and free the old token.
 * @param event The event.
 * @param token The token.
 */
void _lf_replace_token(event_t* event, lf_token_t* token) {
    if (event->token != token) {
        // Free the existing token, if any
        __done_using(event->token);
    }
    // Replace the token with ours.
    event->token = token;
}

/**
 * Schedule events at a specific tag (time, microstep), provided
 * that the tag is in the future relative to the current tag.
 * The input time values are absolute.
 * 
 * If there is an event found at the requested tag, the payload
 * is replaced and 0 is returned.
 * 
 * Note that this function is an internal API that must
 * be called with tags that are in order for a given
 * trigger. This means that the following order is illegal:
 * _lf_schedule_at_tag(trigger1, bigger_tag, ...);
 * _lf_schedule_at_tag(trigger1, smaller_tag, ...);
 * where bigger_tag > smaller_tag. This function is primarily
 * used for network communication (which is assumed to be
 * in order).
 * 
 * This function assumes the caller holds the mutex lock.
 *
 * @param trigger The trigger to be invoked at a later logical time.
 * @param tag Logical tag of the event
 * @param token The token wrapping the payload or NULL for no payload.
 * 
 * @return 1 for success, 0 if no new event was scheduled (instead, the payload was updated),
 *  or -1 for error (the tag is equal to or less than the current tag).
 */
int _lf_schedule_at_tag(trigger_t* trigger, tag_t tag, lf_token_t* token) {

    tag_t current_logical_tag = get_current_tag();

    DEBUG_PRINT("_lf_schedule_at_tag() called with tag (%lld, %u) at tag (%lld, %u).",
                  tag.time - start_time, tag.microstep,
                  current_logical_tag.time - start_time, current_logical_tag.microstep);
    if (compare_tags(tag, current_logical_tag) <= 0) {
        warning_print("_lf_schedule_at_tag(): requested to schedule an event in the past.");
        return -1;
    }

    // Increment the reference count of the token.
    if (token != NULL) {
        token->ref_count++;
    }

    // Do not schedule events if the tag is after the stop tag
    if (_lf_is_tag_after_stop_tag(tag)) {
        warning_print("_lf_schedule_at_tag: event time is past the timeout. Discarding event.");
        __done_using(token);
        return -1;
    }
    
    event_t* e = _lf_get_new_event();
    // Set the event time
    e->time = tag.time;
    
    tracepoint_schedule(trigger, tag.time - current_logical_tag.time);

    // Make sure the event points to this trigger so when it is
    // dequeued, it will trigger this trigger.
    e->trigger = trigger;

    // Set the payload.
    e->token = token;

#ifdef FEDERATED_DECENTRALIZED
    // Set the intended tag
    e->intended_tag = trigger->intended_tag;
#endif

    event_t* found = (event_t *)pqueue_find_equal_same_priority(event_q, e);
    if (found != NULL) {
        if (tag.microstep == 0u) {
                // The microstep is 0, which means that the event is being scheduled
                // at a future time and at the beginning of the skip list of events 
                // at that time.
                // In case the event is a dummy event
                // convert it to a real event.                
                found->is_dummy = false;
                switch (trigger->policy) {
                    case drop:
                        if (found->token != token) {
                            __done_using(token);
                        }
                        _lf_recycle_event(e);
                        return(0);
                        break;
                    case replace:
                        // Replace the payload of the event at the head with our
                        // current payload.
                        _lf_replace_token(found, token);
                        _lf_recycle_event(e);
                        return 0;
                        break;
                    default:
                        // Adding a microstep to the original
                        // intended tag.
                        if (_lf_is_tag_after_stop_tag((tag_t) {.time=found->time,.microstep=1})) {
                            // Scheduling e will incur a microstep after the stop tag, 
                            // which is illegal.
                            _lf_recycle_event(e);
                            return 0;
                        }
                        if (found->next != NULL) {
                            error_print("_lf_schedule_at_tag: in-order contract violated.");
                            return -1;
                        }
                        found->next = e;
                }
        } else {
            // We are requesting a microstep greater than 0
            // where there is already an event for this trigger on the event queue.
            // That event may itself be a dummy event for a real event that is
            // also at a microstep greater than 0.
            // We have to insert our event into the chain or append it
            // to the end of the chain, depending on which microstep is lesser.
            microstep_t microstep_of_found = 0;
            if (tag.time == current_logical_tag.time) {
                // This is a situation where the head of the queue
                // is an event with microstep == current_microstep + 1
                // which should be reflected in our steps calculation.
                microstep_of_found += current_logical_tag.microstep + 1; // Indicating that
                                                            // the found event
                                                            // is at this microstep.
            }
            // Follow the chain of events until the right point
            // to insert the new event.
            while (microstep_of_found < tag.microstep - 1) {
                if (found->next == NULL) {
                    // The chain stops short of where we want to be.
                    // If it exactly one microstep short of where we want to be,
                    // then we don't need a dummy. Otherwise, we do.
                    microstep_t undershot_by = (tag.microstep - 1) - microstep_of_found;
                    if (undershot_by > 0) {
                        found->next = _lf_create_dummy_events(trigger, tag.time, e, undershot_by);
                    } else {
                        found->next = e;
                    }
                    return 1;
                }
                found = found->next;
                microstep_of_found++;
            }
            // At this point, microstep_of_found == tag.microstep - 1.
            if (found->next == NULL) {
                found->next = e;
            } else {
                switch (trigger->policy) {
                    case drop:
                        if (found->next->token != token) {
                            __done_using(token);
                        }
                        _lf_recycle_event(e);
                        return 0;
                        break;
                    case replace:
                        // Replace the payload of the event at the head with our
                        // current payload.
                        _lf_replace_token(found->next, token);
                        _lf_recycle_event(e);
                        return 0;
                        break;
                    default:
                        // Adding a microstep to the original
                        // intended tag.
                        if (_lf_is_tag_after_stop_tag((tag_t){.time=found->time,.microstep=microstep_of_found+1})) {
                            // Scheduling e will incur a microstep at timeout, 
                            // which is illegal.
                            _lf_recycle_event(e);
                            return 0;
                        }
                        if (found->next->next != NULL) {
                            error_print("_lf_schedule_at_tag: in-order contract violated.");
                            return -1;
                        }
                        found->next->next = e;
                }
            }
        }
    } else {
        // No existing event queued.
        microstep_t relative_microstep = tag.microstep;
        if (tag.time == current_logical_tag.time) {
            relative_microstep -= current_logical_tag.microstep;
        }
        if (((tag.time == current_logical_tag.time) && (relative_microstep == 1)) ||
                tag.microstep == 0) {
            // Do not need a dummy event if we are scheduling at 1 microstep
            // in the future at current time or at microstep 0 in a future time.
            pqueue_insert(event_q, e);
        } else {
            // Create a dummy event. Insert it into the queue, and let its next
            // pointer point to the actual event.
            pqueue_insert(event_q, _lf_create_dummy_events(trigger, tag.time, e, relative_microstep));
        }
    }
    return 1;
}

/**
 * Schedule the specified trigger at current_tag.time plus the offset of the
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
 *  trigger's minimum delay, if it has one. If this number is negative,
 *  then zero is used instead.
 * @param token The token wrapping the payload or NULL for no payload.
 * @return A handle to the event, or 0 if no new event was scheduled, or -1 for error.
 */
handle_t __schedule(trigger_t* trigger, interval_t extra_delay, lf_token_t* token) {
    if (_lf_is_tag_after_stop_tag(current_tag)) {
        // If schedule is called after stop_tag
        // This is a critical condition.
        __done_using(token);
        warning_print("schedule() called after stop tag.");
        return 0;
    }

    if (extra_delay < 0LL) {
        warning_print("schedule called with a negative extra_delay %lld. Replacing with zero.", extra_delay);
        extra_delay = 0LL;
    }

    DEBUG_PRINT("__schedule: scheduling trigger %p with delay %lld and token %p.",
            trigger, extra_delay, token);
    
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
    interval_t intended_time = current_tag.time + delay;
    DEBUG_PRINT("__schedule: current_tag.time = %lld. Total logical delay = %lld",
            current_tag.time, delay);
    interval_t min_spacing = trigger->period;

    event_t* e = _lf_get_new_event();
    
    // Initialize the next pointer.
    e->next = NULL;

    // Set the payload.
    e->token = token;

    // Make sure the event points to this trigger so when it is
    // dequeued, it will trigger this trigger.
    e->trigger = trigger;

    // If the trigger is physical, then we need to check whether
    // physical time is larger than the intended time and, if so,
    // modify the intended time.
    if (trigger->is_physical) {
        // Get the current physical time and assign it as the intended time.
        intended_time = get_physical_time() + delay;
    } else {
        // FIXME: We need to verify that we are executing within a reaction?
        // See reactor_threaded.
        // If a logical action is scheduled asynchronously (which should never be
        // done) the computed tag can be smaller than the current tag, in which case
        // it needs to be adjusted.
        // FIXME: This can go away once:
        // - we have eliminated the possibility to have a negative additional delay; and
        // - we detect the asynchronous use of logical actions
        if (intended_time < current_tag.time) {
            warning_print("Attempting to schedule an event earlier than current time by %lld nsec! "
                    "Revising to the current time %lld.",
                    current_tag.time - intended_time, current_tag.time);
            intended_time = current_tag.time;
        }
    }

#ifdef FEDERATED_DECENTRALIZED
    // Event inherits the original intended_tag of the trigger
    // set by the network stack (or the default, which is (NEVER,0))
    e->intended_tag = trigger->intended_tag;
#endif
    
    event_t* existing = (event_t*)(trigger->last);
    // Check for conflicts (a queued event with the same trigger and time).
    if (trigger->period < 0) {
        // No minimum spacing defined.
        tag_t intended_tag = (tag_t) {.time = intended_time, .microstep = 0u};
        e->time = intended_tag.time;
        event_t* found = (event_t *)pqueue_find_equal_same_priority(event_q, e);
        // Check for conflicts. Let events pile up in super dense time.
        if (found != NULL) {
            intended_tag.microstep++;
            // Skip to the last node in the linked list.
            while(found->next != NULL) {
                found = found->next;
                intended_tag.microstep++;
            }
            if (_lf_is_tag_after_stop_tag(intended_tag)) {
                DEBUG_PRINT("Attempt to schedule an event after stop_tag was rejected.");
                // Scheduling an event will incur a microstep
                // after the stop tag.
                _lf_recycle_event(e);
                return 0;
            }
            // Hook the event into the list.
            found->next = e;
            return(0); // FIXME: return value
        }
        // If there are not conflicts, schedule as usual. If intended time is
        // equal to the current logical time, the event will effectively be 
        // scheduled at the next microstep.
    } else if (!trigger->is_timer && existing != NULL) { 
        // There exists a previously scheduled event. It determines the
        // earliest time at which the new event can be scheduled.
        // Check to see whether the event is too early. 
        instant_t earliest_time = existing->time + min_spacing;
        DEBUG_PRINT("There is a previously scheduled event; earliest possible time "
                "with min spacing: %lld",
                earliest_time);
        // If the event is early, see which policy applies.
        if (earliest_time >= intended_time) {
            DEBUG_PRINT("Event is early.");
            switch(trigger->policy) {
                case drop:
                    DEBUG_PRINT("Policy is drop. Dropping the event.");
                    if (min_spacing > 0 || 
                            pqueue_find_equal_same_priority(event_q, existing) != NULL) {
                        // Recycle the new event and the token.
                        if (existing->token != token) {
                            __done_using(token);
                        }
                        _lf_recycle_event(e);
                        return(0);
                    }
                case replace:
                    DEBUG_PRINT("Policy is replace. Replacing the previous event.");
                    // If the existing event has not been handled yet, update
                    // it. WARNING: If provide a mechanism for unscheduling, we
                    // can no longer rely on the tag of the existing event to
                    // determine whether or not it has been recycled (the
                    // existing->time < current_tag.time case below).
                    // NOTE: Because microsteps are not explicit, if the tag of
                    // the preceding event is equal to the current time, then
                    // we search the event queue to figure out whether it has
                    // been handled yet.
                    if (existing->time > current_tag.time ||
                            (existing->time == current_tag.time &&
                            pqueue_find_equal_same_priority(event_q, existing) != NULL)) {
                        // Recycle the existing token and the new event                        
                        // and update the token of the existing event.
                        _lf_replace_token(existing, token);
                        _lf_recycle_event(e);
                        return(0);
                    }
                    // If the preceding event _has_ been handled, then adjust
                    // the tag to defer the event.
                    intended_time = earliest_time;
                    break;
                default:
                    if (existing->time == current_tag.time &&
                            pqueue_find_equal_same_priority(event_q, existing) != NULL) {
                        if (_lf_is_tag_after_stop_tag((tag_t){.time=existing->time,.microstep=get_microstep()+1})) {
                            // Scheduling e will incur a microstep at timeout, 
                            // which is illegal.
                            _lf_recycle_event(e);
                            return 0;
                        }
                        // If the last event hasn't been handled yet, insert
                        // the new event right behind.
                        existing->next = e;
                        return 0; // FIXME: return a value
                    } else {
                         // Adjust the tag.
                        intended_time = earliest_time;
                    }
                    break;
            }
        }
    }

    // Check if the intended time is in the future
    // This is a sanity check for the logic above
    // FIXME: This is a development assertion and might
    // not be necessary for end-user LF programs
    if (intended_time < current_tag.time) {
        error_print("Attempting to schedule an event earlier than current time by %lld nsec! "
                "Revising to the current time %lld.",
                current_tag.time - intended_time, current_tag.time);
        intended_time = current_tag.time;
    }

    // Set the tag of the event.
    e->time = intended_time;

    // Do not schedule events if if the event time is past the stop time
    // (current microsteps are checked earlier).
    DEBUG_PRINT("Comparing event with elapsed time %lld against stop time %lld.", e->time - start_time, stop_tag.time - start_time);
    if (e->time > stop_tag.time) {
        DEBUG_PRINT("__schedule: event time is past the timeout. Discarding event.");
        __done_using(token);
        _lf_recycle_event(e);
        return(0);
    }
    
    // Store a pointer to the current event in order to check the min spacing
    // between this and the following event. Only necessary for actions
    // that actually specify a min spacing.
    trigger->last = (event_t*)e;

    // Queue the event.
    // NOTE: There is no need for an explicit microstep because
    // when this is called, all events at the current tag
    // (time and microstep) have been pulled from the queue,
    // and any new events added at this tag will go into the reaction_q
    // rather than the event_q, so anything put in the event_q with this
    // same time will automatically be executed at the next microstep.
    LOG_PRINT("Inserting event in the event queue with elapsed time %lld.",
            e->time - start_time);
    pqueue_insert(event_q, e);

    tracepoint_schedule(trigger, e->time - current_tag.time);

    // FIXME: make a record of handle and implement unschedule.
    // NOTE: Rather than wrapping around to get a negative number,
    // we reset the handle on the assumption that much earlier
    // handles are irrelevant.
    int return_value = __handle++;
    if (__handle < 0) {
        __handle = 1;
    }
    return return_value;
}

/**
 * Insert reactions triggered by trigger to the reaction queue... 
 * 
 * @param trigger The trigger
 * @param token The token wrapping the payload or NULL for no payload.
 * @return 1 if successful, or 0 if no new reaction was scheduled because the function
 *  was called incorrectly.
 */
handle_t _lf_insert_reactions_for_trigger(trigger_t* trigger, lf_token_t* token) {
    // The trigger argument could be null, meaning that nothing is triggered.
    // Doing this after incrementing the reference count ensures that the
    // payload will be freed, if there is one.
	if (trigger == NULL) {
        warning_print("_lf_schedule_init_reactions() called with a NULL trigger");
	    __done_using(token);
	    return 0;
	}
    
    // Check to see if the trigger is not a timer
    // and not a physical action
    if (trigger->is_timer || trigger->is_physical) {
        warning_print("_lf_schedule_init_reactions() called on a timer or physical action.");
        return 0;
    }

    // Increment the reference count of the token.
	if (token != NULL) {
	    token->ref_count++;
	}

    // Check if the trigger has violated the STP offset
    bool is_STP_violated = false;
#ifdef FEDERATED
    if (compare_tags(trigger->intended_tag, get_current_tag()) < 0) {
        is_STP_violated = true;
    }
#ifdef FEDERATED_CENTRALIZED
    // Check for STP violation in the centralized coordination, which is a 
    // critical error.
    if (is_STP_violated) {
        error_print_and_exit("Attempted to insert reactions for a trigger that had violated the STP offset"
                             " in centralized coordination.");
    }
#endif
#endif

    // Copy the token pointer into the trigger struct so that the
    // reactions can access it. This overwrites the previous template token,
    // for which we decrement the reference count.
    if (trigger->token != token && trigger->token != NULL) {
        // Mark the previous one ok_to_free so we don't get a memory leak.
        trigger->token->ok_to_free = OK_TO_FREE;
        // Free the token if its reference count is zero. Since __done_using
        // decrements the reference count, first increment it here.
        trigger->token->ref_count++;
        __done_using(trigger->token);
    }
    trigger->token = token;
    // Prevent this token from being freed. It is the new template.
    // This might be null if there are no reactions to the action.
    if (token != NULL) {
        token->ok_to_free = no;
    }

    // Mark the trigger present.
    trigger->status = present;
    
    // Push the corresponding reactions for this trigger
    // onto the reaction queue.
    for (int i = 0; i < trigger->number_of_reactions; i++) {
        reaction_t* reaction = trigger->reactions[i];
        // Do not enqueue this reaction twice.
        if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
            reaction->is_STP_violated = is_STP_violated;
            pqueue_insert(reaction_q, reaction);
            LOG_PRINT("Enqueued reaction %s at time %lld.", reaction->name, get_logical_time());
        }
    }

    return 1;
}

/**
 * Utility function to convert a pointer to action struct into
 * a pointer to the corresponding trigger struct.  The type of the
 * action struct is defined by a generated typedef and differs for different
 * actions, which is why the point to the action struct is a void*.
 * All such structs, however, share a common feature, which is tht the
 * first entry in the struct is a pointer to the corresponding trigger_t
 * struct.  This function uses this fact to return a pointer to that
 * trigger_t struct.
 * @param action A pointer to an action struct.
 * @return A pointer to the corresponding trigger struct.
 */
trigger_t* _lf_action_to_trigger(void* action) {
    return *((trigger_t**)action);
}

/**
 * Advance from the current tag to the next. If the given next_time is equal to
 * the current time, then increase the microstep. Otherwise, update the current
 * time and set the microstep to zero.
 * 
 * @param next_time The time step to advance to.
 */ 
void _lf_advance_logical_time(instant_t next_time) {
    // FIXME: The following checks that _lf_advance_logical_time()
    // is being called correctly. Namely, check if logical time
    // is being pushed past the head of the event queue. This should
    // never happen if _lf_advance_logical_time() is called correctly.
    // This is commented out because it will add considerable overhead
    // to the ordinary execution of LF programs. Instead, there might
    // be a need for a target property that enables these kinds of logic
    // assertions for development purposes only.
    /*
    event_t* next_event = (event_t*)pqueue_peek(event_q);
    if (next_event != NULL) {
        if (next_time > next_event->time) {
            error_print_and_exit("_lf_advance_logical_time(): Attempted to move time to %lld, which is "
                    "past the head of the event queue, %lld.", 
                    next_time - start_time, next_event->time - start_time);
        }
    }
    */

    if (current_tag.time < next_time) {
        current_tag.time = next_time;
        current_tag.microstep = 0;
    } else if (current_tag.time == next_time) {
        current_tag.microstep++;
    } else {
        error_print_and_exit("_lf_advance_logical_time(): Attempted to move tag back in time.");
    }
    LOG_PRINT("Advanced (elapsed) tag to (%lld, %u)", next_time - start_time, current_tag.microstep);
}

/**
 * Variant of schedule_value when the value is an integer.
 * See reactor.h for documentation.
 * @param action Pointer to an action on the self struct.
 */
handle_t _lf_schedule_int(void* action, interval_t extra_delay, int value) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    // NOTE: This doesn't acquire the mutex lock in the multithreaded version
    // until schedule_value is called. This should be OK because the element_size
    // does not change dynamically.
    if (trigger->element_size != sizeof(int)) {
        error_print("Action type is not an integer.");
        return -1;
    }
    int* container = (int*)malloc(sizeof(int));
    *container = value;
    return _lf_schedule_value(action, extra_delay, container, 1);
}

/**
 * Library function for allocating memory for an array to be sent on an output.
 * This turns over "ownership" of the allocated memory to the output, so
 * the allocated memory will be freed downstream.
 * @param token The token to use as a template (or if it is free, to use).
 * @param length The length of the array.
 * @param num_destinations The number of destinations (for initializing the reference count).
 * @return A pointer to the new or reused token or null if the template token
 *  is incompatible with this usage.
 */
lf_token_t* __set_new_array_impl(lf_token_t* token, int length, int num_destinations) {
    // If the template token cannot carry a payload, then it is incompatible.
    if (token->element_size == 0) {
        error_print("set_new_array: specified token cannot carry an array. It has zero element_size.");
        return NULL;
    }
    // First, initialize the token, reusing the one given if possible.
    lf_token_t* new_token = __initialize_token(token, length);
    new_token->ref_count = num_destinations;
    DEBUG_PRINT("__set_new_array_impl: Allocated memory for payload %p.", new_token->value);
    return new_token;
}

/**
 * For the specified reaction, if it has produced outputs, insert the
 * resulting triggered reactions into the reaction queue.
 * This procedure assumes the mutex lock is NOT held and grabs
 * the lock only when it actually inserts something onto the reaction queue.
 * @param reaction The reaction that has just executed.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution (for tracing).
 */
void schedule_output_reactions(reaction_t* reaction, int worker) {
    if (reaction->is_a_control_reaction) {
        // Control reactions will not produce an output but can have
        // effects in order to have certain precedence requirements.
        // No need to execute this function if the reaction is a control
        // reaction.
        return;
    }
    // If the reaction produced outputs, put the resulting triggered
    // reactions into the reaction queue. As an optimization, if exactly one
    // downstream reaction is enabled by this reaction, then it may be
    // executed immediately in this same thread
    // without going through the reaction queue.
    reaction_t* downstream_to_execute_now = NULL;
    int num_downstream_reactions = 0;
#ifdef FEDERATED_DECENTRALIZED // Only pass down STP violation for federated programs that use decentralized coordination.
    // Extract the inherited STP violation
    bool inherited_STP_violation = reaction->is_STP_violated;
    LOG_PRINT("Reaction %s has STP violation status: %d.", reaction->name, reaction->is_STP_violated);
#endif
    DEBUG_PRINT("There are %d outputs from reaction %s.", reaction->num_outputs, reaction->name);
    for (int i=0; i < reaction->num_outputs; i++) {
        if (*(reaction->output_produced[i])) {
            DEBUG_PRINT("Output %d has been produced.", i);
            trigger_t** triggerArray = (reaction->triggers)[i];
            DEBUG_PRINT("There are %d trigger arrays associated with output %d.", reaction->triggered_sizes[i], i);
            for (int j=0; j < reaction->triggered_sizes[i]; j++) {
                trigger_t* trigger = triggerArray[j];
                if (trigger != NULL) {
                    DEBUG_PRINT("Trigger %p lists %d reactions.", trigger, trigger->number_of_reactions);
                    for (int k=0; k < trigger->number_of_reactions; k++) {
                        reaction_t* downstream_reaction = trigger->reactions[k];
#ifdef FEDERATED_DECENTRALIZED // Only pass down tardiness for federated LF programs
                        // Set the is_STP_violated for the downstream reaction
                        if (downstream_reaction != NULL) {
                            downstream_reaction->is_STP_violated = inherited_STP_violation;
                            DEBUG_PRINT("Passing is_STP_violated of %d to the downstream reaction: %s",
                            		downstream_reaction->is_STP_violated, downstream_reaction->name);
                        }
#endif
                        if (downstream_reaction != NULL && downstream_reaction != downstream_to_execute_now) {
                            num_downstream_reactions++;
                            // If there is exactly one downstream reaction that is enabled by this
                            // reaction, then we can execute that reaction immediately without
                            // going through the reaction queue. In multithreaded execution, this
                            // avoids acquiring a mutex lock.
                            // FIXME: Check the earliest deadline on the reaction queue.
                            // This optimization could violate EDF scheduling otherwise.
                            if (num_downstream_reactions == 1 && downstream_reaction->last_enabling_reaction == reaction) {
                                // So far, this downstream reaction is a candidate to execute now.
                                downstream_to_execute_now = downstream_reaction;
                            } else {
                                // If there is a previous candidate reaction to execute now,
                                // it is no longer a candidate.
                                if (downstream_to_execute_now != NULL) {
                                    // More than one downstream reaction is enabled.
                                    // In this case, if we were to execute the downstream reaction
                                    // immediately without changing any queues, then the second
                                    // downstream reaction would be blocked because this reaction
                                    // remains on the executing queue. Hence, the optimization
                                    // is not valid. Put the candidate reaction on the queue.
                                    _lf_enqueue_reaction(downstream_to_execute_now);
                                    downstream_to_execute_now = NULL;
                                }
                                // Queue the reaction.
                                _lf_enqueue_reaction(downstream_reaction);
                            }
                        }
                    }
                }
            }
        }
    }
    if (downstream_to_execute_now != NULL) {
        LOG_PRINT("Worker %d: Optimizing and executing downstream reaction now: %s", worker, downstream_to_execute_now->name);
        bool violation = false;
#ifdef FEDERATED_DECENTRALIZED // Only use the STP handler for federated programs that use decentralized coordination
        // If the is_STP_violated for the reaction is true,
        // an input trigger to this reaction has been triggered at a later
        // logical time than originally anticipated. In this case, a special
        // STP handler will be invoked.             
        // FIXME: Note that the STP handler will be invoked
        // at most once per logical time value. If the STP handler triggers the
        // same reaction at the current time value, even if at a future superdense time,
        // then the reaction will be invoked and the STP handler will not be invoked again.
        // However, input ports to a federate reactor are network port types so this possibly should
        // be disallowed.
        // @note The STP handler and the deadline handler are not mutually exclusive.
        //  In other words, both can be invoked for a reaction if it is triggered late
        //  in logical time (STP offset is violated) and also misses the constraint on 
        //  physical time (deadline).
        // @note In absence of a STP handler, the is_STP_violated will be passed down the reaction
        //  chain until it is dealt with in a downstream STP handler.
        if (downstream_to_execute_now->is_STP_violated == true) {
            // Tardiness has occurred
            LOG_PRINT("Event has STP violation.");
            reaction_function_t handler = downstream_to_execute_now->STP_handler;
            // Invoke the STP handler if there is one.
            if (handler != NULL) {
                // There is a violation and it is being handled here
                // If there is no STP handler, pass the is_STP_violated
                // to downstream reactions.
                violation = true;
                LOG_PRINT("Invoke tardiness handler.");
                (*handler)(downstream_to_execute_now->self);

                // If the reaction produced outputs, put the resulting
                // triggered reactions into the queue or execute them directly if possible.
                schedule_output_reactions(downstream_to_execute_now, worker);
                
                // Reset the tardiness because it has been dealt with in the
                // STP handler
                downstream_to_execute_now->is_STP_violated = false;
                DEBUG_PRINT("Reset reaction's is_STP_violated field to false: %s",
                		downstream_to_execute_now->name);
            }
        }
#endif
        if (downstream_to_execute_now->deadline > 0LL) {
            // Get the current physical time.
            instant_t physical_time = get_physical_time();
            // Check for deadline violation.
            if (physical_time > current_tag.time + downstream_to_execute_now->deadline) {
                // Deadline violation has occurred.
                violation = true;
                // Invoke the local handler, if there is one.
                reaction_function_t handler = downstream_to_execute_now->deadline_violation_handler;
                if (handler != NULL) {
                    // Assume the mutex is still not held.
                    (*handler)(downstream_to_execute_now->self);

                    // If the reaction produced outputs, put the resulting
                    // triggered reactions into the queue or execute them directly if possible.
                    schedule_output_reactions(downstream_to_execute_now, worker);
                }
            }
        }
        if (!violation) {
            // Invoke the downstream_reaction function.
            tracepoint_reaction_starts(downstream_to_execute_now, worker);
            downstream_to_execute_now->function(downstream_to_execute_now->self);
            tracepoint_reaction_ends(downstream_to_execute_now, worker);

            // If the downstream_reaction produced outputs, put the resulting triggered
            // reactions into the queue (or execute them directly, if possible).
            schedule_output_reactions(downstream_to_execute_now, worker);
        }
            
        // Reset the is_STP_violated because it has been passed
        // down the chain
        downstream_to_execute_now->is_STP_violated = false;
        DEBUG_PRINT("Finally, reset reaction's is_STP_violated field to false: %s",
        		downstream_to_execute_now->name);
    } else if (num_downstream_reactions > 0) {
        // If we are running a multithreaded setting, the following function
        // may wake up other worker threads to execute the newly queued reactions.
        _lf_notify_workers();
    }
}

/**
 * Return a writable copy of the specified token.
 * If the reference count is 1, this returns the original token rather than a copy.
 * The reference count will still be 1.
 * If the size of the token payload is zero, this also returns the original token.
 * Otherwise, this returns a new token with a reference count of 0.
 * To ensure that the allocated memory is not leaked, this new token must be
 * either passed to an output using set_token() or scheduled with a action
 * using schedule_token().
 */
lf_token_t* writable_copy(lf_token_t* token) {
    DEBUG_PRINT("writable_copy: Requesting writable copy of token %p with reference count %d.", token, token->ref_count);
    if (token->ref_count == 1) {
        DEBUG_PRINT("writable_copy: Avoided copy because reference count is %d.", token->ref_count);
        return token;
   } else {
        DEBUG_PRINT("writable_copy: Copying array because reference count is greater than 1. It is %d.", token->ref_count);
        size_t size = token->element_size * token->length;
        if (size == 0) {
            return token;
        }
        void* copy = malloc(size);
        DEBUG_PRINT("Allocating memory for writable copy %p.", copy);
        memcpy(copy, token->value, size);
        // Count allocations to issue a warning if this is never freed.
        __count_payload_allocations++;
        // Create a new, dynamically allocated token.
        lf_token_t* result = create_token(token->element_size);
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
    printf("  -i, --id <n>\n");
    printf("   The ID of the federation that this reactor will join.\n\n");

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
 * The ID of the federation that this reactor will join.
 * This should be overridden with a command-line -i option to ensure
 * that each federate only joins its assigned federation.
 */
char* federation_id = "Unidentified Federation";

/**
 * Process the command-line arguments. If the command line arguments are not
 * understood, then print a usage message and return 0. Otherwise, return 1.
 * @return 1 if the arguments processed successfully, 0 otherwise.
 */
int process_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--fast") == 0) {
            if (argc < i + 2) {
                error_print("--fast needs a boolean.");
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
                error_print("Invalid value for --fast: %s", fast_spec);
            }
       } else if (strcmp(argv[i], "-o") == 0
               || strcmp(argv[i], "--timeout") == 0
               || strcmp(argv[i], "-timeout") == 0) {
           // Tolerate -timeout for legacy uses.
           if (argc < i + 3) {
               error_print("--timeout needs time and units.");
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
               error_print("Invalid time value: %s", time_spec);
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
               error_print("Invalid time units: %s", units);
        	   usage(argc, argv);
        	   return 0;
           }
       } else if (strcmp(argv[i], "-k") == 0 || strcmp(argv[i], "--keepalive") == 0) {
    	   if (argc < i + 2) {
    	       error_print("--keepalive needs a boolean.");
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
    	       error_print("Invalid value for --keepalive: %s", keep_spec);
    	   }
       } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--threads") == 0) {
    	   if (argc < i + 2) {
    	       error_print("--threads needs an integer argument.s");
    		   usage(argc, argv);
    		   return 0;
    	   }
    	   i++;
    	   char* threads_spec = argv[i++];
    	   _lf_number_of_threads = atoi(threads_spec);
    	   if (_lf_number_of_threads <= 0) {
    	       error_print("Invalid value for --threads: %s", threads_spec);
    	   }
       } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--id") == 0) {
           if (argc < i + 2) {
               error_print("--id needs a string argument.");
               usage(argc, argv);
               return 0;
           }
           i++;
           info_print("Federation ID for executable %s: %s", argv[0], argv[i]);
           federation_id = argv[i++];
       } else {
           error_print("Unrecognized command-line argument: %s", argv[i]);
    	   usage(argc, argv);
    	   return 0;
       }
    }
    return 1;
}

/**
 * Calculate the necessary offset to bring _LF_CLOCK in parity
 * with the epoch time.
 */
void calculate_epoch_offset() {
    if (_LF_CLOCK == CLOCK_REALTIME) {
        // Set the epoch offset to zero (see tag.h)
        _lf_epoch_offset = 0LL;
    } else {
        // Initialize _lf_epoch_offset to the difference between what is
        // reported by whatever clock LF is using (e.g. CLOCK_MONOTONIC)
        // and what is reported by CLOCK_REALTIME.
        struct timespec physical_clock_snapshot, real_time_start;

        clock_gettime(_LF_CLOCK, &physical_clock_snapshot);
        instant_t physical_clock_snapshot_ns = physical_clock_snapshot.tv_sec * BILLION + physical_clock_snapshot.tv_nsec;

        clock_gettime(CLOCK_REALTIME, &real_time_start);
        instant_t real_time_start_ns = real_time_start.tv_sec * BILLION + real_time_start.tv_nsec;

        _lf_epoch_offset = real_time_start_ns - physical_clock_snapshot_ns;
    }
    LOG_PRINT("Clock sync: Initial epoch offset set to %lld.", _lf_epoch_offset);
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
	// NOTE: The recycle and next queue does not need to be sorted. But here it is.
    recycle_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, in_no_particular_order, get_event_time,
            get_event_position, set_event_position, event_matches, print_event);
    next_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, in_no_particular_order, get_event_time,
            get_event_position, set_event_position, event_matches, print_event);

    // Initialize the trigger table.
    __initialize_trigger_objects();

    physical_start_time = get_physical_time();
    current_tag.time = physical_start_time;
    start_time = current_tag.time;

    struct timespec physical_time_timespec = {physical_start_time / BILLION, physical_start_time % BILLION};

    info_print("---- Start execution at time %s---- plus %ld nanoseconds.",
            ctime(&physical_time_timespec.tv_sec), physical_time_timespec.tv_nsec);
    
    if (duration >= 0LL) {
        // A duration has been specified. Calculate the stop time.
        _lf_set_stop_tag((tag_t) {.time = current_tag.time + duration, .microstep = 0});
    }
}

/**
 * Report elapsed logical and physical times and report if any
 * memory allocated by set_new, set_new_array, or writable_copy
 * has not been freed.
 */
void termination() {
    // Invoke the code generated termination function.
    terminate_execution();

    // Stop any tracing, if it is running.
    stop_trace();

    // In order to free tokens, we perform the same actions we would have for a new time step.
    __start_time_step();

    // If the event queue still has events on it, report that.
    if (event_q != NULL && pqueue_size(event_q) > 0) {
        warning_print("---- There are %zu unprocessed future events on the event queue.", pqueue_size(event_q));
        event_t* event = (event_t*)pqueue_peek(event_q);
        interval_t event_time = event->time - start_time;
        warning_print("---- The first future event has timestamp %lld after start time.", event_time);
    }
    // Issue a warning if a memory leak has been detected.
    if (__count_payload_allocations > 0) {
        warning_print("Memory allocated for messages has not been freed.");
        warning_print("Number of unfreed messages: %d.", __count_payload_allocations);
    }
    if (__count_token_allocations > 0) {
        warning_print("Memory allocated for tokens has not been freed!");
        warning_print("Number of unfreed tokens: %d.", __count_token_allocations);
    }
    // Print elapsed times.
    // If these are negative, then the program failed to start up.
    interval_t elapsed_time = get_elapsed_logical_time();
    if (elapsed_time >= 0LL) {
        char time_buffer[28]; // 28 bytes is enough for the largest 64 bit number: 9,223,372,036,854,775,807
        readable_time(time_buffer, elapsed_time);
        info_print("---- Elapsed logical time (in nsec): %s", time_buffer);

        // If physical_start_time is 0, then execution didn't get far enough along
        // to initialize this.
        if (physical_start_time > 0LL) {
            readable_time(time_buffer, get_elapsed_physical_time());
            info_print("---- Elapsed physical time (in nsec): %s", time_buffer);
        }
    }
}
