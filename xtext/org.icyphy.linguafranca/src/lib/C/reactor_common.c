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


/** The number of reactions in the system. */
unsigned int number_of_reactions = 100; // FIXME set this in the generated code

/**
 * Current time in nanoseconds since January 1, 1970.
 * This is not in scope for reactors.
 */
instant_t current_time = 0LL;

/** Logical time at the start of execution. */
interval_t start_time = 0LL;

/** Physical time at the start of the execution. */
struct timespec physicalStartTime;

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
    return physicalTime.tv_sec * BILLION + physicalTime.tv_nsec -
    (physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec);
}

/**
 * Specialized version of malloc used by Lingua Franca for action values
 * and messages contained in dynamically allocated memory.
 * @param size The size of the memory block to allocate.
 * @return A pointer to the allocated memory block.
 */
void* lf_malloc(size_t size) {
    // NOTE: For now, this just delegates to malloc.
    // But in the future, we expect to use it to attach a reference count to the
    // allocated object.
    return malloc(size);
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
 * Return whether or not the given events have matching tags.
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
    fprintf(out, "time: %lld, trigger: %p, value: %p\n",
			e->time, e->trigger, e->value);
}

// ********** Priority Queue Support End

/** Counter used to issue a warning if memory is allocated and never freed. */
static int __count_allocations;

/**
 * Library function to decrement the reference count and free the memory, if
 * appropriate, for messages carried by a token_t struct.
 */
void __done_using(token_t* token) {
    token->ref_count--;
    // printf("****** After reacting, ref_count = %d.\n", token->ref_count);
    if (token->ref_count == 0) {
        // Count frees to issue a warning if this is never freed.
        __count_allocations--;
        free(token->value);
        // printf("DEBUG: Freed allocated memory %p\n", token->value);
    }
}

/** 
 * Schedule the specified trigger at current_time plus the offset of the
 * specified trigger plus the delay. The value is required to be a pointer
 * returned by malloc because it will be freed after having been delivered to
 * all relevant destinations unless it is NULL, in which case it will be
 * ignored. If the trigger offset plus the extra delay is greater than zero and
 * stop has been requested, then ignore this and return 0. Also, if the trigger
 * argument is null, ignore and return 0. Otherwise, return a handle to the
 * scheduled trigger, which is an integer greater than 0.
 */
handle_t __schedule(trigger_t* trigger, interval_t extra_delay, void* value) {
	// The trigger argument could be null, meaning that nothing is triggered.
	if (trigger == NULL) return 0;
    // Compute the tag.  How we do that depends on whether
    // this is a logical or physical action.
    interval_t tag = current_time;
    event_t* existing = NULL;

    // Recycle event_t structs, if possible.    
    event_t* e = pqueue_pop(recycle_q);
    if (e == NULL) {
        e = malloc(sizeof(struct event_t));
    }
    
    e->trigger = trigger;
    e->value = value;
    // For logical actions, the logical time of the new event is just
    // the current logical time plus the minimum offset (action parameter)
    // plus the extra delay specified in the call to schedule.
    e->time = tag + trigger->offset + extra_delay;

    if (trigger->is_physical) {
        // If the trigger is physical, then we need to use
        // physical time and the time of the last invocation to adjust the tag.
        // Specifically, the timestamp assigned to the action event will be
        // the maximum of the current logical time, the
        // current physical time, and the time of last
        // invocation plus the minTime (action parameter) plus the
        // extra_delay (argument to this function).
        // If the action has never been scheduled before, then the
        // timestamp will be the maximum of the current logical time,
        // the current physical time,
        // and the start time + minTime + extra_delay.

        // Get the current physical time.
        struct timespec current_physical_time;
        clock_gettime(CLOCK_REALTIME, &current_physical_time);
        // Convert to an instant.
        instant_t physical_time =
                current_physical_time.tv_sec * BILLION
                + current_physical_time.tv_nsec;
        if (physical_time > current_time) {
            tag = physical_time;
        }

        interval_t min_inter_arrival = trigger->offset + extra_delay;
        // Compute the earliest time that this event can be scheduled.
        instant_t earliest_time;
        if (trigger->scheduled == NEVER) {
            earliest_time = start_time + min_inter_arrival;
        } else {
            earliest_time = trigger->scheduled + min_inter_arrival;
        }
        
        if (earliest_time > tag) {
            // The event is early. See which policy applies.
            if (trigger->policy == UPDATE) {
                // Update existing event if it exists.   
                e->time = tag;
                // See if there is an existing event up to but not including
                // the earliest time this event can be scheduled.
                existing = pqueue_find_equal(event_q, e, earliest_time-1);
                if (existing != NULL) {
                    // Update the value of the existing event.
                    existing->value = value;
                }
            }
            if (trigger->policy == DROP || existing != NULL) {
                // Recycle the new event.
                e->value = NULL;    // FIXME: Memory leak.
                pqueue_insert(recycle_q, e);
                return(0);
            }
            if (trigger->policy == DEFER 
                || (trigger->policy == UPDATE && existing == NULL)) {
                // Adjust the tag.
                tag = earliest_time;
            }
        }

        // Record the tag.
        trigger->scheduled = tag;        
        e->time = tag;                        
    }
    
    // Do not schedule events if a stop has been requested.
    if (tag != current_time && stop_requested) {
        return 0;
    }

    // Handle duplicate events for logical actions.
    if (!trigger->is_physical) {
        existing = pqueue_find_equal_same_priority(event_q, e);
        if (existing != NULL) {
            existing->value = value;
            // Recycle the new event.
            e->value = NULL;    // FIXME: Memory leak.
            pqueue_insert(recycle_q, e);
            return(0);
        }
    }
    // NOTE: There is no need for an explicit microstep because
    // when this is called, all events at the current tag
    // (time and microstep) have been pulled from the queue,
    // and any new events added at this tag will go into the reaction_q
    // rather than the event_q, so anything put in the event_q with this
    // same time will automatically be executed at the next microstep.
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
 * Library function for allocating memory for an array output.
 * This turns over "ownership" of the allocated memory to the output.
 */
void* __set_new_array_impl(token_t* token, int length) {
    // FIXME: Error checking needed.
    token->value = malloc(token->element_size * length);
    // printf("DEBUG: Allocated %p\n", token->value);
    token->ref_count = token->initial_ref_count;
    // Count allocations to issue a warning if this is never freed.
    __count_allocations++;
    // printf("****** Allocated object with starting ref_count = %d.\n", token->ref_count);
    token->length = length;
    return token->value;
}

/**
 * Library function for returning a writable copy of a token.
 * If the reference count is 1, it returns the original rather than a copy.
 */
void* __writable_copy_impl(token_t* token) {
    // printf("****** Requesting writable copy with reference count %d.\n", token->ref_count);
    if (token->ref_count < 2) {
        // printf("****** Avoided copy because reference count is less than two.\n");
        // Decrement the reference count to avoid the automatic free().
        token->ref_count--;
        return token->value;
    } else {
        // printf("****** Copying array because reference count is greater than 1. It is %d.\n", token->ref_count);
        int size = token->element_size * token->length;
        void* copy = malloc(size);
        memcpy(copy, token->value, size);
        // Count allocations to issue a warning if this is never freed.
        __count_allocations++;
        return copy;
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
    __count_allocations = 0;
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
    clock_gettime(CLOCK_REALTIME, &physicalStartTime);
    printf("---- Start execution at time %s---- plus %ld nanoseconds.\n",
            ctime(&physicalStartTime.tv_sec), physicalStartTime.tv_nsec);
    current_time = physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec;
    start_time = current_time;
    
    if (duration >= 0LL) {
        // A duration has been specified. Calculate the stop time.
        stop_time = current_time + duration;
    }
}



// Check that memory allocated by set_new, set_new_array, or writable_copy
// has been freed and print a warning message if not.
void termination() {
    if (__count_allocations != 0) {
        printf("**** WARNING: Memory allocated by set_new, set_new_array, or writable_copy has not been freed!\n");
        printf("**** Number of unfreed tokens: %d.\n", __count_allocations);
    }
    // Print elapsed times.
    interval_t elapsed_logical_time
        = current_time - (physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec);
    printf("---- Elapsed logical time (in nsec): %lld\n", elapsed_logical_time);
    
    struct timespec physicalEndTime;
    clock_gettime(CLOCK_REALTIME, &physicalEndTime);
    interval_t elapsed_physical_time
        = (physicalEndTime.tv_sec * BILLION + physicalEndTime.tv_nsec)
        - (physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec);
    printf("---- Elapsed physical time (in nsec): %lld\n", elapsed_physical_time);
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
