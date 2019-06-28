/*
 FIXME: License, copyright, authors.
 */

#include "reactor.h"

/** Global constants. */
bool False = false;
bool True = true;

// Indicator of whether to wait for physical time to match logical time.
// By default, execution will wait. The command-line argument -fast will
// eliminate the wait and allow logical time to exceed physical time.
bool fast = false;

// By default, execution is not threaded and this variable will have value 0.
int number_of_threads = 0;

// Current time in nanoseconds since January 1, 1970.
// This is not in scope for reactors.
instant_t current_time = 0LL;

// Logical time at the start of execution.
interval_t start_time = 0LL;

// Indicator that the execution should stop after the completion of the
// current logical time. This can be set to true by calling the stop()
// function in a reaction.
bool stop_requested = false;

// Duration, or -1 if no stop time has been given.
instant_t duration = -1LL;

// Stop time, or 0 if no stop time has been given.
instant_t stop_time = 0LL;

// Indicator of whether the wait command-line option was given.
bool wait_specified = false;

/////////////////////////////
// The following functions are in scope for all reactors:

// Return the elpased logical time in nanoseconds since the start of execution.
interval_t get_elapsed_logical_time() {
    return current_time - start_time;
}

// Return the current logical time in nanoseconds since January 1, 1970.
instant_t get_logical_time() {
    return current_time;
}

// Stop execution at the conclusion of the current logical time.
void stop() {
    stop_requested = true;
}

/////////////////////////////
// The following is not in scope for reactors:

// Priority queues.
pqueue_t* event_q;     // For sorting by time.
pqueue_t* reaction_q;  // For sorting by index (precedence sort)
pqueue_t* recycle_q;   // For recycling malloc'd events.
pqueue_t* free_q;      // For free malloc'd payloads carried by events.

handle_t __handle = 0;
struct timespec physicalStartTime;

// ********** Priority Queue Support Start

// Compare two priorities.
static int cmp_pri(pqueue_pri_t next, pqueue_pri_t curr) {
    return (next > curr);
}
// Compare two events. They are "equal" if they refer to the same trigger.
static int eql_evt(void* next, void* curr) {
    return (((event_t*)next)->trigger == ((event_t*)curr)->trigger);
}
// Compare two reactions.
static int eql_rct(void* next, void* curr) {
    return (next == curr);
}
// Get priorities based on time.
// Used for sorting event_t structs.
static pqueue_pri_t get_evt_pri(void *a) {
    return (pqueue_pri_t)(((event_t*) a)->time);
}
// Get priorities based on indices, which reflect topological sort.
// Used for sorting reaction_t structs.
static pqueue_pri_t get_rct_pri(void *a) {
    return ((reaction_t*) a)->index;
}

// Get position in the queue of the specified event.
static size_t get_evt_pos(void *a) {
    return ((event_t*) a)->pos;
}

// Get position in the queue of the specified event.
static size_t get_rct_pos(void *a) {
    return ((reaction_t*) a)->pos;
}

// Set position of the specified event.
static void set_evt_pos(void *a, size_t pos) {
    ((event_t*) a)->pos = pos;
}

// Set position of the specified event.
static void set_rct_pos(void *a, size_t pos) {
    ((reaction_t*) a)->pos = pos;
}

static void prt_rct(FILE *out, void *a) {
	reaction_t *n = a;
    fprintf(out, "index: %lld, reaction: %p\n",
			n->index, n);
}

static void prt_evt(FILE *out, void *a) {
	event_t *n = a;
    fprintf(out, "time: %lld, trigger: %p, payload: %p\n",
			n->time, n->trigger, n->payload);
}

// ********** Priority Queue Support End

// Schedule the specified trigger at current_time plus the
// offset of the specified trigger plus the delay.
// The payload is required to be a pointer returned by malloc
// because it will be freed after having been delivered to
// all relevant destinations unless it is NULL, in which case
// it will be ignored.
handle_t __schedule(trigger_t* trigger, interval_t delay, void* payload) {
    // Recycle event_t structs, if possible.
    event_t* e = pqueue_pop(recycle_q);
    if (e == NULL) {
        e = malloc(sizeof(struct event_t));
    }
    e->time = current_time + trigger->offset + delay;
    e->trigger = trigger;
    e->payload = payload;
    
    // NOTE: There is no need for an explicit microstep because
    // when this is called, all events at the current tag
    // (time and microstep) have been pulled from the queue,
    // and any new events added at this tag will go into the reaction_q
    // rather than the event_q, so anything put in the event_q with this
    // same time will automatically be executed at the next microstep.
    pqueue_insert(event_q, e);
    // FIXME: make a record of handle and implement unschedule.
    return __handle++;
}

// For the specified reaction, if it has produced outputs, put the
// resulting triggered reactions into the reaction queue.
void trigger_output_reactions(reaction_t* reaction) {
	// If the reaction produced outputs, put the resulting triggered
    // reactions into the queue.
    for(int i=0; i < reaction->num_outputs; i++) {
        if (*(reaction->output_produced[i])) {
            trigger_t** triggerArray = (reaction->triggers)[i];
            for (int j=0; j < reaction->triggered_sizes[i]; j++) {
            	trigger_t* trigger = triggerArray[j];
                for (int k=0; k < trigger->number_of_reactions; k++) {
                    reaction_t* reaction = trigger->reactions[k];
                    pqueue_insert(reaction_q, trigger->reactions[k]);
             		// printf("Pushed on reaction_q reaction with level: %lld\n", trigger->reactions[k]->index);
             		// printf("Reaction pointer: %p\n", trigger->reactions[k]);
               }
            }
        }
	}
}

// Print a usage message.
void usage(char* command) {
    printf("\nCommand-line arguments: \n\n");
    printf("  -fast\n");
    printf("   Do not wait for physical time to match logical time.\n\n");
    printf("  -stop <duration> <units>\n");
    printf("   Stop after the specified amount of logical time, where units are one of\n");
    printf("   nsec, usec, msec, sec, minute, hour, day, week, or the plurals of those.\n\n");
    printf("  -wait\n");
    printf("   Do not stop execution even if there are no events to process. Just wait.\n\n");
    printf("  -threads <n>\n");
    printf("   Executed in <n> threads if possible (optional feature).\n\n");
}

// Process the command-line arguments.
// If the command line arguments are not understood, then
// print a usage message and return 0.
// Otherwise, return 1.
int process_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-fast") == 0) {
            fast = true;
        } else if (strcmp(argv[i], "-stop") == 0) {
            if (argc < i + 3) {
                usage(argv[0]);
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
                usage(argv[0]);
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
            } else if (strncmp(units, "minute", 6) == 0) {
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
                usage(argv[0]);
                return 0;
            }
        } else if (strcmp(argv[i], "-wait") == 0) {
            wait_specified = true;
        } else if (strcmp(argv[i], "-threads") == 0) {
            if (argc < i + 2) {
                usage(argv[0]);
                return 0;
            }
            i++;
            char* threads_spec = argv[i++];
            number_of_threads = atoi(threads_spec);           
        } else {
            usage(argv[0]);
            return 0;
        }
    }
    return 1;
}

// Initialize the priority queues and set logical time to match
// physical time. This also prints a message reporting the start time.
void initialize() {
#if _WIN32 || WIN32
    HMODULE ntdll = GetModuleHandleA("ntdll.dll");
    if (ntdll) {
        NtDelayExecution = (NtDelayExecution_t *)GetProcAddress(ntdll, "NtDelayExecution");
        NtQueryPerformanceCounter = (NtQueryPerformanceCounter_t *)GetProcAddress(ntdll, "NtQueryPerformanceCounter");
        NtQuerySystemTime = (NtQuerySystemTime_t *)GetProcAddress(ntdll, "NtQuerySystemTime");
    }
#endif

    // Initialize our priority queues.
    event_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, cmp_pri, get_evt_pri,
            get_evt_pos, set_evt_pos, eql_evt, prt_evt);
    reaction_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, cmp_pri, get_rct_pri,
            get_rct_pos, set_rct_pos, eql_rct, prt_rct);
	// NOTE: The recycle queue does not need to be sorted. But here it is.
    recycle_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, cmp_pri, get_evt_pri,
            get_evt_pos, set_evt_pos, eql_evt, prt_evt);
	// NOTE: The free queue does not need to be sorted. But here it is.
    free_q = pqueue_init(INITIAL_EVENT_QUEUE_SIZE, cmp_pri, get_evt_pri,
            get_evt_pos, set_evt_pos, eql_evt, prt_evt);

    // Initialize the trigger table.
    __initialize_trigger_objects();

    // Initialize logical time to match physical time.
    clock_gettime(CLOCK_REALTIME, &physicalStartTime);
    printf("Start execution at time %splus %ld nanoseconds.\n",
    		ctime(&physicalStartTime.tv_sec), physicalStartTime.tv_nsec);
    current_time = physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec;
    start_time = current_time;
    
    if (duration >= 0LL) {
        // A duration has been specified. Calculate the stop time.
        stop_time = current_time + duration;
    }
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
