/*
 FIXME: License, copyright, authors.
 */

#include "reactor.h"

// This is not in scope for reactors.
instant_t current_time = 0LL;

// The following should be in scope for reactors:
// FIXME: This probably should not be a global, at least not for parallel execution.
long long get_logical_time() {
	return current_time;
}

// Priority queues.
pqueue_t* event_q;     // For sorting by time.
pqueue_t* reaction_q;  // For sorting by index (topological sort)

handle_t __handle = 0;
struct timespec __physicalStartTime;

// ********** Priority Queue Support Start

// Compare two priorities.
static int cmp_pri(pqueue_pri_t next, pqueue_pri_t curr) {
  return (next > curr);
}
// Compare two events.
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

// ********** Priority Queue Support End

// Schedule the specified trigger at current_time plus the delay.
handle_t __schedule(trigger_t* trigger, interval_t delay) {
    // FIXME: Recycle event_t structs.
    event_t* e = malloc(sizeof(struct event_t));
    e->time = current_time + delay;
    e->trigger = trigger;

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

// Schedule the specified trigger at current_time plus the
// offset declared in the trigger plus the extra_delay.
handle_t schedule(trigger_t* trigger, interval_t extra_delay) {
	return __schedule(trigger, trigger->offset + extra_delay);
}


// Wait until physical time matches or exceeds the start time of execution
// plus the current_time plus the specified logical time.  If this is not
// interrupted, then advance current_time by the specified logical_delay. 
// Return 0 if time advanced to the time of the event and -1 if the wait
// was interrupted.
int wait_until(event_t* event) {
    // printf("-------- Waiting for logical time %lld.\n", event->time);
    long long logical_time_ns = event->time;
    
    // Get the current physical time.
    struct timespec current_physical_time;
    clock_gettime(CLOCK_REALTIME, &current_physical_time);
    
    long long ns_to_wait = logical_time_ns
    - (current_physical_time.tv_sec * BILLION
    + current_physical_time.tv_nsec);
    
    if (ns_to_wait <= 0) {
        // Advance current time.
        current_time = event->time;
        return 0;
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
    if (current_physical_time_ns < event->time) {
    	current_time = current_physical_time_ns;
    	return -1;
    }
    	} else {
    // Advance current time.
    current_time = event->time;
    // FIXME: Make sure that the microstep is dealt with correctly.
            return -1;
        }
    }
    // Advance current time.
    current_time = event->time;
    return 0;
}
// Wait until physical time matches or exceeds the time of the least tag
// on the event queue. If theres is no event in the queue, return 0.
// After this wait, advance current_time to match
// this tag. Then pop the next event(s) from the
// event queue that all have the same tag, and extract from those events
// the reactions that are to be invoked at this logical time.
// Sort those reactions by index (determined by a topological sort)
// and then execute the reactions in order. Each reaction may produce
// outputs, which places additional reactions into the index-ordered
// priority queue. All of those will also be executed in order of indices.
// Finally, return 1.
int next() {
	event_t* event = pqueue_peek(event_q);
	if (event == NULL) {
	    // No event in the queue.
        return 0;
	}
	// Wait until physical time >= event.time
	if (wait_until(event) < 0) {
        // FIXME: sleep was interrupted. Handle that somehow here!
	}
	
	// Invoke code that must execute before starting a new logical time round,
	// such as initializing outputs to be absent.
	__start_time_step();
	
  	// Pop all events from event_q with timestamp equal to current_time
  	// stick them into reaction.
  	do {
  	 	event = pqueue_pop(event_q);
  	 	for (int i = 0; i < event->trigger->number_of_reactions; i++) {
  	 	    // printf("Pushed on reaction_q: %p\n", event->trigger->reactions[i]);
  	 	    // printf("Pushed reaction args: %p\n", event->trigger->reactions[i]->args);
  	        pqueue_insert(reaction_q, event->trigger->reactions[i]);
  	 	}
  	 	if (event->trigger->period > 0) {
  	        // Reschedule the trigger.
  	        __schedule(event->trigger, event->trigger->period);
  	 	}
 	 	
      // FIXME: Recycle this event instead of freeing it.
      free(event);

      event = pqueue_peek(event_q);
  	} while(event != NULL && event->time == current_time);

  	// Handle reactions.
  	while(pqueue_size(reaction_q) > 0) {
        reaction_t* reaction = pqueue_pop(reaction_q);
        // printf("Popped from reaction_q: %p\n", reaction);
    	// printf("Popped reaction function: %p\n", reaction->function);
        reaction->function(reaction->this);

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
                    }
                }
            }
        }
  	}
	return 1;
}

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
                       get_evt_pos, set_evt_pos, eql_evt);
	reaction_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, cmp_pri, get_rct_pri,
                       get_rct_pos, set_rct_pos, eql_rct);

	// Initialize logical time to match physical time.
	clock_gettime(CLOCK_REALTIME, &__physicalStartTime);
	printf("Start execution at time %splus %ld nanoseconds.\n",
	ctime(&__physicalStartTime.tv_sec), __physicalStartTime.tv_nsec);
	current_time = __physicalStartTime.tv_sec * BILLION	+ __physicalStartTime.tv_nsec;
	
  // Initialize the trigger table.
  __initialize_trigger_objects();
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

int main(int argc, char* argv[]) {
	initialize();
	__start_timers();
	// FIXME: Need stopping conditions.
	while (next() != 0);
	return 0;
}
