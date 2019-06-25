/*
 Version of the C library supporting multithreaded execution.
 */

#include "reactor_common.c"
#include <pthread.h>

// Number of idle worker threads.
int number_of_idle_threads = 0;

// Indicator that we are between logical times.
bool between_logical_times = false;

// Queue of currently executing reactions.
pqueue_t* executing_q;  // Sorted by index (precedence sort)

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t wake = PTHREAD_COND_INITIALIZER;
pthread_cond_t number_of_idle_threads_increased = PTHREAD_COND_INITIALIZER;
pthread_cond_t end_logical_time = PTHREAD_COND_INITIALIZER;

// Schedule the specified trigger at current_time plus the
// offset declared in the trigger plus the extra_delay.
// If the offset of the trigger and the extra_delay are both zero,
// then schedule the trigger to occur one microstep later in superdense time.
// If this call occurs between logical times, then instead of adding
// the offset and the extra_delay to the current_time, add them to the
// greater of the current_physical time and the current logical time (current_time).
// The payload is required to be a pointer returned by malloc
// because it will be freed after having been delivered to
// all relevant destinations unless it is NULL, in which case
// it will be ignored.
handle_t schedule(trigger_t* trigger, interval_t extra_delay, void* payload) {
 	pthread_mutex_lock(&mutex); // FIXME: is only necessary for _async_ calls, see MEMOCODE paper
	// If we are between logical times, this is an asynchronous callback
	// and we need to use physical time to adjust the delay.
 	if (between_logical_times) {
 		// Get the current physical time.
        struct timespec current_physical_time;
        clock_gettime(CLOCK_REALTIME, &current_physical_time);
    
        interval_t time_adjustment =
                current_physical_time.tv_sec * BILLION
                + current_physical_time.tv_nsec
                - current_time;
        if (time_adjustment > 0LL) {
        	extra_delay += time_adjustment;
        }
 	}
    int return_value = __schedule(trigger, trigger->offset + extra_delay, payload);
 	pthread_mutex_unlock(&mutex);
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
// If the -stop option has been given on the command line, then return
// 0 when the logical time duration matches the specified duration.
// Also return 0 if there are no more events in the queue and
// the wait command-line option has not been given.
// Otherwise, return 1.
int next() {
 	pthread_mutex_lock(&mutex);
 	// Wait for the reaction queue to be empty and
 	// all worker threads to be idle, indicating that
 	// the previous logical time is complete.
 	while (pqueue_size(reaction_q) > 0 || number_of_idle_threads != number_of_threads) {
 		if (stop_requested) {
     		pthread_mutex_unlock(&mutex);
            return 0;
 		}
        // Wait for some activity on the number of idle threads.
        pthread_cond_wait(&number_of_idle_threads_increased, &mutex);
	}
 	// All worker threads are idle and lock is held.
	
 	// Free any action payloads from the previous logical time that need
 	// to be freed and recycle the event carrying them.
    event_t* free_event = pqueue_pop(free_q);
    while (free_event != NULL) {
    	free(free_event->payload);
    	pqueue_insert(recycle_q, free_event);
    	free_event = pqueue_pop(free_q);
    }
    // Check whether the previous logical time executed was to be the
    // last logical time executed.
    if (stop_time > 0LL && current_time >= stop_time) {
		stop_requested = true;
     	pthread_mutex_unlock(&mutex);
 		// Signal the worker threads.
		pthread_cond_broadcast(&wake);
       	return 0;
    }

 	// Peek at the earliest event in the event queue.
	event_t* event = pqueue_peek(event_q);
	
    // If there is no next event and -wait has been specified
    // on the command line, then we will wait the maximum time possible.
    instant_t next_time = LLONG_MAX;
    if (event == NULL) {
        // No event in the queue and -wait was not specified.
        // Execution is finished.
        if (!wait_specified) {
     		pthread_mutex_unlock(&mutex);
            return 0;
       }
    } else {
        next_time = event->time;
    }
    // Wait until physical time >= event.time.
    // Do not hold the lock during that time.
    // NOTE: We should release the lock even if there will no physical time wait
    // to allow other threads to sneak in. Perhaps also do a yield?
    // The wait_until function will advance current_time.
    between_logical_times = true;
    pthread_mutex_unlock(&mutex);
    pthread_cond_broadcast(&end_logical_time);
    if (wait_until(next_time) < 0) {
        // Sleep was interrupted or the stop time has been reached.
        // Time has not advanced to the time of the event.
        // There may be a new earlier event on the queue.
 		pthread_mutex_lock(&mutex);
        event_t* new_event = pqueue_peek(event_q);
        if (new_event == event) {
            // There is no new event. If the stop time has been reached,
            // or if the maximum time has been reached (unlikely), then return.
            if ((stop_time > 0LL && current_time >= stop_time) || new_event == NULL) {
            	stop_requested = true;
     			between_logical_times = false;
    			pthread_mutex_unlock(&mutex);
				// Signal the worker threads.
				pthread_cond_broadcast(&wake);
                return 0;
            }
        } else {
        	// Handle the new event.
        	event = new_event;
        	next_time = event->time;
        }
    }
    // Reacquire the lock.
 	pthread_mutex_lock(&mutex);
    between_logical_times = false;
        
    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    __start_time_step();
    
    // Pop all events from event_q with timestamp equal to current_time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    do {
        event = pqueue_pop(event_q);
        
        // Push the corresponding reactions onto the reaction queue.
        for (int i = 0; i < event->trigger->number_of_reactions; i++) {
            // printf("Pushed on reaction_q: %p\n", event->trigger->reactions[i]);
            // printf("Pushed reaction args: %p\n", event->trigger->reactions[i]->args);
            pqueue_insert(reaction_q, event->trigger->reactions[i]);
        }
        // If the trigger is a periodic clock, create a new event for its next execution.
        if (event->trigger->period > 0) {
            // Reschedule the trigger.
            // Note that the delay here may be negative because the __schedule
            // function will add the trigger->offset, which we don't want at this point.
            // NULL argument indicates that there is no payload.
            __schedule(event->trigger, event->trigger->period - event->trigger->offset, NULL);
        }
        // Copy the payload pointer into the trigger struct so that the
        // reactions can access it.
        event->trigger->payload = event->payload;
        
        // If the payload is non-null, record the event to free the payload
        // at the end of the current logical time. Otherwise, recycle the event.
        // In either case, so that sorting doesn't cost anything,
        // give all recycled events the same zero time stamp.
        event->time = 0LL;
        if (event->payload == NULL) {
       		pqueue_insert(recycle_q, event);
       	} else {
       		pqueue_insert(free_q, event);
       	}
		// Peek at the next event in the event queue.
        event = pqueue_peek(event_q);
    } while(event != NULL && event->time == current_time);

	// Release the lock and allow the threads to execute the reactions.    
    pthread_mutex_unlock(&mutex);
    
    // Signal the worker threads.
	pthread_cond_broadcast(&wake);

    return 1;
}

// Worker thread for the thread pool.
void* worker(void* arg) {
	printf("Worker thread started.\n");
	// Keep track of whether we have decremented the idle thread count.
	bool have_been_busy = false;
	pthread_mutex_lock(&mutex);
	while (!stop_requested) {
		// Obtain a reaction from the reaction_q.
		reaction_t* reaction = pqueue_peek(reaction_q);
		reaction_t* executing = pqueue_peek(executing_q);
		// Check whether there is a reaction ready to execute.
		// A reaction that is the earliest one on the rection queue
		// is ready to execute if there are no currently executing reactions
		// with levels less than the level of the reaction.
        // FIXME: This is conservative, since the index just denotes the level in the precedence
        // graph. This can be improved using a binary encoding of dependencies.
        /*
        if (reaction != NULL) {
        	printf("Considering running reaction with index: %lld\n", reaction->index);
        }
        if (executing != NULL) {
        	printf("Some other thread is running reaction with index: %lld\n", executing->index);
        }
        */
		if (reaction == NULL || (executing != NULL && executing->index < reaction->index)) {
			// There are no reactions ready to run.
			// If we were previously busy, count this thread as idle now.
			if (have_been_busy) {
				number_of_idle_threads++;
				have_been_busy = false;
				// Notify the main thread that there is an idle thread.
				pthread_cond_signal(&number_of_idle_threads_increased);
			}
			// Wait for something to change (either a stop request or
			// something went on the reaction queue.
			// printf("Waiting for items on the reaction queue.\n");
			pthread_cond_wait(&wake, &mutex);
		} else {
	    	// This thread will no longer be idle.
	    	if (!have_been_busy) {
	    		number_of_idle_threads--;
	    		have_been_busy = true;
	    	}
	    	
        	reaction_t* reaction = pqueue_pop(reaction_q);
        	// printf("Popped from reaction_q reaction with index: %lld\n", reaction->index);
        	
        	// Push the reaction on the executing queue in order to prevent any
        	// reactions that may depend on it from executing before this reaction is finished.
        	pqueue_insert(executing_q, reaction);
        
        	// If the reaction has a deadline, compare to current physical time
        	// and invoke the deadline violation reaction before the reaction function
        	// if a violation has occurred.
        	if (reaction->deadline > 0LL) {
            	// Get the current physical time.
            	struct timespec current_physical_time;
            	clock_gettime(CLOCK_REALTIME, &current_physical_time);
            	// Convert to instant_t.
            	instant_t physical_time = 
                    	current_physical_time.tv_sec * BILLION
                    	+ current_physical_time.tv_nsec;
            	// Check for deadline violation.
            	if (physical_time > current_time + reaction->deadline) {
                	// Deadline violation has occurred.
                	// Invoke the violation reactions, if there are any.
                	trigger_t* trigger = reaction->deadline_violation;
                	if (trigger != NULL) {
                    	for (int i = 0; i < trigger->number_of_reactions; i++) {
                    		// Unlock the mutex to run the reaction.
 							pthread_mutex_unlock(&mutex);
                       		trigger->reactions[i]->function(trigger->reactions[i]->self);
							pthread_mutex_lock(&mutex);
                        	// If the reaction produced outputs, put the resulting
                        	// triggered reactions into the queue.
                        	// FIXME: The following causes a stack overflow on DeadlineC.lf!  Why???
         					// trigger_output_reactions(trigger->reactions[i]);
         					// trigger_output_reactions(trigger->reactions[i]);
                    	}
                	}
            	}
        	}
        
            // Unlock the mutex to run the reaction.
 			pthread_mutex_unlock(&mutex);
        	// Invoke the reaction function.
        	reaction->function(reaction->self);
        	// Reacquire the mutex lock.
 			pthread_mutex_lock(&mutex);
        	// If the reaction produced outputs, put the resulting triggered
        	// reactions into the queue while holding the mutex lock.
        	trigger_output_reactions(reaction);
        	// There may be new reactions on the reaction queue, so notify other threads.
			pthread_cond_broadcast(&wake);
        	// Remove the reaction from the executing queue.
        	pqueue_remove(executing_q, reaction);
    	}
	}
 	pthread_mutex_unlock(&mutex);
	// Stop has been requested.
	return NULL;
}

// Array of thread IDs (to be dynamically allocated).
pthread_t* __thread_ids;

// Start threads in the thread pool.
void start_threads() {
	__thread_ids = malloc(number_of_threads * sizeof(pthread_t));
	number_of_idle_threads = number_of_threads;
	for (int i = 0; i < number_of_threads; i++) {
		pthread_create(&__thread_ids[i], NULL, worker, NULL);
	}
}

// Print elapsed logical and physical times.
void wrapup() {
	// Signal worker threads to exit.
	// Assume the following write is atomic and therefore need not be guarded.
	stop_requested = true;
	// Signal the worker threads.
	pthread_cond_broadcast(&wake);
	
	// Wait for the worker threads to exit.
	void* worker_thread_exit_status;
	for (int i = 0; i < number_of_threads; i++) {
		pthread_join(__thread_ids[i], &worker_thread_exit_status);
		printf("Worker thread exited.\n");
	}
	free(__thread_ids);
	
    interval_t elapsed_logical_time
        = current_time - (physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec);
    printf("Elapsed logical time (in nsec): %lld\n", elapsed_logical_time);
    
    struct timespec physicalEndTime;
    clock_gettime(CLOCK_REALTIME, &physicalEndTime);
    interval_t elapsed_physical_time
        = (physicalEndTime.tv_sec * BILLION + physicalEndTime.tv_nsec)
        - (physicalStartTime.tv_sec * BILLION + physicalStartTime.tv_nsec);
    printf("Elapsed physical time (in nsec): %lld\n", elapsed_physical_time);
}

int main(int argc, char* argv[]) {
    if (process_args(argc, argv)) {
 		pthread_mutex_lock(&mutex);
        initialize();
        
        // Create a queue on which to put reactions that are currently executing.
        executing_q = pqueue_init(number_of_threads, cmp_pri, get_rct_pri,
            	get_rct_pos, set_rct_pos, eql_rct, prt_rct);

        __start_timers();
        start_threads();
 		pthread_mutex_unlock(&mutex);
        while (next() != 0 && !stop_requested);
        wrapup();
    	return 0;
    } else {
    	return -1;
    }
}
