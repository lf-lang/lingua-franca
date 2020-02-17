/* Runtime infrastructure for the threaded version of the C target of Lingua Franca. */

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

/** Runtime infrastructure for the threaded version of the C target of Lingua Franca.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 */

#include "reactor_common.c"
#include <pthread.h>

// Number of idle worker threads.
int number_of_idle_threads = 0;

// Queue of currently executing reactions.
pqueue_t* executing_q;  // Sorted by index (precedence sort)

// The one and only mutex lock.
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// Condition variables used for notification between threads.
pthread_cond_t event_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t reaction_or_executing_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t number_of_idle_threads_changed = PTHREAD_COND_INITIALIZER;

// Schedule the specified trigger at current_time plus the
// offset declared in the trigger plus the extra_delay.
// If the offset of the trigger and the extra_delay are both zero,
// then schedule the trigger to occur one microstep later in superdense time.
// If this call occurs between logical times, then instead of adding
// the offset and the extra_delay to the current_time, add them to the
// greater of the current_physical time and the current logical time (current_time).
// The value is required to be a pointer returned by malloc
// because it will be freed after having been delivered to
// all relevant destinations unless it is NULL, in which case
// it will be ignored.
handle_t schedule(trigger_t* trigger, interval_t extra_delay, void* value) {
    pthread_mutex_lock(&mutex);
	int return_value = __schedule(trigger, extra_delay, value);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
 	pthread_mutex_unlock(&mutex);
 	return return_value;
}

/** If the -fast option was not given, then wait until physical time
 *  matches the lesser of the specified time or the
 *  timeout time, if a timeout time has been given.
 *  If an event is put on the event queue during the wait, then
 *  stop the wait.
 *  Return true if the specified time is reached and false if
 *  either the timeout time is less than the specified time,
 *  or the wait is interrupted before the specified time is reached.
 *  The mutex lock is assumed to be held by the calling thread.
 *  Note this this could return true even if the a new event
 *  was placed on the queue if that event time matches the
 *  specified stop time.
 */
bool __wait_until(instant_t logical_time_ns) {
    bool return_value = true;
    if (stop_time > 0LL && logical_time_ns > stop_time) {
    	// Modify the time to wait until to be the timeout time.
        logical_time_ns = stop_time;
        // Indicate on return that the time of the event was not reached.
        // We still wait for time to elapse in case asynchronous events come in.
        return_value = false;
    }
    if (!fast) {
        // Convert the logical time to a timespec.
        // timespec is seconds and nanoseconds.
        struct timespec wait_until_time = {(time_t)logical_time_ns / BILLION, (long)logical_time_ns % BILLION};

        // printf("-------- Waiting for physical time to match logical time %llu.\n", logical_time_ns);
    	// printf("-------- which is %splus %ld nanoseconds.\n", ctime(&wait_until_time.tv_sec), wait_until_time.tv_nsec);

        if (pthread_cond_timedwait(&event_q_changed, &mutex, &wait_until_time) != ETIMEDOUT) {
        	// printf("-------- Wait interrupted.\n");
        	
            // Wait did not time out, which means that there
            // may have been an asynchronous call to schedule(), or
            // it may have been a control-C to stop the process.
            // Do not adjust current_time here. If there was an asynchronous
            // call to schedule(), it will have put an event on the event queue,
            // and current_time will be set to that time when that event is pulled.
            return_value = false;
        }
        // printf("-------- Returned from wait.\n");
    }
    return return_value;
}

/** Free any action values that need to be freed and recycle
 *  the event carrying them.
 */
void __free_action_values() {
	event_t* free_event = pqueue_pop(free_q);
	while (free_event != NULL) {
		if (free_event->value != NULL) {
			free(free_event->value);
		}
		if (free_event->trigger != NULL) {
			// Make sure the trigger is not pointing to freed memory.
			free_event->trigger->value = NULL;
		}
		pqueue_insert(recycle_q, free_event);
		free_event = pqueue_pop(free_q);
	}
}

/** Internal version of next() that does not acquire the mutex lock.
 *  It assumes the lock is already held.
 *
 *  First, wait until the reaction and executing queues are empty.
 *  This indicates that the previous logical time is finished.
 *  Then, if stop() has been called, return false.
 *
 *  Then if there is at least one event in the event queue, then
 *  wait until physical time matches or exceeds the time of the least tag
 *  on the event queue; pop the next event(s) from the
 *  event queue that all have the same tag; extract from those events
 *  the reactions that are to be invoked at this logical time and
 *  transfer them to the reaction queue (which is sorted by precedence);
 *  wait until both the reaction and execution queues are empty;
 *  and finally, return true.
 *
 *  If there is no event in the queue and the
 *  keepalive command-line option was not given, return false.
 *  If keepalive was given, then wait for either stop() to be
 *  called, in which case return false, or an event appears in the
 *  event queue, in which case, perform the above sequence of actions.
 *
 *  If a timeout option was specified, then when the next logical time
 *  from the event queue exceeds the value of that timeout,
 *  return false.
 *
 *  @return false if the program should be terminated.
 */
bool __next() {
 	// Wait for the reaction and executing queues to be empty,
	// indicating that the previous logical time is complete.
	// printf("DEBUG: next(): number_of_idle_threads = %d\n", number_of_idle_threads);
	// printf("DEBUG: next(): reaction_q size = %ld\n", pqueue_size(reaction_q));
 	while (pqueue_size(reaction_q) > 0 || pqueue_size(executing_q) > 0) {
 		// Do not check for stop_requested here because stopping should occur
 		// only between logical times!

        // Wait for some activity on the reaction and executing queues.
        pthread_cond_wait(&reaction_or_executing_q_changed, &mutex);
    	// printf("DEBUG: next(): number_of_idle_threads = %d\n", number_of_idle_threads);
    	// printf("DEBUG: next(): reaction_q size = %ld\n", pqueue_size(reaction_q));
	}
 	// printf("DEBUG: next(): continuing.\n");

 	// Previous logical time is complete.
	
 	__free_action_values();

	if (stop_requested) {
        return false;
	}

 	// Peek at the earliest event in the event queue.
	event_t* event = pqueue_peek(event_q);
    instant_t next_time = LLONG_MAX;
    if (event != NULL) {
    	// There is an event in the event queue.
    	next_time = event->time;
    } else {
    	// There is no event on the event queue.
     	// printf("DEBUG: next(): event queue is empty.\n");
    	if (!keepalive_specified) {
    	 	// printf("DEBUG: next(): requesting stop.\n");
    		stop_requested = true;
			// Signal the worker threads.
			pthread_cond_broadcast(&reaction_or_executing_q_changed);
    		return false;
    	}
		// Check whether physical time exceeds the stop time, if a
		// stop time is given. If it does, return false.
		if (stop_time > 0LL) {
			if (get_physical_time() >= stop_time) {
				// printf("DEBUG: next(): stop time reached by physical clock. Requesting stop.\n");
				stop_requested = true;
				// Signal the worker threads.
				pthread_cond_broadcast(&reaction_or_executing_q_changed);
				return false;
			}
		}
    }

    // Check whether the new logical time exceeds the timeout, if a
    // timeout was specified. Note that this will execute all microsteps
    // at the stop time.
    if (stop_time > 0LL && (event != NULL && next_time > stop_time)) {
	 	// printf("DEBUG: next(): logical stop time reached. Requesting stop.\n");
		stop_requested = true;
 		// Signal the worker threads. Since both the queues are
		// empty, the threads will exit without doing anything further.
		pthread_cond_broadcast(&reaction_or_executing_q_changed);
       	return false;
    }

    instant_t wait_until_time = next_time;
    if (stop_time > 0LL && stop_time < next_time) {
    	wait_until_time = stop_time;
    }

    // Wait until physical time >= event.time (or max time or stop
    // time, if there is no event).
    // Do not hold the lock during the wait.
    // NOTE: We should release the lock even if there will be no physical time wait
    // to allow other threads to sneak in. Perhaps also do a yield?
    // The __wait_until function will release the lock while waiting.
    while (!stop_requested) {
    	// printf("DEBUG: next(): Waiting until time %lld.\n", (wait_until_time - start_time));
    	if (!__wait_until(wait_until_time)) {
    		// printf("DEBUG: next(): Wait until time interrupted.\n");
    		// Sleep was interrupted or the timeout time has been reached.
    		// Time has not advanced to the time of the event.
    		// There may be a new earlier event on the queue.
    		// Mutex lock was reacquired by wait_until.
    		event_t* new_event = pqueue_peek(event_q);
    		if (new_event == event) {
    			// There is no new event. If the timeout time has been reached,
    			// or there is also no old event,
    			// or if the maximum time has been reached (unlikely), then return.
    			if ((stop_time > 0LL && event->time > stop_time) || new_event == NULL) {
    				stop_requested = true;
    				// Signal the worker threads.
    				pthread_cond_broadcast(&reaction_or_executing_q_changed);
    				return false;
    			}
    		} else {
    			// Handle the new event.
    			event = new_event;
    			next_time = event->time;
    			break;
    		}
		} else {
			// printf("DEBUG: next(): Done waiting until time.\n");
			// Arrived at the wait_until_time, but there may still be no
			// event, in which case, we should stop, unless keepalive was
			// specified or physical time has reached the stop time.
			event = pqueue_peek(event_q);
			if (event == NULL) {
				if (!keepalive_specified) {
					// printf("DEBUG: next(): No event. Quitting.\n");
					stop_requested = true;
					// Signal the worker threads.
					pthread_cond_broadcast(&reaction_or_executing_q_changed);
					return false;
				}
				// Check whether physical time exceeds the stop time, if a
				// stop time is given. If it does, return false.
				if (stop_time > 0LL) {
					if (get_physical_time() >= stop_time) {
						// printf("DEBUG: next(): stop time reached by physical clock. Requesting stop.\n");
						stop_requested = true;
						// Signal the worker threads.
						pthread_cond_broadcast(&reaction_or_executing_q_changed);
						return false;
					}
				}
				continue; // Keep waiting.
			} else {
				break;
			}
		}
    }

    // At this point, finally, we have an event to process.

    // Advance current time to match that of the first event on the queue.
    current_time = event->time;
            
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

    // Signal the worker threads.
	pthread_cond_broadcast(&reaction_or_executing_q_changed);

    return true;
}

/** First, wait until the reaction and executing queues are empty.
 *  This indicates that the previous logical time is finished.
 *  Then, if stop() has been called, return false.
 *
 *  If there is at least one event in the event queue, then
 *  wait until physical time matches or exceeds the time of the least tag
 *  on the event queue; pop the next event(s) from the
 *  event queue that all have the same tag; extract from those events
 *  the reactions that are to be invoked at this logical time and
 *  transfer them to the reaction queue (which is sorted by precedence);
 *  wait until both the reaction and execution queues are empty;
 *  and finally, return true.
 *
 *  If there is no event in the queue and the
 *  keepalive command-line option was not given, return false.
 *  If keepalive was given, then wait for either stop() to be
 *  called, in which case return false, or an event appears in the
 *  event queue, in which case, perform the above sequence of actions.
 *
 *  @return false if the program should be terminated.
 */
bool next() {
 	pthread_mutex_lock(&mutex);
 	bool return_value = __next();
	pthread_mutex_unlock(&mutex);
	return return_value;
}

// Stop execution at the conclusion of the current logical time.
void stop() {
    pthread_mutex_lock(&mutex);
    stop_requested = true;
    // In case any thread is waiting on a condition, notify all.
    pthread_cond_broadcast(&reaction_or_executing_q_changed);
    pthread_cond_signal(&event_q_changed);
    pthread_cond_signal(&number_of_idle_threads_changed);
    pthread_mutex_unlock(&mutex);
}

// Worker thread for the thread pool.
void* worker(void* arg) {
	printf("Worker thread started.\n");
	// Keep track of whether we have decremented the idle thread count.
	bool have_been_busy = false;
	pthread_mutex_lock(&mutex);
	// Iterate until stop is requested and the reaction_q is empty (signaling
	// that the current time instant is done).
	while (!stop_requested || pqueue_size(reaction_q) > 0) {
		// Obtain a reaction from the reaction_q.
		reaction_t* reaction = pqueue_peek(reaction_q);
		reaction_t* executing = pqueue_peek(executing_q);
		// Check whether there is a reaction ready to execute.
		// A reaction that is the earliest one on the reaction queue
		// is ready to execute if there are no currently executing reactions
		// with levels less than the level of the reaction.
        // FIXME: This is conservative, since the index just denotes the level in the precedence
        // graph. This can be improved using a binary encoding of dependencies.
		if (reaction == NULL || (executing != NULL && executing->index < reaction->index)) {
			// There are no reactions ready to run.
			// If we were previously busy, count this thread as idle now.
			if (have_been_busy) {
				number_of_idle_threads++;
				have_been_busy = false;
			}
			// Notify the main thread that there is an idle thread.
			// Do this even if we were not previously busy because this
			// could be a startup condition.
			pthread_cond_signal(&number_of_idle_threads_changed);
			// Wait for something to change (either a stop request or
			// something went on the reaction queue.
			// printf("DEBUG: worker: Waiting for items on the reaction queue.\n");
			pthread_cond_wait(&reaction_or_executing_q_changed, &mutex);
            // printf("DEBUG: worker: Done waiting.\n");
		} else {
	    	// This thread will no longer be idle.
	    	if (!have_been_busy) {
	    		number_of_idle_threads--;
	    		have_been_busy = true;
	    	}
			// Notify the main thread that there is one fewer idle thread.
			pthread_cond_signal(&number_of_idle_threads_changed);

        	reaction_t* reaction = pqueue_pop(reaction_q);
        	// printf("DEBUG: worker: Popped from reaction_q reaction with index: %lld\n and deadline %lld.\n", reaction->index, reaction->local_deadline);
        	
        	// Push the reaction on the executing queue in order to prevent any
        	// reactions that may depend on it from executing before this reaction is finished.
        	pqueue_insert(executing_q, reaction);
        
        	// Notify while still holding the lock.
		    pthread_cond_broadcast(&reaction_or_executing_q_changed);

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
                		// Unlock the mutex to run the reaction.
                        pthread_mutex_unlock(&mutex);
                        (*handler)(reaction->self);
						pthread_mutex_lock(&mutex);
                        // If the reaction produced outputs, put the resulting
                        // triggered reactions into the queue.
                        schedule_output_reactions(reaction);
                	    // There may be new reactions on the reaction queue, so notify other threads.
        			    pthread_cond_broadcast(&reaction_or_executing_q_changed);
                	    // Remove the reaction from the executing queue.
                	    pqueue_remove(executing_q, reaction);
                    }
                }
            }
            if (!violation) {
                // Unlock the mutex to run the reaction.
 			    pthread_mutex_unlock(&mutex);
        	    // Invoke the reaction function.
 			    // printf("DEBUG: worker: Invoking reaction.\n");
        	    reaction->function(reaction->self);
        	    // Reacquire the mutex lock.
 			    pthread_mutex_lock(&mutex);
        	    // If the reaction produced outputs, put the resulting triggered
        	    // reactions into the queue while holding the mutex lock.
        	    schedule_output_reactions(reaction);
        	    // Remove the reaction from the executing queue.
        	    pqueue_remove(executing_q, reaction);
        	    // There may be new reactions on the reaction queue, so notify other threads.
			    pthread_cond_broadcast(&reaction_or_executing_q_changed);
 			    // printf("DEBUG: worker: Done invoking reaction.\n");
            }
    	}
	}
	// This thread is exiting, so don't count it anymore.
	number_of_threads--;

	// Notify the main thread that there is one fewer idle thread.
	pthread_cond_signal(&number_of_idle_threads_changed);

	// printf("DEBUG: worker: Stop requested. Exiting.\n");
 	pthread_mutex_unlock(&mutex);
	// timeout has been requested.
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

/** Execute finalization functions, including reactions triggered
 *  by shutdown. This is assumed to be called in the main thread
 *  after all worker threads have exited (or at least finished
 *  all their reaction invocations).
 *  Print elapsed logical and physical times.
 *
 */
void wrapup() {
	// Signal worker threads to exit, in case they haven't already.
	// Assume the following write is atomic and therefore need not be guarded.
 	pthread_mutex_lock(&mutex);
	stop_requested = true;
	// Signal the worker threads.
	pthread_cond_broadcast(&reaction_or_executing_q_changed);
 	pthread_mutex_unlock(&mutex);

	// Wait for the worker threads to exit.
	void* worker_thread_exit_status;
	for (int i = 0; i < number_of_threads; i++) {
		pthread_join(__thread_ids[i], &worker_thread_exit_status);
		printf("Worker thread exited.\n");
	}
	free(__thread_ids);

	// Invoke any code-generated wrapup. If this returns true,
    // then actions have been scheduled at the next microstep.
    // Invoke next() one more time to react to those actions.
	// printf("DEBUG: wrapup invoked.\n");
    if (__wrapup()) {
    	// __wrapup() returns true if it has put shutdown events
    	// onto the event queue.  We need to run reactions to those
    	// events.
 		// printf("DEBUG: __wrapup returned true.\n");

    	// To make sure next() does it's work, unset stop_requested.
    	stop_requested = false;
    	// To make sure next() doesn't wait for events that will
    	// never arrive, unset keepalive.
    	keepalive_specified = false;

    	// Execute one iteration of next(), which will process the
    	// next timestamp on the event queue, moving its reactions
    	// to the reaction queue. This returns false if there was
    	// in fact nothing on the event queue.
        if (next()) {
        	// printf("DEBUG: wrapup: next() returned\n");
        	// printf("DEBUG: reaction_q size = %ld, executing_q = %ld\n", pqueue_size(reaction_q), pqueue_size(executing_q));
        	// Without relying on the worker threads, execute whatever is on the reaction_q.
        	// NOTE: deadlines on these reactions are ignored.
        	// Is that the right thing to do?
        	while (pqueue_size(reaction_q) > 0) {
        		reaction_t* reaction = pqueue_pop(reaction_q);

        		// Invoke the reaction function.
        		// printf("DEBUG: wrapup(): Invoking reaction.\n");
        		reaction->function(reaction->self);

        		// If the reaction produced outputs, put the resulting triggered
        		// reactions into the reaction queue.
        		schedule_output_reactions(reaction);
        		// printf("DEBUG: wrapup(): Done invoking reaction.\n");
        	}
 		}
    }
}

int main(int argc, char* argv[]) {
    // Invoke the function that optionally provides default command-line options.
    __set_default_command_line_options();
    
    // Initialize the one and only mutex to be recursive, meaning that it is OK
    // for the same thread to lock and unlock the mutex even if it already holds
    // the lock.
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&mutex, &attr);

    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {
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
        termination();
    	return 0;
    } else {
        termination();
    	return -1;
    }
}
