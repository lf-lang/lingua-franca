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

pqueue_t* transfer_q;  // To store reactions that are still blocked by other reactions.

// The one and only mutex lock.
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// Condition variables used for notification between threads.
pthread_cond_t event_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t reaction_or_executing_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t number_of_idle_threads_changed = PTHREAD_COND_INITIALIZER;

/**
 * Schedule the specified trigger at current_time plus the offset of the
 * specified trigger plus the delay. If the offset of the trigger and
 * the extra_delay are both zero, then the event will occur one
 * microstep later in superdense time (it gets put on the event queue,
 * which will not be examined until all events on the reaction queue
 * have been processed).
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
 * @param token The token wrapping the payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_token(trigger_t* trigger, interval_t extra_delay, token_t* token) {
    pthread_mutex_lock(&mutex);
	int return_value = __schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
 	pthread_mutex_unlock(&mutex);
 	return return_value;
}

/** Placeholder for code-generated function that will, if used in a federated
 *  execution, coordinate the advancement of time.  Given a request to advance
 *  to the specified time, an implementation of this function may block until
 *  it is safe for time to advance to this point. It may return a lesser time
 *  if such blocking was interrupted by either a new event on the event queue
 *  (e.g. a physical action) or the RTI granting advancement to a lesser time.
 *  @param time The time to which to advance.
 *  @return The time to which it is safe to advance.
 */
instant_t request_time_advance(instant_t time);

/** If the -fast option was not given, then wait until physical time
 *  matches the lesser of the specified time or the
 *  timeout time, if a timeout time has been given.
 *  If an event is put on the event queue during the wait, then
 *  stop the wait.
 *  If this execution is part of a federation, then, if necessary,
 *  request that the RTI grant a time advance to the specified time.
 *  Return true if the specified time is reached and false if
 *  either the timeout time is less than the specified time,
 *  the wait is interrupted before the specified time is reached,
 *  or the RTI grants time advance to a lesser time.
 *  The mutex lock is assumed to be held by the calling thread.
 *  Note this this could return true even if the a new event
 *  was placed on the queue if that event time matches the
 *  specified stop time.
 */
bool wait_until(instant_t logical_time_ns) {
    bool return_value = true;
    if (stop_time > 0LL && logical_time_ns > stop_time) {
    	// Modify the time to wait until to be the timeout time.
        logical_time_ns = stop_time;
        // Indicate on return that the time of the event was not reached.
        // We still wait for time to elapse in case asynchronous events come in.
        return_value = false;
    }
    // In case this is in a federation, check whether time can advance
    // to the next time. This call may block waiting for a response from
    // the RTI. If an action triggers during that wait, it will unblock
    // and return with a time less than the next_time.
    instant_t grant_time = request_time_advance(logical_time_ns);
    if (grant_time < logical_time_ns) {
        return_value = false;
    } else if (!fast) {
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

/**
 * Internal version of next() that does not acquire the mutex lock. It assumes
 * the lock is already held.
 *
 * First, wait until the reaction and executing queues are empty. This indicates
 * that the previous logical time is finished. Then, if stop() has been called,
 * return false.
 *
 * Then if there is at least one event in the event queue, then wait until
 * physical time matches or exceeds the time of the least tag on the event
 * queue; pop the next event(s) from the event queue that all have the same tag;
 * extract from those events the reactions that are to be invoked at this
 * logical time and insert them into the reaction queue. The reaction queue is
 * sorted by index, the upper 48 bits of which consist of a deadline and the
 * lower 16 bits denote a level that must be greater than the levels of all
 * reactions that precede it in the precedence graph. Before executing a
 * reaction, each worker verifies that no reactions with a lower level and
 * matching chain id are executing concurrently.
 *
 * Wait until both the reaction and executing queues are empty; and finally,
 * return true.
 *
 * If there is no event in the queue and the keepalive command-line option was
 * not given, return false. If keepalive was given, then wait for either stop()
 * to be called, in which case return false, or an event appears in the event
 * queue, in which case, perform the above sequence of actions.
 *
 * If a timeout option was specified, then when the next logical time from the
 * event queue exceeds the value of that timeout, return false.
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
    	// If this is a federate receiving messages
    	// from another federate, then we may need to consult
    	// the RTI to advance logical time to this physical time.
		if (stop_time > 0LL) {
			if (get_physical_time() >= stop_time) {
			    instant_t grant_time = request_time_advance(stop_time);
			    if (grant_time == stop_time) {
			        // printf("DEBUG: next(): stop time reached by physical clock. Requesting stop.\n");
			        stop_requested = true;
			        // Signal the worker threads.
			        pthread_cond_broadcast(&reaction_or_executing_q_changed);
			        return false;
			    } else {
			        // RTI has granted advance to an earlier time. Continue executing.
			        return true;
			    }
			}
		}
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
    // The wait_until function will release the lock while waiting.
    while (!stop_requested) {
    	// printf("DEBUG: next(): Waiting until time %lld.\n", (wait_until_time - start_time));
    	if (!wait_until(wait_until_time)) {
    		// printf("DEBUG: next(): Wait until time interrupted.\n");
    		// Sleep was interrupted or the timeout time has been reached.
    		// Time has not advanced to the time of the event.
    		// There may be a new earlier event on the queue.
    		// Mutex lock was reacquired by wait_until.
    		event_t* new_event = pqueue_peek(event_q);
    		if (new_event != NULL) next_time = new_event->time;

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

    	    if (new_event == event) {
    			// There is no new event. If the timeout time has been reached,
    			// or there is also no old event,
    			// or if the maximum time has been reached (unlikely), then return.
    			if (new_event == NULL || (stop_time > 0LL && event->time > stop_time)) {
    				stop_requested = true;
    				// Signal the worker threads.
    				pthread_cond_broadcast(&reaction_or_executing_q_changed);
    				return false;
    			}
    		} else {
    			// Handle the new event.
    			event = new_event;
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
        
        token_t* token = event->token;

        // Push the corresponding reactions onto the reaction queue.
        for (int i = 0; i < event->trigger->number_of_reactions; i++) {
            // printf("Pushed on reaction_q: %p\n", event->trigger->reactions[i]);
            // printf("Pushed reaction args: %p\n", event->trigger->reactions[i]->args);
            pqueue_insert(reaction_q, event->trigger->reactions[i]);
        }
        // If the trigger is a periodic clock, create a new event for its next execution.
        // FIXME: This isn't quite right. A logical action can also have a non-zero period field.
        if (!(event->trigger->is_physical) && event->trigger->period > 0) {
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

/**
 * Return `true` if there is currently another reaction blocking the given
 * reaction.
 * @return true if this reaction is blocked, false otherwise.
 */
bool is_blocked(reaction_t* reaction) {
    for (int i = 1; i < executing_q->size; i++) {
        reaction_t* running = executing_q->d[i];
        if (OVERLAPPING(reaction->chain_id, running->chain_id) 
                && LEVEL(reaction->index) > LEVEL(running->index)) {
            return true;
        }
    }
    return false;
}

/**
 * Return the first ready (i.e., unblocked) reaction in the reaction queue if
 * there is one. Return `NULL` if all pending reactions are blocked.
 * @return the first-ranked reaction that is ready to execute, NULL if there is
 * none.
 */ 
reaction_t* first_ready_reaction() {
    
    reaction_t* r;
    reaction_t* b;

    // Find a reaction that is ready to execute.
    while ((r = pqueue_pop(reaction_q)) != NULL) {
        if (is_blocked(r)) {
            // Move blocked reaction onto another queue.
            // NOTE: This could also just be be a FIFO.
            pqueue_insert(transfer_q, r);
        } else {
            break;
        }
    }
    
    // Push blocked reactions back onto the reaction queue.
    if (pqueue_size(reaction_q) >= pqueue_size(transfer_q)) {
        while ((b = pqueue_pop(transfer_q)) != NULL) {
            pqueue_insert(reaction_q, b);
        }
    } else {
        pqueue_t* tmp;
        while ((b = pqueue_pop(reaction_q)) != NULL) {
            pqueue_insert(transfer_q, b);
        }
        tmp = reaction_q;
        reaction_q = transfer_q;
        transfer_q = tmp;
    }
    return r;
}

/**
 * Worker thread for the thread pool.
 */
void* worker(void* arg) {
	// printf("DEBUG: Worker thread started.\n");
	// Keep track of whether we have decremented the idle thread count.
	bool have_been_busy = false;
	pthread_mutex_lock(&mutex);
	// Iterate until stop is requested and the reaction_q is empty (signaling
	// that the current time instant is done).
	while (!stop_requested || pqueue_size(reaction_q) > 0) {
		// Obtain a reaction from the reaction_q that is ready to execute
        // (i.e., it is not blocked by concurrently executing reactions
        // that it depends on).
        reaction_t* reaction = first_ready_reaction();
		if (reaction == NULL) {
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

        	//reaction_t* reaction = pqueue_pop(reaction_q);
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
                        // There may be new reactions on the reaction_queue, so notify other threads.
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

void print_snapshot() {
    printf(">>> START Snapshot\n");
    printf("Ready:\n");
    pqueue_dump(reaction_q, stdout, reaction_q->prt);
    printf("Executing:\n");
    pqueue_dump(executing_q, stdout, executing_q->prt);    
    printf(">>> END Snapshot\n");
}

// Array of thread IDs (to be dynamically allocated).
pthread_t* __thread_ids;

// Start threads in the thread pool.
void start_threads() {
    // printf("DEBUG: Starting %d worker threads.\n", number_of_threads);
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
        
    	// Execute one iteration of next(), which will process the next
    	// timestamp on the event queue, moving its reactions to the reaction
    	// queue. This returns false if there was in fact nothing on the event
    	// queue.
        if (next()) {
        	// printf("DEBUG: wrapup: next() returned\n");
        	// Without relying on the worker threads, execute whatever is on the reaction_q.
        	// NOTE: deadlines on these reactions are ignored.
        	// Is that the right thing to do?
        	while (pqueue_size(reaction_q) > 0) {
        		reaction_t* reaction = pqueue_pop(reaction_q);

        		// Invoke the reaction function.
        		// printf("DEBUG: wrapup(): Invoking reaction.\n");
        		reaction->function(reaction->self);

        		// If the reaction produced outputs, insert the resulting triggered
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
        transfer_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        // Create a queue on which to put reactions that are currently executing.
        executing_q = pqueue_init(number_of_threads, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        __start_timers();
        start_threads();
 		pthread_mutex_unlock(&mutex);
 		// printf("DEBUG: Starting main loop.\n");
        while (next() != 0 && !stop_requested);
        wrapup();
        termination();
    	return 0;
    } else {
        termination();
    	return -1;
    }
}
