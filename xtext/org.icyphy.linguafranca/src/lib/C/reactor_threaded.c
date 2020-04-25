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
pthread_cond_t reaction_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t executing_q_emptied = PTHREAD_COND_INITIALIZER;

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
    // printf("DEBUG: pthread_mutex_lock schedule_token\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
    int return_value = __schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
    // printf("DEBUG: pthread_mutex_unlock schedule_token\n");
    pthread_mutex_unlock(&mutex);
     return return_value;
}

/** Placeholder for code-generated function that will, in a federated
 *  execution, be used to coordinate the advancement of time.  It will notify
 *  the runtime infrastructure (RTI) that all reactions at the specified
 *  logical time have completed.
 *  @param time The logical time that has been completed.
 */
void logical_time_complete(instant_t time);

/** Placeholder for code-generated function that will, in a federated
 *  execution, be used to coordinate the advancement of time.  It will notify
 *  the runtime infrastructure (RTI) of the logical time of the next event
 *  on the event queue (or the stop time, if that is less than any event on the
 *  event queue). An implementation of this function may block until
 *  it is safe for logical time to advance to the specified time.
 *  This function returns either the specified time or a lesser time.
 *  It will return a lesser time if its blocking was interrupted by
 *  either a new event on the event queue (e.g. a physical action) or
 *  if the RTI grants advancement to a lesser time.
 *  @param time The time to which to advance.
 *  @return The time to which it is safe to advance.
 */
instant_t next_event_time(instant_t time);

/** Wait until physical time matches or exceeds the specified logical time,
 *  unless -fast is given.
 *
 *  If an event is put on the event queue during the wait, then the wait is
 *  interrupted and this function returns false. It also returns false if the
 *  timeout time is reached before the wait has completed.
 *
 *  The mutex lock is assumed to be held by the calling thread.
 *  Note this this could return true even if the a new event
 *  was placed on the queue if that event time matches or exceeds
 *  the specified time.
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

// Indicator used to identify the first invocation of __next().
bool __first_invocation = true;

/**
 * If there is at least one event in the event queue, then wait until
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
 * If there is no event in the queue and the keepalive command-line option was
 * not given, set stop_requested to true and return.
 * If keepalive was given, then wait for either stop()
 * to be called or an event appears in the event queue and then return.
 *
 * If a timeout option was specified, then when the next logical time from the
 * event queue exceeds the value of that timeout, set stop_requested to true
 * and return.
 *
 * This does not acquire the mutex lock. It assumes the lock is already held.
 *
 *  @return false if the program should be terminated, true otherwise.
 */
bool __next() {
     // Previous logical time is complete.
    if (stop_requested) {
        return false;
    }

     // Peek at the earliest event in the event queue.
    event_t* event = pqueue_peek(event_q);
    instant_t next_time = FOREVER;
    if (event != NULL) {
        // There is an event in the event queue.
        next_time = event->time;

        // If a stop time was given, adjust the next_time from the
        // event time to that stop time.
        if (stop_time > 0LL && next_time > stop_time) {
            next_time = stop_time;
        }

        // In case this is in a federation, check whether time can advance
        // to the next time. If there are upstream federates, then this call
        // will block waiting for a response from the RTI.
        // If an action triggers during that wait, it will unblock
        // and return with a time (typically) less than the next_time.
        instant_t grant_time = next_event_time(next_time);
        if (grant_time != next_time) {
            // RTI has granted time advance to an earlier time or the wait
            // for the RTI response was interrupted by a local physical action.
            // Continue executing. The event queue may have changed.
            return true;
        }

        // Wait for physical time to advance to the next event time (or stop time).
        // This can be interrupted if a physical action triggers (e.g., a message
        // arrives from an upstream federate or a local physical action triggers).
        // printf("DEBUG: next(): Waiting until time %lld.\n", (next_time - start_time));
        if (!wait_until(next_time)) {
            // printf("DEBUG: next(): Wait until time interrupted.\n");
            // Sleep was interrupted or the timeout time has been reached.
            // Time has not advanced to the time of the event.
            // There may be a new earlier event on the queue.
            // Mutex lock was reacquired by wait_until.
            return true;
        }

        // If the event queue has changed, return to iterate.
        if (pqueue_peek(event_q) != event) return true;

        // If the event time was past the stop time, it is now safe to stop execution.
        if (event->time != next_time) {
            // printf("DEBUG: __next(): logical stop time reached. Requesting stop.\n");
            stop_requested = true;
            // Signal all the worker threads. Since both the queues are
            // empty, the threads will exit without doing anything further.
            pthread_cond_broadcast(&reaction_q_changed);
            return false;
        }

        // At this point, finally, we have an event to process.
        // Advance current time to match that of the first event on the queue.
        current_time = next_time;
        // printf("DEBUG: __next(): ********* Advanced logical time to %lld.\n", current_time - start_time);

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

        return true;
    } else {
        // There is no event on the event queue.
        // printf("DEBUG: __next(): event queue is empty.\n");
        // If a stop time was given, adjust the next_time from FOREVER
        // to that stop time.
        if (stop_time > 0LL) {
            next_time = stop_time;
        }
        // Ask the RTI to advance time to either stop_time or FOREVER.
        // This will be granted if there are no upstream federates.
        // If there are upstream federates, then the call will block
        // until the upstream federates can grant some time advance,
        // and in that case, the returned grant may be less than the
        // requested advance.
        instant_t grant_time = next_event_time(next_time);
        if (grant_time == next_time) {
            // RTI is OK with advancing time to stop_time or FOREVER.
            if (!keepalive_specified) {
                // Since keepalive was not specified, quit.
                // If this is a federate, it will resign from the federation.
                // printf("DEBUG: __next(): requesting stop.\n");
                // Can't call stop() because we already hold a mutex.
                stop_requested = true;
                // Signal all the worker threads.
                pthread_cond_broadcast(&reaction_q_changed);
                return false;
            }
        } else {
            // RTI has granted advance to an earlier time.
            // This means there is an upstream federate that
            // could send messages. There may be a message on
            // the event queue. Continue executing.
            return true;
        }
        // If we get here, the RTI has granted time advance to the stop time
        // (or there is only federate) and keepalive has been specified.
        // If the event queue is no longer empty (e.g. an input message
        // has arrived exactly at the stop time), then return true to
        // iterate again and process that message.
        event = pqueue_peek(event_q);
        if (event != NULL) return true;

        // The event queue is still empty, but since keepalive has been
        // specified, we should not stop unless physical time exceeds the
        // stop_time.
        wait_until(next_time);

        // If the event queue is no longer empty, return true to iterate.
        event = pqueue_peek(event_q);
        if (event != NULL) return true;

        // printf("DEBUG: __next(): Reached stop time. Requesting stop.\n");
        stop_requested = true;
        // Signal the worker threads.
        pthread_cond_broadcast(&reaction_q_changed);
        return false;
    }
}


// Stop execution at the conclusion of the current logical time.
void stop() {
    // printf("DEBUG: pthread_mutex_lock stop\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
    stop_requested = true;
    // Notify the RTI that nothing more will happen.
    next_event_time(FOREVER);
    // In case any thread is waiting on a condition, notify all.
    pthread_cond_broadcast(&reaction_q_changed);
    pthread_cond_signal(&event_q_changed);
    // printf("DEBUG: pthread_mutex_unlock stop\n");
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
    // This will swap the two queues if the transfer_q has
    // gotten larger than the reaction_q.
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

/** Indicator that a worker thread has already taken charge of
 *  advancing time. When another worker thread encouters a true
 *  value to this variable, it should wait for events to appear
 *  on the reaction queue rather than advance time.
 */
bool __advancing_time = false;

/**
 * Worker thread for the thread pool.
 */
void* worker(void* arg) {
    // printf("DEBUG: Worker thread started.\n");
    // Keep track of whether we have decremented the idle thread count.
    bool have_been_busy = false;
    // printf("DEBUG: pthread_mutex_lock worker\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
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

            // If there are also no reactions in progress, then advance time,
            // unless some other worker thread is already advancing time.
            if (pqueue_size(executing_q) == 0 && !__advancing_time) {
                // The following will set stop_requested if there are
                // no events to process. It may block waiting for events
                // to appear on the event queue, but in any case, it will
                // either set stop_request to true or populate the reaction
                // queue with reactions to execute. Note that we already
                // hold the mutex lock.
                if (!__first_invocation) {
                    logical_time_complete(current_time);
                    __first_invocation = false;
                }
                __advancing_time = true;
                __next();
                __advancing_time = false;
                // printf("DEBUG: worker: Done waiting for __next().\n");
            } else {
                // Wait for something to change (either a stop request or
                // something went on the reaction queue.
                // printf("DEBUG: worker: Waiting for items on the reaction queue.\n");
                pthread_cond_wait(&reaction_q_changed, &mutex);
                // printf("DEBUG: worker: Done waiting.\n");
            }
        } else {
            // Got a reaction that is ready to run.

            // If there are additional reactions on the reaction_q, notify one other
            // idle thread, if there is one, so that it can attempt to execute
            // that reaction.
            if (pqueue_size(reaction_q) > 0 && number_of_idle_threads > 0) {
                pthread_cond_signal(&reaction_q_changed);
            }

            // This thread will no longer be idle.
            if (!have_been_busy) {
                number_of_idle_threads--;
                have_been_busy = true;
            }

            //reaction_t* reaction = pqueue_pop(reaction_q);
            // printf("DEBUG: worker: Popped from reaction_q reaction with index: %lld\n and deadline %lld.\n", reaction->index, reaction->local_deadline);

            // Push the reaction on the executing queue in order to prevent any
            // reactions that may depend on it from executing before this reaction is finished.
            pqueue_insert(executing_q, reaction);
        
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
                        // printf("DEBUG: pthread_mutex_unlock to run deadline handler.\n");
                        pthread_mutex_unlock(&mutex);
                        (*handler)(reaction->self);
                        // printf("DEBUG: pthread_mutex_lock worker after running deadline handler\n");
                        pthread_mutex_lock(&mutex);
                        // printf("DEBUG: pthread_mutex_locked\n");
                        // If the reaction produced outputs, put the resulting
                        // triggered reactions into the queue.
                        schedule_output_reactions(reaction);
                        // Remove the reaction from the executing queue.
                        // This thread holds the mutex lock, so if this is the last
                        // reaction of the current time step, this thread will also
                        // be the one to advance time.
                        pqueue_remove(executing_q, reaction);
                    }
                }
            }
            if (!violation) {
                // Unlock the mutex to run the reaction.
                // printf("DEBUG: pthread_mutex_unlock to invoke reaction function\n");
                pthread_mutex_unlock(&mutex);
                // Invoke the reaction function.
                 // printf("DEBUG: worker: Invoking reaction.\n");
                reaction->function(reaction->self);
                // Reacquire the mutex lock.
                // printf("DEBUG: pthread_mutex_lock worker after invoking reaction function\n");
                pthread_mutex_lock(&mutex);
                // printf("DEBUG: pthread_mutex_locked\n");
                // If the reaction produced outputs, put the resulting triggered
                // reactions into the queue while holding the mutex lock.
                schedule_output_reactions(reaction);
                // Remove the reaction from the executing queue.
                // This thread holds the mutex lock, so if this is the last
                // reaction of the current time step, this thread will also
                // be the one to advance time.
                pqueue_remove(executing_q, reaction);

                 // printf("DEBUG: worker: Done invoking reaction.\n");
            }
        }
    }
    // This thread is exiting, so don't count it anymore.
    number_of_threads--;

    // printf("DEBUG: worker: Stop requested. Exiting.\n");
    // printf("DEBUG: pthread_mutex_unlock worker\n");
    // Signal the main thread.
    pthread_cond_signal(&executing_q_emptied);
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
    // Invoke any code-generated wrapup. If this returns true,
    // then actions have been scheduled at the next microstep.
    // Invoke __next() one more time to react to those actions.
    // printf("DEBUG: wrapup invoked.\n");
    if (__wrapup()) {
        // __wrapup() returns true if it has put shutdown events
        // onto the event queue.  We need to run reactions to those
        // events.
         // printf("DEBUG: __wrapup returned true.\n");

        // To make sure next() does its work, unset stop_requested.
        stop_requested = false;
        // To make sure next() doesn't wait for events that will
        // never arrive, unset keepalive.
        keepalive_specified = false;
        
        // Execute one iteration of next(), which will process the next
        // timestamp on the event queue, moving its reactions to the reaction
        // queue. This returns false if there was in fact nothing on the event
        // queue.
        if (__next()) {
            if (!__first_invocation) {
                logical_time_complete(current_time);
                __first_invocation = false;
            }
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
         } else {
             // Notify the RTI that the current logical time is complete.
            logical_time_complete(current_time);
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
        // printf("DEBUG: pthread_mutex_lock main\n");
        pthread_mutex_lock(&mutex);
        // printf("DEBUG: pthread_mutex_locked\n");
        initialize();
        transfer_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        // Create a queue on which to put reactions that are currently executing.
        executing_q = pqueue_init(number_of_threads, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        __start_timers();
        start_threads();
        // printf("DEBUG: pthread_mutex_unlock main\n");
         pthread_mutex_unlock(&mutex);
         // printf("DEBUG: Waiting for worker threads to exit.\n");

         // Wait for the worker threads to exit.
         void* worker_thread_exit_status;
         for (int i = 0; i < number_of_threads; i++) {
             pthread_join(__thread_ids[i], &worker_thread_exit_status);
             printf("Worker thread exited.\n");
         }
         free(__thread_ids);

         wrapup();
        termination();
        return 0;
    } else {
        termination();
        return -1;
    }
}
