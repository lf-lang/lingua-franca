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
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#include "reactor_common.c"
#include <pthread.h>

// Number of idle worker threads.
volatile int number_of_idle_threads = 0;

/*
 * A struct representing a barrier in threaded 
 * Lingua Franca programs that can prevent advancement 
 * of tag if
 * 1- Number of requestors is larger than 0
 * 2- Value of horizon is not (FOREVER, 0)
 */
typedef struct _lf_tag_advancement_barrier {
    int requestors; // Used to indicate the number of
                    // requestors that have asked
                    // for a barrier to be raised
                    // on tag.
    tag_t horizon;  // If semaphore is larger than 0
                    // then the runtime should not
                    // advance its tag beyond the
                    // horizon.
} _lf_tag_advancement_barrier;


/**
 *  Create a global tag barrier and
 * initialize the barrier's semaphore to 0 and its horizon to (FOREVER, 0).
 */
_lf_tag_advancement_barrier _lf_global_tag_advancement_barrier = {0, 
                                                                  (tag_t) {
                                                                    .time = FOREVER,
                                                                    .microstep = 0 
                                                                  }
                                                                 };

// Queue of currently executing reactions.
pqueue_t* executing_q;  // Sorted by index (precedence sort)

pqueue_t* transfer_q;  // To store reactions that are still blocked by other reactions.

// The one and only mutex lock.
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// Condition variables used for notification between threads.
pthread_cond_t event_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t reaction_q_changed = PTHREAD_COND_INITIALIZER;
pthread_cond_t executing_q_emptied = PTHREAD_COND_INITIALIZER;
// A condition variable that notifies threads whenever the number
// of requestors on the tag barrier reaches zero.
pthread_cond_t global_tag_barrier_requestors_reached_zero = PTHREAD_COND_INITIALIZER;

/**
 * Raise a barrier on tag at future_tag if possible (or freeze 
 * the current tag) and increment the total number of requestors 
 * waiting on the barrier. There should always be a subsequent
 * call to _lf_decrement_global_tag_barrier() or 
 * _lf_decrement_global_tag_barrier_already_locked() to release
 * the barrier.
 * 
 * If there is already a barrier raised at a later tag, this 
 * function will move it to future_tag or the current tag, whichever
 * is larger. If the existing barrier is earlier 
 * than future_tag, this function will not move the barrier. If there are 
 * no existing barriers and future_tag is in the past relative to the 
 * current tag, this function will raise a barrier at the current tag.
 * 
 * This function assumes the mutex lock is already held, thus, it will not
 * acquire it itself.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in the federated execution.
 * 
 * @param future_tag A desired tag for the barrier. This function will guarantee
 * that current logical time will not go past future_tag if it is in the future.
 * If future_tag is in the past (or equals to current logical time), the runtime
 * will freeze advancement of logical time.
 */
void _lf_increment_global_tag_barrier_already_locked(tag_t future_tag) {
    tag_t current_tag = get_current_tag();
    // Check to see if future_tag is actually in the future
    if (compare_tags(current_tag, future_tag) < 0) {
        if (compare_tags(future_tag, _lf_global_tag_advancement_barrier.horizon) < 0) {
            // The future tag is smaller than the current horizon of the barrier.
            // Therefore, we should prevent logical time from reaching the
            // expected tag.
            _lf_global_tag_advancement_barrier.horizon.time = future_tag.time;
            _lf_global_tag_advancement_barrier.horizon.microstep = future_tag.microstep - 1; // Just before the requested tag.
                                                                                             // Could be -1 if future_tag has 
                                                                                             // 0 microsteps. In this case,
                                                                                             // the logical time will not advance
                                                                                             // to future_tag.time at all
            DEBUG_PRINT("Raised barrier at tag (%lld, %u).",
                        _lf_global_tag_advancement_barrier.horizon.time,
                        _lf_global_tag_advancement_barrier.horizon.microstep);
        } 
    } else {
            // future_tag is not in the future.
            // Therefore, hold the current logical time.
            _lf_global_tag_advancement_barrier.horizon = current_tag;
            DEBUG_PRINT("Raised barrier at current tag (%lld, %u).",
                        _lf_global_tag_advancement_barrier.horizon.time,
                        _lf_global_tag_advancement_barrier.horizon.microstep);
    }
    // Increment the number of requestors
    _lf_global_tag_advancement_barrier.requestors++;
}

/**
 * Raise a barrier on tag at future_tag if possible (or freeze 
 * the current tag) and increment the total number of requestors 
 * waiting on the barrier. There should always be a subsequent
 * call to _lf_decrement_global_tag_barrier() or 
 * _lf_decrement_global_tag_barrier_already_locked() to release
 * the barrier.
 * 
 * If there is already a barrier raised at a later tag, this 
 * function will move it to future_tag or the current tag, whichever
 * is larger. If the existing barrier is earlier 
 * than future_tag, this function will not move the barrier. If there are 
 * no existing barriers and future_tag is in the past relative to the 
 * current tag, this function will raise a barrier at the current tag.
 * 
 * This function assumes the mutex lock is already held, thus, it will not
 * acquire it itself.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in the federated execution.
 * 
 * @param future_tag A desired tag for the barrier. This function will guarantee
 * that current tag will not go past future_tag if it is in the future.
 * If future_tag is in the past (or equals to current tag), the runtime
 * will freeze advancement of tag.
 */
void _lf_increment_global_tag_barrier(tag_t future_tag) {
    pthread_mutex_lock(&mutex);
    _lf_increment_global_tag_barrier_already_locked(future_tag);
    pthread_mutex_unlock(&mutex);
}

/**
 * Decrement the total number of requestors for the global tag barrier.
 * If the total number of requestors reaches zero, this function resets the
 * tag barrier to (FOREVER, 0) and notifies all threads that are waiting 
 * on the barrier that the number of requestors has reached zero.
 * 
 * This function assumes that the caller already holds the mutex lock.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in the federated execution.
 */
void _lf_decrement_global_tag_barrier_already_locked() {
    // Decrement the number of requestors for the tag barrier.
    _lf_global_tag_advancement_barrier.requestors--;
    // Check to see if the semaphore is negative, which indicates that
    // a mismatched call was placed for this function.
    if (_lf_global_tag_advancement_barrier.requestors < 0) {
        fprintf(stderr, "Mismatched use of _lf_increment_global_tag_barrier()"
         " and  _lf_decrement_global_tag_barrier().\n");
        exit(1);
    } else if (_lf_global_tag_advancement_barrier.requestors == 0) {
        // When the semaphore reaches zero, reset the horizon to forever.
        _lf_global_tag_advancement_barrier.horizon = (tag_t) { .time = FOREVER, .microstep = 0 };
        // Notify waiting threads that the semaphore has reached zero.
        pthread_cond_broadcast(&global_tag_barrier_requestors_reached_zero);
    }
    DEBUG_PRINT("Barrier is at tag (%lld, %u).",
                 _lf_global_tag_advancement_barrier.horizon.time,
                 _lf_global_tag_advancement_barrier.horizon.microstep);
}

/**
 * @see _lf_decrement_global_tag_barrier_already_locked()
 * A variant of _lf_decrement_global_tag_barrier_already_locked() that
 * assumes the caller does not hold the mutex lock, thus, it will acquire it
 * itself.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in the federated execution.
 */
void _lf_decrement_global_tag_barrier() {
    pthread_mutex_lock(&mutex);
    // Call the original function
    _lf_decrement_global_tag_barrier_already_locked();
    pthread_mutex_unlock(&mutex);
}

/**
 * A function that will wait if the proposed tag
 * is larger than a requested barrier on tag until
 * that barrier is lifted.
 * 
 * 
 * This function assumes the mutex is already locked.
 * Thus, it unlocks the mutex while it's waiting to allow
 * the tag barrier to change.
 * 
 * @param proposed_tag The tag that the runtime wants
 *  to advance to.
 */
void _lf_wait_on_global_tag_barrier(tag_t proposed_tag) {
    // Do not wait for FOREVER
    if (proposed_tag.time == FOREVER) {
        return;
    }
    // Wait if the global barrier semaphore on logical time is zero
    // and the proposed_time is larger than the horizon.
    while (compare_tags(proposed_tag, _lf_global_tag_advancement_barrier.horizon) &&
          _lf_global_tag_advancement_barrier.requestors > 0) {
        DEBUG_PRINT("Waiting on barrier for tag (%lld, %u).", proposed_tag.time, proposed_tag.microstep);
        // Wait until no requestor remains for the barrier on logical time
        pthread_cond_wait(&global_tag_barrier_requestors_reached_zero, &mutex);
    }
}

/**
 * Schedule the specified trigger at current_tag.time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_token(void* action, interval_t extra_delay, token_t* token) {
    trigger_t* trigger = _lf_action_to_trigger(action);
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

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_copy(void* action, interval_t offset, void* value, int length) {
    if (value == NULL) {
        return _lf_schedule_token(action, offset, NULL);
    }
    trigger_t* trigger = _lf_action_to_trigger(action);

    if (trigger == NULL || trigger->token == NULL || trigger->token->element_size <= 0) {
        fprintf(stderr, "ERROR: schedule: Invalid trigger or element size.\n");
        return -1;
    }
    // printf("DEBUG: pthread_mutex_lock schedule_token\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
    // Initialize token with an array size of length and a reference count of 0.
    token_t* token = __initialize_token(trigger->token, length);
    // Copy the value into the newly allocated memory.
    memcpy(token->value, value, token->element_size * length);
    // The schedule function will increment the reference count.
    handle_t result = __schedule(trigger, offset, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
    // printf("DEBUG: pthread_mutex_unlock schedule_token\n");
    pthread_mutex_unlock(&mutex);
    return result;
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, int length) {
    trigger_t* trigger = _lf_action_to_trigger(action);

    // printf("DEBUG: pthread_mutex_lock schedule_token\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
    token_t* token = create_token(trigger->element_size);
    token->value = value;
    token->length = length;
    int return_value = __schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
    // printf("DEBUG: pthread_mutex_unlock schedule_token\n");
    pthread_mutex_unlock(&mutex);
    return return_value;
}

/** 
 * Placeholder for code-generated function that will, in a federated
 * execution, be used to coordinate the advancement of tag. It will notify
 * the runtime infrastructure (RTI) that all reactions at the specified
 * logical tag have completed. This function should be called only while
 * holding the mutex lock.
 * @param time The logical time that has been completed.
 * @param microstep The logical microstep that has been completed.
 */
void logical_time_complete(instant_t time, microstep_t microstep);

/** 
 * Placeholder for code-generated function that will, in a federated
 * execution, be used to coordinate the advancement of time.  It will notify
 * the runtime infrastructure (RTI) of the logical time of the next event
 * on the event queue (or the stop time, if that is less than any event on the
 * event queue). An implementation of this function may block until
 * it is safe for logical time to advance to the specified time.
 * This function returns either the specified time or a lesser time.
 * It will return a lesser time if its blocking was interrupted by
 * either a new event on the event queue (e.g. a physical action) or
 * if the RTI grants advancement to a lesser time.
 * @param time The time to which to advance.
 * @param microstep The microstep to which to advance.
 * @return The time to which it is safe to advance.
 */
tag_t next_event_tag(instant_t time, microstep_t microstep);

/**
 * Wait until physical time matches or exceeds the specified logical time,
 * unless -fast is given. If a barrier on logical time is raised at an earlier
 * logical time, wait until the barrier is removed.
 *
 * If an event is put on the event queue during the wait, then the wait is
 * interrupted and this function returns false. It also returns false if the
 * timeout time is reached before the wait has completed.
 * 
 * In certain cases, if a reaction is added to the reaction queue during the
 * wait, this function returns false to allow for execution of those reactions. * 
 *
 * The mutex lock is assumed to be held by the calling thread.
 * Note this this could return true even if the a new event
 * was placed on the queue if that event time matches or exceeds
 * the specified time.
 *
 * @param logical_time_ns Logical time to wait on.
 * @param mirostep The microstep to wait on used exclusively for federated 
 *  applications with decentralized coordination. Ignored otherwise. 
 * 
 * @return False if the wait is interrupted either because of an event
 *  queue signal or if the wait time was interrupted early by reaching
 *  the stop time, if one was specified.
 */
bool wait_until(instant_t logical_time_ns, microstep_t microstep) {
    bool return_value = true;
    if (timeout_time != NEVER && logical_time_ns > timeout_time) {
        // Modify the time to wait until to be the timeout time.
        logical_time_ns = timeout_time;
        // Indicate on return that the time of the event was not reached.
        // We still wait for time to elapse in case asynchronous events come in.
        return_value = false;
    }
    interval_t wait_until_time_ns = logical_time_ns;
#ifdef _LF_COORD_DECENTRALIZED // Only apply the STP offset if coordination is decentralized
    // Apply the STP offset to the logical time
    wait_until_time_ns += _lf_global_time_STP_offset;
#endif
    if (!fast) {
        // Convert the logical time to a timespec.
        // timespec is seconds and nanoseconds.
        struct timespec wait_until_time = {(time_t)wait_until_time_ns / BILLION, (long)wait_until_time_ns % BILLION};

        // printf("DEBUG: -------- Waiting for physical time to match logical time %llu.\n", logical_time_ns);
        // printf("DEBUG: -------- which is %splus %ld nanoseconds.\n", ctime(&wait_until_time.tv_sec), wait_until_time.tv_nsec);

        if (pthread_cond_timedwait(&event_q_changed, &mutex, &wait_until_time) != ETIMEDOUT) {
            // printf("DEBUG: -------- Wait interrupted.\n");

            // Wait did not time out, which means that there
            // may have been an asynchronous call to schedule().
            // Do not adjust current_tag.time here. If there was an asynchronous
            // call to schedule(), it will have put an event on the event queue,
            // and current_tag.time will be set to that time when that event is pulled.
            return_value = false;
        }
        // printf("DEBUG: -------- Returned from wait.\n");

    }
#ifdef _LF_IS_FEDERATED // Only wait on the tag barrier in federated LF programs
    // Wait until the global barrier on tag (horizon) is larger
    // than the tag.
    // This can effectively add to the STP offset in certain cases, for example,
    // when a message with timestamp (0,0) has arrived at (0,0).
    _lf_wait_on_global_tag_barrier((tag_t) { .time = logical_time_ns, .microstep = microstep } );
#endif
    return return_value;
}

// Indicator used to identify the first invocation of __next().
bool __first_invocation = true;

/**
 * If there is at least one event in the event queue, then wait until
 * physical time matches or exceeds the time of the least tag on the event
 * queue; pop the next event(s) from the event queue that all have the same tag;
 * extract from those events the reactions that are to be invoked at this
 * logical time and insert them into the reaction queue. The event queue is
 * sorted by time tag.
 *
 * If there is no event in the queue and the keepalive command-line option was
 * not given, set stop_requested to true and return.
 * If keepalive was given, then wait for either request_stop()
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
    event_t* event = (event_t*)pqueue_peek(event_q);
    tag_t next_tag = { .time = FOREVER, .microstep = UINT_MAX };

    bool next_event_tag_is_after_timeout = false;
    if (event != NULL) {
        // There is an event in the event queue.
        next_tag.time = event->time;
        if (next_tag.time == current_tag.time) {
            next_tag.microstep =  get_microstep() + 1;
        } else {
            next_tag.microstep = 0;
        }

        // DEBUG_PRINT("Got event with time %lld off the event queue.", next_time);
        // If a timeout tag was given, adjust the next_tag from the
        // event tag to that timeout tag.
        // FIXME: This is primarily done to let the RTI
        // know about the timeout time? 
        if (_lf_is_tag_after_timeout(next_tag)) {
            next_event_tag_is_after_timeout = true;
            next_tag = (tag_t) { .time = timeout_time, .microstep = 0 };
        }

#ifdef _LF_IS_FEDERATED
        // In case this is in a federation, check whether tag can advance
        // to the next tag. If there are upstream federates, then this call
        // will block waiting for a response from the RTI.
        // If an action triggers during that wait, it will unblock
        // and return with a time (typically) less than the next_time.
        tag_t grant_tag = next_event_tag(next_tag.time, next_tag.microstep);
        if (compare_tags(grant_tag, next_tag) != 0) {
            // RTI has granted tag advance to an earlier tag or the wait
            // for the RTI response was interrupted by a local physical action.
            // Continue executing. The event queue may have changed.
            return true;
        }

        // Since next_event_tag releases the mutex lock internally, we need to check
        // if the timeout time has changed or if stop is requested in case a 
        // request_stop() has been called while the mutex was unlocked.
        if (stop_requested) {
            return false;
        } else if (_lf_is_tag_after_timeout(next_tag)) {
            next_event_tag_is_after_timeout = true;
            next_tag.time = timeout_time;
            next_tag.microstep = 0;
        }
#endif

        // Wait for physical time to advance to the next event time (or stop time).
        // This can be interrupted if a physical action triggers (e.g., a message
        // arrives from an upstream federate or a local physical action triggers).
        // printf("DEBUG: next(): Waiting until time %lld.\n", (next_time - start_time));
        if (!wait_until(next_tag.time, next_tag.microstep)) {
            // printf("DEBUG: __next(): Wait until time interrupted.\n");
            // Sleep was interrupted or the stop time has been reached.
            // Time has not advanced to the time of the event.
            // There may be a new earlier event on the queue.
            // Mutex lock was reacquired by wait_until.
            return true;
        }
        
        // If the event queue has changed, return to iterate.
        if ((event_t*)pqueue_peek(event_q) != event) {
            return true;
        }
        
        // Since wait_until releases the mutex lock internally, we need to check
        // if the timeout time has changed or if stop is requested in case a 
        // request_stop() has been called while the mutex was unlocked.
        // If the event tag was past the timeout time, it is now safe to stop the
        // execution.
        if (stop_requested) {
            return false;
        }else if (_lf_is_tag_after_timeout(next_tag) || next_event_tag_is_after_timeout) {
            stop_requested = true;
            // Signal all the worker threads. Since both the queues are
            // empty, the threads will exit without doing anything further.
            pthread_cond_broadcast(&reaction_q_changed);
            return false;
        }
                
        // At this point, finally, we have an event to process.
        // Advance current time to match that of the first event on the queue.
        _lf_advance_logical_time(next_tag.time);

        // DEBUG_PRINT("__next(): ********* Advanced logical time to %lld.", current_tag.time - start_time);

        // Invoke code that must execute before starting a new logical time round,
        // such as initializing outputs to be absent.
        __start_time_step();
        // Pop all events from event_q with timestamp equal to current_tag.time,
        // extract all the reactions triggered by these events, and
        // stick them into the reaction queue.
        __pop_events();
        
        return true;
    } else {
        // There is no event on the event queue.
        // printf("DEBUG: __next(): event queue is empty.\n");
        // If a timeout tag was given, adjust the next_tag's time
        // from FOREVER to timeout time.
        if (timeout_time != NEVER) {
            next_tag.time = timeout_time;
            next_tag.microstep = 0;
        }

        // Ask the RTI to advance time to either timeout_time or FOREVER.
        // This will be granted if there are no upstream federates.
        // If there are upstream federates, then the call will block
        // until the upstream federates can grant some time advance,
        // and in that case, the returned grant may be less than the
        // requested advance.
        // printf("DEBUG: __next(): next event time to RTI %lld.\n", next_time - start_time);
        // FIXME: verify
        tag_t grant_tag = next_event_tag(next_tag.time, next_tag.microstep);
        // printf("DEBUG: __next(): RTI grants time advance to %lld.\n", grant_time - start_time);
        if (compare_tags(grant_tag, next_tag) == 0) {
            // RTI is OK with advancing tag to (timeout_time, 0) or (FOREVER, 0).
            // Note that keepalive is always set for a federated execution.
            if (!keepalive_specified) {
                // Since keepalive was not specified, quit.
                // printf("DEBUG: __next(): requesting stop.\n");
                // Can't call request_stop() because we already hold a mutex.
                stop_requested = true;
                // Signal all the worker threads.
                pthread_cond_broadcast(&reaction_q_changed);
                return false;
            }
            // printf("DEBUG: __next(): keepalive is specified or in a federation. Keep going.\n");
        } else {
            // RTI has granted advance to an earlier time.
            // This means there is an upstream federate that
            // could send messages. There may be a message on
            // the event queue. Continue executing.
            return true;
        }

        // Since next_event_tag releases the mutex lock internally, we need to check
        // if the timeout time has changed or if stop is requested in case a 
        // request_stop() has been called while the mutex was unlocked.
        if (_lf_is_tag_after_timeout(next_tag)) {
            // Set the next tag to the current tag to stop as soon as possible
            next_tag = current_tag;
        }


        // If we get here, the RTI has granted time advance to the stop time
        // (or there is only federate, or to FOREVER is there is no stop time)
        // and keepalive has been specified.
        // If the event queue is no longer empty (e.g. an input message
        // has arrived exactly at the stop time), then return true to
        // iterate again and process that message.
        event = (event_t*)pqueue_peek(event_q);
        if (event != NULL) return true;

        // The event queue is still empty, but since keepalive has been
        // specified, we should not stop unless physical time exceeds the
        // timeout_time.  If --fast has been specified, however, this will
        // return immediately.
        // FIXME: --fast becomes useless for federated execution because
        // the federates proceed immediately to the stop time or FOREVER!
        // printf("DEBUG: __next(): Empty queue. Waiting until time %lld.\n", next_time - start_time);
        if (!wait_until(next_tag.time, next_tag.microstep)) {
            // printf("DEBUG: __next(): Wait until time interrupted.\n");
            // Sleep was interrupted or the stop time has been reached.
            // Time has not advanced to the time of the event.
            // There may be a new earlier event on the queue.
            // Mutex lock was reacquired by wait_until.
            // Iterate in case something has appeared on the event queue.
            // If the event queue is still empty and we have reached the stop
            // time, then the next iteration will stop.
            return true;
        }
        // wait_until successfully waited until next_time.
        // If the event queue is no longer empty, return true to iterate.
        event = (event_t*)pqueue_peek(event_q);
        if (event != NULL) return true;

        // Nothing on the event queue and we have reached the stop time.
        // printf("DEBUG: __next(): Reached stop time of %lld. Requesting stop.\n", next_time - start_time);
        stop_requested = true;
        // Signal the worker threads.
        pthread_cond_broadcast(&reaction_q_changed);
        return false;
    }
}

/**
 * Request a stop to execution as soon as possible.
 * In a non-federated execution, this will occur
 * at the conclusion of the current logical time.
 * In a federated execution, it will likely occur at
 * a later logical time determined by the RTI so that
 * all federates stop at the same logical time.
 */
void request_stop() {
    // printf("DEBUG: pthread_mutex_lock stop\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
#ifdef _LF_IS_FEDERATED
    _lf_fd_send_stop_request_to_rti();
    // Notify the RTI that nothing more will happen.
    next_event_tag(FOREVER, 0);
    // Do not set stop_requested
    // since the RTI might grant a
    // later stop tag than the current
    // tag. The _lf_fd_send_request_stop_to_rti() 
    // will raise a barrier at the current
    // logical time.
#else
    stop_requested = true;
    // In case any thread is waiting on a condition, notify all.
    pthread_cond_broadcast(&reaction_q_changed);
    pthread_cond_signal(&event_q_changed);
#endif
    // printf("DEBUG: pthread_mutex_unlock stop\n");
    pthread_mutex_unlock(&mutex);
}

/**
 * Return `true` if there is currently another reaction on
 * the executing queue that is blocking the given
 * reaction. A reaction blocks the specified reaction if it has a
 * level less than that of the specified reaction and it also has
 * an overlapping chain ID, meaning that it is (possibly) upstream
 * of the specified reaction.
 * @return true if this reaction is blocked, false otherwise.
 */
bool is_blocked(reaction_t* reaction) {
    for (int i = 1; i < executing_q->size; i++) {
        reaction_t* running = (reaction_t*) executing_q->d[i];
        if (LEVEL(running->index) < LEVEL(reaction->index)
                && OVERLAPPING(reaction->chain_id, running->chain_id)) {
            return true;
        }
    }
    // Note that there is no need to check the transfer_q, which contains
    // reactions popped from the reaction_q that have previously been
    // determined to be blocked by executing reactions. The reason that
    // we don't have to check the transfer_q is that if there is a reaction
    // on that queue blocking this one, then there must also be a reaction
    // on the executing queue blocking this one. Blocking is transitive.

    // printf("Not blocking for reaction with chainID %llu and level %llu\n", reaction->chain_id, reaction->index);
    // pqueue_dump(executing_q, stdout, executing_q->prt);
    return false;
}

/**
 * Return the first ready (i.e., unblocked) reaction in the reaction queue if
 * there is one. Return `NULL` if all pending reactions are blocked.
 *
 * The reaction queue is sorted by index, where a lower index appears
 * earlier in the queue. The first reaction in the reaction queue is
 * ready to execute if it is not blocked by any reaction that is currently
 * executing in another thread. If that first reaction is blocked, then
 * the second reaction is ready to execute if it is not blocked by any
 * reaction that is currently executing (if it is blocked by the first
 * reaction, then it is also blocked by a currently executing reaction because
 * the first reaction is blocked).
 *
 * The upper 48 bits of the index are the deadline and the
 * lower 16 bits denote a level in the precedence graph.  Reactions that
 * do not depend on any upstream reactions have level 0, and greater values
 * indicate the length of the longest upstream path to a reaction with level 0.
 * If a reaction has no specified deadline and is not upstream of any reaction
 * with a specified deadline, then its deadline is the largest 48 bit number.
 * Also, all reactions that precede a reaction r that has a deadline D
 * are are assigned a deadline D' <= D.
 *
 * A reaction r is blocked by an executing reaction e if e has a lower level
 * and the chain ID of e overlaps (shares at least one bit) with the chain ID
 * of r. If the two chain IDs share no bits, then we are assured that e is not
 * upstream of r and hence cannot block r.
 *
 * @return the first-ranked reaction that is ready to execute, NULL if there is
 * none.
 */ 
reaction_t* first_ready_reaction() {
    
    reaction_t* r;
    reaction_t* b;

    // Find a reaction that is ready to execute.
    while ((r = (reaction_t*)pqueue_pop(reaction_q)) != NULL) {
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
        while ((b = (reaction_t*)pqueue_pop(transfer_q)) != NULL) {
            pqueue_insert(reaction_q, b);
        }
    } else {
        pqueue_t* tmp;
        while ((b = (reaction_t*)pqueue_pop(reaction_q)) != NULL) {
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
volatile bool __advancing_time = false;

/**
 * Put the specified reaction on the reaction queue.
 * This version acquires a mutex lock.
 * @param reaction The reaction.
 */
void _lf_enqueue_reaction(reaction_t* reaction) {
    // Acquire the mutex lock.
    // printf("DEBUG: pthread_mutex_lock worker to queue downstream reaction.\n");
    pthread_mutex_lock(&mutex);
    // printf("DEBUG: pthread_mutex_locked\n");
    // Do not enqueue this reaction twice.
    if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
        // printf("DEBUG: Enqueing downstream reaction %p.\n", reaction);
        pqueue_insert(reaction_q, reaction);
        pthread_cond_signal(&reaction_q_changed);
    }
    pthread_mutex_unlock(&mutex);
    // printf("DEBUG: pthread_mutex_unlock after queueing downstream reaction.\n");
}

/** For logging and debugging, each worker thread is numbered. */
int worker_thread_count = 0;

/**
 * Worker thread for the thread pool.
 */
void* worker(void* arg) {
    // printf("DEBUG: Worker thread started.\n");
    // Keep track of whether we have decremented the idle thread count.
    bool have_been_busy = false;
    // printf("DEBUG: pthread_mutex_lock worker\n");
    pthread_mutex_lock(&mutex);

    int worker_number = ++worker_thread_count;

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

            // If there are no reactions in progress and no reactions on
            // the reaction queue, then advance time,
            // unless some other worker thread is already advancing time.
            if (pqueue_size(reaction_q) == 0
                    && pqueue_size(executing_q) == 0
                    && !__advancing_time)
            {
                //if (!__first_invocation) {
                logical_time_complete(current_tag.time, current_tag.microstep);
                //}
                //__first_invocation = false;
                // The following will set stop_requested if there are
                // no events to process or if __next() returns false.
                // __next() may block waiting for events
                // to appear on the event queue, but in any case, it will
                // either set stop_request to true or populate the reaction
                // queue with reactions to execute. Note that we already
                // hold the mutex lock.
                __advancing_time = true;
                __next();
                __advancing_time = false;
                // printf("DEBUG: worker %d: Done waiting for __next().\n", worker_number);
            } else {
                // Wait for something to change (either a stop request or
                // something went on the reaction queue.
                // printf("DEBUG: worker %d: Waiting for items on the reaction queue.\n", worker_number);
                pthread_cond_wait(&reaction_q_changed, &mutex);
                // printf("DEBUG: worker %d: Done waiting.\n", worker_number);
            }
        } else {
            // Got a reaction that is ready to run.
            // printf("DEBUG: worker %d: Popped from reaction_q reaction with index: %lld\n and deadline %lld.\n", worker_number, reaction->index, reaction->deadline);

            // This thread will no longer be idle.
            if (!have_been_busy) {
                number_of_idle_threads--;
                have_been_busy = true;
            }

            // Push the reaction on the executing queue in order to prevent any
            // reactions that may depend on it from executing before this reaction is finished.
            pqueue_insert(executing_q, reaction);

            // If there are additional reactions on the reaction_q, notify one other
            // idle thread, if there is one, so that it can attempt to execute
            // that reaction.
            if (pqueue_size(reaction_q) > 0 && number_of_idle_threads > 0) {
                pthread_cond_signal(&reaction_q_changed);
            }
        

            bool violation = false;
            // If the reaction is tardy,
            // an input trigger to this reaction has been triggered at a later
            // logical time than originally anticipated. In this case, a special
            // tardy reaction will be invoked.             
            // FIXME: Note that the tardy reaction will be invoked
            // at most once per logical time value. If the tardy reaction triggers the
            // same reaction at the current time value, even if at a future superdense time,
            // then the reaction will be invoked and the tardy reaction will not be invoked again.
            // However, inputs ports to a federate reactor are network port types so this possibly should
            // be disallowed.
            // @note The tardy handler and the deadline handler are not mutually exclusive.
            //  In other words, both can be invoked for a reaction if it is triggered late
            //  in logical time (tardy) and also misses the constraint on physical time (deadline).
            // @note In absence of a tardy handler, the is_tardy will be passed down the reaction
            //  chain until it is dealt with in a downstream tardy handler.
            if (reaction->is_tardy == true) {
                reaction_function_t handler = reaction->tardy_handler;
                DEBUG_PRINT("Worker %d: Invoking tardiness handler %p.", worker_number, handler);
                // Invoke the tardy handler if there is one.
                if (handler != NULL) {
                    // There is a violation
                    violation = true;
                    // Unlock the mutex to run the reaction.
                    DEBUG_PRINT("Worker %d: pthread_mutex_unlock to run tardiness handler.\n", worker_number);
                    pthread_mutex_unlock(&mutex);
                    (*handler)(reaction->self);

                    // If the reaction produced outputs, put the resulting
                    // triggered reactions into the queue or execute them directly if possible.
                    schedule_output_reactions(reaction, worker_number);
                    
                    // Reset the is_tardy because it has been dealt with
                    reaction->is_tardy = false;
                }
            }
            // If the reaction has a deadline, compare to current physical time
            // and invoke the deadline violation reaction instead of the reaction function
            // if a violation has occurred. Note that the violation reaction will be invoked
            // at most once per logical time value. If the violation reaction triggers the
            // same reaction at the current time value, even if at a future superdense time,
            // then the reaction will be invoked and the violation reaction will not be invoked again.
            if (reaction->deadline > 0LL) {
                // Get the current physical time.
                struct timespec current_physical_time;
                clock_gettime(CLOCK_REALTIME, &current_physical_time);
                // Convert to instant_t.
                instant_t physical_time =
                        current_physical_time.tv_sec * BILLION
                        + current_physical_time.tv_nsec;
                // Check for deadline violation.
                if (physical_time > current_tag.time + reaction->deadline) {
                    // Deadline violation has occurred.
                    violation = true;
                    // Invoke the local handler, if there is one.
                    reaction_function_t handler = reaction->deadline_violation_handler;
                    if (handler != NULL) {
                        // Unlock the mutex to run the reaction.
                        // printf("DEBUG: Worker %d: pthread_mutex_unlock to run deadline handler.\n", worker_number);
                        pthread_mutex_unlock(&mutex);
                        (*handler)(reaction->self);

                        // If the reaction produced outputs, put the resulting
                        // triggered reactions into the queue or execute them directly if possible.
                        schedule_output_reactions(reaction, worker_number);
                        // Remove the reaction from the executing queue.
                    }
                }
            }
            if (violation) {
                // Need to acquire the mutex lock to remove this from the executing queue
                // and to obtain the next reaction to execute.
                // printf("DEBUG: Worker %d: pthread_mutex_lock worker after running deadline handler\n", worker_number);
                pthread_mutex_lock(&mutex);
                // printf("DEBUG: Worker %d: pthread_mutex_locked\n", worker_number);

                // The reaction is not going to be executed. However,
                // this thread holds the mutex lock, so if this is the last
                // reaction of the current time step, this thread will also
                // be the one to advance time.
                pqueue_remove(executing_q, reaction);
            } else {
                // Unlock the mutex to run the reaction.
                // printf("DEBUG: Worker %d: pthread_mutex_unlock to invoke reaction function\n", worker_number);
                pthread_mutex_unlock(&mutex);
                // Invoke the reaction function.
                DEBUG_PRINT("Worker %d: Invoking reaction.", worker_number);
                tracepoint_reaction_starts(reaction, worker_number);
                reaction->function(reaction->self);
                tracepoint_reaction_ends(reaction, worker_number);
                DEBUG_PRINT("Worker %d: Done invoking reaction.", worker_number);
                // If the reaction produced outputs, put the resulting triggered
                // reactions into the queue or execute them immediately.
                schedule_output_reactions(reaction, worker_number);

                // Reacquire the mutex lock.
                // printf("DEBUG: Worker %d: pthread_mutex_lock worker after invoking reaction function\n", worker_number);
                pthread_mutex_lock(&mutex);
                // printf("DEBUG: Worker %d: pthread_mutex_locked\n", worker_number);

                // Remove the reaction from the executing queue.
                // This thread holds the mutex lock, so if this is the last
                // reaction of the current time step, this thread will also
                // be the one to advance time.
                pqueue_remove(executing_q, reaction);
            }
            // Reset the is_tardy because it has been passed
            // down the chain
            reaction->is_tardy = false;

            // printf("DEBUG: Worker %d: Done invoking reaction.\n", worker_number);
        }
    } // while (!stop_requested || pqueue_size(reaction_q) > 0)
    // This thread is exiting, so don't count it anymore.
    _lf_number_of_threads--;

    // printf("DEBUG: Worker %d: Stop requested. Exiting.\n", worker_number);
    // printf("DEBUG: Worker %d: pthread_mutex_unlock\n", worker_number);
    // Signal the main thread.
    pthread_cond_signal(&executing_q_emptied);
    pthread_mutex_unlock(&mutex);
    // timeout has been requested.
    return NULL;
}

void print_snapshot() {
    printf(">>> START Snapshot\n");
    printf("Pending:\n");
    pqueue_dump(reaction_q, stdout, reaction_q->prt);
    printf("Executing:\n");
    pqueue_dump(executing_q, stdout, executing_q->prt);    
    printf(">>> END Snapshot\n");
}

// Array of thread IDs (to be dynamically allocated).
pthread_t* __thread_ids;

// Start threads in the thread pool.
void start_threads() {
    // printf("DEBUG: Starting %d worker threads.\n", _lf_number_of_threads);
    __thread_ids = (pthread_t*)malloc(_lf_number_of_threads * sizeof(pthread_t));
    number_of_idle_threads = _lf_number_of_threads;
    for (int i = 0; i < _lf_number_of_threads; i++) {
        pthread_create(&__thread_ids[i], NULL, worker, NULL);
    }
}

/** Execute all reactions on the reaction queue in sequence.
 *  This function should be called with the mutex locked.
 *  It will return with the mutex still locked, but it will
 *  unlock it to run the reactions.
 */
void __execute_reactions_during_wrapup() {
    // NOTE: deadlines on these reactions are ignored.
    // Is that the right thing to do?
    while (pqueue_size(reaction_q) > 0) {
        pthread_mutex_lock(&mutex);
        reaction_t* reaction = (reaction_t*)pqueue_pop(reaction_q);
        // Check if the reaction is tardy
        if (reaction->is_tardy && (reaction->tardy_handler != NULL)) {
            // Invoke the tardy handler if it exists.
            pthread_mutex_unlock(&mutex);
            DEBUG_PRINT("wrapup(): Invoking the tardy handler.\n");
            reaction->tardy_handler(reaction->self);
        } else {
            pthread_mutex_unlock(&mutex);
            // Invoke the reaction function.
            DEBUG_PRINT("wrapup(): Invoking reaction.\n");
            tracepoint_reaction_starts(reaction, 0); // 0 indicates main thread.
            reaction->function(reaction->self);
            tracepoint_reaction_ends(reaction, 0); // 0 indicates main thread.
        }

        // If the reaction produced outputs, insert the resulting triggered
        // reactions into the reaction queue.
        schedule_output_reactions(reaction, 0); // Worker threads have exited, so 0 thread.
        DEBUG_PRINT("wrapup(): Done invoking reaction.");
    }
}

/** 
 * Execute reactions that are already in the reaction queue
 * and reactions triggered at shutdown's tag. This is assumed to 
 * be called in the main thread after all worker threads have 
 * exited (or at least finished adding all their final reactions).
 * 
 * This function assumes the caller does not hold the mutex lock.
 * Note that this function assumes that all worker threads have
 * exited, and thus does not acquire the lock when accessing the
 * reaction_q. It does however still need to acquire
 * the mutex lock when the federate wants to communicate with the RTI
 * (e.g., by calling logical_time_complete()) and when it accesses
 * the event_q (because of async calls to __schedule()).
 */
void wrapup() {
    if (current_tag.time != timeout_time) {
        // request_stop() has been called or starvation occurred.
        // Shutdown reactions need to be executed
        // at the next microstep. First, execute
        // the reactions at the current microstep.
        __execute_reactions_during_wrapup();
    
        // Acquire the mutex lock to send logical_time_complete to the
        // RTI and to access the event_q.
        pthread_mutex_lock(&mutex);
        // Notify the RTI that the current logical tag is complete.
        // This function informs the RTI of the current_tag in a
        // non-blocking manner.
        logical_time_complete(current_tag.time, current_tag.microstep);

        _lf_advance_logical_time(current_tag.time);
        // Invoke code that must execute before starting a new logical time round,
        // such as initializing outputs to be absent.
        __start_time_step();

        event_t* event = (event_t*)pqueue_peek(event_q);
        if (event != NULL) {
            if (event->time == current_tag.time) {
                // Pop events one last time to retrieve events
                // scheduled at the next microstep.
                __pop_events();
            }
        }
        // Release the lock
        pthread_mutex_unlock(&mutex);
    }

    // Invoke any code-generated wrapup. If this returns true,
    // then reactions have been scheduled at the current microstep.
    // printf("DEBUG: wrapup invoked.\n");
    if (__wrapup()) {
        // __wrapup() returns true if it has put shutdown events
        // onto the event queue.  We need to run reactions to those
        // events.
        DEBUG_PRINT("__wrapup returned true.");
        
        // Execute shutdown reactions.
        __execute_reactions_during_wrapup();
    }
    
    // Acquire the mutex before calling logical_time_complete().
    pthread_mutex_lock(&mutex);
    // Notify the RTI that the current logical tag is complete.
    // This function informs the RTI of the current_tag in a
    // non-blocking manner.
    logical_time_complete(current_tag.time, current_tag.microstep);    
    // Release the lock
    pthread_mutex_unlock(&mutex);
}

int main(int argc, char* argv[]) {
    // Invoke the function that optionally provides default command-line options.
    __set_default_command_line_options();
    
    // Initialize the one and only mutex to be recursive, meaning that it is OK
    // for the same thread to lock and unlock the mutex even if it already holds
    // the lock.
    // FIXME: This is dangerous. The docs say this: "It is advised that an
    // application should not use a PTHREAD_MUTEX_RECURSIVE mutex with
    // condition variables because the implicit unlock performed for a
    // pthread_cond_wait() or pthread_cond_timedwait() may not actually
    // release the mutex (if it had been locked multiple times).
    // If this happens, no other thread can satisfy the condition
    // of the predicate.”  This seems like a bug in the implementation of
    // pthreads. Maybe it has been fixed?
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&mutex, &attr);

    if (atexit(termination) != 0) {
        fprintf(stderr, "WARNING: Failed to register termination function!");
    }

    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {
        // printf("DEBUG: pthread_mutex_lock main\n");
        pthread_mutex_lock(&mutex);
        // printf("DEBUG: pthread_mutex_locked\n");
        initialize();
        transfer_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        // Create a queue on which to put reactions that are currently executing.
        executing_q = pqueue_init(_lf_number_of_threads, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        __trigger_startup_reactions();
        __initialize_timers();

        // At this time, reactions (startup, etc.) are added to the 
        // reaction queue that will be executed at tag (0,0).
        // Before we could do so, we need to ask the RTI if it is 
        // okay.
        tag_t grant_time = next_event_tag(start_time, 0u);
        if (grant_time.time != start_time || grant_time.microstep != 0u) {
            // This is a critical condition
            fprintf(stderr, "Federate received a grant time earlier than start time or with an incorrect starting microstep (%lld, %u).\n", grant_time.time - start_time, grant_time.microstep);
            exit(1);
        }
        
        _lf_execution_started = true;
        DEBUG_PRINT("Started execution.");

        start_threads();
        // printf("DEBUG: pthread_mutex_unlock main\n");
        pthread_mutex_unlock(&mutex);
        // printf("DEBUG: Waiting for worker threads to exit.\n");

        // Wait for the worker threads to exit.
        void* worker_thread_exit_status;
        // printf("DEBUG: number of threads: %d\n", _lf_number_of_threads);
        
        int ret = 0;
        for (int i = 0; i < _lf_number_of_threads; i++) {
            ret = MAX(pthread_join(__thread_ids[i], &worker_thread_exit_status), ret);
        }

        if (ret == 0) {
            printf("---- All worker threads exited successfully.\n");
        } else {
            printf("Unable to successfully join worker threads: %s", strerror(ret));
        }
        
        free(__thread_ids);

        wrapup();
        return 0;
    } else {
        return -1;
    }
}
