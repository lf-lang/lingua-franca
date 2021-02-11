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
#include <signal.h>

/**
 * The maximum amount of time a worker thread should stall
 * before checking the reaction queue again.
 * This is not currently used.
 */
#define MAX_STALL_INTERVAL MSEC(1)

/**
 * Unless the "fast" option is given, an LF program will wait until
 * physical time matches logical time before handling an event with
 * a given logical time. The amount of time is less than this given
 * threshold, then no wait will occur. The purpose of this is
 * to prevent unnecessary delays caused by simply setting up and
 * performing the wait.
 */
#define MIN_WAIT_TIME USEC(10)

// Number of idle worker threads.
int number_of_idle_threads = 0;

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
pqueue_t* executing_q; // Sorted by index (precedence sort)

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
 * Raise a barrier to prevent the current tag from advancing to or
 * beyond the value of the future_tag argument, if possible.
 * If the current tag is already at or beyond future_tag, then
 * prevent any further advances. This function will increment the
 * total number of pending barrier requests. For each call to this
 * function, there should always be a subsequent call to
 * _lf_decrement_global_tag_barrier() or
 * _lf_decrement_global_tag_barrier_already_locked()
 * to release the barrier.
 * 
 * If there is already a barrier raised at a tag later than future_tag, this
 * function will change the barrier to future_tag or the current tag, whichever
 * is larger. If the existing barrier is earlier 
 * than future_tag, this function will not change the barrier. If there are
 * no existing barriers and future_tag is in the past relative to the 
 * current tag, this function will raise a barrier to the current tag.
 * 
 * This function assumes the mutex lock is already held, thus, it will not
 * acquire it itself.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in a federated execution.
 * 
 * @param future_tag A desired tag for the barrier. This function will guarantee
 * that current logical time will not go past future_tag if it is in the future.
 * If future_tag is in the past (or equals to current logical time), the runtime
 * will freeze advancement of logical time.
 */
void _lf_increment_global_tag_barrier_already_locked(tag_t future_tag) {
    // Check if future_tag is after stop tag.
    // This will only occur when a federate receives a timed message with 
    // a tag that is after the stop tag
    if (_lf_is_tag_after_stop_tag(future_tag)) {
        warning_print("Attempting to raise a barrier after the stop tag.");
        future_tag = stop_tag;
    }
    tag_t current_tag = get_current_tag();
    // Check to see if future_tag is actually in the future (or present).
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
            DEBUG_PRINT("Raised barrier at elapsed tag (%lld, %u).",
                        _lf_global_tag_advancement_barrier.horizon.time - start_time,
                        _lf_global_tag_advancement_barrier.horizon.microstep);
        } 
    } else {
            // future_tag is not in the future.
            // Therefore, hold the current logical time.
            _lf_global_tag_advancement_barrier.horizon = current_tag;
            DEBUG_PRINT("Raised barrier at elapsed tag (%lld, %u).",
                        _lf_global_tag_advancement_barrier.horizon.time - start_time,
                        _lf_global_tag_advancement_barrier.horizon.microstep);
    }
    // Increment the number of requestors
    _lf_global_tag_advancement_barrier.requestors++;
}

/**
 * Raise a barrier to prevent the current tag from advancing to or
 * beyond the value of the future_tag argument, if possible.
 * If the current tag is already at or beyond future_tag, then
 * prevent any further advances. This function will increment the
 * total number of pending barrier requests. For each call to this
 * function, there should always be a subsequent call to
 * _lf_decrement_global_tag_barrier() or
 * _lf_decrement_global_tag_barrier_already_locked()
 * to release the barrier.
 * 
 * If there is already a barrier raised at a tag later than future_tag, this
 * function will change the barrier to future_tag or the current tag, whichever
 * is larger. If the existing barrier is earlier 
 * than future_tag, this function will not change the barrier. If there are
 * no existing barriers and future_tag is in the past relative to the 
 * current tag, this function will raise a barrier to the current tag.
 * 
 * This function acquires the mutex lock .
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in a federated execution.
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
 * Decrement the total number of pending barrier requests for the global tag barrier.
 * If the total number of requests reaches zero, this function resets the
 * tag barrier to (FOREVER, 0) and notifies all threads that are waiting 
 * on the barrier that the number of requests has reached zero.
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
        error_print_and_exit("Mismatched use of _lf_increment_global_tag_barrier()"
                " and  _lf_decrement_global_tag_barrier().");
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
 * is larger than a barrier set by a call to
 * _lf_increment_global_tag_barrier or
 * _lf_increment_global_tag_barrier_already_locked.
 * It will return when that barrier is lifted.
 * 
 * This function assumes the mutex is already locked.
 * Thus, it unlocks the mutex while it's waiting to allow
 * the tag barrier to change.
 * 
 * @param proposed_tag The tag that the runtime wants
 *  to advance to.
 */
void _lf_wait_on_global_tag_barrier(tag_t proposed_tag) {
    // Do not wait for tags after the stop tag
    if (_lf_is_tag_after_stop_tag(proposed_tag)) {
        proposed_tag = stop_tag;
    }
    // Do not wait forever
    if (proposed_tag.time == FOREVER) {
        warning_print("Global tag barrier should not handle FOREVER proposed tags.");
        return;
    }
    // Wait if the global barrier semaphore on logical time is zero
    // and the proposed_time is larger than the horizon.
    while (_lf_global_tag_advancement_barrier.requestors > 0
            && compare_tags(proposed_tag, _lf_global_tag_advancement_barrier.horizon) > 0
    ) {
        DEBUG_PRINT("Waiting on barrier for tag (%lld, %u).", proposed_tag.time - start_time, proposed_tag.microstep);
        // Wait until no requestor remains for the barrier on logical time
        pthread_cond_wait(&global_tag_barrier_requestors_reached_zero, &mutex);
        
        // Do not wait for tags after the stop tag
        if (_lf_is_tag_after_stop_tag(proposed_tag)) {
            proposed_tag = stop_tag;
        }
    }
}

/**
 * Schedule the specified trigger at current_tag.time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_token(void* action, interval_t extra_delay, lf_token_t* token) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    pthread_mutex_lock(&mutex);
    int return_value = __schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
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
        error_print("schedule: Invalid trigger or element size.");
        return -1;
    }
    pthread_mutex_lock(&mutex);
    // Initialize token with an array size of length and a reference count of 0.
    lf_token_t* token = __initialize_token(trigger->token, length);
    // Copy the value into the newly allocated memory.
    memcpy(token->value, value, token->element_size * length);
    // The schedule function will increment the reference count.
    handle_t result = __schedule(trigger, offset, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
    pthread_mutex_unlock(&mutex);
    return result;
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, int length) {
    trigger_t* trigger = _lf_action_to_trigger(action);

    pthread_mutex_lock(&mutex);
    lf_token_t* token = create_token(trigger->element_size);
    token->value = value;
    token->length = length;
    int return_value = __schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    pthread_cond_signal(&event_q_changed);
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
void logical_tag_complete(instant_t time, microstep_t microstep);

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
 * unless -fast is given.
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
 * @param logical_time_ns Logical time to wait until physical time matches it.
 * @param return_if_interrupted If this is false, then wait_util will wait
 *  until physical time matches the logical time regardless of whether new
 *  events get put on the event queue. This is useful, for example, for
 *  synchronizing the start of the program.
 * 
 * @return Return false if the wait is interrupted either because of an event
 *  queue signal or if the wait time was interrupted early by reaching
 *  the stop time, if one was specified. Return true if the full wait time
 *  was reached.
 */
bool wait_until(instant_t logical_time_ns) {
    DEBUG_PRINT("-------- Waiting until physical time matches logical time %lld", logical_time_ns);
    bool return_value = true;
    if (logical_time_ns > stop_tag.time) {
        DEBUG_PRINT("-------- Waiting until the stop time instead: %lld", stop_tag.time);
        // Modify the time to wait until to be the timeout time.
        logical_time_ns = stop_tag.time;
        // Indicate on return that the time of the event was not reached.
        // We still wait for time to elapse in case asynchronous events come in.
        return_value = false;
    }
    interval_t wait_until_time_ns = logical_time_ns;
#ifdef _LF_COORD_DECENTRALIZED // Only apply the STP offset if coordination is decentralized
    // Apply the STP offset to the logical time
    // Prevent an overflow
    if (wait_until_time_ns < FOREVER - _lf_global_time_STP_offset) {
        // If wait_time is not forever
        wait_until_time_ns += _lf_global_time_STP_offset;
    }
#endif
    if (!fast) {
        // Get physical time as adjusted by clock synchronization offset.
        instant_t current_physical_time = get_physical_time();
        // We want to wait until that adjusted time matches the logical time.
        interval_t ns_to_wait = wait_until_time_ns - current_physical_time;
        // We should not wait if that adjusted time is already ahead
        // of logical time.
        if (ns_to_wait < MIN_WAIT_TIME) {
            return return_value;
        }

        // We will use pthread_cond_wait, which takes as an argument the absolute
        // time to wait until. However, that will not include the offset that we
        // have calculated with clock synchronization. So we need to instead ensure
        // that the time it waits is ns_to_wait.
        // We need the current clock value as obtained using CLOCK_REALTIME because
        // that is what pthread_cond_timedwait will use.
        // The above call to setPhysicalTime() set the
        // _lf_last_reported_unadjusted_physical_time_ns to the CLOCK_REALTIME value
        // unadjusted by clock synchronization.
        // Note that if ns_to_wait is large enough, then the following addition could
        // overflow. This could happen, for example, if wait_until_time_ns == FOREVER.
        instant_t unadjusted_wait_until_time_ns = FOREVER;
        if (FOREVER - _lf_last_reported_unadjusted_physical_time_ns > ns_to_wait) {
            unadjusted_wait_until_time_ns = _lf_last_reported_unadjusted_physical_time_ns + ns_to_wait;
        }
        DEBUG_PRINT("-------- Clock offset is %lld ns.", current_physical_time - _lf_last_reported_unadjusted_physical_time_ns);

        // Convert the absolute time to a timespec.
        // timespec is seconds and nanoseconds.
        struct timespec unadjusted_wait_until_time
                = {(time_t)unadjusted_wait_until_time_ns / BILLION, (long)unadjusted_wait_until_time_ns % BILLION};

        DEBUG_PRINT("-------- Waiting %lld ns for physical time to match logical time %llu.", ns_to_wait, logical_time_ns);
        DEBUG_PRINT("-------- which is %splus %ld nanoseconds.", ctime(&unadjusted_wait_until_time.tv_sec), unadjusted_wait_until_time.tv_nsec);

        // pthread_cond_timedwait returns 0 if it is awakened before the timeout.
        // Hence, we want to run it repeatedly until either it returns non-zero or the
        // current physical time matches or exceeds the logical time.
        if (pthread_cond_timedwait(&event_q_changed, &mutex, &unadjusted_wait_until_time) != ETIMEDOUT) {
            DEBUG_PRINT("-------- Wait interrupted.");

            // Wait did not time out, which means that there
            // may have been an asynchronous call to schedule().
            // Continue waiting.
            // Do not adjust current_tag.time here. If there was an asynchronous
            // call to schedule(), it will have put an event on the event queue,
            // and current_tag.time will be set to that time when that event is pulled.
            return_value = false;
        } else {
            // Reached timeout.
            // Unfortunately, at least on Macs, pthread_cond_timedwait appears
            // to be implemented incorrectly and it returns well short of the target
            // time.  Check for this condition and wait again if necessary.
            interval_t ns_to_wait = wait_until_time_ns - get_physical_time();
            // We should not wait if that adjusted time is already ahead
            // of logical time.
            if (ns_to_wait < MIN_WAIT_TIME) {
                return true;
            }
            DEBUG_PRINT("-------- pthread_cond_timedwait claims to have timed out, "
                    "but it did not reach the target time. Waiting again.");
            return wait_until(wait_until_time_ns);
        }

        DEBUG_PRINT("-------- Returned from wait, having waited %lld ns.", get_physical_time() - current_physical_time);
    }
    return return_value;
}

// Indicator used to identify the first invocation of __next().
bool __first_invocation = true;

/**
 * Return the tag of the next event on the event queue.
 * If the event queue is empty then return (FOREVER, UINT_MAX).
 * If the next event tag is greater than the stop tag, then
 * return the stop tag.
 */
tag_t get_next_event_tag() {
    // Peek at the earliest event in the event queue.
    event_t* event = (event_t*)pqueue_peek(event_q);
    tag_t next_tag = { .time = FOREVER, .microstep = UINT_MAX };
    if (event != NULL) {
        // There is an event in the event queue.
        if (event->time < current_tag.time) {
            error_print_and_exit("get_next_event_tag(): Earliest event on the event queue (%lld) is "
                                  "earlier than the current time (%lld).",
                                  event->time - start_time,
                                  current_tag.time - start_time);
        }

        next_tag.time = event->time;
        if (next_tag.time == current_tag.time) {
            next_tag.microstep =  get_microstep() + 1;
        } else {
            next_tag.microstep = 0;
        }
    }

    // If a timeout tag was given, adjust the next_tag from the
    // event tag to that timeout tag.
    // FIXME: Why is this needed? 
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        next_tag = stop_tag;
    }
    LOG_PRINT("Next event tag is (%lld, %d). Event queue has size %d.",
            next_tag.time - start_time, next_tag.microstep, pqueue_size(event_q));
    return next_tag;
}

/**
 * If there is at least one event in the event queue, then wait until
 * physical time matches or exceeds the time of the least tag on the event
 * queue; pop the next event(s) from the event queue that all have the same tag;
 * extract from those events the reactions that are to be invoked at this
 * logical time and insert them into the reaction queue. The event queue is
 * sorted by time tag.
 *
 * If there is no event in the queue and the keepalive command-line option was
 * not given, set the stop tag to the current tag.
 * If keepalive was given, then wait for either request_stop()
 * to be called or an event appears in the event queue and then return.
 *
 * Every time tag is advanced, it is checked against stop tag and if they are
 * equal, shutdown reactions are triggered.
 *
 * This does not acquire the mutex lock. It assumes the lock is already held.
 */
void __next() {
    // Previous logical time is complete.
    tag_t next_tag = get_next_event_tag();
    if (next_tag.time == FOREVER && !keepalive_specified) {
        // No event in the queue
        // keepalive is not set so we should stop.
        // Note that federated programs always have
        // keepalive = true         
        _lf_set_stop_tag((tag_t){.time=current_tag.time,.microstep=current_tag.microstep+1});
    }

    // If a timeout tag was given, adjust the next_tag from the
    // event tag to that timeout tag.
    // FIXME: This is primarily done to let the RTI
    // know about the timeout time? 
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        next_tag = stop_tag;
    }

#ifdef _LF_COORD_CENTRALIZED
    // In case this is in a federation with centralized coordination, notify 
    // the RTI of the next earliest tag at which this federate might produce 
    // an event. This function may block until it is safe to advance the current 
    // tag to the next tag. Specifically, it blocks if there are upstream 
    // federates. If an action triggers during that wait, it will unblock
    // and return with a time (typically) less than the next_time.
    tag_t grant_tag = next_event_tag(next_tag.time, next_tag.microstep);
    if (compare_tags(grant_tag, next_tag) < 0) {
        // RTI has granted tag advance to an earlier tag or the wait
        // for the RTI response was interrupted by a local physical action with
        // a tag earlier than requested. FIXME: Isn't this a race condition?
        // What if RTI has granted a TAG and is in-transit when the physical action
        // arrives?
        // Continue executing. The event queue may have changed.
        return;
    }

    // Since next_event_tag releases the mutex lock internally, we need to check
    // next_tag again in case the stop_tag has changed while the mutex was unlocked.
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        next_tag = stop_tag;
    }
#endif // _LF_COORD_CENTRALIZED

    // Peek at the earliest event in the event queue.
    event_t* event = (event_t*)pqueue_peek(event_q);

    // Wait for physical time to advance to the next event time (or stop time).
    // This can be interrupted if a physical action triggers (e.g., a message
    // arrives from an upstream federate or a local physical action triggers).
    LOG_PRINT("Waiting until elapsed time %lld.", (next_tag.time - start_time));
    if (!wait_until(next_tag.time)) {
        DEBUG_PRINT("__next(): Wait until time interrupted.");
        // Sleep was interrupted.

        // Since wait_until releases the mutex lock internally, we need to check
        // again for any changes in stop_tag while the mutex was unlocked.
        if (_lf_is_tag_after_stop_tag(next_tag)) {
            // Interrupt could have been due to a request_stop()
            // Return to the worker loop to check
            return;
        }

        // The cause of interruption is not a call to request_stop().
        // May have been an asynchronous call to schedule().
        // Before returning from this __next(), check if the new event
        // that has caused wait_until() to be interrupted is earlier than the
        // current event.
        event_t* next_event = (event_t*)pqueue_peek(event_q);
        next_tag.time = next_event->time;
        if (next_event != event) {
            // Get the current physical time.
            instant_t current_physical_time_ns = get_physical_time();
            // Set current time to match physical time, but not less than
            // current logical time nor more than next time in the event queue.
            if (current_physical_time_ns > current_tag.time) {
                if (current_physical_time_ns < next_tag.time) {
                    // In federated programs, barriers could be raised on a specific tag.
                    // If next_tag is larger than the barrier, next() should wait until the barrier
                    // is removed before advancing the tag
#ifdef _LF_IS_FEDERATED // Only wait on the tag barrier in federated LF programs
                    // Wait until the global barrier on tag (horizon) is larger than the next_tag.
                    // This can effectively add to the STP offset in certain cases, for example,
                    // when a message with timestamp (0,0) has arrived at (0,0).
                    _lf_wait_on_global_tag_barrier((tag_t){
                                                    .time = current_physical_time_ns,
                                                    .microstep = 0});
#endif // _LF_IS_FEDERATED
                    // Advance tag.
                    _lf_advance_logical_time(current_physical_time_ns);
                    return;
                }
            } else {
                // Current physical time does not exceed current logical
                // time, so do not advance tag.
                return;
            }
        } else {
            warning_print("wait_until() was interrupted but neither the event queue nor the stop tag have changed.");
            // This could cause the program to execute instructions in the worker thread for virtually no reason
            return;
        }
    }
    
    DEBUG_PRINT("Physical time is ahead of next tag time by %lld. This should be small.",
                get_physical_time() - next_tag.time);
    
    // If the event queue has changed, return to iterate.
    if ((event_t*)pqueue_peek(event_q) != event) {
        return;
    }

    // Since wait_until releases the mutex lock internally, we need to check
    // again for any changes in stop_tag while the mutex was unlocked.
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        next_tag = stop_tag;
    }
    
    // In federated programs, barriers could be raised on a specific tag.
    // If next_tag is larger than the barrier, next() should wait until the barrier
    // is removed before advancing the tag
#ifdef _LF_IS_FEDERATED // Only wait on the tag barrier in federated LF programs
    // A barrier tag may have been set either by a stop request or by an incoming
    // message with a logical tag.
    // Wait until the global barrier on tag (horizon) is larger than the next_tag.
    // This can effectively add to the STP offset in certain cases, for example,
    // when a message with timestamp (0,0) has arrived at (0,0).

    // FIXME: If the event queue is empty, next_tag.time == FOREVER, resulting
    // in a warning being printed.
    _lf_wait_on_global_tag_barrier(next_tag);
#endif

    // Since _lf_wait_on_global_tag_barrier releases the mutex lock internally, we need to check
    // again for any changes in stop_tag while the mutex was unlocked.
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        next_tag = stop_tag;
    }

    // If the event queue has changed, return to iterate.
    if ((event_t*)pqueue_peek(event_q) != event) {
        return;
    }

    // At this point, finally, we have an event to process.
    // Advance current time to match that of the first event on the queue.
    _lf_advance_logical_time(next_tag.time);

    if (compare_tags(current_tag, stop_tag) >= 0) {
        // Pop shutdown events
        DEBUG_PRINT("Scheduling shutdown reactions.");
        __trigger_shutdown_reactions();
    }

    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    __start_time_step();
    // Pop all events from event_q with timestamp equal to current_tag.time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    __pop_events();
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
    pthread_mutex_lock(&mutex);
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
    // In a non-federated program, the stop_tag will be the next microstep
    _lf_set_stop_tag((tag_t) {.time = current_tag.time, .microstep = current_tag.microstep+1});
    // In case any thread is waiting on a condition, notify all.
    pthread_cond_broadcast(&reaction_q_changed);
    // We signal instead of broadcast under the assumption that only
    // one worker thread can call wait_until at a given time because
    // the call to wait_until is protected by a mutex lock
    pthread_cond_signal(&event_q_changed);
#endif
    pthread_mutex_unlock(&mutex);
}

/**
 * If the reaction is blocked by a currently executing
 * reaction, return true. Otherwise, return false.
 * A reaction blocks the specified reaction if it has a
 * level less than that of the specified reaction and it also has
 * an overlapping chain ID, meaning that it is (possibly) upstream
 * of the specified reaction.
 * This function assumes the mutex is held because it accesses
 * the executing_q.
 * @param reaction The reaction.
 * @return true if this reaction is blocked, false otherwise.
 */
bool _lf_is_blocked_by_executing_reaction(reaction_t* reaction) {
    if (reaction == NULL) {
        return false;
    }
    for (size_t i = 1; i < executing_q->size; i++) {
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
 * This function assumes the mutex is held.
 *
 * @return the first-ranked reaction that is ready to execute, NULL if there is
 * none.
 */ 
reaction_t* first_ready_reaction() {    
    reaction_t* r;
    reaction_t* b;

    // Find a reaction that is ready to execute.
    while ((r = (reaction_t*)pqueue_pop(reaction_q)) != NULL) {
        if (_lf_is_blocked_by_executing_reaction(r)) {
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

/** 
 * Indicator that a worker thread has already taken charge of
 * advancing time. When another worker thread encouters a true
 * value to this variable, it should wait for events to appear
 * on the reaction queue rather than advance time.
 * This variable should only be accessed while holding the mutex lock.
 */
bool __advancing_time = false;

/**
 * Put the specified reaction on the reaction queue.
 * This version acquires a mutex lock.
 * @param reaction The reaction.
 */
void _lf_enqueue_reaction(reaction_t* reaction) {
    // Acquire the mutex lock.
    pthread_mutex_lock(&mutex);
    // Do not enqueue this reaction twice.
    if (pqueue_find_equal_same_priority(reaction_q, reaction) == NULL) {
        DEBUG_PRINT("Enqueing downstream reaction %p.", reaction);
        pqueue_insert(reaction_q, reaction);
        // NOTE: We could notify another thread so it can execute this reaction.
        // However, this notification is expensive!
        // It is now handled by schedule_output_reactions() in reactor_common,
        // which calls the _lf_notify_workers() function defined below.
        // pthread_cond_signal(&reaction_q_changed);
    }
    pthread_mutex_unlock(&mutex);
}

/**
 * Notify workers that something has changed on the reaction_q.
 * Notification is performed only if there is a reaction on the
 * reaction queue that is ready to execute and there is an idle
 * worker thread.
 * This function assumes the caller holds the mutex lock.
 */
void _lf_notify_workers_locked() {
    if (number_of_idle_threads > 0) {
        reaction_t* next_ready_reaction = (reaction_t*)pqueue_peek(reaction_q);
        if (next_ready_reaction != NULL
                && !_lf_is_blocked_by_executing_reaction(next_ready_reaction)
        ) {
            // FIXME: In applications without parallelism, this notification
            // proves very expensive. Perhaps we should be checking execution times.
            pthread_cond_signal(&reaction_q_changed);
            DEBUG_PRINT("Notify another worker of a reaction on the reaction queue.");
        }
    }
}

/**
 * Notify workers that something has changed on the reaction_q.
 * Notification is performed only if there is a reaction on the
 * reaction queue that is ready to execute and there is an idle
 * worker thread.
 * This function acquires the mutex lock.
 */
void _lf_notify_workers() {
    pthread_mutex_lock(&mutex);
    _lf_notify_workers_locked();
    pthread_mutex_unlock(&mutex);
}

/** For logging and debugging, each worker thread is numbered. */
int worker_thread_count = 0;

/**
 * Worker thread for the thread pool.
 * This acquires the mutex lock and releases it to wait for time to
 * elapse or for asynchronous events and also releases it to execute reactions.
 */
void* worker(void* arg) {
    // Keep track of whether we have decremented the idle thread count.
    bool have_been_busy = false;
    pthread_mutex_lock(&mutex);

    int worker_number = ++worker_thread_count;
    LOG_PRINT("Worker thread %d started.", worker_number);

    // Iterate until the stop_tag is reached or reaction queue is empty
    while (true) {
        // Obtain a reaction from the reaction_q that is ready to execute
        // (i.e., it is not blocked by concurrently executing reactions
        // that it depends on).
        reaction_t* current_reaction_to_execute = first_ready_reaction();
        if (current_reaction_to_execute == NULL) {
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
                    && !__advancing_time) {
                logical_tag_complete(current_tag.time, current_tag.microstep);
                
                // If we are at the stop tag, do not call __next()
                // to prevent advancing the logical time.
                if (compare_tags(current_tag, stop_tag) >= 0) {
                    // Break out of the while loop and notify other
                    // worker threads potentially waiting to continue.
                    pthread_cond_broadcast(&reaction_q_changed);
                    pthread_cond_signal(&event_q_changed);
                    break;
                }
                
                // __next() may block waiting for events
                // to appear on the event queue. Note that we already
                // hold the mutex lock.
                __advancing_time = true;
                tracepoint_worker_advancing_time_starts(worker_number);
                __next();
                tracepoint_worker_advancing_time_ends(worker_number);
                __advancing_time = false;
                DEBUG_PRINT("Worker %d: Done waiting for __next().", worker_number);
            } else {
                // Logical time is not complete, and nothing on the reaction queue
                // is ready to run.
                // Wait for something to change (either a stop request or
                // something went on the reaction queue.
                DEBUG_PRINT("Worker %d: Waiting for items on the reaction queue.", worker_number);
                tracepoint_worker_wait_starts(worker_number);
                // NOTE: Could use a timedwait here to ensure that the worker thread
                // wakes up periodically. But this appears to be unnecessary. When there
                // is a ready reaction on the reaction queue, there will be a notification
                // and notification occurs while holding the mutex lock so it should not be missed.
                // Nevertheless, we keep the commented out code for a timedwait in case we later
                // want to put this back in:
                //
                // struct timespec physical_time;
                // clock_gettime(CLOCK_REALTIME, &physical_time);
                // physical_time.tv_nsec += MAX_STALL_INTERVAL;
                // pthread_cond_timedwait(&reaction_q_changed, &mutex, &physical_time);
                pthread_cond_wait(&reaction_q_changed, &mutex);
                tracepoint_worker_wait_ends(worker_number);
                DEBUG_PRINT("Worker %d: Done waiting.", worker_number);
            }
        } else {
            // Got a reaction that is ready to run.
            DEBUG_PRINT("Worker %d: Popped from reaction_q reaction with index: "
                    "%lld and deadline %lld.", worker_number,
                    current_reaction_to_execute->index,
                    current_reaction_to_execute->deadline);

            // This thread will no longer be idle.
            if (!have_been_busy) {
                number_of_idle_threads--;
                have_been_busy = true;
            }

            // Push the reaction on the executing queue in order to prevent any
            // reactions that may depend on it from executing before this reaction is finished.
            pqueue_insert(executing_q, current_reaction_to_execute);

            // If there are additional reactions on the reaction_q, notify one other
            // idle thread, if there is one, so that it can attempt to execute
            // that reaction.
            _lf_notify_workers_locked();

            // Unlock the mutex to run the reaction.
            pthread_mutex_unlock(&mutex);
            DEBUG_PRINT("Worker %d: Running a reaction (or its fault variants).", worker_number);

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
            if (current_reaction_to_execute->is_tardy == true) {
                reaction_function_t handler = current_reaction_to_execute->tardy_handler;
                LOG_PRINT("Worker %d: Invoking tardiness handler.", worker_number);
                // Invoke the tardy handler if there is one.
                if (handler != NULL) {
                    // There is a violation
                    violation = true;
                    (*handler)(current_reaction_to_execute->self);
                    
                    // If the reaction produced outputs, put the resulting
                    // triggered reactions into the queue or execute them directly if possible.
                    schedule_output_reactions(current_reaction_to_execute, worker_number);
                    
                    // Reset the is_tardy because it has been dealt with
                    current_reaction_to_execute->is_tardy = false;
                }
            }
            // If the reaction has a deadline, compare to current physical time
            // and invoke the deadline violation reaction instead of the reaction function
            // if a violation has occurred. Note that the violation reaction will be invoked
            // at most once per logical time value. If the violation reaction triggers the
            // same reaction at the current time value, even if at a future superdense time,
            // then the reaction will be invoked and the violation reaction will not be invoked again.
            if (current_reaction_to_execute->deadline > 0LL) {
                // Get the current physical time.
                instant_t physical_time = get_physical_time();
                // Check for deadline violation.
                if (physical_time > current_tag.time + current_reaction_to_execute->deadline) {
                    // Deadline violation has occurred.
                    violation = true;
                    // Invoke the local handler, if there is one.
                    reaction_function_t handler = current_reaction_to_execute->deadline_violation_handler;
                    if (handler != NULL) {
                        LOG_PRINT("Worker %d: Deadline violation. Invoking deadline handler.",
                                worker_number);
                        (*handler)(current_reaction_to_execute->self);

                        // If the reaction produced outputs, put the resulting
                        // triggered reactions into the queue or execute them directly if possible.
                        schedule_output_reactions(current_reaction_to_execute, worker_number);
                        // Remove the reaction from the executing queue.
                    }
                }
            }
            if (violation) {
                // Need to acquire the mutex lock to remove this from the executing queue
                // and to obtain the next reaction to execute.
                pthread_mutex_lock(&mutex);

                // The reaction is not going to be executed. However,
                // this thread holds the mutex lock, so if this is the last
                // reaction of the current time step, this thread will also
                // be the one to advance time.
                pqueue_remove(executing_q, current_reaction_to_execute);
            } else {
                // Invoke the reaction function.
                LOG_PRINT("Worker %d: Invoking reaction at elapsed tag (%lld, %d).",
                        worker_number,
                        current_tag.time - start_time,
                        current_tag.microstep);
                tracepoint_reaction_starts(current_reaction_to_execute, worker_number);
                current_reaction_to_execute->function(current_reaction_to_execute->self);
                tracepoint_reaction_ends(current_reaction_to_execute, worker_number);
                // If the reaction produced outputs, put the resulting triggered
                // reactions into the queue or execute them immediately.
                schedule_output_reactions(current_reaction_to_execute, worker_number);

                // Reacquire the mutex lock.
                pthread_mutex_lock(&mutex);

                // Remove the reaction from the executing queue.
                // This thread holds the mutex lock, so if this is the last
                // reaction of the current time step, this thread will also
                // be the one to advance time.
                pqueue_remove(executing_q, current_reaction_to_execute);
            }
            // Reset the is_tardy because it has been passed
            // down the chain
            current_reaction_to_execute->is_tardy = false;

            DEBUG_PRINT("Worker %d: Done invoking reaction.", worker_number);
        }
    } // while (!stop_requested || pqueue_size(reaction_q) > 0)
    // This thread is exiting, so don't count it anymore.
    _lf_number_of_threads--;

    DEBUG_PRINT("Worker %d: Stop requested. Exiting.", worker_number);
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
    LOG_PRINT("Starting %d worker threads.", _lf_number_of_threads);
    __thread_ids = (pthread_t*)malloc(_lf_number_of_threads * sizeof(pthread_t));
    number_of_idle_threads = _lf_number_of_threads;
    for (unsigned int i = 0; i < _lf_number_of_threads; i++) {
        pthread_create(&__thread_ids[i], NULL, worker, NULL);
    }
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
        warning_print("Failed to register termination function!");
    }
    // The above handles only "normal" termination (via a call to exit).
    // As a consequence, we need to also trap ctrl-C, which issues a SIGINT,
    // and cause it to call exit.
    signal(SIGINT, exit);

    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {
        pthread_mutex_lock(&mutex);
        initialize();
        transfer_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        // Create a queue on which to put reactions that are currently executing.
        executing_q = pqueue_init(_lf_number_of_threads, in_reverse_order, get_reaction_index,
            get_reaction_position, set_reaction_position, reaction_matches, print_reaction);

        __trigger_startup_reactions();
        __initialize_timers();

        // If the stop_tag is (0,0), also insert the shutdown
        // reactions. This can only happen if the timeout time
        // was set to 0.
        if (compare_tags(current_tag, stop_tag) >= 0) {
            __trigger_shutdown_reactions();
        }

        // At this time, reactions (startup, etc.) are added to the 
        // reaction queue that will be executed at tag (0,0).
        // Before we could do so, we need to ask the RTI if it is 
        // okay.
        tag_t grant_time = next_event_tag(start_time, 0u);
        if (grant_time.time != start_time || grant_time.microstep != 0u) {
            // This is a critical condition
            error_print_and_exit("Federate received a grant time earlier than start time "
                    "or with an incorrect starting microstep (%lld, %u).",
                    grant_time.time - start_time, grant_time.microstep);
        }
        
        _lf_execution_started = true;

        start_threads();
        pthread_mutex_unlock(&mutex);
        DEBUG_PRINT("Waiting for worker threads to exit.");

        // Wait for the worker threads to exit.
        void* worker_thread_exit_status;
        DEBUG_PRINT("Number of threads: %d.", _lf_number_of_threads);
        
        int ret = 0;
        for (int i = 0; i < _lf_number_of_threads; i++) {
            ret = MAX(pthread_join(__thread_ids[i], &worker_thread_exit_status), ret);
        }

        if (ret == 0) {
            LOG_PRINT("---- All worker threads exited successfully.");
        } else {
            error_print("Unable to successfully join worker threads: %s", strerror(ret));
        }
        
        free(__thread_ids);
        return 0;
    } else {
        return -1;
    }
}
