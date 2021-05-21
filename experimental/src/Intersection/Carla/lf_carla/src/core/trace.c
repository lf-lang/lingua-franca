/**
 * @file
 * @author Edward A. Lee
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley and TU Dresden

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

 * @section DESCRIPTION
 * Include this file instead of trace.h to get tracing.
 * See trace.h file for instructions.
 */

#include "util.h"
#include "platform.h"

/** Macro to use when access to trace file fails. */
#define _LF_TRACE_FAILURE(trace_file) \
    do { \
        fprintf(stderr, "WARNING: Access to trace file failed.\n"); \
        fclose(trace_file); \
        trace_file = NULL; \
        lf_mutex_unlock(&_lf_trace_mutex); \
        return -1; \
    } while(0)

// Mutex used to provent collisions between threads writing to the file.
lf_mutex_t _lf_trace_mutex;
// Condition variable used to indicate when flushing a buffer is finished.
lf_cond_t _lf_flush_finished;
// Condition variable used to indicate when a new trace needs to be flushed.
lf_cond_t _lf_flush_needed;
// The thread that flushes to a file.
lf_thread_t _lf_flush_trace_thread;

/**
 * Array of buffers into which traces are written.
 * When each buffer gets full, the trace is flushed to the trace file.
 * We use a double buffering strategy. When a buffer becomes full,
 * tracing continues in a new buffer while a separate thread writes
 * the old buffer to the file.
 */
trace_record_t** _lf_trace_buffer = NULL;
int* _lf_trace_buffer_size = NULL;
trace_record_t** _lf_trace_buffer_to_flush = NULL;
int* _lf_trace_buffer_size_to_flush = NULL;

/** The number of trace buffers allocated when tracing starts. */
int _lf_number_of_trace_buffers;

/** Marker that tracing is stopping or has stopped. */
int _lf_trace_stop = 1;

/** The file into which traces are written. */
FILE* _lf_trace_file;

/**
 * Table of pointers to a description of the object.
 */
object_description_t _lf_trace_object_descriptions[TRACE_OBJECT_TABLE_SIZE];
int _lf_trace_object_descriptions_size = 0;

/**
 * Register a trace event.
 * @param pointer1 Pointer that identifies the object, typically to a reactor self struct.
 * @param pointer2 Further identifying pointer, typically to a trigger (action or timer) or NULL if irrelevant.
 * @param type The type of trace object.
 * @param description The human-readable description of the object.
 * @return 1 if successful, 0 if the trace object table is full.
 */
int _lf_register_trace_event(void* pointer1, void* pointer2, _lf_trace_object_t type, char* description) {
    lf_mutex_lock(&_lf_trace_mutex);
    if (_lf_trace_object_descriptions_size >= TRACE_OBJECT_TABLE_SIZE) {
        return 0;
    }
    _lf_trace_object_descriptions[_lf_trace_object_descriptions_size].pointer = pointer1;
    _lf_trace_object_descriptions[_lf_trace_object_descriptions_size].trigger = pointer2;
    _lf_trace_object_descriptions[_lf_trace_object_descriptions_size].type = type;
    _lf_trace_object_descriptions[_lf_trace_object_descriptions_size].description = description;
    _lf_trace_object_descriptions_size++;
    lf_mutex_unlock(&_lf_trace_mutex);
    return 1;
}

/**
 * Register a user trace event. This should be called once, providing a pointer to a string
 * that describes a phenomenon being traced. Use the same pointer as the first argument to
 * tracepoint_user_event() and tracepoint_user_value().
 * @param description Pointer to a human-readable description of the event.
 * @return 1 if successful, 0 if the trace object table is full.
 */
int register_user_trace_event(char* description) {
    return _lf_register_trace_event(description, NULL, trace_user, description);
}

/** Indicator that the trace header information has been written to the file. */
bool _lf_trace_header_written = false;

/**
 * Write the trace header information.
 * See trace.h.
 * @return The number of items written to the object table or -1 for failure.
 */
int write_trace_header() {
    if (_lf_trace_file != NULL) {
        lf_mutex_lock(&_lf_trace_mutex);
        // The first item in the header is the start time.
        // This is both the starting physical time and the starting logical time.
        instant_t start_time = get_start_time();
        // printf("DEBUG: Start time written to trace file is %lld.\n", start_time);
        size_t items_written = fwrite(
                &start_time,
                sizeof(instant_t),
                1,
                _lf_trace_file
        );
        if (items_written != 1) _LF_TRACE_FAILURE(_lf_trace_file);

        // The next item in the header is the size of the
        // _lf_trace_object_descriptions table.
        // printf("DEBUG: Table size written to trace file is %d.\n", _lf_trace_object_descriptions_size);
        items_written = fwrite(
                &_lf_trace_object_descriptions_size,
                sizeof(int),
                1,
                _lf_trace_file
        );
        if (items_written != 1) _LF_TRACE_FAILURE(_lf_trace_file);

        // Next we write the table.
        for (int i = 0; i < _lf_trace_object_descriptions_size; i++) {
            // printf("DEBUG: Object pointer: %p.\n", _lf_trace_object_descriptions[i].pointer);
            // Write the pointer to the self struct.
            items_written = fwrite(
                        &_lf_trace_object_descriptions[i].pointer,
                        sizeof(void*),
                        1,
                        _lf_trace_file
            );
            if (items_written != 1) _LF_TRACE_FAILURE(_lf_trace_file);

            // Write the pointer to the trigger_t struct.
            items_written = fwrite(
                        &_lf_trace_object_descriptions[i].trigger,
                        sizeof(trigger_t*),
                        1,
                        _lf_trace_file
            );
            if (items_written != 1) _LF_TRACE_FAILURE(_lf_trace_file);

            // Write the object type.
            items_written = fwrite(
                        &_lf_trace_object_descriptions[i].type, // Write the pointer value.
                        sizeof(_lf_trace_object_t),
                        1,
                        _lf_trace_file
            );
            if (items_written != 1) _LF_TRACE_FAILURE(_lf_trace_file);

            // Write the description.
            int description_size = strlen(_lf_trace_object_descriptions[i].description);
            // printf("DEBUG: Object description: %s.\n", _lf_trace_object_descriptions[i].description);
            items_written = fwrite(
                        _lf_trace_object_descriptions[i].description,
                        sizeof(char),
                        description_size + 1, // Include null terminator.
                        _lf_trace_file
            );
            if (items_written != description_size + 1) _LF_TRACE_FAILURE(_lf_trace_file);
        }
        lf_mutex_unlock(&_lf_trace_mutex);
    }
    return _lf_trace_object_descriptions_size;
}

/**
 * Thread that actually flushes the buffers to a file.
 */
void* flush_trace(void* args) {
    lf_mutex_lock(&_lf_trace_mutex);
    while (_lf_trace_file != NULL) {
        // Look for a buffer to flush.
        int worker = -1;
        for (int i = 0; i < _lf_number_of_trace_buffers; i++) {
            if (_lf_trace_buffer_size_to_flush && _lf_trace_buffer_size_to_flush[i] > 0) {
                // Found one.
                worker = i;
                break;
            }
        }
        if (worker < 0) {
            // There is no trace ready to flush.
            if (_lf_trace_stop != 0) {
                // Tracing has stopped.
                lf_mutex_unlock(&_lf_trace_mutex);
                return NULL;
            }
            // Wait for notification that there is a buffer ready to flush
            // (or that tracing is being stopped).
            lf_cond_wait(&_lf_flush_needed, &_lf_trace_mutex);
            continue;
        }
        // Unlock the mutex to write to the file.
        lf_mutex_unlock(&_lf_trace_mutex);

        // If the trace header has not been written, write it now.
        // This is deferred to here so that user trace objects can be
        // registered in startup reactions.
        if (!_lf_trace_header_written) {
            write_trace_header();
            _lf_trace_header_written = true;
        }

        // Write first the length of the array.
        size_t items_written = fwrite(
                &_lf_trace_buffer_size_to_flush[worker],
                sizeof(int),
                1,
                _lf_trace_file
        );
        if (items_written != 1) {
            fprintf(stderr, "WARNING: Access to trace file failed.\n");
            fclose(_lf_trace_file);
            _lf_trace_file = NULL;
        } else {
            // Write the contents.
            items_written = fwrite(
                    _lf_trace_buffer_to_flush[worker],
                    sizeof(trace_record_t),
                    _lf_trace_buffer_size_to_flush[worker],
                    _lf_trace_file
            );
            if (items_written != _lf_trace_buffer_size_to_flush[worker]) {
                fprintf(stderr, "WARNING: Access to trace file failed.\n");
                fclose(_lf_trace_file);
                _lf_trace_file = NULL;
            }
        }
        // Acquire the mutex to update the size.
        lf_mutex_lock(&_lf_trace_mutex);
        _lf_trace_buffer_size_to_flush[worker] = 0;
        // There may be more than one worker thread blocked waiting for a flush,
        // so broadcast rather than just signal.
        lf_cond_broadcast(&_lf_flush_finished);
    }
    return NULL;
}

/**
 * Flush trace so far to the trace file. This version assumes the mutex
 * is already held.
 * @param worker The worker thread or 0 for the main (or only) thread.
 */
void flush_trace_to_file_locked(int worker) {
    if (_lf_trace_file != NULL) {
        // printf("DEBUG: Writing %d trace records.\n", _lf_trace_buffer_size[worker]);

        // If the previous flush for this worker is not finished, wait for it to finish.
        while (_lf_trace_buffer_size_to_flush[worker] != 0) {
            lf_cond_wait(&_lf_flush_finished, &_lf_trace_mutex);
        }
        _lf_trace_buffer_size_to_flush[worker] = _lf_trace_buffer_size[worker];

        // Swap the double buffer.
        trace_record_t* tmp = _lf_trace_buffer[worker];
        _lf_trace_buffer[worker] = _lf_trace_buffer_to_flush[worker];
        _lf_trace_buffer_to_flush[worker] = tmp;

        _lf_trace_buffer_size[worker] = 0;
        lf_cond_signal(&_lf_flush_needed);
    }
}

/**
 * Flush trace so far to the trace file. We use double buffering, so unless
 * the flush thread is busy flushing the previous buffer for this worker, this
 * returns immediately.
 * @param worker The worker thread or 0 for the main (or only) thread.
 */
void flush_trace_to_file(int worker) {
    if (_lf_trace_file != NULL) {
        // printf("DEBUG: Writing %d trace records.\n", _lf_trace_buffer_size[worker]);
        lf_mutex_lock(&_lf_trace_mutex);
        flush_trace_to_file_locked(worker);
        lf_mutex_unlock(&_lf_trace_mutex);
    }
}

/**
 * Open a trace file and start tracing.
 * @param filename The filename for the trace file.
 */
void start_trace(char* filename) {
    lf_mutex_init(&_lf_trace_mutex);
    lf_cond_init(&_lf_flush_finished);
    lf_cond_init(&_lf_flush_needed);
    // FIXME: location of trace file should be customizable.
    _lf_trace_file = fopen(filename, "w");
    if (_lf_trace_file == NULL) {
        fprintf(stderr, "WARNING: Failed to open log file with error code %d."
                "No log will be written.\n", errno);
    }
    // Do not write the trace header information to the file yet
    // so that startup reactions can register user-defined trace objects.
    // write_trace_header();
    _lf_trace_header_written = false;

    // Allocate an array of arrays of trace records, one per worker thread plus one
    // for the 0 thread (the main thread, or in an unthreaded program, the only thread).
    _lf_number_of_trace_buffers = _lf_number_of_threads + 1;
    _lf_trace_buffer = (trace_record_t**)malloc(sizeof(trace_record_t*) * _lf_number_of_trace_buffers);
    for (int i = 0; i < _lf_number_of_trace_buffers; i++) {
        _lf_trace_buffer[i] = (trace_record_t*)malloc(sizeof(trace_record_t) * TRACE_BUFFER_CAPACITY);
    }
    // Array of counters that track the size of each trace record (per thread).
    _lf_trace_buffer_size = (int*)calloc(sizeof(int), _lf_number_of_trace_buffers);

    // Allocate memory for double buffering.
    _lf_trace_buffer_to_flush = (trace_record_t**)malloc(sizeof(trace_record_t*) * _lf_number_of_trace_buffers);
    for (int i = 0; i < _lf_number_of_trace_buffers; i++) {
        _lf_trace_buffer_to_flush[i] = (trace_record_t*)malloc(sizeof(trace_record_t) * TRACE_BUFFER_CAPACITY);
    }
    // Array of counters that track the size of each trace record (per thread).
    _lf_trace_buffer_size_to_flush = (int*)calloc(sizeof(int), _lf_number_of_trace_buffers);

    _lf_trace_stop = 0;

    // In case the user forgets to stop to the trace in wrapup.
    if (atexit(stop_trace) != 0) {
        warning_print("Failed to register stop_trace function for execution upon termination.");
    }

    lf_thread_create(&_lf_flush_trace_thread, flush_trace, NULL);

    DEBUG_PRINT("Started tracing.");
}

/**
 * Trace an event identified by a type and an identifying pointer.
 * The pointer can be, for example, to the self struct of the reactor instance.
 * This is a generic tracepoint function. It is better to use one of the specific functions.
 * @param event_type The type of event (see trace_event_t in trace.h)
 * @param pointer The identifying pointer.
 * @param reaction_number The number of the reaction or -1 if the trace is not of a reaction
 *  or the reaction number if not known.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution
 *  or -1 for an unknown thread.
 * @param physical_time If the caller has already accessed physical time, provide it here.
 *  Otherwise, provide NULL. This argument avoids a second call to get_physical_time
 *  and ensures that the physical time in the trace is the same as that used by the caller.
 * @param trigger Pointer to the trigger_t struct for calls to schedule or NULL otherwise.
 * @param extra_delay The extra delay passed to schedule(). If not relevant for this event
 *  type, pass 0.
 */
void tracepoint(
        trace_event_t event_type,
        void* pointer,
        int reaction_number,
        int worker,
        instant_t* physical_time,
        trigger_t* trigger,
        interval_t extra_delay
) {
    // printf("DEBUG: Creating trace record.\n");
    // Flush the buffer if it is full.
    int index = (worker >= 0) ? worker : 0;
    if (_lf_trace_buffer_size[index] >= TRACE_BUFFER_CAPACITY) {
        // No more room in the buffer. Write the buffer to the file.
        flush_trace_to_file(index);
    }
    // The above flush_trace_to_file resets the write pointer.
    int i = _lf_trace_buffer_size[index];
    // Write to memory buffer.
    _lf_trace_buffer[index][i].event_type = event_type;
    _lf_trace_buffer[index][i].pointer = pointer;
    _lf_trace_buffer[index][i].reaction_number = reaction_number;
    _lf_trace_buffer[index][i].worker = worker;
    _lf_trace_buffer[index][i].logical_time = get_logical_time();
    _lf_trace_buffer[index][i].microstep = get_microstep();
    if (physical_time != NULL) {
        _lf_trace_buffer[index][i].physical_time = *physical_time;
    } else {
        _lf_trace_buffer[index][i].physical_time = get_physical_time();
    }
    _lf_trace_buffer_size[index]++;
    _lf_trace_buffer[index][i].trigger = trigger;
    _lf_trace_buffer[index][i].extra_delay = extra_delay;
}

/**
 * Trace the start of a reaction execution.
 * @param reaction Pointer to the reaction_t struct for the reaction.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_reaction_starts(reaction_t* reaction, int worker) {
    tracepoint(reaction_starts, reaction->self, reaction->number, worker, NULL, NULL, 0);
}

/**
 * Trace the end of a reaction execution.
 * @param reaction Pointer to the reaction_t struct for the reaction.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_reaction_ends(reaction_t* reaction, int worker) {
    tracepoint(reaction_ends, reaction->self, reaction->number, worker, NULL, NULL, 0);
}

/**
 * Trace a call to schedule.
 * @param trigger Pointer to the trigger_t struct for the trigger.
 * @param extra_delay The extra delay passed to schedule().
 */
void tracepoint_schedule(trigger_t* trigger, interval_t extra_delay) {
    // schedule() can only trigger reactions within the same reactor as the action
    // or timer. If there is such a reaction, find its reactor's self struct and
    // put that into the tracepoint. We only have to look at the first reaction.
    // If there is no reaction, insert NULL for the reactor.
    void* reactor = NULL;
    if (trigger->number_of_reactions > 0
            && trigger->reactions[0] != NULL) {
        reactor = trigger->reactions[0]->self;
    }
    tracepoint(schedule_called, reactor, 0, 0, NULL, trigger, extra_delay);
}

/**
 * Trace a user-defined event. Before calling this, you must call
 * register_user_trace_event() with a pointer to the same string
 * or else the event will not be recognized.
 * @param description Pointer to the description string.
 */
void tracepoint_user_event(char* description) {
    // -1s indicate unknown reaction number and worker thread.
    tracepoint(user_event, description,  -1, -1, NULL, NULL, 0);
}

/**
 * Trace a user-defined event with a value.
 * Before calling this, you must call
 * register_user_trace_event() with a pointer to the same string
 * or else the event will not be recognized.
 * @param description Pointer to the description string.
 * @param value The value of the event. This is a long long for
 *  convenience so that time values can be passed unchanged.
 *  But int values work as well.
 */
void tracepoint_user_value(char* description, long long value) {
    // -1s indicate unknown reaction number and worker thread.
    tracepoint(user_value, description,  -1, -1, NULL, NULL, value);
}

/**
 * Trace the start of a worker waiting for something to change on the event or reaction queue.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_worker_wait_starts(int worker) {
    tracepoint(worker_wait_starts, NULL, -1, worker, NULL, NULL, 0);
}

/**
 * Trace the end of a worker waiting for something to change on the event or reaction queue.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_worker_wait_ends(int worker) {
    tracepoint(worker_wait_ends, NULL, -1, worker, NULL, NULL, 0);
}

/**
 * Trace the start of a worker waiting for logical time to advance or an event to
 * appear on the event queue.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_worker_advancing_time_starts(int worker) {
    tracepoint(worker_advancing_time_starts, NULL, -1, worker, NULL, NULL, 0);
}

/**
 * Trace the end of a worker waiting for logical time to advance or an event to
 * appear on the event queue.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_worker_advancing_time_ends(int worker) {
    tracepoint(worker_advancing_time_ends, NULL, -1, worker, NULL, NULL, 0);
}

/**
 * Flush any buffered trace records to the trace file and
 * close the file.
 */
void stop_trace() {
    if (_lf_trace_stop) {
        // Trace was already stopped. Nothing to do.
        return;
    }
    lf_mutex_lock(&_lf_trace_mutex);
    // In multithreaded execution, thread 0 invokes wrapup reactions, so we
    // put that trace last. However, it could also include some startup events.
    // In any case, the trace file does not guarantee any ordering.
    for (int i = 1; i < _lf_number_of_trace_buffers; i++) {
        // Flush the buffer if it has data.
        // printf("DEBUG: Trace buffer %d has %d records.\n", i, _lf_trace_buffer_size[i]);
        if (_lf_trace_buffer_size && _lf_trace_buffer_size[i] > 0) {
            flush_trace_to_file_locked(i);
        }
    }
    if (_lf_trace_buffer_size && _lf_trace_buffer_size[0] > 0) {
        flush_trace_to_file_locked(0);
    }
    _lf_trace_stop = 1;
    // Wake up the trace_flush thread.
    lf_cond_signal(&_lf_flush_needed);
    lf_mutex_unlock(&_lf_trace_mutex);

    // Join trace_flush thread.
    void* flush_trace_thread_exit_status;
    lf_thread_join(_lf_flush_trace_thread, &flush_trace_thread_exit_status);

    fclose(_lf_trace_file);
    _lf_trace_file = NULL;
    DEBUG_PRINT("Stopped tracing.");
}
