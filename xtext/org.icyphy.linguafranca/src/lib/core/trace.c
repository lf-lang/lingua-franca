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

/**
 * Buffer into which traces are written. When this gets full,
 * the trace is flushed to the trace file.
 */
trace_record_t _lf_trace_buffer[TRACE_BUFFER_CAPACITY];
int _lf_trace_buffer_size = 0;

/** The file into which traces are written. */
FILE* _lf_trace_file;

/**
 * Table of pointers to a description of the object.
 */
// FIXME: Size should be code generated.
object_description_t _lf_trace_object_descriptions[1024];
int _lf_trace_object_descriptions_size = 0;

/**
 * Flush trace so far to the trace file.
 * @return The number of items written or -1 for failure.
 */
int flush_trace_to_file() {
    // printf("DEBUG: Writing %d trace records.\n", _lf_trace_buffer_size);
    if (_lf_trace_file != NULL) {
        // Write first the length of the array.
        size_t items_written = fwrite(
                &_lf_trace_buffer_size,
                sizeof(int),
                1,
                _lf_trace_file
        );
        if (items_written != 1) {
            _LF_TRACE_FAILURE(_lf_trace_file);
        } else {
            // Write the contents.
            items_written = fwrite(
                    _lf_trace_buffer,
                    sizeof(trace_record_t),
                    _lf_trace_buffer_size,
                    _lf_trace_file
            );
            if (items_written != _lf_trace_buffer_size) {
                _LF_TRACE_FAILURE(_lf_trace_file);
            }
        }
    }
    int result = _lf_trace_buffer_size;
    _lf_trace_buffer_size = 0;
    return result;
}

/**
 * Write the trace header information.
 * See trace.h.
 * @return The number of items written to the object table or -1 for failure.
 */
int write_trace_header() {
    if (_lf_trace_file != NULL) {
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
            // printf("DEBUG: Object pointer: %p.\n", _lf_trace_object_descriptions[i].object);
            items_written = fwrite(
                        &_lf_trace_object_descriptions[i].object, // Write the pointer value.
                        sizeof(void*),
                        1,
                        _lf_trace_file
            );
            if (items_written != 1) _LF_TRACE_FAILURE(_lf_trace_file);

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
    }
    return _lf_trace_object_descriptions_size;
}

/**
 * Open a trace file and start tracing.
 */
void start_trace() {
    // FIXME: filename and location should be customizable.
    _lf_trace_file = fopen("trace.lft", "w");
    if (_lf_trace_file == NULL) {
        fprintf(stderr, "WARNING: Failed to open log file with error code %d."
                "No log will be written.\n", errno);
    }
    write_trace_header();
}

/**
 * Trace an event identified by a type and a pointer to the self struct of the reactor instance.
 * This is a generic tracepoint function. It is better to use one of the specific functions.
 * @param event_type The type of event (see trace_event_t in trace.h)
 * @param reaction_number The number of the reaction or -1 if the trace is not of a reaction.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 * @param physical_time If the caller has already accessed physical time, provide it here.
 *  Otherwise, provide NULL. This argument avoids a second call to get_physical_time
 *  and ensures that the physical time in the trace is the same as that used by the caller.
 */
void tracepoint(
            trace_event_t event_type,
            void* self_struct,
            int reaction_number,
            int worker,
            instant_t* physical_time
) {
    // printf("DEBUG: Creating trace record.\n");
    // Flush the buffer if it is full.
    if (_lf_trace_buffer_size >= TRACE_BUFFER_CAPACITY) {
        // No more room in the buffer. Write the buffer to the file.
        flush_trace_to_file();
    }
    // Write to memory buffer.
    _lf_trace_buffer[_lf_trace_buffer_size].event_type = event_type;
    _lf_trace_buffer[_lf_trace_buffer_size].self_struct = self_struct;
    _lf_trace_buffer[_lf_trace_buffer_size].reaction_number = reaction_number;
    _lf_trace_buffer[_lf_trace_buffer_size].worker = worker;
    _lf_trace_buffer[_lf_trace_buffer_size].logical_time = get_logical_time();
    _lf_trace_buffer[_lf_trace_buffer_size].microstep = get_microstep();
    if (physical_time != NULL) {
        _lf_trace_buffer[_lf_trace_buffer_size].physical_time = *physical_time;
    } else {
        _lf_trace_buffer[_lf_trace_buffer_size].physical_time = get_physical_time();
    }
    _lf_trace_buffer_size++;
}

/**
 * Trace the start of a reaction execution.
 * @param reaction Pointer to the reaction_t struct for the reaction.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_reaction_starts(reaction_t* reaction, int worker) {
    tracepoint(reaction_starts, reaction->self, reaction->number, worker, NULL);
}

/**
 * Trace the end of a reaction execution.
 * @param reaction Pointer to the reaction_t struct for the reaction.
 * @param worker The thread number of the worker thread or 0 for unthreaded execution.
 */
void tracepoint_reaction_ends(reaction_t* reaction, int worker) {
    tracepoint(reaction_ends, reaction->self, reaction->number, worker, NULL);
}

/**
 * Flush any buffered trace records to the trace file and
 * close the file.
 */
void stop_trace() {
    // Flush the buffer if it has data.
    if (_lf_trace_buffer_size > 0) {
        flush_trace_to_file();
    }
    fclose(_lf_trace_file);
    _lf_trace_file = NULL;
}
