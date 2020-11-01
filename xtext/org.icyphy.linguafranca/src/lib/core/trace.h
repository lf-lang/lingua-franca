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
 * Definitions of tracepoint events for use with the C code generator and any other
 * code generator that uses the C infrastructure (such as the Python code generator).
 *
 * FIXME: How to use.
 *
 * The trace file is named trace.lft and is a binary file with the following format:
 *
 * Header:
 * * instant_t: The start time. This is both the starting physical time and the starting logical time.
 * * int: Size N of the table mapping pointers to descriptions.
 * This is followed by N records each of which has:
 * * A pointer value (the key).
 * * A null-terminated string (the description).
 *
 * Traces:
 * A sequence of traces, each of which begins with an int giving the length of the trace
 * followed by binary representations of the trace_record struct written using fwrite().
 */
#ifndef TRACE_H
#define TRACE_H

/**
 * Trace event types. If you update this, be sure to update the
 * string representation below. Also, create a tracepoint function
 * for each event type.
 */
typedef enum {
    reaction_starts,
    reaction_ends
} trace_event_t;

/**
 * String description of event types.
 */
static const char* trace_event_names[] = {
        "Reaction starts",
        "Reaction ends"
};

#ifdef LINGUA_FRANCA_TRACE

/** Macro to use when access to trace file fails. */
#define _LF_TRACE_FAILURE(trace_file) \
    do { \
        fprintf(stderr, "WARNING: Access to trace file failed.\n"); \
        fclose(trace_file); \
        trace_file = NULL; \
        return -1; \
    } while(0)

// FIXME: Target property should specify the capacity of the trace buffer.
#define TRACE_BUFFER_CAPACITY 2048

typedef struct trace_record_t {
    trace_event_t event_type;
    void* self_struct;
    int reaction_number;
    instant_t logical_time;
    instant_t physical_time;
} trace_record_t;

/**
 * Struct for table of pointers to a description of the object.
 */
typedef struct object_description_t object_description_t;
struct object_description_t {
    void* object;      // Pointer to the object (e.g., a reaction_t).
    char* description; // A NULL terminated string.
};

void start_trace();

/**
 * Trace an event identified by a type and a pointer to the self struct of the reactor instance.
 * This is a generic tracepoint function. It is better to use one of the specific functions.
 * @param event_type The type of event (see trace_event_t in trace.h)
 * @param self_struct The pointer to the self struct of the reactor instance in the trace table.
 * @param reaction_index The index of the reaction or -1 if the trace is not of a reaction.
 * @param physical_time If the caller has already accessed physical time, provide it here.
 *  Otherwise, provide NULL. This argument avoids a second call to get_physical_time
 *  and ensures that the physical time in the trace is the same as that used by the caller.
 */
void tracepoint(trace_event_t event_type, void* self_struct, int reaction_index, instant_t* physical_time);

/**
 * Trace the start of a reaction execution.
 * @param reaction Pointer to the reaction_t struct for the reaction.
 */
void tracepoint_reaction_starts(reaction_t* reaction);

/**
 * Trace the end of a reaction execution.
 * @param reaction Pointer to the reaction_t struct for the reaction.
 */
void tracepoint_reaction_ends(reaction_t* reaction);

void stop_trace();

#else

// empty definition in case we compile without tracing
#define tracepoint(...)
#define tracepoint_reaction_starts(...)
#define tracepoint_reaction_ends(...)
#define start_trace(...)
#define stop_trace(...)

#endif // LINGUA_FRANCA_TRACE
#endif // TRACE_H
