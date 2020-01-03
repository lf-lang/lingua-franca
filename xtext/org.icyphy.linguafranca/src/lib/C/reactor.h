/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
 * @author Marten Lohstroh (marten@berkeley.edu)
 * @author Chris Gill (cdgill@wustl.edu)
 * @author Mehrdad Niknami (mniknami@berkeley.edu)
 *
 * @section LICENSE
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

 * @section DESCRIPTION
 * Runtime infrastructure for the C target of Lingua Franca.
 * This file contains header information used by both the threaded and
 * non-threaded versions of the C runtime.
 */

#ifndef REACTOR_H
#define REACTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <errno.h>
#include "pqueue.h"

//  ======== Macros ========  //

#define BILLION 1000000000LL

// FIXME: May want these to application dependent, hence code generated.
#define INITIAL_EVENT_QUEUE_SIZE 10
#define INITIAL_REACT_QUEUE_SIZE 10

/* Conversion of time to nanoseconds. */
#define NSEC(t) (t * 1LL)
#define NSECS(t) (t * 1LL)
#define USEC(t) (t * 1000LL)
#define USECS(t) (t * 1000LL)
#define MSEC(t) (t * 1000000LL)
#define MSECS(t) (t * 1000000LL)
#define SEC(t)  (t * 1000000000LL)
#define SECS(t) (t * 1000000000LL)
#define MINUTE(t)   (t * 60000000000LL)
#define MINUTES(t)  (t * 60000000000LL)
#define HOUR(t)  (t * 3600000000000LL)
#define HOURS(t) (t * 3600000000000LL)
#define DAY(t)   (t * 86400000000000LL)
#define DAYS(t)  (t * 86400000000000LL)
#define WEEK(t)  (t * 604800000000000LL)
#define WEEKS(t) (t * 604800000000000LL)

/** Set the specified output to the specified value.
 *  This copies the value into the field in the self struct designated
 *  for this outputs value. This also sets the _is_present variable
 *  in the self struct to true.
 *  This is a macro terminated with a semicolon.
 *  @param out A pointer to the output in the self struct.
 *  @param value The value to insert into the self struct.
 */
#define set(out, value) (*out) = value; (*out ## _is_present) = true;

/** Version of set that hands to an output a dynamically allocated array.
 *  This will only work with an output that declared with an array type,
 *  type[]. The deallocation is delegated to downstream reactors, which
 *  automatically deallocate when the reference count drops to zero.
 *  This is a macro terminated with a semicolon.
 *  @param out A pointer to the output in the self struct.
 *  @param val A pointer to the array to send.
 *  @param length The length of the array to send.
 */
#define set_array(out, val, len) out->value = val; out->length = len; out->ref_count = out->initial_ref_count; (*out ## _is_present) = true;

/** Version of set() that allocates a new object of the type of a
 *  specified output port, sets the specified output to send that object,
 *  and sets the corresponding _is_present variable in the self struct to true.
 *  This returns a pointer to the
 *  object that has been allocated so that the user code can populate it.
 *  The freeing of the dynamically allocated object will be handled automatically
 *  when the last downstream reader of the message has finished.
 *  This is a macro terminated with a semicolon.
 *  @param out A pointer to the output in the self struct.
 */
#define set_new(out) __set_new_array_impl(out, 1); (*out ## _is_present) = true;

/** Version of set() that allocates a new array of the specified length,
 *  sets the specified output to send that array, and sets the corresponding
 *  _is_present variable in the self struct to true. This returns the
 *  array that has been allocated so that the user code can populate the array.
 *  The freeing of the dynamically allocated array will be handled automatically
 *  when the last downstream reader of the message has finished.
 *  This is a macro terminated with a semicolon.
 *  @param out A pointer to the output in the self struct.
 *  @param length The length of the array to be sent.
 */
#define set_new_array(out, length) __set_new_array_impl(out, length); (*out ## _is_present) = true;

/** Set the _is_present variable corresponding to the specified output
 *  to true. This is normally used with array outputs with fixed sizes
 *  and statically allocated structs. In these cases, the values in the
 *  output are normally written directly to the array or struct.
 *  This is a macro terminated with a semicolon.
 *  @param out A pointer to the output in the self struct.
 */
#define set_present(out) (*out ## _is_present) = true;

/** Version of set that hands to an output a dynamically allocated object.
 *  This will only work with an output that declared with a pointer type,
 *  type*. The deallocation is delegated to downstream reactors, which
 *  automatically deallocate when the reference count drops to zero.
 *  This is a macro terminated with a semicolon.
 *  @param out A pointer to the output in the self struct.
 *  @param val A pointer to the object to send.
 */
#define set_token(out, val) out->value = val; out->length = 1; out->ref_count = out->initial_ref_count; (*out ## _is_present) = true;

/** Return a writable copy of the specified input, which must be
 *  a message carried by a token_t struct. If the reference count
 *  is exactly one, this returns the message itself without copying.
 *  Otherwise, it returns a copy.
 *  This is a macro that converts an input name into a reference
 *  in the self struct.
 */
#define writable_copy(input) __writable_copy_impl(self->__ ## input)

//  ======== Type definitions ========  //

/** Booleans. */
typedef enum { false, true } bool;

/** Handles for scheduled triggers. */
typedef int handle_t;

/** Time instants.
    WARNING: If this code is used after about the year 2262,
    then representing time as a long long will be insufficient. */
typedef long long instant_t;

/** Intervals of time. */
typedef long long interval_t;

/** Topological order index for reactions. */
typedef pqueue_pri_t index_t;

/** Reaction function type. */
typedef void(*reaction_function_t)(void*);

/** Token type for dynamically allocated arrays and structs sent as messages. */
typedef struct token_t {
    /** Pointer to a struct or array to be sent as a message. */
    void* value;
    /** Size of the struct or array element. */
    int element_size;
    /** Length of the array or 1 for a struct. */
    int length;
    /** The number of destination input ports for the message. */
    int initial_ref_count;
    /** The number of input ports that have not already reacted to the message. */
    int ref_count;
} token_t;

/** Reaction activation record to push onto the reaction queue. */
typedef struct trigger_t trigger_t;

/** Reaction activation record to push onto the reaction queue. */
typedef struct reaction_t reaction_t;
struct reaction_t {
    reaction_function_t function;
    void* self;    // Pointer to a struct with the reactor's state.
    index_t index; // Inverse priority determined by dependency analysis.
    size_t pos;    // Current position in the priority queue.
    int num_outputs;  // Number of outputs that may possibly be produced by this function.
    bool** output_produced;   // Array of pointers to booleans indicating whether outputs were produced.
    int* triggered_sizes;     // Pointer to array of ints with number of triggers per output.
    trigger_t ***triggers;    // Array of pointers to arrays of pointers to triggers triggered by each output.
    bool running;             // Indicator that this reaction has already started executing.
    interval_t local_deadline;// Local deadline relative to the time stamp for invocation of the reaction.
    reaction_function_t deadline_violation_handler; // Local deadline violation handler.
    // NOTE: The next three items are for deadlines set in the container. These may go away.
    interval_t deadline;      // Container deadline relative to the time stamp for invocation of the reaction.
    trigger_t* deadline_violation; // Trigger to fire in the event of a container deadline violation.
    time_t violation_handled; // The time at which the most recent deadline violation has been handled
                              // (to prevent it from being handled again).
};

/** Reaction activation record to push onto the reaction queue. */
struct trigger_t {
    reaction_t** reactions;   // Reactions sensitive to this trigger.
    int number_of_reactions;  // Number of reactions sensitive to this trigger.
	interval_t offset;        // For an action, this will be a minimum delay.
	interval_t period;        // For periodic timers (not for actions).
    void* value;              // Pointer to malloc'd value (or NULL).
    bool is_physical;         // Indicator that this denotes a physical action 
                              // (i.e., to be scheduled relative to physical time)
};

/** Event activation record to push onto the event queue. */
typedef struct event_t {
    instant_t time;     // Time of release.
    trigger_t* trigger; // Associated trigger.
    size_t pos;         // Position in the priority queue.
    void* value;      // Pointer to malloc'd value (or NULL).
} event_t;

//  ======== Function Declarations ========  //

/**
 * Return the elpased logical time in nanoseconds
 * since the start of execution.
 * @return a time interval
 */
interval_t get_elapsed_logical_time();

/**
 * Return the current logical time in nanoseconds
 * since January 1, 1970.
 * @return a time instant
 */
instant_t get_logical_time();

/**
 * Return the current physical time in nanoseconds
 * since January 1, 1970.
 * @return a time instant
 */
instant_t get_physical_time();

/**
 * Function to request stopping execution at the end of the current logical time.
 */
void stop();

/** 
 * Generated function that optionally sets default command-line options.
 */
void __set_default_command_line_options();

/** 
 * Generated function that resets outputs to be absent at the
 * start of a new time step.
 */
void __start_time_step();

/** 
 * Generated function that produces a table containing all triggers
 * (i.e., inputs, timers, and actions).
 */
void __initialize_trigger_objects();

/** 
 * Internal version of the schedule() function, used by generated 
 * __start_timers() function. 
 * @param trigger The action or timer to be triggered.
 * @param delay Offset of the event release.
 * @param value The malloc'd value.
 */
handle_t __schedule(trigger_t* trigger, interval_t delay, void* value);

/**
 * Function (to be code generated) to start timers.
 */
void __start_timers();

/**
 * Function (to be code generated) to wrap up execution.
 * If this returns true, then one more invocation of next()
 * be executed in order to invoke reactions that are triggered
 * by shutdown.
 */
bool __wrapup();

/** Global constants. */
bool False;
bool True;

/** By default, execution is not threaded and this variable will have value 0. */
int number_of_threads;

/**
 * External version of schedule, callable from within reactors.
 * @param trigger The action or timer to be triggered.
 * @param delay Extra offset of the event release.
 * @param value The malloc'd value.
*/
handle_t schedule(trigger_t* trigger, interval_t extra_delay, void* value);

/**
 * Specialized version of malloc used by Lingua Franca for action values
 * and messages contained in dynamically allocated memory.
 * @param size The size of the memory block to allocate.
 * @return A pointer to the allocated memory block.
 */
void* lf_malloc(size_t size);

//  ******** Begin Windows Support ********  //
// Windows is not POSIX, so we include here compatibility definitions.
#if _WIN32 || WIN32
#pragma warning(disable: 4204 4255 4459 4710)
#ifdef  _M_X64
typedef long long intptr_t;
#else
typedef int intptr_t;
#endif
typedef intptr_t INTPTR_T;
typedef struct HINSTANCE__ *HINSTANCE;
typedef HINSTANCE HMODULE;
typedef INTPTR_T (__stdcall *FARPROC)();
HMODULE __stdcall GetModuleHandleA(char const *lpModuleName);
FARPROC __stdcall GetProcAddress(HMODULE hModule, char const *lpProcName);
typedef long NTSTATUS;
typedef union _LARGE_INTEGER *PLARGE_INTEGER;
typedef NTSTATUS __stdcall NtDelayExecution_t(unsigned char Alertable,
  PLARGE_INTEGER Interval);
NtDelayExecution_t *NtDelayExecution;
typedef NTSTATUS __stdcall NtQueryPerformanceCounter_t(
  PLARGE_INTEGER PerformanceCounter, PLARGE_INTEGER PerformanceFrequency);
NtQueryPerformanceCounter_t *NtQueryPerformanceCounter;
typedef NTSTATUS __stdcall NtQuerySystemTime_t(PLARGE_INTEGER SystemTime); 
NtQuerySystemTime_t *NtQuerySystemTime;
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
typedef int clockid_t;
int clock_gettime(clockid_t clk_id, struct timespec *tp);
int nanosleep(const struct timespec *req, struct timespec *rem);
#endif
//  ******** End Windows Support ********  //

// Patmos is missing posix time as well
#ifdef __PATMOS__
#define TIMER_US_LOW *((volatile _IODEV int *) (PATMOS_IO_TIMER + 0xc))
int clock_gettime(clockid_t clk_id, struct timespec *tp);
int nanosleep(const struct timespec *req, struct timespec *rem);
#endif

#endif /* REACTOR_H */
/** @} */