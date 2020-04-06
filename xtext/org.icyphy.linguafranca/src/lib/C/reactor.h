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

#define NEVER -9223372036854775807LL
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

// NOTE: According to the "Swallowing the Semicolon" section on this page:
//    https://gcc.gnu.org/onlinedocs/gcc-3.0.1/cpp_3.html
// the following macros should use an odd do-while construct to avoid
// problems with if ... else statements that do not use braces around the
// two branches.

////////////////////////////////////////////////////////////
//// Functions for producing outputs.

/** Set the specified output (or input of a contained reactor)
 *  to the specified value.
 *  This version is used for primitive type such as int,
 *  double, etc. as well as the built-in types bool and string.
 *  The value is copied and therefore the variable carrying the
 *  value can be subsequently modified without changing the output.
 *  This can also be used for structs with a type defined by a typedef
 *  so that the type designating string does not end in '*'.
 *  @param port The output port (by name) or input of a contained
 *   reactor in form input_name.port_name.
 *  @param value The value to insert into the self struct.
 */
#define set(out, value) \
do { \
    out ## _is_present = true; \
    self->__ ## out = value; \
    self->__ ## out ## _is_present = true; \
} while(0)

/** Version of set for output types given as 'type[]' where you
 *  want to send a previously dynamically allocated array.
 *  The deallocation is delegated to downstream reactors, which
 *  automatically deallocate when the reference count drops to zero.
 *  It also sets the corresponding _is_present variable in the self
 *  struct to true (which causes the object message to be sent).
 *  @param out The output port (by name).
 *  @param val The array to send (a pointer to the first element).
 *  @param length The length of the array to send.
 */
#define set_array(out, val, len) \
do { \
    out ## _is_present = true; \
    __initialize_token(self->__ ## out, val, self->__ ## out->element_size, len, self->__ ## out ## _num_destinations); \
    self->__ ## out ## _is_present = true; \
} while(0)

/** Version of set() for output types given as 'type*' that
 *  allocates a new object of the type of the specified output port.
 *  It also sets the corresponding _is_present variable in the self
 *  struct to true (which causes the object message to be sent),
 *  and sets the variable named by the argument to the newly allocated
 *  object so that the user code can then populate it.
 *  The freeing of the dynamically allocated object will be handled automatically
 *  when the last downstream reader of the message has finished.
 *  @param out The output port (by name).
 */
#define set_new(out) \
do { \
    out ## _is_present = true; \
    token_t* token = __set_new_array_impl(self->__ ## out, 1, self->__ ## out ## _num_destinations); \
    out = token->value; \
    self->__ ## out ## _is_present = true; \
    self->__ ## out = token; \
} while(0)

/** Version of set() for output types given as 'type[]'.
 *  This allocates a new array of the specified length,
 *  sets the corresponding _is_present variable in the self struct to true
 *  (which causes the array message to be sent), and sets the variable
 *  given by the first argument to point to the new array so that the
 *  user code can populate the array. The freeing of the dynamically
 *  allocated array will be handled automatically
 *  when the last downstream reader of the message has finished.
 *  @param out The output port (by name).
 *  @param length The length of the array to be sent.
 */
#define set_new_array(out, length) \
do { \
    out ## _is_present = true; \
    token_t* token = __set_new_array_impl(self->__ ## out, length, self->__ ## out ## _num_destinations); \
    out = token->value; \
    self->__ ## out ## _is_present = true; \
    self->__ ## out = token; \
} while(0)

/** Version of set() for output types given as 'type[number]'.
 *  This sets the _is_present variable corresponding to the specified output
 *  to true (which causes the array message to be sent). The values in the 
 *  output are normally written directly to the array or struct before or
 *  after this is called.
 *  @param out The output port (by name).
 */
#define set_present(out) \
do { \
    out ## _is_present = true; \
    self->__ ## out ## _is_present = true; \
} while(0)

/** Version of set() for output types given as 'type*' or 'type[]'where you want
 *  to forward an input or action without copying it.
 *  The deallocation of memory is delegated to downstream reactors, which
 *  automatically deallocate when the reference count drops to zero.
 *  @param out The output port (by name).
 *  @param token A pointer to token obtained from an input or action.
 */
#define set_token(out, token) \
do { \
    out ## _is_present = true; \
    self->__ ## out = token; \
    token->ref_count += self->__ ## out ## _num_destinations; \
    self->__ ## out ## _is_present = true; \
} while(0)

#define LEVEL(index) (index & 0xFFFF)

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#define OVERLAPPING(chain1, chain2) ((chain1 & chain2) != 0)

#define DEADLINE(index) (index & 0x7FFFFFFFFFFF0000)

//  ======== Type definitions ========  //

/** Booleans. */
typedef enum {false, true} bool;

/** Handles for scheduled triggers. */
typedef int handle_t;

/** Time instants.
    WARNING: If this code is used after about the year 2262,
    then representing time as a long long will be insufficient. */
typedef long long instant_t;

/** Intervals of time. */
typedef long long interval_t;

/** String type so that we don't have use {= char* =}. */
#ifndef string
typedef char* string;
#endif

/** Topological order index for reactions. */
typedef pqueue_pri_t index_t;

/** Reaction function type. */
typedef void(*reaction_function_t)(void*);

/** Trigger struct representing an output, timer, action, or input. */
typedef struct trigger_t trigger_t;

/** Token type for dynamically allocated arrays and structs sent as messages. */
typedef struct token_t {
    /** Pointer to a struct or array to be sent as a message. Must be first. */
    void* value;
    /** Size of the struct or array element. */
    size_t element_size;
    /** Length of the array or 1 for a struct. */
    int length;
    /** The number of input ports that have not already reacted to the message. */
    int ref_count;
    /** Indicator of whether this token is expected to be freed.
     *  Tokens that are created at the start of execution and associated with output
     *  ports or actions are not expected to be freed. They can be reused instead.
     */
    bool ok_to_free;
    /** For recycling, a pointer to the next token in the recycling bin. */
    struct token_t* next_free;
} token_t;

/** Reaction activation record to push onto the reaction queue. */
typedef struct reaction_t reaction_t;
struct reaction_t {
    reaction_function_t function;
    void* self;    // Pointer to a struct with the reactor's state.
    index_t index; // Inverse priority determined by dependency analysis.
    unsigned long long chain_id; // Binary encoding of the branches that this reaction has upstream in the dependency graph.
    size_t pos;    // Current position in the priority queue.
    int num_outputs;  // Number of outputs that may possibly be produced by this function.
    bool** output_produced;   // Array of pointers to booleans indicating whether outputs were produced.
    int* triggered_sizes;     // Pointer to array of ints with number of triggers per output.
    trigger_t ***triggers;    // Array of pointers to arrays of pointers to triggers triggered by each output.
    bool running;             // Indicator that this reaction has already started executing.
    interval_t local_deadline;// Local deadline relative to the time stamp for invocation of the reaction.
    reaction_function_t deadline_violation_handler; // Local deadline violation handler.
};

/** 
 * Enumeration of different policies for handling events that succeed one
 * another more rapidly than is allowed by a physical action's min. inter-
 * arrival time.
 *
 * For logical actions, the policy should always be `NONE`. 
 *
 * For physical actions, the default policy is `DEFER`, which is to increase the
 * offsets of newly-scheduled events so that the min. interarrival time is
 * satisfied. This means that no events will be ignored, but they will occur
 * later. This policy has the drawback that it may cause the event queue to grow
 * indefinitely. 
 *
 * The `DROP` policy ignores events that are scheduled too close to one another.
 *
 * The `UPDATE` policy do the following. If the event that a newly-scheduled
 * event is in too close proximity of is still on the event queue, the value
 * carried by that event will be updated with the value of the newly-scheduled
 * event. If this is not possible because the original event has already been
 * popped off the queue, the `DEFER` policy applies.
 * */
typedef enum queuing_policy_t {
    NONE,
    DEFER, 
    DROP, 
    UPDATE
} queuing_policy_t;

/** Trigger struct representing an output, timer, action, or input. */
struct trigger_t {
    reaction_t** reactions;   // Reactions sensitive to this trigger.
    int number_of_reactions;  // Number of reactions sensitive to this trigger.
	interval_t offset;        // For an action, this will be a minimum delay.
	interval_t period;        // For an action, this denotes the minimal interarrival time. // FIXME: add a field to distinguish between actions and timers.
    token_t* token;           // Pointer to a token wrapping the payload (or NULL if there is none).
    bool is_physical;         // Indicator that this denotes a physical action (i.e., to be scheduled relative to physical time).
    instant_t scheduled;      // Tag of the last event that was scheduled for this action.
    queuing_policy_t policy;  // Indicates the policy for handling events that succeed one another more rapidly than allowable by the specified min. interarrival time. Only applies to physical actions.
    size_t element_size;      // The size of the payload, if there is one, zero otherwise.
                              // If the payload is an array, then this is the size of an element of the array.
    bool is_present;          // Indicator at any given logical time of whether the trigger is present.
};

/** Event activation record to push onto the event queue. */
typedef struct event_t {
    instant_t time;           // Time of release.
    trigger_t* trigger;       // Associated trigger.
    size_t pos;               // Position in the priority queue.
    token_t* token;           // Pointer to the token wrapping the value or a template token for a trigger.
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
 * Print a snapshot of the priority queues used during execution.
 */
void print_snapshot();

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
 * @param token The token payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t __schedule(trigger_t* trigger, interval_t delay, token_t* token);

/**
 * Function (to be code generated) to start timers.
 */
void __start_timers();

/**
 * Function (to be code generated) to terminate execution.
 * This will be invoked after all shutdown actions have completed.
 */
void __termination();

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
 * Create a new token and initialize it.
 * The value pointer will be NULL and the length will be 0.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A new or recycled token_t struct.
 */
token_t* create_token(size_t element_size);

/**
 * Schedule the specified action with an integer value at a later logical
 * time that depends on whether the action is logical or physical and
 * what its parameter values are. See schedule_value().
 */
handle_t schedule_int(trigger_t* trigger, interval_t extra_delay, int value);

/**
 * Schedule the specified action with the specified token as a payload.
 * This will trigger an event at a later logical time that depends
 * on whether the action is logical or physical and what its parameter
 * values are.
 *
 * logical action: A logical action has an offset (default is zero)
 * and a minimum interarrival time (MIT), which also defaults to zero.
 * The logical time at which this scheduled event will trigger is
 * the current time plus the offset plus the delay argument given to
 * this function. If, however, that time is not greater than a prior
 * triggering of this logical action by at least the MIT, then the
 * one of two things can happen depending on the policy specified
 * for the action. If the action's policy is DROP (default), then the
 * action is simply dropped and the memory pointed to by value argument
 * is freed. If the policy is DEFER, then the time will be increased
 * to equal the time of the most recent triggering plus the MIT.
 *
 * For the above, "current time" means the logical time of the
 * reaction that is calling this function, or if this function is
 * being called from outside a reaction (asynchronously), then the
 * most recent logical time relevant to the reactor that owns the
 * action.
 *
 * physical action: A physical action has all the same parameters
 * as a logical action, but its timestamp will be the larger of the
 * current physical time and the time it would be assigned if it
 * were a logical action.
 *
 * The token is required to be either NULL or a pointer to
 * a token created using create_token().
 *
 * There are three conditions under which this function will not
 * actually put an event on the event queue and decrement the reference count
 * of the token (if there is one), which could result in the payload being
 * freed. In all three cases, this function returns 0. Otherwise,
 * it returns a handle to the scheduled trigger, which is an integer
 * greater than 0.
 *
 * The first condition is that a stop has been requested and the time offset
 * of this event is greater than zero.
 * The second condition is that the logical time of the event
 * is greater that the requested stop time (timeout).
 * The third condition is that the trigger argument is null.
 *
 * @param action The action to be triggered.
 * @param extra_delay Extra offset of the event release above that in the action.
 * @param token The token to carry the payload or null for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_token(trigger_t* action, interval_t extra_delay, token_t* token);

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * The value is required to be malloc'd memory with a size equal to the
 * element_size of the specifies action times the length parameter.
 * See schedule_token() for details.
 * @param action The action to be triggered.
 * @param extra_delay Extra offset of the event release above that in the action.
 * @param value Dynamically allocated memory containing the value to send.
 * @param length The length of the array, if it is an array, or 1 for a
 *  scalar and 0 for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_value(trigger_t* action, interval_t extra_delay, void* value, int length);

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value. If the value is non-null,
 * then it will be copied into newly allocated memory under the assumption
 * that its size given in the trigger's token object's element_size field
 * multiplied by the specified length.
 * See schedule_token(), which this uses, for details.
 * @param trigger Pointer to a trigger object (typically an action on a self struct).
 * @param offset The time offset over and above that in the action.
 * @param value A pointer to the value to copy.
 * @param length The length, if an array, 1 if a scalar, and 0 if value is NULL.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_copy(trigger_t* trigger, interval_t offset, void* value, int length);

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
