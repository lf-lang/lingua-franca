/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
 * @author Marten Lohstroh (marten@berkeley.edu)
 * @author Chris Gill (cdgill@wustl.edu)
 * @author Mehrdad Niknami (mniknami@berkeley.edu)
 *
 * @section LICENSE
 * Copyright (c) 2019, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * Header file for the infrastructure for the C target of Lingua Franca.
 * This file contains header information used by both the threaded and
 * non-threaded versions of the C runtime. A generated C program will have
 * either #include reactor.c or #include reactor_threaded.c. Those two files
 * #include this header file.
 *
 * This header file defines the functions and macros that programmers use
 * in the body of reactions for reading and writing inputs and outputs and
 * scheduling future events. The LF compiler does not parse that C code.
 * This fact strongly affects the design.
 *
 * The intent of the C target for Lingua Franca not to provide a safe
 * programming environment (The C++ and TypeScript targets are better
 * choices for that), but rather to find the lowest possible overhead
 * implementation of Lingua Franca. The API herein can easily be misused,
 * leading to memory leaks, nondeterminism, or program crashes.
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

#define CONSTRUCTOR(classname) (new_ ## classname)
#define SELF_STRUCT_T(classname) (classname ## _self_t)


// Commonly used time values.
#define NEVER -0xFFFFFFFFFFFFFFFFLL
#define FOREVER 0x7FFFFFFFFFFFFFFFLL

// Convenience for converting times
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

/**
 * Macro for extracting the deadline from the index of a reaction.
 * The reaction queue is sorted according to this index, and the
 * use of the deadline here results in an earliest deadline first
 * (EDF) scheduling poicy.
 */
#define DEADLINE(index) (index & 0x7FFFFFFFFFFF0000)

/**
 * Macro for extracting the level from the index of a reaction.
 * A reaction that has no upstream reactions has level 0.
 * Other reactions have a level that is the length of the longest
 * upstream chain to a reaction with level 0 (inclusive).
 * This is used, along with the deadline, to sort reactions
 * in the reaction queue. It ensures that reactions that are
 * upstream in the dependence graph execute before reactions
 * that are downstream.
 */
#define LEVEL(index) (index & 0xFFFF)

/** Utility for finding the maximum of two values. */
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

/** Utility for finding the minimum of two values. */
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

/**
 * Macro for determining whether two reactions are in the
 * same chain (one depends on the other). This is conservative.
 * If it returns false, then they are surely not in the same chain,
 * but if it returns true, they may be in the same chain.
 * This is in reactor_threaded.c to execute reactions in parallel
 * on multiple cores even if their levels are different.
 */
#define OVERLAPPING(chain1, chain2) ((chain1 & chain2) != 0)

//  ======== Type definitions ========  //

/**
 * Booleans. This needs to be defined only if the target language
 * is C and the compiler is not a C++ compiler.
 */
#ifndef __cplusplus
typedef enum {false, true} bool;
#endif

/**
 * Handles for scheduled triggers. These handles are returned
 * by schedule() functions. The intent is that the handle can be
 * used to cancel a future scheduled event, but this is not
 * implemented yet.
 */
typedef int handle_t;

/**
 * Time instant. Both physical and logical times are represented
 * using this typedef. FIXME: Perhaps distinct typedefs should
 * be used.
 * WARNING: If this code is used after about the year 2262,
 * then representing time as a long long will be insufficient.
 */
typedef long long instant_t;

/** Interval of time. */
typedef long long interval_t;

/**
 * String type so that we don't have to use {= char* =}.
 * Use this for strings that are not dynamically allocated.
 * For dynamically allocated strings that have to be freed after
 * being consumed downstream, use type char*.
 */
#ifndef string
typedef char* string;
#else
#warning "string typedef has been previously given."
#endif

/** Topological order index for reactions. */
typedef pqueue_pri_t index_t;

/**
 * Reaction function type. The argument passed to one of
 * these reaction functions is a pointer to the self struct
 * for the reactor.
 */
typedef void(*reaction_function_t)(void*);

/** Trigger struct representing an output, timer, action, or input. See below. */
typedef struct trigger_t trigger_t;

/**
 * Token type for dynamically allocated arrays and structs sent as messages.
 *
 * In the C LF target, a type for an output that ends in '*' is
 * treated specially. The value carried by the output is assumed
 * to be in dynamically allocated memory, and, using reference
 * counting, after the last downstream reader of the value has
 * finished, the memory will be freed.  To prevent this freeing
 * from occurring, the output type can be specified using the
 * syntax {= type* =}; this will not be treated as dynamically
 * allocated memory. Alternatively, the programmer can give a typedef
 * in the preamble that masks the trailing *.
 *
 * This struct is the wrapper around the dynamically allocated memory
 * that carries the message.  The message can be an array of values,
 * where the size of each value is element_size (in bytes). If it is
 * not an array, the length == 1.
 */
typedef struct token_t {
    /** Pointer to dynamically allocated memory containing a message. */
    void* value;
    /** Size of the struct or array element. */
    size_t element_size;
    /** Length of the array or 1 for a struct. */
    int length;
    /** The number of input ports that have not already reacted to the message. */
    int ref_count;
    /**
     * Indicator of whether this token is expected to be freed.
     * Tokens that are created at the start of execution and associated with output
     * ports or actions are not expected to be freed. They can be reused instead.
     */
    bool ok_to_free;
    /** For recycling, a pointer to the next token in the recycling bin. */
    struct token_t* next_free;
} token_t;

/** A struct with a pointer to a token_t and an _is_present variable
 *  for use to initialize actions in start_time_step().
 */
typedef struct token_present_t {
    token_t** token;
    bool* is_present;
    bool reset_is_present; // True to set is_present to false after calling done_using().
} token_present_t;

/**
 * Reaction activation record to push onto the reaction queue.
 * Some of the information in this struct is common among all instances
 * of the reactor, and some is specific to each particular instance.
 * These are marked below COMMON or INSTANCE accordingly.
 * The COMMON information is set in the constructor.
 * The fields marked RUNTIME have values that change
 * during execution.
 * Instances of this struct are put onto the reaction queue (reaction_q).
 */
typedef struct reaction_t reaction_t;
struct reaction_t {
    reaction_function_t function; // The reaction function. COMMON.
    void* self;    // Pointer to a struct with the reactor's state. INSTANCE.
    index_t index; // Inverse priority determined by dependency analysis. INSTANCE.
    unsigned long long chain_id; // Binary encoding of the branches that this reaction has upstream in the dependency graph. INSTANCE.
    size_t pos;       // Current position in the priority queue. RUNTIME.
    int num_outputs;  // Number of outputs that may possibly be produced by this function. COMMON.
    bool** output_produced;   // Array of pointers to booleans indicating whether outputs were produced. COMMON.
    int* triggered_sizes;     // Pointer to array of ints with number of triggers per output. INSTANCE.
    trigger_t ***triggers;    // Array of pointers to arrays of pointers to triggers triggered by each output. INSTANCE.
    bool running;             // Indicator that this reaction has already started executing. RUNTIME.
    interval_t deadline;// Deadline relative to the time stamp for invocation of the reaction. INSTANCE.
    reaction_function_t deadline_violation_handler; // Deadline violation handler. COMMON.
};

/**
 * Trigger struct representing an output, timer, action, or input.
 * Instances of this struct are put onto the event queue (event_q).
 */
struct trigger_t {
    reaction_t** reactions;   // Array of pointers to reactions sensitive to this trigger.
    int number_of_reactions;  // Number of reactions sensitive to this trigger.
    bool is_timer;            // True if this is a timer (a special kind of action), false otherwise.
    interval_t offset;        // Minimum delay of an action. For a timer, this is also the maximum delay.
    interval_t period;        // Minimum interarrival time of an action. For a timer, this is also the maximal interarrival time.
    token_t* token;           // Pointer to a token wrapping the payload (or NULL if there is none).
    bool is_physical;         // Indicator that this denotes a physical action.
    instant_t scheduled;      // Tag of the last event that was scheduled for this action.
    bool drop;                // Whether or not to drop events if they succeed one another more quickly than the minimum interarrival time allows.
    size_t element_size;      // The size of the payload, if there is one, zero otherwise.
                              // If the payload is an array, then this is the size of an element of the array.
    bool is_present;          // Indicator at any given logical time of whether the trigger is present.
};

/** Event activation record to push onto the event queue. */
typedef struct event_t {
    instant_t time;           // Time of release.
    trigger_t* trigger;       // Associated trigger.
    size_t pos;               // Position in the priority queue.
    token_t* token;           // Pointer to the token wrapping the value.
} event_t;

//  ======== Function Declarations ========  //

/**
 * Return the elapsed logical time in nanoseconds
 * since the start of execution.
 * @return A time interval.
 */
interval_t get_elapsed_logical_time();

/**
 * Return the current logical time in nanoseconds.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent.
 * @return A time instant.
 */
instant_t get_logical_time();

/**
 * Return the current physical time in nanoseconds.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent.
 * @return A time instant.
 */
instant_t get_physical_time();

/**
 * Print a snapshot of the priority queues used during execution
 * (for debugging).
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
 * Pop all events from event_q with timestamp equal to current_time, extract all
 * the reactions triggered by these events, and stick them into the reaction
 * queue.
 */
void __pop_events();

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

/**
 * Indicator for the absence of values for ports that remain disconnected.
 */
extern bool absent;

/**
 * Create a new token and initialize it.
 * The value pointer will be NULL and the length will be 0.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A pointer to a new or recycled token_t struct.
 */
token_t* create_token(size_t element_size);

/**
 * Schedule the specified action with an integer value at a later logical
 * time that depends on whether the action is logical or physical and
 * what its parameter values are. This wraps a copy of the integer value
 * in a token. See schedule_token() for more details.
 * @param action The action to be triggered.
 * @param extra_delay Extra offset of the event release above that in the action.
 * @param value The value to send.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_int(void* action, interval_t extra_delay, int value);

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
 * reaction that is calling this function. Logical actions should
 * always be scheduled within a reaction invocation, never asynchronously
 * from the outside. FIXME: This needs to be checked.
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
 * The first condition is that stop() has been called and the time offset
 * of this event is greater than zero.
 * The second condition is that the logical time of the event
 * is greater that the stop time (timeout) that is specified in the target
 * properties or on the command line.
 * The third condition is that the trigger argument is null.
 *
 * @param action The action to be triggered.
 * @param extra_delay Extra offset of the event release above that in the action.
 * @param token The token to carry the payload or null for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_token(void* action, interval_t extra_delay, token_t* token);

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
handle_t schedule_value(void* action, interval_t extra_delay, void* value, int length);

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value. If the value is non-null,
 * then it will be copied into newly allocated memory under the assumption
 * that its size is given in the trigger's token object's element_size field
 * multiplied by the specified length.
 * See schedule_token(), which this uses, for details.
 * @param action Pointer to an action on a self struct.
 * @param offset The time offset over and above that in the action.
 * @param value A pointer to the value to copy.
 * @param length The length, if an array, 1 if a scalar, and 0 if value is NULL.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_copy(void* action, interval_t offset, void* value, int length);

/**
 * For a federated execution, broadcast stop() to all federates.
 */
void __broadcast_stop();

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

#endif /* REACTOR_H */
/** @} */
