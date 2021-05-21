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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <errno.h>
#include "pqueue.h"
#include "util.h"
#include "tag.h"    // Time-related types and functions.

// The following file is also included, but must be included
// after its requirements are met, so the #include appears at
// then end.
// #include "trace.h"

//  ======== Macros ========  //
#define CONSTRUCTOR(classname) (new_ ## classname)
#define SELF_STRUCT_T(classname) (classname ## _self_t)

// FIXME: May want these to application dependent, hence code generated.
#define INITIAL_EVENT_QUEUE_SIZE 10
#define INITIAL_REACT_QUEUE_SIZE 10

////////////////////////////////////////////////////////////
//// Macros for producing outputs.

// NOTE: According to the "Swallowing the Semicolon" section on this page:
//    https://gcc.gnu.org/onlinedocs/gcc-3.0.1/cpp_3.html
// the following macros should use an odd do-while construct to avoid
// problems with if ... else statements that do not use braces around the
// two branches.

/**
 * Set the specified output (or input of a contained reactor)
 * to the specified value.
 *
 * This version is used for primitive types such as int,
 * double, etc. as well as the built-in types bool and string.
 * The value is copied and therefore the variable carrying the
 * value can be subsequently modified without changing the output.
 * This can also be used for structs with a type defined by a typedef
 * so that the type designating string does not end in '*'.
 * @param out The output port (by name) or input of a contained
 *  reactor in form input_name.port_name.
 * @param value The value to insert into the self struct.
 */
#define _LF_SET(out, val) \
do { \
    out->value = val; \
    out->is_present = true; \
} while(0)

/**
 * Version of set for output types given as 'type[]' where you
 * want to send a previously dynamically allocated array.
 *
 * The deallocation is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * It also sets the corresponding _is_present variable in the self
 * struct to true (which causes the object message to be sent).
 * @param out The output port (by name).
 * @param val The array to send (a pointer to the first element).
 * @param length The length of the array to send.
 * @see lf_token_t
 */
#ifndef __cplusplus
#define _LF_SET_ARRAY(out, val, element_size, length) \
do { \
    out->is_present = true; \
    lf_token_t* token = __initialize_token_with_value(out->token, val, length); \
    token->ref_count = out->num_destinations; \
    out->token = token; \
    out->value = token->value; \
} while(0)
#else
#define _LF_SET_ARRAY(out, val, element_size, length) \
do { \
    out->is_present = true; \
    lf_token_t* token = __initialize_token_with_value(out->token, val, length); \
    token->ref_count = out->num_destinations; \
    out->token = token; \
    out->value = static_cast<decltype(out->value)>(token->value); \
} while(0)
#endif

/**
 * Version of set() for output types given as 'type*' that
 * allocates a new object of the type of the specified output port.
 *
 * This macro dynamically allocates enough memory to contain one
 * instance of the output datatype and sets the variable named
 * by the argument to point to the newly allocated memory.
 * The user code can then populate it with whatever value it
 * wishes to send.
 *
 * This macro also sets the corresponding _is_present variable in the self
 * struct to true (which causes the object message to be sent),
 * @param out The output port (by name).
 */
#define _LF_SET_NEW(out) \
do { \
    out->is_present = true; \
    lf_token_t* token = __set_new_array_impl(out->token, 1, out->num_destinations); \
    out->value = token->value; \
    out->token = token; \
} while(0)

/**
 * Version of set() for output types given as 'type[]'.
 *
 * This allocates a new array of the specified length,
 * sets the corresponding _is_present variable in the self struct to true
 * (which causes the array message to be sent), and sets the variable
 * given by the first argument to point to the new array so that the
 * user code can populate the array. The freeing of the dynamically
 * allocated array will be handled automatically
 * when the last downstream reader of the message has finished.
 * @param out The output port (by name).
 * @param length The length of the array to be sent.
 */
#ifndef __cplusplus
#define _LF_SET_NEW_ARRAY(out, len) \
do { \
    out->is_present = true; \
    lf_token_t* token = __set_new_array_impl(out->token, len, out->num_destinations); \
    out->value = token->value; \
    out->token = token; \
    out->length = len; \
} while(0)
#else
#define _LF_SET_NEW_ARRAY(out, len) \
do { \
    out->is_present = true; \
    lf_token_t* token = __set_new_array_impl(out->token, len, out->num_destinations); \
    out->value = static_cast<decltype(out->value)>(token->value); \
    out->token = token; \
    out->length = len; \
} while(0)
#endif
/**
 * Version of set() for output types given as 'type[number]'.
 *
 * This sets the _is_present variable corresponding to the specified output
 * to true (which causes the array message to be sent). The values in the
 * output are normally written directly to the array or struct before or
 * after this is called.
 * @param out The output port (by name).
 */
#define _LF_SET_PRESENT(out) \
do { \
    out->is_present = true; \
} while(0)

/**
 * Version of set() for output types given as 'type*' or 'type[]' where you want
 * to forward an input or action without copying it.
 *
 * The deallocation of memory is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * @param out The output port (by name).
 * @param token A pointer to token obtained from an input or action.
 */
#ifndef __cplusplus
#define _LF_SET_TOKEN(out, newtoken) \
do { \
    out->is_present = true; \
    out->value = newtoken->value; \
    out->token = newtoken; \
    newtoken->ref_count += out->num_destinations; \
    out->is_present = true; \
    out->length = newtoken->length; \
} while(0)
#else
#define _LF_SET_TOKEN(out, newtoken) \
do { \
    out->is_present = true; \
    out->value = static_cast<decltype(out->value)>(newtoken->value); \
    out->token = newtoken; \
    newtoken->ref_count += out->num_destinations; \
    out->is_present = true; \
    out->length = newtoken->length; \
} while(0)
#endif


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
 * ushort type. Redefine here for portability if sys/types.h is not included.
 * @see sys/types.h
 * 
 * @note using sizeof(ushort) should be okay but not sizeof ushort.
 */
#ifndef _SYS_TYPES_H
typedef unsigned short int ushort;
#endif

/**
 * Policy for handling scheduled events that violate the specified
 * minimum interarrival time.
 * The default policy is `defer`: adjust the tag to that the minimum
 * interarrival time is satisfied.
 * The `drop` policy simply drops events that are scheduled too early.
 * The `replace` policy will attempt to replace the value of the event
 * that it preceded it. Unless the preceding event has already been
 * handled, its gets assigned the value of the new event. If the
 * preceding event has already been popped off the event queue, the
 * `defer` policy is fallen back to.
 */
typedef enum {defer, drop, replace} lf_spacing_policy_t;

 /* An enum that enables the C core library to
 * ignore freeing the void* inside a token if the void*
 * value is garbage collected by an external controller
 */
typedef enum {no=0, token_and_value, token_only} ok_to_free_t;


/**
 * Status of a given port at a given logical time.
 * 
 * If the value is 'present', it is an indicator that the port is present at the given logical time.
 * If the value is 'absent', it is an indicator that the port is absent at the given logical time.
 * If the value is 'unknown', it is unknown whether the port is present or absent (e.g., in a distributed application).
 * 
 * @note For non-network ports, unknown is unused.
 * @note The absent and present fields need to be compatible with false and true
 *  respectively because for non-network ports, the status can either be present
 *  or absent (no possibility of unknown).
 */
typedef enum {absent = false, present = true, unknown} port_status_t;

/**
 * The flag OK_TO_FREE is used to indicate whether
 * the void* in toke_t should be freed or not.
 */ 
#ifdef __GARBAGE_COLLECTED
#define OK_TO_FREE token_only
#else
#define OK_TO_FREE token_and_value
#endif

/**
 * Handles for scheduled triggers. These handles are returned
 * by schedule() functions. The intent is that the handle can be
 * used to cancel a future scheduled event, but this is not
 * implemented yet.
 */
typedef int handle_t;

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
 * Global STP offset uniformly applied to advancement of each 
 * time step in federated execution. This can be retrieved in 
 * user code by calling get_stp_offset() and adjusted by 
 * calling set_stp_offset(interval_t offset).
 */
extern interval_t _lf_global_time_STP_offset;

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
typedef struct lf_token_t {
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
    ok_to_free_t ok_to_free;
    /** For recycling, a pointer to the next token in the recycling bin. */
    struct lf_token_t* next_free;
} lf_token_t;

/** A struct with a pointer to a lf_token_t and an _is_present variable
 *  for use to initialize actions in start_time_step().
 */
typedef struct token_present_t {
    lf_token_t** token;
    port_status_t* status; // FIXME: This structure is used to present the status of tokens
                           // for both ports and actions.
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
    int number;    // The number of the reaction in the reactor (0 is the first reaction).
    index_t index; // Inverse priority determined by dependency analysis. INSTANCE.
    unsigned long long chain_id; // Binary encoding of the branches that this reaction has upstream in the dependency graph. INSTANCE.
    size_t pos;       // Current position in the priority queue. RUNTIME.
    reaction_t* last_enabling_reaction; // The last enabling reaction, or NULL if there is none. Used for optimization. INSTANCE.
    int num_outputs;  // Number of outputs that may possibly be produced by this function. COMMON.
    bool** output_produced;   // Array of pointers to booleans indicating whether outputs were produced. COMMON.
    int* triggered_sizes;     // Pointer to array of ints with number of triggers per output. INSTANCE.
    trigger_t ***triggers;    // Array of pointers to arrays of pointers to triggers triggered by each output. INSTANCE.
    bool running;             // Indicator that this reaction has already started executing. RUNTIME.
    interval_t deadline;      // Deadline relative to the time stamp for invocation of the reaction. INSTANCE.
    bool is_STP_violated;     // Indicator of STP violation in one of the input triggers to this reaction. default = false.
                              // Value of True indicates to the runtime that this reaction contains trigger(s)
                              // that are triggered at a later logical time that was originally anticipated.
                              // Currently, this is only possible if logical
                              // connections are used in a decentralized federated
                              // execution. COMMON.
    reaction_function_t deadline_violation_handler; // Deadline violation handler. COMMON.
    reaction_function_t STP_handler;   // STP handler. Invoked when a trigger to this reaction
                                       // was triggered at a later logical time than originally
                                       // intended. Currently, this is only possible if logical
                                       // connections are used in a decentralized federated
                                       // execution. COMMON.
    bool is_a_control_reaction; // Indicates whether this reaction is a control reaction. Control
                                // reactions will not set ports or actions and don't require scheduling
                                // any output reactions. Default is false.
    char* name;                 // If logging is set to LOG or higher, then this will
                                // point to the full name of the reactor followed by
    							// the reaction number.
};

/** Typedef for event_t struct, used for storing activation records. */
typedef struct event_t event_t;

/** Event activation record to push onto the event queue. */
struct event_t {
    instant_t time;           // Time of release.
    trigger_t* trigger;       // Associated trigger, NULL if this is a dummy event.
    size_t pos;               // Position in the priority queue.
    lf_token_t* token;        // Pointer to the token wrapping the value.
    bool is_dummy;            // Flag to indicate whether this event is merely a placeholder or an actual event.
#ifdef FEDERATED
    tag_t intended_tag;       // The intended tag.
#endif
    event_t* next;            // Pointer to the next event lined up in superdense time.
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
    lf_token_t* token;           // Pointer to a token wrapping the payload (or NULL if there is none).
    bool is_physical;         // Indicator that this denotes a physical action.
    event_t* last;            // Pointer to the last event that was scheduled for this action.
    lf_spacing_policy_t policy;          // Indicates which policy to use when an event is scheduled too early.
    size_t element_size;      // The size of the payload, if there is one, zero otherwise.
                              // If the payload is an array, then this is the size of an element of the array.
    port_status_t status;     // Determines the status of the port at the current logical time. Therefore, this
                              // value needs to be reset at the beginning of each logical time.
                              //
                              // This status is especially needed for the distributed execution because the receiver logic will need 
                              // to know what it should do if it receives a message with 'intended tag = current tag' from another 
                              // federate. 
                              // - If status is 'unknown', it means that the federate has still no idea what the status of 
                              //   this port is and thus has refrained from executing any reaction that has that port as its input.
                              //   This means that the receiver logic can directly inject the triggered reactions into the reaction
                              //   queue at the current logical time.
                              // - If the status is absent, it means that the federate has assumed that the port is 'absent' 
                              //   for the current logical time. Therefore, receiving a message with 'intended tag = current tag'
                              //   is an error that should be handled, for example, as a violation of the STP offset in the decentralized 
                              //   coordination. 
                              // - Finally, if status is 'present', then this is an error since multiple 
                              //   downstream messages have been produced for the same port for the same logical time.
#ifdef FEDERATED
    tag_t last_known_status_tag;        // Last known status of the port, either via a timed message, a port absent, or a
                                        // TAG from the RTI.
    bool is_a_control_reaction_waiting; // Indicates whether at least one control reaction is waiting for this trigger
                                        // if it belongs to a network input port. Must be false by default.
    tag_t intended_tag;                 // The amount of discrepency in logical time between the original intended
                                        // trigger time of this trigger and the actual trigger time. This currently
                                        // can only happen when logical connections are used using a decentralized coordination
                                        // mechanism (@see https://github.com/icyphy/lingua-franca/wiki/Logical-Connections).
    instant_t physical_time_of_arrival; // The physical time at which the message has been received on the network according to the local clock.
                                        // Note: The physical_time_of_arrival is only passed down one level of the hierarchy. Default: NEVER.
#endif
};
//  ======== Function Declarations ========  //

/**
 * Return the time of the start of execution in nanoseconds.
 * This is both the starting physical and starting logical time.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent.
 * @return A time instant.
 */
instant_t get_start_time();

/**
 * Return the global STP offset on advancement of logical
 * time for federated execution.
 */
interval_t get_stp_offset();

/**
 * Set the global STP offset on advancement of logical
 * time for federated execution.
 * 
 * @param offset A positive time value to be applied
 *  as the STP offset.
 */
void set_stp_offset(interval_t offset);

/**
 * Print a snapshot of the priority queues used during execution
 * (for debugging).
 */
void print_snapshot();

/**
 * Request a stop to execution as soon as possible.
 * In a non-federated execution, this will occur
 * at the conclusion of the current logical time.
 * In a federated execution, it will likely occur at
 * a later logical time determined by the RTI so that
 * all federates stop at the same logical time.
 */
void request_stop();

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
handle_t __schedule(trigger_t* trigger, interval_t delay, lf_token_t* token);

/**
 * Function (to be code generated) to schedule timers.
 */
void __initialize_timers();

/**
 * Function (to be code generated) to trigger startup reactions.
 */
void __trigger_startup_reactions();


/**
 * Function (to be code generated) to terminate execution.
 * This will be invoked after all shutdown actions have completed.
 */
void terminate_execution();

/**
 * Function (to be code generated) to trigger shutdown reactions.
 */
bool __trigger_shutdown_reactions();

/**
 * Create a new token and initialize it.
 * The value pointer will be NULL and the length will be 0.
 * @param element_size The size of an element carried in the payload or
 *  0 if there is no payload.
 * @return A pointer to a new or recycled lf_token_t struct.
 */
lf_token_t* create_token(size_t element_size);

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
handle_t _lf_schedule_int(void* action, interval_t extra_delay, int value);

/**
 * Get a new event. If there is a recycled event available, use that.
 * If not, allocate a new one. In either case, all fields will be zero'ed out.
 */
event_t* _lf_get_new_event();

/**
 * Recycle the given event.
 * Zero it out and pushed it onto the recycle queue.
 */
void _lf_recycle_event(event_t* e);

/**
 * Schedule events at a specific tag (time, microstep), provided
 * that the tag is in the future relative to the current tag.
 * The input time values are absolute.
 * 
 * If there is an event found at the requested tag, the payload
 * is replaced and 0 is returned.
 *
 * @param trigger The trigger to be invoked at a later logical time.
 * @param tag Logical tag of the event
 * @param token The token wrapping the payload or NULL for no payload.
 * 
 * @return 1 for success, 0 if no new event was scheduled (instead, the payload was updated), or -1 for error.
 */
int _lf_schedule_at_tag(trigger_t* trigger, tag_t tag, lf_token_t* token);

/**
 * Create a dummy event to be used as a spacer in the event queue.
 */
event_t* _lf_create_dummy_event(trigger_t* trigger, instant_t time, event_t* next, unsigned int offset);

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
handle_t _lf_schedule_token(void* action, interval_t extra_delay, lf_token_t* token);

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
handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, int length);

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
handle_t _lf_schedule_copy(void* action, interval_t offset, void* value, int length);

/**
 * For a federated execution, send a STOP_REQUEST message
 * to the RTI.
 */
void _lf_fd_send_stop_request_to_rti();

/**
 * Advance from the current tag to the next. If the given next_time is equal to
 * the current time, then increase the microstep. Otherwise, update the current
 * time and set the microstep to zero.
 */ 
void _lf_advance_logical_time(instant_t next_time);

/**
 * If multithreaded, notify workers that something has changed
 * on the reaction_q. Otherwise, do nothing.
 */
void _lf_notify_workers();

/**
 * If multithreaded and the reaction is blocked by
 * a currently executing reaction, return true.
 * Otherwise, return false.
 * @param reaction The reaction.
 */
bool _lf_is_blocked_by_executing_reaction();

//  ******** Global Variables ********  //

/**
 * The number of worker threads for threaded execution.
 * By default, execution is not threaded and this variable will have value 0,
 * meaning that the execution is not threaded.
 */
extern unsigned int _lf_number_of_threads;

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

#include "trace.h"

#endif /* REACTOR_H */
/** @} */
