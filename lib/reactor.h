/*
 FIXME: License, copyright, authors.
 */

/**
 * @file  reactor.h
 * @brief Reactor core macros, type definitions, and function declarations.
 *
 * @{
 */

#ifndef REACTOR_H
#define REACTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include "pqueue.h"

//  ======== Macros ========  //

#define BILLION 1000000000LL

// FIXME: May want these to application dependent, hence code generated.
#define INITIAL_EVENT_QUEUE_SIZE 10
#define INITIAL_REACT_QUEUE_SIZE 10

/** Conversion of time to nanoseconds. */
#define NSEC(t) t ## LL
#define USEC(t) (t * 1000LL)
#define MSEC(t) (t * 1000000LL)
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

//  ======== Type definitions ========  //

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
typedef void(*reaction_function_t)(void*, void*);

/** Reaction activation record to push onto the reaction queue. */
typedef struct reaction_t {
  reaction_function_t function;
  void* this;    // Pointer to a struct with the reactor's state.
  void* args;    // Pointer to a struct with accessible ports and actions.
  index_t index; // Index determined by topological sort.
  size_t pos;    // Current position in the priority queue.
} reaction_t;

/** Reaction activation record to push onto the reaction queue. */
typedef struct {
	reaction_t** reactions;  // Reactions sensitive to this trigger.
	int number_of_reactions; // Number of reactions sensitive to this trigger.
	interval_t offset;       // For an action, this will be a minimum delay.
	interval_t period;       // For periodic timers (not for actions).
} trigger_t;

/** Event activation record to push onto the event queue. */
typedef struct event_t {
  instant_t time;     // Time of release.
  trigger_t* trigger; // Associated trigger.
  size_t pos;         // Position in the priority queue.
} event_t;

//  ======== Function Declarations ========  //

/**
 * Function to get the current logical time.
 * @return a time instant
 */
instant_t get_logical_time(); // FIXME: this should not be global

/** 
 * Generated function that produces a table containing all triggers
 * (i.e., inputs, timers, and actions).
 */
void __initialize_trigger_table();

/** 
 * Internal version of the schedule() function, used by generated 
 * __start_timers() function. 
 * @param trigger the action or timer to be triggered
 * @param delay offset of the event release
 */
handle_t __schedule(trigger_t* trigger, interval_t delay);

/**
 * Function (to be code generated) to start timers.
 */
void __start_timers();

/**
 * External version of schedule, callable from within reactors.
 * @param trigger the action or timer to be triggered
 * @param delay extra offset of the event release
 */
handle_t schedule(trigger_t* trigger, interval_t extra_delay);

/**
 * Function to set the value of an output. This is called from within
 * a reaction.
 * @param output_index index of output port into the connection_table
 * @param value pointer to the value to be sent
 */
void set(int output_index, void* value);

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
typedef int clockid_t;
int clock_gettime(clockid_t clk_id, struct timespec *tp);
int nanosleep(const struct timespec *req, struct timespec *rem);
#endif
//  ******** End Windows Support ********  //

#endif /* REACTOR_H */
/** @} */