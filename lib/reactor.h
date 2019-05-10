#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include "pqueue.h"

#define BILLION 1000000000LL

// FIXME: May want these to application dependent, hence code generated.
#define INITIAL_TAG_QUEUE_SIZE 10
#define INITIAL_INDEX_QUEUE_SIZE 10

// ********* Type definitions included for all actors.
// WARNING: If this code is used after about the year 2262,
// then representing time as a long long will be insufficient.
typedef long long instant_t;

// Intervals of time.
typedef long long interval_t;

// Topological sort index for reactions.
typedef pqueue_pri_t index_t;

// Handles for scheduled triggers.
typedef int handle_t;

// Reaction function type
typedef void(*reaction_function_t)(void);

// A reaction.
typedef struct reaction_t {
  reaction_function_t function;
  index_t index;
  size_t pos; // Used by priority queue.
} reaction_t;

typedef struct {
	reaction_t** reactions;
	int number_of_reactions;
	interval_t offset; // For an action, this will be a minimum delay.
	interval_t period;
} trigger_t;

// Event to put in the event queue.
typedef struct event_t {
  instant_t time;
  trigger_t* trigger;
  size_t pos;         // position in the priority queue 
} event_t;

// Macros for conversion of time to nanoseconds.
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

long long get_logical_time();
// Function (to be code generated) to start timers.
void __start_timers();
// Internal version of schedule() function, used by generated __start_timers() function.
handle_t __schedule(trigger_t* trigger, interval_t delay);
// External version of schedule, callable from within reactors.
handle_t schedule(trigger_t* trigger, interval_t extra_delay);
// Generated function that produces a table containing all triggers (inputs, timers, and actions).
void __initialize_trigger_table();

// Function to set the value of an output. This is called from within
// a reaction, where the first argument is the index of an output port
// into the connection_table , and the second
// argument is a pointer to the value to be sent.
void set(int output_index, void* value);

// ********** Begin Windows Support
// Windows is not POSIX, so we include here compatibility definitions.
#if _WIN32 || WIN32
#pragma warning(disable: 4204 4255 4459 4710)
#ifdef  _M_X64
typedef long long intptr_t;
#else
typedef int intptr_t;
#endif
intptr_t __cdecl _loaddll(char *);
int __cdecl _unloaddll(intptr_t);
int (__cdecl * __cdecl _getdllprocaddr(intptr_t, char *, intptr_t))(void);
typedef long NTSTATUS;
typedef union _LARGE_INTEGER *PLARGE_INTEGER;
typedef NTSTATUS __stdcall NtDelayExecution_t(unsigned char Alertable, PLARGE_INTEGER Interval);
NtDelayExecution_t *NtDelayExecution = NULL;
typedef NTSTATUS __stdcall NtQueryPerformanceCounter_t(PLARGE_INTEGER PerformanceCounter, PLARGE_INTEGER PerformanceFrequency);
NtQueryPerformanceCounter_t *NtQueryPerformanceCounter = NULL;
typedef NTSTATUS __stdcall NtQuerySystemTime_t(PLARGE_INTEGER SystemTime); NtQuerySystemTime_t *NtQuerySystemTime = NULL;
typedef enum { CLOCK_REALTIME = 0 } clockid_t;
int clock_gettime(clockid_t clk_id, struct timespec *tp);
int nanosleep(const struct timespec *req, struct timespec *rem);
#endif
// ********** End Windows Support
