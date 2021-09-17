/* Windows API support for the C target of Lingua Franca. */

/*************
Copyright (c) 2021, The University of California at Berkeley.

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

/** Windows API support for the C target of Lingua Franca.
 *  
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 * 
 * All functions return 0 on success.
 * 
 * @see https://gist.github.com/Soroosh129/127d1893fa4c1da6d3e1db33381bb273
 */

#include <windows.h>
#include <process.h>
#include <errno.h>
#include "lf_windows_support.h"
#include "../platform.h"
#include <time.h>

/**
 * Offset to _LF_CLOCK that would convert it
 * to epoch time.
 * For CLOCK_REALTIME, this offset is always zero.
 * For CLOCK_MONOTONIC, it is the difference between those
 * clocks at the start of the execution.
 */
LARGE_INTEGER _lf_epoch_offset;

/**
 * Indicates wether or not the underlying hardware
 * supports Windows' high-resolution counter. It should
 * always be supported for Windows Xp and later.
 */
int _lf_use_performance_counter = 0;

/**
 * The denominator to convert the performance counter
 * to nanoseconds.
 */
double _lf_frequency_to_ns = 1;

#define BILLION 1000000000

/**
 * Calculate the necessary offset to bring _LF_CLOCK in parity with the epoch
 * time.
 */
void calculate_epoch_offset() {
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;

    _lf_epoch_offset = t;
}

/**
 * Initialize the LF clock.
 */
void lf_initialize_clock() {    
    // Check if the performance counter is available
    LARGE_INTEGER performanceFrequency;
    _lf_use_performance_counter = QueryPerformanceFrequency(&performanceFrequency);
    if (_lf_use_performance_counter) {
        QueryPerformanceCounter(&_lf_epoch_offset);
        _lf_frequency_to_ns = (double)performanceFrequency.QuadPart / 1000000000.;
    } else {
        error_print(
            "High resolution performance counter is not supported on this machine.");
        calculate_epoch_offset();
        _lf_frequency_to_ns = 0.01;
    }
}

#ifdef NUMBER_OF_WORKERS
#if __STDC_VERSION__ < 201112L || defined (__STDC_NO_THREADS__) // (Not C++11 or later) or no threads support

/**
 * Create a new thread, starting with execution of lf_thread
 * getting passed arguments. The new handle is stored in thread.
 * 
 * @return 0 on success, 1 otherwise.
 */
int lf_thread_create(_lf_thread_t* thread, void *(*lf_thread) (void *), void* arguments) {
    uintptr_t handle = _beginthread(lf_thread, 0, arguments);
	thread = (HANDLE)handle;
	if(thread == (HANDLE)-1){
		return 1;
	}else{
		return 0;
	}
}

/**
 * Make calling thread wait for termination of the thread.  The
 * exit status of the thread is stored in thread_return, if thread_return
 * is not NULL.
 * 
 * @return 0 on success, EINVAL otherwise.
 */
int lf_thread_join(_lf_thread_t thread, void** thread_return) {    
	DWORD retvalue = WaitForSingleObject(thread, INFINITE);
	if(retvalue == WAIT_OBJECT_0){
		return 0;
	}else{
		return EINVAL;
	}
}

/**
 * Initialize a critical section.
 * 
 * @return 0 on success, 1 otherwise.
 */
int lf_mutex_init(_lf_critical_section_t* critical_section) {
    // Set up a recursive mutex
	InitializeCriticalSection((CRITICAL_SECTION*)critical_section);
	if(critical_section != NULL){
		return 0;
	}else{
		return 1;
	}
}

/** 
 * Lock a critical section.
 * 
 * From https://docs.microsoft.com/en-us/windows/win32/api/synchapi/nf-synchapi-entercriticalsection:
 *    "This function can raise EXCEPTION_POSSIBLE_DEADLOCK if a wait operation on the critical section times out.
 *     The timeout interval is specified by the following registry value: 
 *     HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\Session Manager\CriticalSectionTimeout.
 *     Do not handle a possible deadlock exception; instead, debug the application."
 * 
 * @return 0
 */
int lf_mutex_lock(_lf_critical_section_t* critical_section) {
    // The following Windows API does not return a value. It can
    // raise a EXCEPTION_POSSIBLE_DEADLOCK. See synchapi.h.
	EnterCriticalSection((CRITICAL_SECTION*)critical_section);
    return 0;
}

/** 
 * Leave a critical_section.
 * 
 * @return 0
 */
int lf_mutex_unlock(_lf_critical_section_t* critical_section) {
    // The following Windows API does not return a value.
    LeaveCriticalSection((CRITICAL_SECTION*)critical_section);
    return 0;
}

/** 
 * Initialize a conditional variable.
 * 
 * @return 0
 */
int lf_cond_init(_lf_cond_t* cond) {
    // The following Windows API does not return a value.
    InitializeConditionVariable((CONDITION_VARIABLE*)cond);
    return 0;
}

/** 
 * Wake up all threads waiting for condition variable cond.
 * 
 * @return 0
 */
int lf_cond_broadcast(_lf_cond_t* cond) {
    // The following Windows API does not return a value.
    WakeAllConditionVariable((CONDITION_VARIABLE*)cond);
    return 0;
}

/** 
 * Wake up one thread waiting for condition variable cond.
 * 
 * @return 0
 */
int lf_cond_signal(_lf_cond_t* cond) {
    // The following Windows API does not return a value.
    WakeConditionVariable((CONDITION_VARIABLE*)cond);
    return 0;
}

/** 
 * Wait for condition variable "cond" to be signaled or broadcast.
 * "mutex" is assumed to be locked before.
 * 
 * @return 0 on success, 1 otherwise.
 */
int lf_cond_wait(_lf_cond_t* cond, _lf_critical_section_t* critical_section) {
    // According to synchapi.h, the following Windows API returns 0 on failure,
    // and non-zero on success.
    int return_value =
     (int)SleepConditionVariableCS(
         (CONDITION_VARIABLE*)cond, 
         (CRITICAL_SECTION*)critical_section, 
         INFINITE
     );
     switch (return_value) {
        case 0:
            // Error
            return 1;
            break;
        
        default:
            // Success
            return 0;
            break;
     }
}

/** 
 * Block current thread on the condition variable until condition variable
 * pointed by "cond" is signaled or time pointed by "absolute_time_ns" in
 * nanoseconds is reached.
 * 
 * @return 0 on success and LF_TIMEOUT on timeout, 1 otherwise.
 */
int lf_cond_timedwait(_lf_cond_t* cond, _lf_critical_section_t* critical_section, instant_t absolute_time_ns) {
    // Convert the absolute time to a relative time
    DWORD relative_time_ms = (absolute_time_ns - get_physical_time())/1000000LL;

    int return_value =
     (int)SleepConditionVariableCS(
         (CONDITION_VARIABLE*)cond, 
         (CRITICAL_SECTION*)critical_section, 
         relative_time_ms
     );
     switch (return_value) {
        case 0:
            // Error
            if (GetLastError() == ERROR_TIMEOUT) {
                return _LF_TIMEOUT;
            }
            return 1;
            break;
        
        default:
            // Success
            return 0;
            break;
     }
}


#else
#include "lf_C11_threads_support.c"
#endif
#endif

/**
 * Fetch the value of _LF_CLOCK (see lf_windows_support.h) and store it in t.
 * The timestamp value in 't' will always be epoch time, which is the number of
 * nanoseconds since January 1st, 1970.
 *
 * @return 0 for success, or -1 for failure. In case of failure, errno will be
 *  set to EINVAL or EFAULT.
 */
int lf_clock_gettime(instant_t* t) {
    // Adapted from gclib/GResUsage.cpp 
    // (https://github.com/gpertea/gclib/blob/8aee376774ccb2f3bd3f8e3bf1c9df1528ac7c5b/GResUsage.cpp)
    // License: https://github.com/gpertea/gclib/blob/master/LICENSE.txt
    int result = -1;
    if (t == NULL) {
        // The t argument address references invalid memory
        errno = EFAULT;
        return result;
    }
    LARGE_INTEGER windows_time;
    FILETIME f;

    if (_lf_use_performance_counter) {
        QueryPerformanceCounter(&windows_time);
    }
    else {
        GetSystemTimeAsFileTime(&f);
        windows_time.QuadPart = f.dwHighDateTime;
        windows_time.QuadPart <<= 32;
        windows_time.QuadPart |= f.dwLowDateTime;
    }

    windows_time.QuadPart -= _lf_epoch_offset.QuadPart;
    *t = (instant_t)((double)windows_time.QuadPart / _lf_frequency_to_ns);
    return (0);
}

/**
 * Pause execution for a number of nanoseconds.
 *
 * @return 0 for success, or -1 for failure. In case of failure, errno will be
 *  set to 
 *   - EINTR: The sleep was interrupted by a signal handler
 *   - EINVAL: All other errors
 */
int lf_nanosleep(instant_t requested_time) {
    /* Declarations */
	HANDLE timer;	/* Timer handle */
	LARGE_INTEGER li;	/* Time defintion */
	/* Create timer */
	if(!(timer = CreateWaitableTimer(NULL, TRUE, NULL)))
		return FALSE;
	/* Set timer properties */
	li.QuadPart = -requested_time;
	if(!SetWaitableTimer(timer, &li, 0, NULL, NULL, FALSE)){
		CloseHandle(timer);
		return FALSE;
	}
	/* Start & wait for timer */
	WaitForSingleObject(timer, INFINITE);
	/* Clean resources */
	CloseHandle(timer);
	/* Slept without problems */
	return TRUE;
}