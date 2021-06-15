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
#include "lf_windows_support.h"
#include "../platform.h"


#include "lf_unix_clock_support.c"

/**
 * Offset to _LF_CLOCK that would convert it
 * to epoch time.
 * For CLOCK_REALTIME, this offset is always zero.
 * For CLOCK_MONOTONIC, it is the difference between those
 * clocks at the start of the execution.
 */
interval_t _lf_epoch_offset = 0LL;

/**
 * Initialize the LF clock.
 */
void lf_initialize_clock() {
    // FIXME: We don't strictly need to convert Windows clock to epoch. It is
    // done here for better uniformity across platforms. This requires access to
    // an epoch-based clock to begin with, which many baremetal target platforms
    // will most likely not have.
    _lf_epoch_offset = calculate_epoch_offset(_LF_CLOCK);
}

/**
 * Calculate the necessary offset to bring _LF_CLOCK in parity with the epoch
 * time.
 */
void calculate_epoch_offset() {
    if (_LF_CLOCK == CLOCK_REALTIME) {
        // Set the epoch offset to zero (see tag.h)
        _lf_epoch_offset = 0LL;
    } else {
        // Initialize _lf_epoch_offset to the difference between what is
        // reported by whatever clock LF is using (e.g. CLOCK_MONOTONIC) and
        // what is reported by CLOCK_REALTIME.
        struct timespec physical_clock_snapshot, real_time_start;

        clock_gettime(_LF_CLOCK, &physical_clock_snapshot);
        instant_t physical_clock_snapshot_ns = physical_clock_snapshot.tv_sec * BILLION + physical_clock_snapshot.tv_nsec;

        clock_gettime(CLOCK_REALTIME, &real_time_start);
        instant_t real_time_start_ns = real_time_start.tv_sec * BILLION + real_time_start.tv_nsec;

        _lf_epoch_offset = real_time_start_ns - physical_clock_snapshot_ns;
    }
    LOG_PRINT("Clock sync: Initial epoch offset set to %lld.", _lf_epoch_offset);
}

#if __STDC_VERSION__ < 201112L || defined (__STDC_NO_THREADS__) // (Not C++11 or later) or no threads support

NtDelayExecution_t *NtDelayExecution = NULL;
NtQueryPerformanceCounter_t *NtQueryPerformanceCounter = NULL;
NtQuerySystemTime_t *NtQuerySystemTime = NULL;

/**
 * Create a new thread, starting with execution of lf_thread
 * getting passed arguments. The new handle is stored in thread.
 * 
 * @return 0 on success, 1 otherwise.
 */
int lf_thread_create(_lf_thread_t* thread, void *(*lf_thread) (void *), void* arguments) {
    uintptr_t handle = _beginthread((windows_thread)lf_thread,0,arg);
	thread->handle = (HANDLE)handle;
	if(thread->handle == (HANDLE)-1){
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
	DWORD retvalue = WaitForSingleObject(thread.handle,INFINITE);
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

/**
 * Fetch the value of _LF_CLOCK (see lf_windows_support.h) and store it in tp.
 * The timestamp value in 'tp' will always be epoch time, which is the number of
 * nanoseconds since January 1st, 1970.
 *
 * @return 0 for success, or -1 for failure. In case of failure, errno will be
 *  set to EINVAL.
 */
int lf_clock_gettime(_lf_time_spec_t* tp) {
    int result = -1;
    int days_from_1601_to_1970 = 134774 /* there were no leap seconds during this time, so life is easy */;
    long long timestamp, counts, counts_per_sec;
    switch (_LF_CLOCK) {
        case CLOCK_REALTIME:
            NtQuerySystemTime((PLARGE_INTEGER)&timestamp);
            timestamp -= days_from_1601_to_1970 * 24LL * 60 * 60 * 1000 * 1000 * 10;
            tp->tv_sec = (time_t)(timestamp / (BILLION / 100));
            tp->tv_nsec = (long)((timestamp % (BILLION / 100)) * 100);
            result = 0;
            break;
        case CLOCK_MONOTONIC:
            if ((*NtQueryPerformanceCounter)((PLARGE_INTEGER)&counts, (PLARGE_INTEGER)&counts_per_sec) == 0) {
                tp->tv_sec = counts / counts_per_sec;
                tp->tv_nsec = (long)((counts % counts_per_sec) * BILLION / counts_per_sec);
                result = 0;
            } else {
                errno = EINVAL;
                result = -1;
            }
            break;
        default:
            errno = EINVAL;
            result = -1;
            break;
    }
    // Adjust the clock by the epoch offset, so epoch time is always reported.
    return result + _lf_epoch_offset;
}

/**
 * Pause execution for a number of nanoseconds.
 *
 * @return 0 for success, or -1 for failure. In case of failure, errno will be
 *  set to 
 *   - EINTR: The sleep was interrupted by a signal handler
 *   - EINVAL: All other errors
 */
int lf_nanosleep(const _lf_time_spec_t* requested_time, _lf_time_spec_t* remaining) {
    unsigned char alertable = remaining ? 1 : 0;
    long long duration = -(requested_time->tv_sec * (BILLION / 100) + requested_time->tv_nsec / 100);
    NTSTATUS status = (*NtDelayExecution)(alertable, (PLARGE_INTEGER)&duration);
    int result = status == 0 ? 0 : -1;
    if (alertable) {
        if (status < 0) {
            errno = EINVAL;
        } else if (status > 0 && lf_clock_gettime(clk_id, remaining) == 0) {
            errno = EINTR;
        }
    }
    return result;
}

#else
#include "lf_C11_threads_support.c"
#endif