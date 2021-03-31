/* Platform API support for the C target of Lingua Franca. */

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

/** Platform API support for the C target of Lingua Franca.
 *  
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
   // Windows platforms
   #include "platform/lf_windows_support.h"
#elif __APPLE__
    // Apple platforms
    #include "platform/lf_macos_support.h"
#elif __linux__
    // Linux
    #include "platform/lf_linux_support.h"
#elif __unix__ // all unices not caught above
    // Unix
    #include "platform/lf_POSIX_threads_support.h"
#elif defined(_POSIX_VERSION)
    // POSIX
    #include "platform/lf_POSIX_threads_support.h"
#elif defined(__riscv) || defined(__riscv__) 
    // RISC-V (see https://github.com/riscv/riscv-toolchain-conventions)
    #error "RISC-V not supported"
#else
#error "Platform not supported"
#endif

#ifdef NUMBER_OF_WORKERS
#define LF_TIMEOUT _LF_TIMEOUT

typedef _lf_mutex_t lf_mutex_t;          // Type to hold handle to a mutex
typedef _lf_cond_t lf_cond_t;            // Type to hold handle to a condition variable
typedef _lf_thread_t lf_thread_t;        // Type to hold handle to a thread
#endif

typedef _lf_time_spec_t lf_time_spec_t;  // Type to hold time in a traditional {second, nanosecond} POSIX format
typedef _lf_clock_t lf_clock_t;          // Type to hold a clock identifier (e.g., CLOCK_REALTIME on POSIX)

#ifdef NUMBER_OF_WORKERS

/**
 * Create a new thread, starting with execution of lf_thread
 * getting passed arguments. The new handle is stored in thread_id.
 */
extern int lf_thread_create(lf_thread_t* thread, void *(*lf_thread) (void *), void* arguments);

/**
 * Make calling thread wait for termination of the thread.  The
 * exit status of the thread is stored in thread_return, if thread_return
 * is not NULL.
 */
extern int lf_thread_join(lf_thread_t thread, void** thread_return);

/**
 * Initialize a mutex.
 */
extern int lf_mutex_init(lf_mutex_t* mutex);

/**
 * Lock a mutex.
 */
extern int lf_mutex_lock(lf_mutex_t* mutex);

/** 
 * Unlock a mutex.
 */
extern int lf_mutex_unlock(lf_mutex_t* mutex);


/** 
 * Initialize a conditional variable.
 */
extern int lf_cond_init(lf_cond_t* cond);

/** 
 * Wake up all threads waiting for condition variable cond.
 */
extern int lf_cond_broadcast(lf_cond_t* cond);

/** 
 * Wake up one thread waiting for condition variable cond.
 */
extern int lf_cond_signal(lf_cond_t* cond);

/** 
 * Wait for condition variable "cond" to be signaled or broadcast.
 * "mutex" is assumed to be locked before.
 */
extern int lf_cond_wait(lf_cond_t* cond, lf_mutex_t* mutex);

/** 
 * Block current thread on the condition variable until condition variable
 * pointed by "cond" is signaled or time pointed by "absolute_time_ns" in
 * nanoseconds is reached.
 * 
 * @return 0 on success and LF_TIMEOUT on timeout.
 */
extern int lf_cond_timedwait(lf_cond_t* cond, lf_mutex_t* mutex, long long absolute_time_ns);

#endif

/**
 * Fetch the value of clk_id and store it in tp.
 */
extern int lf_clock_gettime(lf_clock_t clk_id, lf_time_spec_t* tp);

/**
 * Pause execution for a number of nanoseconds.
 */
extern int lf_nanosleep(lf_clock_t clk_id, const lf_time_spec_t* requested_time, lf_time_spec_t* remaining);

#endif // PLATFORM_H