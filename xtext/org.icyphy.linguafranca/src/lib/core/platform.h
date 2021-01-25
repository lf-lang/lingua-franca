/* Platform API support for the C target of Lingua Franca. */

/*************
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
***************/

/** Platform API support for the C target of Lingua Franca.
 *  
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
   // Windows platforms
   #error "Windows not supported"
   #ifdef _WIN64
      //define something for Windows (64-bit only)
   #else
      //define something for Windows (32-bit only)
   #endif
#elif __APPLE__
    // Apple platforms
    #if __STDC_VERSION__ < 201112L || defined (__STDC_NO_THREADS__) // (Not C++11 or later) or no threads support
    #include "platform/lf_POSIX_support.c"
    #else
    #include "platform/lf_C11_threads_support.c"
    #endif
    #if TARGET_IPHONE_SIMULATOR
         // iOS Simulator
    #elif TARGET_OS_IPHONE
        // iOS device
    #elif TARGET_OS_MAC || PLATFORM_NAME == "Mac OS"
        // Other kinds of Mac OS
    #else
    #warning "Unknown Apple platform"
    #endif
#elif __linux__
    // Linux
    #if __STDC_VERSION__ < 201112L || defined (__STDC_NO_THREADS__) // (Not C++11 or later) or no threads support
    #include "platform/lf_POSIX_support.c"
    #else
    #include "platform/lf_C11_threads_support.c"
    #endif
#elif __unix__ // all unices not caught above
    // Unix
    #include "platform/lf_POSIX_support.c"
#elif defined(_POSIX_VERSION)
    // POSIX
    #include "platform/lf_POSIX_support.c"
#elif defined(__riscv) || defined(__riscv__) 
    // RISC-V (see https://github.com/riscv/riscv-toolchain-conventions)
    #error "RISC-V not supported"
#else
#error "Platform not supported"
#endif

#define LF_TIMEOUT __LF_TIMEOUT

typedef __lf_mutex_t lf_mutex_t;
typedef __lf_cond_t lf_cond_t;
typedef __lf_thread_t lf_thread_t;

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
 * Wait for condition variable cond to be signaled or broadcast.
 * mutex is assumed to be locked before.
 */
extern int lf_cond_wait(lf_cond_t* cond, lf_mutex_t* mutex);

/** 
 * Block current thread on the condition variable until condition variable
 * pointed by __COND is signaled or time pointed by __TIME_POINT is
 * reached.
 * 
 * @return 0 on success and LF_TIMEOUT on timeout.
 */
extern int lf_cond_timedwait(lf_cond_t* cond, lf_mutex_t* mutex, instant_t absolute_time);

#endif // PLATFORM_H