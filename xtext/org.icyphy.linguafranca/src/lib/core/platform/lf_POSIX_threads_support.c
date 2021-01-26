/* POSIX API support for the C target of Lingua Franca. */

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

/** POSIX API support for the C target of Lingua Franca.
 *  
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 * 
 * All functions return 0 on success.
 */

#ifndef LF_POSIX_SUPPORT
#define LF_POSIX_SUPPORT


#include <pthread.h>

typedef pthread_mutex_t __lf_mutex_t;
typedef pthread_cond_t __lf_cond_t;
typedef pthread_t __lf_thread_t;
typedef struct timespec __lf_time_spec_t;
typedef clockid_t __lf_clock_t;

#define __LF_TIMEOUT ETIMEDOUT

/**
 * Create a new thread, starting with execution of lf_thread
 * getting passed arguments. The new handle is stored in thread.
 */
int lf_thread_create(__lf_thread_t* thread, void *(*lf_thread) (void *), void* arguments) {
    return pthread_create((pthread_t*)thread, NULL, lf_thread, arguments);
}

/**
 * Make calling thread wait for termination of the thread.  The
 * exit status of the thread is stored in thread_return, if thread_return
 * is not NULL.
 */
int lf_thread_join(__lf_thread_t thread, void** thread_return) {
    return pthread_join((pthread_t)thread, thread_return);
}

/**
 * Initialize a mutex.
 */
int lf_mutex_init(__lf_mutex_t* mutex) {
    // Set up a recursive mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    return pthread_mutex_init((pthread_mutex_t*)mutex, &attr);
}

/* Lock a mutex.  */
int lf_mutex_lock(__lf_mutex_t* mutex) {
    return pthread_mutex_lock((pthread_mutex_t*)mutex);
}

/* Unlock a mutex.  */
int lf_mutex_unlock(__lf_mutex_t* mutex) {
    return pthread_mutex_unlock((pthread_mutex_t*)mutex);
}

/* Initialize a conditional variable. */
int lf_cond_init(__lf_cond_t* cond) {
    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    // Limit the scope of the condition variable to this process (default)
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_PRIVATE);
    return pthread_cond_init(cond, &cond_attr);
}

/* Wake up all threads waiting for condition variable cond.  */
int lf_cond_broadcast(__lf_cond_t* cond) {
    return pthread_cond_broadcast((pthread_cond_t*)cond);
}

/* Wake up one thread waiting for condition variable cond.  */
int lf_cond_signal(__lf_cond_t* cond) {
    return pthread_cond_signal((pthread_cond_t*)cond);
}

/* Wait for condition variable COND to be signaled or broadcast.
   MUTEX is assumed to be locked before. */
int lf_cond_wait(__lf_cond_t* cond, __lf_mutex_t* mutex) {
    return pthread_cond_wait((pthread_cond_t*)cond, (pthread_mutex_t*)mutex);
}

/** 
 * Block current thread on the condition variable until condition variable
 * pointed by __COND is signaled or time pointed by __TIME_POINT is
 * reached.
 * 
 * @return 0 on success and LF_TIMEOUT on timeout.
 */
int lf_cond_timedwait(__lf_cond_t* cond, __lf_mutex_t* mutex, instant_t absolute_time_ns) {
    // Convert the absolute time to a timespec.
    // timespec is seconds and nanoseconds.
    struct timespec timespec_absolute_time
            = {(time_t)absolute_time / BILLION, (long)absolute_time % BILLION};
    return pthread_cond_timedwait(
                        (pthread_cond_t*)cond,
                        (pthread_mutex_t*)mutex,
                        &timespec_absolute_time);
}

#endif // LF_POSIX_SUPPORT