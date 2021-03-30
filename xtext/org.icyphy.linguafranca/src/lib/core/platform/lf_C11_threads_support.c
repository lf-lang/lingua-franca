/* C11 threads support for the C target of Lingua Franca. */

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

/** C11 threads support for the C target of Lingua Franca.
 *  
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#include "lf_C11_threads_support.h"
#include <stdlib.h>

/**
 * Create a new thread, starting with execution of lf_thread
 * getting passed arguments. The new handle is stored in thread.
 */
int lf_thread_create(_lf_thread_t* thread, void *(*lf_thread) (void *), void* arguments) {
    return thrd_create((thrd_t*)thread, (thrd_start_t)lf_thread, arguments);
}

/**
 * Make calling thread wait for termination of the thread.  The
 * exit status of the thread is stored in thread_return, if thread_return
 * is not NULL.
 */
int lf_thread_join(_lf_thread_t thread, void** thread_return) {
    thread_return = (void**)malloc(sizeof(void*));
    *thread_return= (int*)malloc(sizeof(int));
    return thrd_join((thrd_t)thread, (int*)*thread_return);
}

/**
 * Initialize a mutex.
 */
int lf_mutex_init(_lf_mutex_t* mutex) {
    // Set up a timed and recursive mutex (default behavior)
    return mtx_init((mtx_t*)mutex, mtx_timed | mtx_recursive);
}

/**
 * Lock a mutex.
 */
int lf_mutex_lock(_lf_mutex_t* mutex) {
    return mtx_lock((mtx_t*) mutex);
}

/** 
 * Unlock a mutex.
 */
int lf_mutex_unlock(_lf_mutex_t* mutex) {
    return mtx_unlock((mtx_t*) mutex);
}

/** 
 * Initialize a conditional variable.
 */
int lf_cond_init(_lf_cond_t* cond) {
    return cnd_init((cnd_t*)cond);
}

/** 
 * Wake up all threads waiting for condition variable cond.
 */
int lf_cond_broadcast(_lf_cond_t* cond) {
    return cnd_broadcast((cnd_t*)cond);
}

/** 
 * Wake up one thread waiting for condition variable cond.
 */
int lf_cond_signal(_lf_cond_t* cond) {
    return cnd_signal((cnd_t*)cond);
}

/** 
 * Wait for condition variable "cond" to be signaled or broadcast.
 * "mutex" is assumed to be locked before.
 */
int lf_cond_wait(_lf_cond_t* cond, _lf_mutex_t* mutex) {
    return cnd_wait((cnd_t*)cond, (mtx_t*)mutex);
}

/** 
 * Block current thread on the condition variable until condition variable
 * pointed by "cond" is signaled or time pointed by "absolute_time_ns" in
 * nanoseconds is reached.
 * 
 * @return 0 on success and LF_TIMEOUT on timeout.
 */
int lf_cond_timedwait(_lf_cond_t* cond, _lf_mutex_t* mutex, long long absolute_time_ns) {
    // Convert the absolute time to a timespec.
    // timespec is seconds and nanoseconds.
    struct timespec timespec_absolute_time
            = {(time_t)absolute_time_ns / 1000000000LL, (long)absolute_time_ns % 1000000000LL};
    return cnd_timedwait((cnd_t*)cond, (mtx_t*)mutex, &timespec_absolute_time);
}
