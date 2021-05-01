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

/** Baremetal RISC-V support for the C target of Lingua Franca.
 *  
 *  @author{Efsane Soyer <efsanesoyer@berkeley.edu>}
 */

#include "lf_baremetal_riscv_support.h"
#include "../platform.h"

/**
 * Fetch the value of clk_id and store it in tp.
 */
int lf_clock_gettime(_lf_clock_t clk_id, _lf_time_spec_t* tp) {
    return clock_gettime((clockid_t)clk_id, (struct timespec*) tp);
}

/**
 * Pause execution for a number of nanoseconds.
 */
int nanosleep(_lf_time_spec_t *req, _lf_time_spec_t *rem) {

    _lf_time_spec_t start_time = __clock_gettime();
    _lf_time_spec_t ts = start_time;
    while (ts.tv_sec < start_time.tv_sec + req->tv_sec || (ts.tv_sec == start_time.tv_sec + req->tv_sec && ts.tv_nsec <= start_time.tv_nsec + req->tv_nsec)) {
        ts = __clock_gettime();
    }

    return 0;
}

// Default nosys implementation of _sbrk
void *
_sbrk (incr)
     int incr;
{
   extern char   end; /* Set by linker.  */
   static char * heap_end;
   char *        prev_heap_end;

   if (heap_end == 0)
     heap_end = & end;

   prev_heap_end = heap_end;
   heap_end += incr;

   return (void *) prev_heap_end;
}