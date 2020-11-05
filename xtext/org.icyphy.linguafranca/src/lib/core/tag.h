/**
 * @file
 * @author Edward A. Lee
 * @author Soroush Bateni
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley.

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

 * @section DESCRIPTION
 * Header file for tag functions for Lingua Franca programs.
 */

#ifndef TAG_H
#define TAG_H

/* Conversion of time to nanoseconds. */
#define NSEC(t) (t * 1LL)
#define NSECS(t) (t * 1LL)
#define USEC(t) (t * 1000LL)
#define USECS(t) (t * 1000LL)
#define MSEC(t) (t * 1000000LL)
#define MSECS(t) (t * 1000000LL)
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

// Commonly used time values.
#define NEVER -0xFFFFFFFFFFFFFFFFLL
#define FOREVER 0x7FFFFFFFFFFFFFFFLL

// Convenience for converting times
#define BILLION 1000000000LL

/**
 * Time instant. Both physical and logical times are represented
 * using this typedef.
 * WARNING: If this code is used after about the year 2262,
 * then representing time as a long long will be insufficient.
 */
typedef long long instant_t;

/**
 * Interval of time.
 */
typedef long long interval_t;

/**
 * Microstep instant.
 */
typedef unsigned int microstep_t;

/**
 * A tag is a time, microstep pair.
 */
typedef struct {
    instant_t time;
    microstep_t microstep;
} tag_t;

/**
 * Compare two tags. Return -1 if the first is less than
 * the second, 0 if they are equal, and +1 if the first is
 * greater than the second. A tag is greater than another if
 * its time is greater or if its time is equal and its microstep
 * is greater.
 * @param tag1
 * @param tag2
 * @return -1, 0, or 1 depending on the relation.
 */
int compare_tags(tag_t tag1, tag_t tag2);

/**
 * Compare two tags. Return -1 if the first is less than
 * the second, 0 if they are equal, and +1 if the first is
 * greater than the second. A tag is greater than another if
 * its time is greater or if its time is equal and its microstep
 * is greater.
 * @param tag1
 * @param tag2
 * @return -1, 0, or 1 depending on the relation.
 */
int compare_tags2(instant_t time1, microstep_t microstep1, instant_t time2, microstep_t microstep2);

#endif // TAG_H
