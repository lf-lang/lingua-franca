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
#define NEVER LLONG_MIN
#define FOREVER LLONG_MAX

#define NEVER_TAG (tag_t){ .time = LLONG_MIN, .microstep = 0 }
#define FOREVER_TAG (tag_t){ .time = LLONG_MAX, .microstep = UINT_MAX }

// Convenience for converting times
#define BILLION 1000000000LL

// The underlying physical clock
#define _LF_CLOCK CLOCK_MONOTONIC

/**
 * Time instant. Both physical and logical times are represented
 * using this typedef.
 * WARNING: If this code is used after about the year 2262,
 * then representing time as a 64-bit long long will be insufficient.
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
 * A tag interval indicates the
 * pairwise difference of two tags.
 */
typedef tag_t tag_interval_t;

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
 * Delay a tag by the specified time interval to realize the "after" keyword.
 * If either the time interval or the time field of the tag is NEVER,
 * return the unmodified tag.
 * If the time interval is 0LL, add one to the microstep, leave
 * the time field alone, and return the result.
 * Otherwise, add the interval to the time field of the tag and reset
 * the microstep to 0.
 * If the sum overflows, saturate the time value at FOREVER.
 *
 * Note that normally it makes no sense to call this with a negative
 * interval (except NEVER), but this is not checked.
 *
 * @param tag The tag to increment.
 * @param interval The time interval.
 */
tag_t delay_tag(tag_t tag, interval_t interval);

/**
 * Return the elapsed logical time in nanoseconds
 * since the start of execution.
 * @return A time interval.
 */
interval_t get_elapsed_logical_time();

/**
 * Return the current logical time in nanoseconds.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent.
 * 
 * @return A time instant.
 */
instant_t get_logical_time();

/**
 * Return the current tag, a logical time, microstep pair.
 */
tag_t get_current_tag();

/**
 * Return the current microstep.
 */
microstep_t get_microstep();

/**
 * Global physical clock offset.
 * Initially set according to the RTI's clock in federated
 * programs.
 */
extern interval_t _lf_global_physical_clock_offset;

/**
 * A test offset that is applied to the clock.
 * The clock synchronization algorithm must correct for this offset.
 * This offset is especially useful to test clock synchronization on the
 * same machine.
 */
extern interval_t _lf_global_test_physical_clock_offset;

/**
 * Offset to _LF_CLOCK that would convert it
 * to epoch time. This is applied to the physical clock
 * to get a more meaningful and universal time.
 * 
 * For CLOCK_REALTIME, this offset is always zero.
 * For CLOCK_MONOTONIC, it is the difference between those
 * clocks at the start of the execution.
 */
extern interval_t _lf_epoch_offset;

/**
 * Return the current physical time in nanoseconds.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent.
 * @return A time instant.
 */
instant_t get_physical_time();

/**
 * Return the elapsed physical time in nanoseconds.
 * This is the time returned by get_physical_time() minus the
 * physical start time as measured by get_physical_time() when
 * the program was started.
 */
instant_t get_elapsed_physical_time();

/**
 * Set a fixed offset to the physical clock.
 * After calling this, the value returned by get_physical_time()
 * and get_elpased_physical_time() will have this specified offset
 * added to what it would have returned before the call.
 */
void set_physical_clock_offset(interval_t offset);

/**
 * Return the physical and logical time of the start of execution in nanoseconds.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent. 
 * @return A time instant.
 */
instant_t get_start_time();

/**
 * For C++ compatibility, take a volatile tag_t and return a non-volatile
 * variant.
 */
#ifdef __cplusplus
tag_t convert_volatile_tag_to_nonvolatile(tag_t volatile const& vtag);
#else
/**
 * @note This is an undefined behavior in C and should
 *  be used with utmost caution. See Section 6.7.2 of the C99 standard.
 */
tag_t convert_volatile_tag_to_nonvolatile(tag_t volatile vtag);
#endif

#endif // TAG_H
