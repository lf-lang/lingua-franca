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
 * Implementation file for tag functions for Lingua Franca programs.
 */

#include "tag.h"
#include "platform.h"

/**
 * Offset to _LF_CLOCK that would convert it
 * to epoch time.
 * For CLOCK_REALTIME, this offset is always zero.
 * For CLOCK_MONOTONIC, it is the difference between those
 * clocks at the start of the execution.
 */
interval_t _lf_epoch_offset = 0LL;

/**
 * Current time in nanoseconds since January 1, 1970
 * This is not in scope for reactors.
 * This should only ever be accessed while holding the mutex lock.
 */
tag_t current_tag = {.time = 0LL, .microstep = 0};

/**
 * Physical time at the start of the execution.
 * This should only ever be accessed while holding the mutex lock.
 */
instant_t physical_start_time = NEVER;

/**
 * Logical time at the start of execution.
 * This should only ever be accessed while holding the mutex lock.
 */
instant_t start_time = NEVER;

/**
 * Global physical clock offset.
 * Initially set according to the RTI's clock in federated
 * programs.
 */
interval_t _lf_global_physical_clock_offset = 0LL;

/**
 * A measure of calculating the drift between the federate's
 * clock and the RTI's clock
 */
interval_t _lf_global_physical_clock_drift = 0LL;

/**
 * A test offset that is applied to the clock.
 * The clock synchronization algorithm must correct for this offset.
 * This offset is especially useful to test clock synchronization on the
 * same machine.
 */
interval_t _lf_global_test_physical_clock_offset = 0LL;

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
int compare_tags(tag_t tag1, tag_t tag2) {
    if (tag1.time < tag2.time) {
        return -1;
    } else if (tag1.time > tag2.time) {
        return 1;
    } else if (tag1.microstep < tag2.microstep) {
        return -1;
    } else if (tag1.microstep > tag2.microstep) {
        return 1;
    } else {
        return 0;
    }
}

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
tag_t delay_tag(tag_t tag, interval_t interval) {
	if (tag.time == NEVER || interval == NEVER) return tag;
	tag_t result = tag;
	if (interval == 0LL) {
		// Note that unsigned variables will wrap on overflow.
		// This is probably the only reasonable thing to do with overflowing
		// microsteps.
		result.microstep++;
	} else {
		// Note that overflow in C is undefined for signed variables.
		if (FOREVER - interval < result.time) {
			result.time = FOREVER;
		} else {
			result.time += interval;
		}
		result.microstep = 0;
	}
	return result;
}

/**
 * Return the elapsed logical time in nanoseconds since the start of execution.
 */
interval_t get_elapsed_logical_time() {
    return current_tag.time - start_time;
}

/**
 * Return the current tag, a logical time, microstep pair.
 */
tag_t get_current_tag() {
    return current_tag;
}

/**
 * Return the current logical time in nanoseconds since January 1, 1970.
 */
instant_t get_logical_time() {
    return current_tag.time;
}

/**
 * Return the current microstep.
 */
microstep_t get_microstep() {
    return current_tag.microstep;
}

/**
 * Stores the last reported absolute snapshot of the 
 * physical clock.
 */
instant_t _lf_last_reported_physical_time_ns = 0LL;

/**
 * Records the most recent time reported by the physical clock
 * when accessed by get_physical_time(). This will be an epoch time
 * (number of nanoseconds since Jan. 1, 1970), as reported when
 * you call lf_clock_gettime(CLOCK_REALTIME, ...). This differs from
 * _lf_last_reported_physical_time_ns by _lf_global_physical_clock_offset
 * plus any calculated drift adjustement, which are adjustments made
 * by clock synchronization.
 */
instant_t _lf_last_reported_unadjusted_physical_time_ns = NEVER;

/**
 * Return the current physical time in nanoseconds since January 1, 1970,
 * adjusted by the global physical time offset.
 */
instant_t get_physical_time() {
    // Get the current clock value
    struct timespec physicalTime;
    lf_clock_gettime(_LF_CLOCK, &physicalTime);
    _lf_last_reported_unadjusted_physical_time_ns = (physicalTime.tv_sec * BILLION + physicalTime.tv_nsec)
            + _lf_epoch_offset;
    
    // Adjust the reported clock with the appropriate offsets
    instant_t adjusted_clock_ns = _lf_last_reported_unadjusted_physical_time_ns
            + _lf_global_physical_clock_offset;

    // Apply the test offset
    adjusted_clock_ns += _lf_global_test_physical_clock_offset;

    // if (_lf_global_physical_clock_drift != 0LL
    //         && _lf_last_clock_sync_instant != 0LL) {
    //     // Apply the calculated drift, if appropriate
    //     interval_t drift = (adjusted_clock_ns - _lf_last_clock_sync_instant) *
    //                        _lf_global_physical_clock_drift;
    //     adjusted_clock_ns += drift;
    //     DEBUG_PRINT("Physical time adjusted for clock drift by %lld.", drift);
    // }
    
    // Check if the clock has progressed since the last reported value
    // This ensures that the clock is monotonic
    if (adjusted_clock_ns > _lf_last_reported_physical_time_ns) {
        _lf_last_reported_physical_time_ns = adjusted_clock_ns;
    }
    
    DEBUG_PRINT("Physical time: %lld. Elapsed: %lld. Offset: %lld",
            _lf_last_reported_physical_time_ns,
            _lf_last_reported_physical_time_ns - start_time,
            _lf_global_physical_clock_offset + _lf_global_test_physical_clock_offset);

    return _lf_last_reported_physical_time_ns;
}

/**
 * Return the physical time of the start of execution in nanoseconds. * 
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent. * 
 * @return A time instant.
 */
instant_t get_start_time() {
    return start_time;
}


/**
 * Return the elapsed physical time in nanoseconds.
 * This is the time returned by get_physical_time() minus the
 * physical start time as measured by get_physical_time() when
 * the program was started.
 */
instant_t get_elapsed_physical_time() {
    return get_physical_time() - physical_start_time;
}

/**
 * Set a fixed offset to the physical clock.
 * After calling this, the value returned by get_physical_time()
 * and get_elpased_physical_time() will have this specified offset
 * added to what it would have returned before the call.
 */
void set_physical_clock_offset(interval_t offset) {
    _lf_global_test_physical_clock_offset += offset;
}

/**
 * For C++ compatibility, take a volatile tag_t and return a non-volatile
 * variant.
 */
#ifdef __cplusplus
tag_t convert_volatile_tag_to_nonvolatile(tag_t volatile const& vtag) {
    tag_t non_volatile_tag;
    non_volatile_tag.time = vtag.time;
    non_volatile_tag.microstep = vtag.microstep;
    return non_volatile_tag;
}
#else
/**
 * @note This is an undefined behavior in C and should
 *  be used with utmost caution. See Section 6.7.2 of the C99 standard.
 */
tag_t convert_volatile_tag_to_nonvolatile(tag_t volatile vtag) {
    return vtag;
}
#endif
