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

/**
 * Current time in nanoseconds since January 1, 1970.
 * This is not in scope for reactors.
 */
tag_t current_tag = {.time = 0LL, .microstep = 0};

/**
 * Physical time at the start of the execution.
 */
instant_t physical_start_time = NEVER;

/**
 * Logical time at the start of execution.
 */
instant_t start_time = NEVER;

/**
 * Global physical clock offset.
 * Initially set according to the RTI's clock in federated
 * programs.
 */
interval_t _lf_global_physical_time_offset = 0LL;

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
 * Compare two tags. Return -1 if the first is less than
 * the second, 0 if they are equal, and +1 if the first is
 * greater than the second. A tag is greater than another if
 * its time is greater or if its time is equal and its microstep
 * is greater.
 * @param tag1
 * @param tag2
 * @return -1, 0, or 1 depending on the relation.
 */
int compare_tags2(instant_t time1, microstep_t microstep1, instant_t time2, microstep_t microstep2) {
    if (time1 < time2) {
        return -1;
    } else if (time1 > time2) {
        return 1;
    } else if (microstep1 < microstep2) {
        return -1;
    } else if (microstep1 > microstep2) {
        return 1;
    } else {
        return 0;
    }
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
 * Return the current physical time in nanoseconds since January 1, 1970,
 * adjusted by the global physical time offset.
 */
instant_t get_physical_time() {
    struct timespec physicalTime;
    clock_gettime(CLOCK_REALTIME, &physicalTime);
    return (physicalTime.tv_sec * BILLION + physicalTime.tv_nsec) + _lf_global_physical_time_offset;
}

/**
 * Return the physical time of the start of execution in nanoseconds.
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent.
 * @return A time instant.
 */
instant_t get_start_time() {
    return start_time;
}


/**
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_elapsed_physical_time() {
    struct timespec physicalTime;
    clock_gettime(CLOCK_REALTIME, &physicalTime);
    return physicalTime.tv_sec * BILLION + physicalTime.tv_nsec - physical_start_time;
}

/**
 * For C++ compatibility, take a volatile tag_t and return a non-volatile
 * variant.
 */
#ifdef __cplusplus
tag_t convert_volatile_tag_to_nonvolatile(tag_t volatile const& vtag) {
    tag_t non_volatile_tag;
    non_volatile_tag.time = vtag.time;
    non_volatile_tag.microstep - vtag.microstep;
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
