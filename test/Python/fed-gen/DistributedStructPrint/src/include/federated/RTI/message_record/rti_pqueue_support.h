/**
 * @file rti_pqueue_support.h
 * @author Soroush Bateni (soroush@berkeley.edu)
 * @brief Header-only support functions for pqueue (in the RTI).
 * @version 0.1
 * @date 2022-06-02
 * 
 * @copyright Copyright (c) 2022, The University of California at Berkeley.

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

#ifndef RTI_PQUEUE_SUPPORT_H
#define RTI_PQUEUE_SUPPORT_H

#include "tag.h"
#include "utils/pqueue.h"
#include "utils/util.h"

// ********** Priority Queue Support Start
/**
 * @brief Represent an in-transit message.
 * 
 */
typedef struct in_transit_message_record {
    tag_t tag;      // Tag of the in-transit message.
    size_t pos;     // Position in the priority queue.
} in_transit_message_record_t;


/**
 * Return whether the first and second argument are given in reverse order.
 */
static int in_reverse_order(pqueue_pri_t thiz, pqueue_pri_t that) {
    return (thiz > that);
}

/**
 * Return false (0) regardless of tag order.
 */
static int in_no_particular_order(pqueue_pri_t thiz, pqueue_pri_t that) {
    return false;
}

/**
 * Return whether or not the given `in_transit_message_record_t` types have the same tag.
 */
static int tags_match(void* next, void* curr) {
    return (lf_tag_compare(
                ((in_transit_message_record_t*)next)->tag, 
                ((in_transit_message_record_t*)curr)->tag
            ) == 0);
}

/**
 * Report a priority equal to the time of the given in-transit message.
 * Used for sorting pointers to in_transit_message_record_t structs.
 */
static pqueue_pri_t get_message_record_index(void *a) {
    return (pqueue_pri_t)(((in_transit_message_record_t*) a)->tag.time);
}

/**
 * Return the given in_transit_message_record_t's position in the queue.
 */
static size_t get_message_record_position(void *a) {
    return ((in_transit_message_record_t*) a)->pos;
}

/**
 * Set the given in_transit_message_record_t's position in the queue.
 */
static void set_message_record_position(void *a, size_t pos) {
    ((in_transit_message_record_t*) a)->pos = pos;
}

/**
 * Print some information about the given in-transit message.
 * 
 * DEBUG function only.
 */
static void print_message_record(void *message) {
	in_transit_message_record_t *r = (in_transit_message_record_t*)message;
    LF_PRINT_DEBUG(
        "Tag of the in_transit_message_record_t: (%ld, %u). "
        "Its position in the priority queue: %u",
        r->tag.time - lf_time_start(),
        r->tag.microstep,
        r->pos
    );
}

// ********** Priority Queue Support End
#endif
