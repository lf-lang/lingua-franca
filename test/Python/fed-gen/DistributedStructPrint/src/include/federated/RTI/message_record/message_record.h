/**
 * @file message_record.h
 * @author Soroush Bateni (soroush@berkeley.edu)
 * @brief Record-keeping for in-transit messages.
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

#ifndef RTI_MESSAGE_RECORD_H
#define RTI_MESSAGE_RECORD_H

#include "rti_pqueue_support.h"

/**
 * @brief Queue to keep a record of in-transit messages.
 * 
 */
typedef struct {
    pqueue_t* main_queue;       // The primary queue.
    pqueue_t* transfer_queue;   // Queue used for housekeeping.
} in_transit_message_record_q_t;

/**
 * @brief Initialize the in-transit message record queue.
 * 
 * @return in_transit_message_record_q 
 */
in_transit_message_record_q_t* initialize_in_transit_message_q();

/**
 * @brief Free the memory occupied by the `queue`.
 * 
 * @param queue The queue to free.
 */
void free_in_transit_message_q(in_transit_message_record_q_t* queue);

/**
 * @brief Add a record of the in-transit message.
 * 
 * @param queue The queue to add to (of type `in_transit_message_record_q`).
 * @param tag The tag of the in-transit message.
 * @return 0 on success.
 */
int add_in_transit_message_record(in_transit_message_record_q_t* queue, tag_t tag);

/**
 * @brief Clean the record of in-transit messages up to and including `tag`.
 * 
 * @param queue The queue to clean (of type `in_transit_message_record_q`).
 * @param tag Will clean all messages with tags <= tag.
 */
void clean_in_transit_message_record_up_to_tag(in_transit_message_record_q_t* queue, tag_t tag);

/**
 * @brief Get the minimum tag of all currently recorded in-transit messages.
 *
 * @param queue The queue to search in (of type `in_transit_message_record_q`).
 * @return tag_t The minimum tag of all currently recorded in-transit messages. Return `FOREVER_TAG` if the queue is empty.
 */
tag_t get_minimum_in_transit_message_tag(in_transit_message_record_q_t* queue);

#endif // RTI_MESSAGE_RECORD_H
