/**
 * @file message_record.c
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

#include "message_record.h"
#include <stdlib.h>

/**
 * @brief Initialize the in-transit message record queue.
 * 
 * @return in_transit_message_record_q 
 */
in_transit_message_record_q_t* initialize_in_transit_message_q() {
    in_transit_message_record_q_t* queue = 
        (in_transit_message_record_q_t*)calloc(
            1, 
            sizeof(in_transit_message_record_q_t)
        );
    queue->main_queue = pqueue_init(
        10, 
        in_reverse_order, 
        get_message_record_index,
        get_message_record_position, 
        set_message_record_position, 
        tags_match, 
        print_message_record
    );

    queue->transfer_queue = pqueue_init(
        10, 
        in_reverse_order, 
        get_message_record_index,
        get_message_record_position, 
        set_message_record_position, 
        tags_match, 
        print_message_record
    );

    return queue;
}

/**
 * @brief Free the memory occupied by the `queue`.
 * 
 * @param queue The queue to free.
 */
void free_in_transit_message_q(in_transit_message_record_q_t* queue) {
    pqueue_free(queue->main_queue);
    pqueue_free(queue->transfer_queue);
    free(queue);
}

/**
 * @brief Add a record of the in-transit message.
 * 
 * @param queue The queue to add to.
 * @param tag The tag of the in-transit message.
 * @return 0 on success.
 */
int add_in_transit_message_record(in_transit_message_record_q_t* queue, tag_t tag) {
    in_transit_message_record_t* in_transit_record = malloc(sizeof(in_transit_message_record_t));
    in_transit_record->tag = tag;
    return pqueue_insert(
        queue->main_queue, 
        (void*)in_transit_record
    );
}

/**
 * @brief Clean the record of in-transit messages up to and including `tag`.
 * 
 * @param queue The queue to clean.
 * @param tag Will clean all messages with tags <= tag.
 */
void clean_in_transit_message_record_up_to_tag(in_transit_message_record_q_t* queue, tag_t tag) {
    in_transit_message_record_t* head_of_in_transit_messages = (in_transit_message_record_t*)pqueue_peek(queue->main_queue);
    while (
        head_of_in_transit_messages != NULL &&              // Queue is not empty
        head_of_in_transit_messages->tag.time <= tag.time   // The head message record has a time less than or equal to
                                                            // `tag.time`.
    ) {
        // Now compare the tags. The message record queue is ordered according to the `time` field, so we need to check
        // all records with that `time` and find those that have smaller or equal full tags.
        if (lf_tag_compare(
                head_of_in_transit_messages->tag,
                tag
            ) <= 0
        ) {
            LF_PRINT_DEBUG(
                "RTI: Removed a message with tag (%ld, %u) from the list of in-transit messages.",
                head_of_in_transit_messages->tag.time - lf_time_start(),
                head_of_in_transit_messages->tag.microstep
            );

            free(pqueue_pop(queue->main_queue));
        } else {
            // Add it to the transfer queue
            pqueue_insert(queue->transfer_queue, pqueue_pop(queue->main_queue));
        }
        head_of_in_transit_messages = (in_transit_message_record_t*)pqueue_peek(queue->main_queue);
    }
    // Empty the transfer queue (which holds messages with equal time but larger microstep) into the main queue.
    pqueue_empty_into(&queue->main_queue, &queue->transfer_queue);
}

/**
 * @brief Get the minimum tag of all currently recorded in-transit messages.
 *
 * @param queue The queue to search in (of type `in_transit_message_record_q`).
 * @return tag_t The minimum tag of all currently recorded in-transit messages. Return `FOREVER_TAG` if the queue is empty.
 */
tag_t get_minimum_in_transit_message_tag(in_transit_message_record_q_t* queue) {
    tag_t minimum_tag = FOREVER_TAG;

    in_transit_message_record_t* head_of_in_transit_messages = (in_transit_message_record_t*)pqueue_peek(queue->main_queue);
    while (head_of_in_transit_messages != NULL) { // Queue is not empty
        // The message record queue is ordered according to the `time` field, so we need to check
        // all records with the minimum `time` and find those that have the smallest tag.
        if (lf_tag_compare(
                head_of_in_transit_messages->tag,
                minimum_tag
            ) <= 0
        ) {
            minimum_tag = head_of_in_transit_messages->tag;
        } else if (head_of_in_transit_messages->tag.time > minimum_tag.time) {
            break;
        }

        // Add the head to the transfer queue.
        pqueue_insert(queue->transfer_queue, pqueue_pop(queue->main_queue));
        
        head_of_in_transit_messages = (in_transit_message_record_t*)pqueue_peek(queue->main_queue);
    }
    // Empty the transfer queue (which holds messages with equal time but larger microstep) into the main queue.
    pqueue_empty_into(&queue->main_queue, &queue->transfer_queue);

    if (head_of_in_transit_messages != NULL) {
        LF_PRINT_DEBUG(
            "RTI: Minimum tag of all in-transit messages: (%ld, %u).",
            head_of_in_transit_messages->tag.time - lf_time_start(),
            head_of_in_transit_messages->tag.microstep
        );
    }

    return minimum_tag;
}
