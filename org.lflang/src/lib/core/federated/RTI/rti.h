/**
 * @file
 * @author Soroush Bateni (soroush@utdallas.edu)
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

*/

#ifndef RTI_H
#define RTI_H

typedef struct RTI_instance_t {
    // The main mutex lock.
    pthread_mutex_t rti_mutex;

    // Condition variable used to signal receipt of all proposed start times.
    pthread_cond_t received_start_times;

    // Condition variable used to signal that a start time has been sent to a federate.
    pthread_cond_t sent_start_time;

    // RTI's decided stop tag for federates
    tag_t max_stop_tag;

    // The federates.
    federate_t federates[NUMBER_OF_FEDERATES];

    // Maximum start time seen so far from the federates.
    int64_t max_start_time;

    // Number of federates that have proposed start times.
    int num_feds_proposed_start;

    // Number of federates handling stop
    int num_feds_handling_stop;

    /**
     * Boolean indicating that all federates have exited.
     * This gets set to true exactly once before the program exits.
     * It is marked volatile because the write is not guarded by a mutex.
     * The main thread makes this true, then calls shutdown and close on
     * the socket, which will cause accept() to return with an error code
     * in respond_to_erroneous_connections().
     */
    volatile bool all_federates_exited;

    /**
     * The ID of the federation that this RTI will supervise.
     * This should be overridden with a command-line -i option to ensure
     * that each federate only joins its assigned federation.
     */
    char* federation_id;

    /************* TCP server information *************/
    /** The final port number that the TCP socket server ends up using. */
    uint16_t final_port_TCP;

    /** The TCP socket descriptor for the socket server. */
    int socket_descriptor_TCP;

    /************* UDP server information *************/
    /** The final port number that the UDP socket server ends up using. */
    uint16_t final_port_UDP;

    /** The UDP socket descriptor for the socket server. */
    int socket_descriptor_UDP;

    /* Thread performing PTP clock sync sessions periodically. */
    pthread_t clock_thread; 
} RTI_instance_t;

#endif // RTI_H