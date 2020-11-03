/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
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
 * Runtime infrastructure for distributed Lingua Franca programs.
 *
 * This implementation creates one thread per federate so as to be able
 * to take advantage of multiple cores. It may be more efficient, however,
 * to use select() instead to read from the multiple socket connections
 * to each federate.
 *
 * This implementation sends messages in little endian order
 * because Intel, RISC V, and Arm processors are little endian.
 * This is not what is normally considered "network order",
 * but we control both ends, and hence, for commonly used
 * processors, this will be more efficient since it won't have
 * to swap bytes.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>      // Defines perror(), errno
#include <sys/socket.h>
#include <sys/types.h>  // Provides select() function to read from multiple sockets.
#include <netinet/in.h> // Defines struct sockaddr_in
#include <arpa/inet.h>  // inet_ntop & inet_pton
#include <unistd.h>     // Defines read(), write(), and close()
#include <netdb.h>      // Defines gethostbyname().
#include <strings.h>    // Defines bzero().
#include <assert.h>
#include <sys/wait.h>   // Defines wait() for process to change state.
#include "util.c"       // Defines error() and swap_bytes_if_big_endian().
#include "rti.h"        // Defines TIMESTAMP. Includes <pthread.h> and "reactor.h".

/** Delay the start of all federates by this amount. */
#define DELAY_START SEC(1)

// The one and only mutex lock.
pthread_mutex_t rti_mutex = PTHREAD_MUTEX_INITIALIZER;

// Condition variable used to signal receipt of all proposed start times.
pthread_cond_t received_start_times = PTHREAD_COND_INITIALIZER;

// Condition variable used to signal that a start time has been sent to a federate.
pthread_cond_t sent_start_time = PTHREAD_COND_INITIALIZER;

// The federates.
federate_t federates[NUMBER_OF_FEDERATES];

// Maximum start time seen so far from the federates.
instant_t max_start_time = 0LL;

// Number of federates that have proposed start times.
int num_feds_proposed_start = 0;

// The start time for an execution.
instant_t start_time = NEVER;

// Boolean indicating that all federates have exited.
volatile bool all_federates_exited = false;

/**
 * The ID of the federation that this RTI will supervise.
 * This should be overridden with a command-line -i option to ensure
 * that each federate only joins its assigned federation.
 */
char* federation_id = "Unidentified Federation";

/** The final port number that the socket server ends up using. */
int final_port = -1;

/** The socket descriptor for the socket server. */
int socket_descriptor = -1;

/** 
 * Create a server and enable listening for socket connections.
 * 
 * @note This function is similar to create_server(...) in 
 * federate.c. However, it contains logs that are specific
 * to the RTI.
 * 
 * @param port The port number to use.
 * @return The socket descriptor on which to accept connections.
 */
int create_server(int specified_port, int port) {
    // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
    int socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_descriptor < 0) {
        error_print_and_exit("Failed to create RTI socket.");
    }

    /*
     * The following used to permit reuse of a port that an RTI has previously
     * used that has not been released. We no longer do this, but instead
     * increment the port number until an available port is found.

    // SO_REUSEPORT (since Linux 3.9)
    //       Permits multiple AF_INET or AF_INET6 sockets to be bound to an
    //       identical socket address.  This option must be set on each
    //       socket (including the first socket) prior to calling bind(2)
    //       on the socket.  To prevent port hijacking, all of the
    //       processes binding to the same address must have the same
    //       effective UID.  This option can be employed with both TCP and
    //       UDP sockets.

    int reuse = 1;
    if (setsockopt(socket_descriptor, SOL_SOCKET, SO_REUSEADDR, 
            (const char*)&reuse, sizeof(reuse)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
    }

    #ifdef SO_REUSEPORT
    if (setsockopt(socket_descriptor, SOL_SOCKET, SO_REUSEPORT, 
            (const char*)&reuse, sizeof(reuse)) < 0)  {
        perror("setsockopt(SO_REUSEPORT) failed");
    }
    #endif
    */

    // Server file descriptor.
    struct sockaddr_in server_fd;
    // Zero out the server address structure.
    bzero((char *) &server_fd, sizeof(server_fd));

    server_fd.sin_family = AF_INET;            // IPv4
    server_fd.sin_addr.s_addr = INADDR_ANY;    // All interfaces, 0.0.0.0.
    // Convert the port number from host byte order to network byte order.
    server_fd.sin_port = htons(port);

    int result = bind(
            socket_descriptor,
            (struct sockaddr *) &server_fd,
            sizeof(server_fd));
    // If the binding fails with this port and no particular port was specified
    // in the LF program, then try the next few ports in sequence.
    while (result != 0
            && specified_port == 0
            && port >= STARTING_PORT
            && port <= STARTING_PORT + PORT_RANGE_LIMIT) {
        printf("RTI failed to get port %d. Trying %d\n", port, port + 1);
        port++;
        server_fd.sin_port = htons(port);
        result = bind(
                socket_descriptor,
                (struct sockaddr *) &server_fd,
                sizeof(server_fd));
    }
    if (result != 0) {
        if (specified_port == 0) {
            error_print_and_exit("Failed to bind the RTI socket. Cannot find a usable port. Consider increasing PORT_RANGE_LIMIT in rti.h.");
        } else {
            error_print_and_exit("Failed to bind the RTI socket. Specified port is not available. Consider leaving the port unspecified");
        }
    }
    printf("RTI for federation %s started using port %d.\n", federation_id, port);

    final_port = port;

    // Enable listening for socket connections.
    // The second argument is the maximum number of queued socket requests,
    // which according to the Mac man page is limited to 128.
    listen(socket_descriptor, 128);

    return socket_descriptor;
}

/** Handle a timed message being received from a federate via the RTI.
 *  @param sending_socket The identifier for the sending socket.
 *  @param buffer The buffer to read into (the first byte is already there).
 */
void handle_timed_message(int sending_socket, unsigned char* buffer) {
    int header_size = 21;
    // Read the header, minus the first byte which is already there.
    read_from_socket(sending_socket, header_size - 1, buffer + 1, "RTI failed to read the timed message header from remote federate.");
    // Extract the header information.
    unsigned short port_id;
    unsigned short federate_id;
    unsigned int length;
    // Extract information from the header.
    extract_header(buffer + 1, &port_id, &federate_id, &length);

    unsigned int total_bytes_to_read = length + header_size;
    unsigned int bytes_to_read = length;
    // Prevent a buffer overflow.
    if (bytes_to_read + header_size > BUFFER_SIZE) {
        bytes_to_read = BUFFER_SIZE - header_size;
    }

    read_from_socket(sending_socket, bytes_to_read, buffer + header_size,
                     "RTI failed to read timed message from federate %d.", federate_id);
    int bytes_read = bytes_to_read + header_size;
    DEBUG_PRINT("Message received by RTI: %s.", buffer + header_size);

    // Need to acquire the mutex lock to ensure that the thread handling
    // messages coming from the socket connected to the destination does not
    // issue a TAG before this message has been forwarded.
    pthread_mutex_lock(&rti_mutex);

    // If the destination federate is no longer connected, issue a warning
    // and return.
    if (federates[federate_id].state == NOT_CONNECTED) {
        pthread_mutex_unlock(&rti_mutex);
        printf("RTI: Destination federate %d is no longer connected. Dropping message.\n",
                federate_id
        );
        return;
    }

    // Forward the message or message chunk.
    int destination_socket = federates[federate_id].socket;

    DEBUG_PRINT("RTI forwarding message to port %d of federate %d of length %d.", port_id, federate_id, length);
    // Need to make sure that the destination federate's thread has already
    // sent the starting TIMESTAMP message.
    while (federates[federate_id].state == NOT_CONNECTED) {
        // Need to wait here.
        pthread_cond_wait(&sent_start_time, &rti_mutex);
    }
    write_to_socket(destination_socket, bytes_read, buffer, "RTI failed to forward message to federate %d.", federate_id);

    // The message length may be longer than the buffer,
    // in which case we have to handle it in chunks.
    int total_bytes_read = bytes_read;
    while (total_bytes_read < total_bytes_to_read) {
        DEBUG_PRINT("Forwarding message in chunks.");
        bytes_to_read = total_bytes_to_read - total_bytes_read;
        if (bytes_to_read > BUFFER_SIZE) bytes_to_read = BUFFER_SIZE;
        read_from_socket(sending_socket, bytes_to_read, buffer, "RTI failed to read message chunks.");
        total_bytes_read += bytes_to_read;

        write_to_socket(destination_socket, bytes_to_read, buffer, "RTI failed to send message chunks.");
    }
    pthread_mutex_unlock(&rti_mutex);
}

/** 
 * Send a tag advance grant (TAG) message to the specified socket
 *  with the specified time and microstep.
 */
void send_tag_advance_grant(federate_t* fed, _lf_fd_tag tag) {
    if (fed->state == NOT_CONNECTED) {
        return;
    }
    unsigned char buffer[1 + sizeof(instant_t) + sizeof(microstep_t)];
    buffer[0] = TIME_ADVANCE_GRANT;
    encode_ll(tag.timestep, &(buffer[1]));
    encode_int(tag.microstep, &(buffer[1 + sizeof(instant_t)]));
    write_to_socket(fed->socket, 1 + sizeof(instant_t) + sizeof(microstep_t), buffer, "RTI failed to send time advance grant to federate %d.", fed->id);
    DEBUG_PRINT("RTI sent to federate %d the TAG (%lld, %u).", fed->id, tag.timestep - start_time, tag.microstep);
}

/** 
 * Find the earliest time at which the specified federate may
 * experience its next event. This is the least next event time
 * of the specified federate and (transitively) upstream federates
 * (with delays of the connections added). For upstream federates,
 * we assume (conservatively) that federate upstream of those
 * may also send an event. If any upstream federate has not sent
 * a next event message, this will return the completion time
 * of the federate (which may be NEVER, if the federate has not
 * yet completed a logical time).
 * @param fed The federate.
 * @param candidate A candidate tag (for the first invocation,
 *  this should be fed->next_event).
 * @param visited An array of booleans indicating which federates
 *  have been visited (for the first invocation, this should be
 *  an array of falses of size NUMBER_OF_FEDERATES).
 */
_lf_fd_tag transitive_next_event(federate_t* fed, _lf_fd_tag candidate, bool visited[]) {
    if (visited[fed->id] || fed->state == NOT_CONNECTED) {
        // DEBUG_PRINT("Federate %d not connected to RTI.", fed->id);
        // Federate has stopped executing or we have visited it before.
        // No point in checking upstream federates.
        return candidate;
    }

    visited[fed->id] = true;
    _lf_fd_tag result = fed->next_event;
    if (candidate.timestep < result.timestep ||
        (candidate.timestep == result.timestep &&
               candidate.microstep < result.microstep)) {
        result = candidate;
    }
    // Check upstream federates to see whether any of them might send
    // an event that would result in an earlier next event.
    for (int i = 0; i < fed->num_upstream; i++) {
        _lf_fd_tag upstream_result = transitive_next_event(
                &federates[fed->upstream[i]], result, visited);
        if (upstream_result.timestep != NEVER) {
            upstream_result.timestep += fed->upstream_delay[i];
        }
        if (upstream_result.timestep < result.timestep ||
            (upstream_result.timestep == result.timestep &&
             upstream_result.microstep < result.microstep)) {
            result = upstream_result;
        }
    }
    if (result.timestep == NEVER) {
        result = fed->completed;
    }
    return result;
}

/** 
 * Determine whether the specified reactor is eligible for a tag advance grant,
 * and, if so, send it one. This first compares the next event tag of the
 * federate to the completion tag of all its upstream federates (plus the
 * delay on the connection to those federates). It finds the minimum of all
 * these tags, with a caveat. If the candidate for minimum has a sufficiently
 * large next event tag that we can be sure it will provide no event before
 * the next smallest minimum, then that candidate is ignored and the next
 * smallest minimum determines the tag. If the resulting tag to advance
 * to does not move time forward for the federate, then no tag advance
 * grant message is sent to the federate.
 *
 * This should be called whenever an upstream federate either registers
 * a next event tag or completes a logical tag.
 *
 * This function assumes that the caller holds the mutex lock.
 *
 * @return True if the TAG message is sent and false otherwise.
 */
bool send_tag_advance_if_appropriate(federate_t* fed) {
    // Determine whether to send a time advance grant to the downstream reactor.
    // The first candidate is its next event time. But we may need to advance
    // it to a lesser time.

    _lf_fd_tag candidate_tag_advance = fed->next_event;

    // Look at its upstream federates (including this one).
    for (int j = 0; j < fed->num_upstream; j++) {
        // First, find the minimum completed time or time of the
        // next event of all federates upstream from this one.
        federate_t* upstream = &federates[fed->upstream[j]];
        // There may be a delay on the connection. Add that to the candidate.
        interval_t delay = fed->upstream_delay[j];
        _lf_fd_tag upstream_completion_tag = upstream->completed;
        upstream_completion_tag.timestep += delay;
        if (delay == 0) {
            // If no delay is given, then the last completed microstep of the
            // upstream federate should be taken into consideration.
            upstream_completion_tag.microstep = upstream->completed.microstep;
        } else {
            // If a positive delay is given, then the event will be processed
            // at microstep 0 in a future timestep
            upstream_completion_tag.microstep = 0;
        }
        // Preserve NEVER.
        if (upstream->completed.timestep == NEVER) {
            upstream_completion_tag.timestep = NEVER;
        }
        // If the completion tag of the upstream federate
        // is less than the candidate tag, then we will need to use that
        // completion tag unless the (transitive) next_event tag of the
        // upstream federate (plus the delay) is larger than the
        // current candidate completion tag.
        if (upstream_completion_tag.timestep < candidate_tag_advance.timestep || 
            (upstream_completion_tag.timestep == candidate_tag_advance.timestep &&
             upstream_completion_tag.microstep < candidate_tag_advance.microstep)) {
            // To handle cycles, need to create a boolean array to keep
            // track of which upstream federates have been visited.
            bool visited[NUMBER_OF_FEDERATES];
            for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
                visited[i] = false;
            }

            // Find the (transitive) next event time upstream.
            _lf_fd_tag upstream_next_event = transitive_next_event(
                    upstream, upstream->next_event, visited);
            DEBUG_PRINT("Upstream next event: (%lld, %u). Upstream completion time: (%lld, %u). Candidate time: (%lld, %u).",
                    upstream_next_event.timestep - start_time,
                    upstream_next_event.microstep,
                    upstream_completion_tag.timestep - start_time,
                    upstream_completion_tag.microstep,
                    candidate_tag_advance.timestep - start_time,
                    candidate_tag_advance.microstep);
            if (upstream_next_event.timestep != NEVER && delay > 0) {
                upstream_next_event.timestep += delay;
                upstream_next_event.microstep = 0;
            }
            // If the upstream federate has disconnected,
            // it will not produce further events. Thus,
            // the next assignment will be unnecessary.
            if (upstream->state != NOT_CONNECTED) {
                if (upstream_next_event.timestep < candidate_tag_advance.timestep ||
                    (upstream_next_event.timestep == candidate_tag_advance.timestep &&
                    upstream_next_event.microstep < candidate_tag_advance.microstep)) {
                    // Cannot advance the federate to the upstream
                    // next event time because that event has presumably not yet
                    // been produced.
                    candidate_tag_advance = upstream_completion_tag;
                }
            }
        } else if (upstream_completion_tag.timestep == candidate_tag_advance.timestep &&
                    upstream_completion_tag.microstep == candidate_tag_advance.microstep &&
                    candidate_tag_advance.microstep == 0) {
            // Note that if the next event for this federate has
            // the tag (0, 0), it should wait for all upstream federates
            // to process tag (0, 0) first. This startup procedure has
            // to be handled separately.
            return false; 
        }
    }

    // If the resulting time will advance time
    // in the federate, send it a time advance grant.
    if (candidate_tag_advance.timestep > fed->completed.timestep || 
        (candidate_tag_advance.timestep == fed->completed.timestep &&
        candidate_tag_advance.microstep > fed->completed.microstep)) {
        DEBUG_PRINT("Sending TAG to fed %d of (%lld, %u) because it is greater than the completed (%lld, %u).", fed->id, candidate_tag_advance.timestep - start_time, candidate_tag_advance.microstep, fed->completed.timestep - start_time, fed->completed.microstep);
        send_tag_advance_grant(fed, candidate_tag_advance);
        return true;
    } else {
        DEBUG_PRINT("Not sending TAG to fed %d of %lld because it is not greater than the completed %lld.", fed->id, candidate_tag_advance.timestep - start_time, fed->completed.timestep - start_time);
    }
    return false;
}

/**
 * Handle a logical tag complete (LTC) message.
 * @param fed The federate that has completed a logical tag.
 */
void handle_logical_time_complete(federate_t* fed) {
    unsigned char buffer[sizeof(instant_t) + sizeof(microstep_t)];
    read_from_socket(fed->socket, sizeof(instant_t), buffer, "RTI failed to read the content of the logical tag complete timestep from federate %d.", fed->id);
    read_from_socket(fed->socket, sizeof(microstep_t), buffer + sizeof(instant_t), "RTI failed to read the content of the logical tag complete microstep from federate %d.", fed->id);

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&rti_mutex);

    fed->completed.timestep = extract_ll(buffer);
    fed->completed.microstep = extract_int(&(buffer[sizeof(instant_t)]));

    DEBUG_PRINT("RTI received from federate %d the logical tag complete (%lld, %u).",
                fed->id, fed->completed.timestep - start_time, fed->completed.microstep);

    // Check downstream federates to see whether they should now be granted a TAG.
    for (int i = 0; i < fed->num_downstream; i++) {
        send_tag_advance_if_appropriate(&federates[fed->downstream[i]]);
    }
    pthread_mutex_unlock(&rti_mutex);
}

/**
 * Handle a next event tag (NET) message.
 * @param fed The federate sending a NET message.
 */
void handle_next_event_time(federate_t* fed) {
    unsigned char buffer[sizeof(instant_t) + sizeof(microstep_t)];
    read_from_socket(fed->socket, sizeof(instant_t), buffer, "RTI failed to read the content of the next event tag timestep from federate %d.", fed->id);
    read_from_socket(fed->socket, sizeof(microstep_t), buffer + sizeof(instant_t), "RTI failed to read the content of the next event tag microstep from federate %d.", fed->id);

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&rti_mutex);

    fed->next_event.timestep = extract_ll(buffer);
    fed->next_event.microstep = extract_int(&(buffer[sizeof(instant_t)]));
    DEBUG_PRINT("RTI received from federate %d the NET (%lld, %u).", fed->id, fed->next_event.timestep - start_time, fed->next_event.microstep);

    // Check to see whether we can reply now with a time advance grant.
    // If the federate has no upstream federates, then it does not wait for
    // nor expect a reply. It just proceeds to advance time.
    if (fed->num_upstream > 0) {
        send_tag_advance_if_appropriate(fed);
    }
    pthread_mutex_unlock(&rti_mutex);
}

/** Handle a STOP message.
 *  @param fed The federate sending a STOP message.
 */
void handle_stop_message(federate_t* fed) {
    union {
        long long ull;
        unsigned char c[sizeof(long long)];
    } buffer;
    read_from_socket(fed->socket, sizeof(long long), (unsigned char*)&buffer.c, "RTI failed to read the stop message timestamp from federate %d.", fed->id);

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&rti_mutex);

    instant_t stop_time = swap_bytes_if_big_endian_ll(buffer.ull);
    DEBUG_PRINT("RTI received from federate %d a STOP request with time %lld.", fed->id, stop_time - start_time);

    // Iterate over federates and send each a STOP message.
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        if (i != fed->id) {
            if (federates[i].state == NOT_CONNECTED) {
                continue;
            }
            unsigned char buffer[9];
            buffer[0] = STOP;
            encode_ll(stop_time, &(buffer[1]));
            write_to_socket(federates[i].socket, 9, buffer, "Failed to send STOP message to federate %d.", i);
            DEBUG_PRINT("RTI sent to federate %d STOP with (elapsed) time %lld.", i, stop_time - start_time);
        }
    }

    pthread_mutex_unlock(&rti_mutex);
}

/** 
 * Handle address query messages.
 * This function reads the body of a ADDRESS_QUERY (@see rti.h) message
 * which is the requested destination federate ID and replies with the stored
 * port value for the socket server of that federate. The port values
 * are initialized to -1. If no ADDRESS_AD message has been received from
 * the destination federate, the RTI will simply reply with -1 for the port.
 * The sending federate is responsible for checking back with the RTI after a 
 * period of time. @see connect_to_federate() in federate.c.
 * @param fed_id The federate sending a ADDRESS_QUERY message.
 */
void handle_address_query(ushort fed_id) {
    // Use buffer both for reading and constructing the reply.
    // The length is what is needed for the reply.
    unsigned char buffer[sizeof(int)];
    int bytes_read = read_from_socket2(federates[fed_id].socket, sizeof(ushort), (unsigned char*)buffer);
    if (bytes_read == 0) {
        error("Failed to read address query.\n");
    }
    ushort remote_fed_id = extract_ushort(buffer);
    
    // debug_print("Received address query from %d for %d.\n", fed_id, remote_fed_id);

    assert(federates[remote_fed_id].server_port < 65536);
    if (federates[remote_fed_id].server_port == -1) {
        //debug_print("Warning: RTI received request for a federate %d server that does not exist yet.\n", remote_fed_id);
    }
    // Retrieve the port number
    encode_int(federates[remote_fed_id].server_port, (unsigned char*)buffer);
    // Send the port number (which could be -1)
    write_to_socket(federates[fed_id].socket, sizeof(int), (unsigned char*)buffer,
                        "Failed to write port number to socket of federate %d.", fed_id);

    // Send the server ip address to federate
    write_to_socket(federates[fed_id].socket, sizeof(federates[remote_fed_id].server_ip_addr),
                        (unsigned char *)&federates[remote_fed_id].server_ip_addr,
                        "Failed to write ip address to socket of federate %d.", fed_id);

    if (federates[remote_fed_id].server_port != -1) {
        DEBUG_PRINT("Replied address query from %d with address %s:%d.", fed_id, federates[remote_fed_id].server_hostname, federates[remote_fed_id].server_port);
    }
}

/**
 * Handle address advertisement messages (@see ADDRESS_AD in rti.h).
 * The federate is expected to send its server port number as the next
 * byte. The RTI will keep a record of this number in the .server_port
 * field of the federates[federate_id] array of structs.
 * 
 * The server_hostname and server_ip_addr fields are assigned
 * in connect_to_federates() upon accepting the socket
 * from the remote federate.
 * 
 * @param federate_id The id of the remote federate that is
 *  sending the address advertisement.
 */
void handle_address_ad(ushort federate_id) {   
    DEBUG_PRINT("Received address advertisement from federate %d.", federate_id);
    // Read the port number of the federate that can be used for physical
    // connections to other federates
    int server_port = -1;
    unsigned char buffer[sizeof(int)];
    int bytes_written = read_from_socket2(federates[federate_id].socket, sizeof(int), (unsigned char *)buffer);

    if (bytes_written == 0) {
        DEBUG_PRINT("Error reading port data from federate %d.", federates[federate_id].id);
    }

    server_port = extract_int(buffer);

    
    pthread_mutex_lock(&rti_mutex);
    federates[federate_id].server_port = server_port;
    pthread_mutex_unlock(&rti_mutex);


    DEBUG_PRINT("Got physical connection server address %s:%d from federate %d.\n", federates[federate_id].server_hostname, federates[federate_id].server_port, federates[federate_id].id);
}

/**
 * A function to handle timestamp messages.
 * This function assumes the caller does not hold the mutex.
 */
void handle_timestamp(federate_t *my_fed) {
    unsigned char buffer[8];
    // Read bytes from the socket. We need 8 bytes.
    int bytes_read = read_from_socket2(my_fed->socket, sizeof(long long), (unsigned char*)&buffer);
    if (bytes_read < 1) {
        error_print("ERROR reading timestamp from federate %d.\n", my_fed->id);
    }


    instant_t timestamp = swap_bytes_if_big_endian_ll(*((long long *)(&buffer)));
    DEBUG_PRINT("RTI received timestamp message: %llx.", timestamp);

    pthread_mutex_lock(&rti_mutex);
    num_feds_proposed_start++;
    if (timestamp > max_start_time) {
        max_start_time = timestamp;
    }
    if (num_feds_proposed_start == NUMBER_OF_FEDERATES) {
        // All federates have proposed a start time.
        pthread_cond_broadcast(&received_start_times);
    } else {
        // Some federates have not yet proposed a start time.
        // wait for a notification.
        while (num_feds_proposed_start < NUMBER_OF_FEDERATES) {
            // FIXME: Should have a timeout here?
            pthread_cond_wait(&received_start_times, &rti_mutex);
        }
    }

    // Send back to the federate the maximum time plus an offset.
    // Start by sending a timestamp marker.
    unsigned char message_marker = TIMESTAMP;
    int bytes_written = write_to_socket2(my_fed->socket, 1, &message_marker);
    if (bytes_written < 1) {
        error_print("ERROR sending timestamp to federate %d.", my_fed->id);
    }

    // Send the timestamp.
    // Add an offset to this start time to get everyone starting together.
    start_time = max_start_time + DELAY_START;
    long long message = swap_bytes_if_big_endian_ll(start_time);
    bytes_written = write_to_socket2(my_fed->socket, sizeof(long long), (unsigned char *)(&message));
    if (bytes_written < 1) {
        error_print("ERROR sending starting time to federate %d.", my_fed->id);
    }

    // Update state for the federate to indicate that the TIMESTAMP
    // message has been sent. That TIMESTAMP message grants time advance to
    // the federate to the start time.
    my_fed->state = GRANTED;
    pthread_cond_broadcast(&sent_start_time);
    pthread_mutex_unlock(&rti_mutex);
    DEBUG_PRINT("RTI sent start time %llx to federate %d.", start_time, my_fed->id);
}

/**
 * A function to handle messages labeled
 * as RESIGN sent by a federate. This 
 * message is sent at the time of termination
 * after all shutdown events are processed
 * on the federate. This function assumes
 * that the caller does not hold the mutex
 * lock.
 * 
 * @note At this point, the RTI might have
 * outgoing messages to the federate. This
 * function thus first performs a shutdown
 * on the socket which sends an EOF. It then
 * waits for the remote socket to be closed
 * before closing the socket itself.
 * 
 * @param my_fed The federate sending a RESIGN message.
 **/
void handle_federate_resign(federate_t *my_fed) {
    //Nothing more to do. Close the socket and exit.
    unsigned char temporary_read_buffer[1];
    pthread_mutex_lock(&rti_mutex);            
    // Shut down the socket before closing
    // it. This indicates to the receiver that
    // there will be no more transmissions, therefore
    // it is safe to close the socket.
    shutdown(my_fed->socket, SHUT_WR);
    int bytes_read = 0;
    do { // Wait until the remote socket is closed
        bytes_read = read_from_socket2(my_fed->socket, 1, temporary_read_buffer);
    } while(bytes_read > 0 || errno == EAGAIN || errno == EWOULDBLOCK);
    my_fed->state = NOT_CONNECTED;
    my_fed->next_event.timestep = NEVER;
    my_fed->next_event.microstep = 0;
    close(my_fed->socket); //  from unistd.h
    pthread_mutex_unlock(&rti_mutex);
    printf("Federate %d has resigned.\n", my_fed->id);
}

/** 
 * Thread handling communication with a federate.
 * @param fed A pointer to an int that is the
 *  socket descriptor for the federate.
 */
void* federate(void* fed) {
    federate_t* my_fed = (federate_t*)fed;

    // Buffer for incoming messages.
    // This does not constrain the message size because messages
    // are forwarded piece by piece.
    unsigned char buffer[BUFFER_SIZE];

    // Listen for messages from the federate.
    while (1) {
        // Read no more than one byte to get the message type.
        int bytes_read = read_from_socket2(my_fed->socket, 1, buffer);
        if (bytes_read == 0) {
            continue;
        } else if (bytes_read < 0) {
            error_print_and_exit("ERROR: RTI socket to federate %d broken.\n", my_fed->id);
        }
        switch(buffer[0]) {
        case TIMESTAMP:
            DEBUG_PRINT("RTI handling TIMESTAMP message.");
            handle_timestamp(my_fed);
            break;
        case ADDRESS_QUERY:
            // debug_print("Handling ADDRESS_QUERY message.\n");
            handle_address_query(my_fed->id);
            break;
        case ADDRESS_AD:
            DEBUG_PRINT("Handling ADDRESS_AD message.");
            handle_address_ad(my_fed->id);
            break;
        case TIMED_MESSAGE:
            DEBUG_PRINT("Handling timed message.");
            if (my_fed->state == NOT_CONNECTED) {
                return NULL;
            }
            handle_timed_message(my_fed->socket, buffer);
            break;
        case RESIGN:
            DEBUG_PRINT("Handling resign.");
            if (my_fed->state == NOT_CONNECTED) {
                return NULL;
            }
            handle_federate_resign(my_fed);
            return NULL;
            break;
        case NEXT_EVENT_TIME:            
            DEBUG_PRINT("Handling next event time.");
            if (my_fed->state == NOT_CONNECTED) {
                return NULL;
            }
            handle_next_event_time(my_fed);
            break;
        case LOGICAL_TIME_COMPLETE:            
            DEBUG_PRINT("Handling logical time completion.");
            if (my_fed->state == NOT_CONNECTED) {
                return NULL;
            }
            handle_logical_time_complete(my_fed);
            break;
        case STOP:
            DEBUG_PRINT("Handling stop.");
            if (my_fed->state == NOT_CONNECTED) {
                return NULL;
            }
            handle_stop_message(my_fed);
            break;
        default:
            error_print("Received from federate %d an unrecognized message type.", my_fed->id);
        }
    }

    // Nothing more to do. Close the socket and exit.
    close(my_fed->socket); //  from unistd.h

    return NULL;
}

/** Wait for one incoming connection request from each federate,
 *  and upon receiving it, create a thread to communicate with
 *  that federate. Return when all federates have connected.
 *  @param socket_descriptor The socket on which to accept connections.
 */
void connect_to_federates(int socket_descriptor) {
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        // Wait for an incoming connection request.
        struct sockaddr client_fd;
        uint32_t client_length = sizeof(client_fd);
        int socket_id = accept(socket_descriptor, &client_fd, &client_length);
        if (socket_id < 0) {
            error_print_and_exit("RTI failed to accept the socket.");
        }

        // The first message from the federate should contain its ID and the federation ID.
        // Buffer for message ID, federate ID, and federation ID length.
        int length = sizeof(ushort) + 2; // This should be 4.
        unsigned char buffer[length];

        // Read bytes from the socket. We need 4 bytes.
        read_from_socket(socket_id, length, buffer, "RTI failed to read from accepted socket.");
        // debug_print("read %d bytes.\n", bytes_read);

        // If any error occurs, this will be set to non-zero.
        unsigned char error_code = 0;

        ushort fed_id;

        // First byte received is the message ID.
        if (buffer[0] != FED_ID) {
            if(buffer[0] == P2P_SENDING_FED_ID || buffer[0] == P2P_TIMED_MESSAGE) {
                // If the connection is a peer-to-peer connection between two
                // federates, reject the connection with the WRONG_SERVER error.
                error_code = WRONG_SERVER;
            } else {
                error_code = UNEXPECTED_MESSAGE;
            }
            fprintf(stderr, "WARNING: RTI expected a FED_ID message. Got %u (see rti.h).\n", buffer[0]);
        } else {
            fed_id = extract_ushort(buffer + 1);
            DEBUG_PRINT("RTI received federate ID: %d.", fed_id);

            // Read the federation ID.
            size_t federation_id_length = buffer[3];
            char federation_id_received[federation_id_length + 1];

            read_from_socket(socket_id, federation_id_length, 
                                (unsigned char*)federation_id_received,
                                "RTI failed to read federation id from federate %d.", fed_id);

            // Terminate the string with a null.
            federation_id_received[federation_id_length] = 0;

            // Compare the received federation ID to mine.
            if (strncmp(federation_id, federation_id_received, federation_id_length) != 0) {
                // Federation IDs do not match. Send back a REJECT message.
                fprintf(stderr,
                        "WARNING: Federate from another federation %s attempted to connect to RTI in federation %s.\n",
                        federation_id_received,
                        federation_id);
                error_code = FEDERATION_ID_DOES_NOT_MATCH;
            } else {
                if (fed_id >= NUMBER_OF_FEDERATES) {
                    // Federate ID is out of range.
                    fprintf(stderr, "WARNING: RTI received federate ID %d, which is out of range.\n", fed_id);
                    error_code = FEDERATE_ID_OUT_OF_RANGE;
                } else {
                    if (federates[fed_id].state != NOT_CONNECTED) {
                        fprintf(stderr, "WARNING: RTI received duplicate federate ID: %d.\n", fed_id);
                        error_code = FEDERATE_ID_IN_USE;
                    }
                }
            }
        }

        // If the FED_ID message was not exactly right, respond with a REJECT,
        // close the socket, and continue waiting for federates to join.
        if (error_code != 0) {
            unsigned char response[2];
            response[0] = REJECT;
            response[1] = error_code;
            // Ignore errors on this response.
            write_to_socket(socket_id, 2, response, "RTI failed to write REJECT message on the socket.");
            // Close the socket.
            close(socket_id);
            // Invalid federate. Try again.
            i--;
            continue;
        } else {
            // Send an ACK message.
            unsigned char response[1];
            response[0] = ACK;
            // Ignore errors on this response.
            write_to_socket(socket_id, 1, response, "RTI failed to write ACK message to federate %d.", fed_id);
        }

        // Assign the address information for federate
        // The IP address is stored here as an in_addr struct (in .server_ip_addr) that can be useful
        // to create sockets and can be efficiently sent over the network. If VERBOSE is defined
        // in the target LF program, the IP address is also stored in a human readable format
        // (stored in .server_hostname) that can be useful for log messages.
        // First, convert the sockaddr structure into a sockaddr_in that contains an internet address.
        struct sockaddr_in* pV4_addr = (struct sockaddr_in*)&client_fd;
        // Then extract the internet address (which is in IPv4 format) and assign it as the federate's socket server
        federates[fed_id].server_ip_addr = pV4_addr->sin_addr;

#ifdef VERBOSE
        // Then create the human readable format and copy that into
        // the .server_hostname field of the federate.
        char str[INET_ADDRSTRLEN];
        inet_ntop( AF_INET, &federates[fed_id].server_ip_addr, str, INET_ADDRSTRLEN );  
        strncpy (federates[fed_id].server_hostname, str, INET_ADDRSTRLEN);     

        DEBUG_PRINT("RTI got address %s from federate %d.", federates[fed_id].server_hostname, fed_id);
#endif
        federates[fed_id].socket = socket_id;

        // Create a thread to communicate with the federate.
        pthread_create(&(federates[fed_id].thread_id), NULL, federate, &(federates[fed_id]));
    }

    DEBUG_PRINT("All federates have connected to RTI.");
}

/**
 * Thread to respond to new connections, which could be federates of other
 * federations who are attempting to join the wrong federation.
 * @param nothing Nothing needed here.
 */
void* respond_to_erroneous_connections(void* nothing) {
    while (true) {
        // Wait for an incoming connection request.
        struct sockaddr client_fd;
        uint32_t client_length = sizeof(client_fd);
        int socket_id = accept(socket_descriptor, &client_fd, &client_length);
        if (socket_id < 0) return NULL;

        if (all_federates_exited) {
            return NULL;
        }

        fprintf(stderr, "WARNING: RTI received an unexpected connection request. Federation is running.\n");
        unsigned char response[2];
        response[0] = REJECT;
        response[1] = FEDERATION_ID_DOES_NOT_MATCH;
        // Ignore errors on this response.
        write_to_socket(socket_id, 2, response,
                            "RTI failed to write FEDERATION_ID_DOES_NOT_MATCH to erroneous incoming connection.");
        // Close the socket.
        close(socket_id);
    }
}

/** Initialize the federate with the specified ID.
 *  @param id The federate ID.
 */
void initialize_federate(int id) {
    federates[id].id = id;
    federates[id].socket = -1;      // No socket.
    federates[id].completed.timestep = NEVER;
    federates[id].completed.microstep = 0u;
    federates[id].next_event.timestep = NEVER;
    federates[id].next_event.microstep = 0u;
    federates[id].state = NOT_CONNECTED;
    federates[id].upstream = NULL;
    federates[id].upstream_delay = NULL;
    federates[id].num_upstream = 0;
    federates[id].downstream = NULL;
    federates[id].num_downstream = 0;
    federates[id].mode = REALTIME;    
    strncpy(federates[id].server_hostname ,"localhost", INET_ADDRSTRLEN);
    federates[id].server_ip_addr.s_addr = 0;
    federates[id].server_port = -1;
}

/** 
 * Launch the specified executable by forking the calling process and converting
 * the forked process into the specified executable.
 * If forking the process fails, this will return -1.
 * Otherwise, it will return the process ID of the created process.
 * 
 * FIXME: Unused function.
 * 
 * @param executable The executable program.
 * @return The PID of the created process or -1 if the fork fails.
 */
pid_t federate_launcher(char* executable) {
    char* command[2];
    command[0] = executable;
    command[1] = NULL;
    pid_t pid = fork();
    if (pid == 0) {
        // This the newly created process. Replace it.
        printf("Federate launcher starting executable: %s.\n", executable);
        execv(executable, command);
        // Remaining part of this function is ignored.
    }
    if (pid == -1) {
        error_print("Forking the RTI process to start the executable: %s\n", executable);
    }
    return pid;
}

/** 
 * Start the socket server for the runtime infrastructure (RTI) and
 * return the socket descriptor.
 * @param num_feds Number of federates.
 * @param port The port on which to listen for socket connections, or
 *  0 to use the default port range.
 */
int start_rti_server(int port) {
    int specified_port = port;
    if (port == 0) {
        // Use the default starting port.
        port = STARTING_PORT;
    }
    socket_descriptor = create_server(specified_port, port);
    printf("RTI: Listening for federates.\n");
    return socket_descriptor;
}

/** Start the runtime infrastructure (RTI) interaction with the federates
 *  and wait for the federates to exit.
 *  @param socket_descriptor The socket descriptor returned by start_rti_server().
 */
void wait_for_federates(int socket_descriptor) {
    // Wait for connections from federates and create a thread for each.
    connect_to_federates(socket_descriptor);

    // All federates have connected.
    printf("RTI: All expected federates have connected. Starting execution.\n");

    // Unfortunately, the socket server will continue to accept connections.
    // In case some other federation's federates are trying to join the wrong
    // federation, need to respond. Start a separate thread to do that.

    pthread_t responder_thread;
    pthread_create(&responder_thread, NULL, respond_to_erroneous_connections, NULL);

    // Wait for federate threads to exit.
    void* thread_exit_status;
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        pthread_join(federates[i].thread_id, &thread_exit_status);
        printf("RTI: Federate %d thread exited.\n", federates[i].id);
    }

    // NOTE: Apparently, closing the socket will not necessarily
    // cause the respond_to_erroneous_connections accept() call to return,
    // so instead, we connect here so that it can check the all_federates_exited
    // variable.
    all_federates_exited = true;

    // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
    int tmp_socket = socket(AF_INET , SOCK_STREAM , 0);
    // If creating the socket fails, assume the thread has already exited.
    if (tmp_socket >= 0) {
        struct hostent *server = gethostbyname("localhost");
        if (server != NULL) {
            // Server file descriptor.
            struct sockaddr_in server_fd;
            // Zero out the server_fd struct.
            bzero((char *) &server_fd, sizeof(server_fd));
            // Set up the server_fd fields.
            server_fd.sin_family = AF_INET;    // IPv4
            bcopy((char *)server->h_addr,
                 (char *)&server_fd.sin_addr.s_addr,
                 server->h_length);
            // Convert the port number from host byte order to network byte order.
            server_fd.sin_port = htons(final_port);
            connect(
                tmp_socket,
                (struct sockaddr *)&server_fd,
                sizeof(server_fd));
            close(tmp_socket);
        }
    }

    // NOTE: In all common TCP/IP stacks, there is a time period,
    // typically between 30 and 120 seconds, called the TIME_WAIT period,
    // before the port is released after this close. This is because
    // the OS is preventing another program from accidentally receiving
    // duplicated packets intended for this program.
    close(socket_descriptor);
}

/**
 * Print a usage message.
 */
void usage(int argc, char* argv[]) {
    printf("\nCommand-line arguments: \n\n");
    printf("  -i, --id <n>\n");
    printf("   The ID of the federation that this RTI will control.\n\n");

    printf("Command given:\n");
    for (int i = 0; i < argc; i++) {
        printf("%s ", argv[i]);
    }
    printf("\n\n");
}

/**
 * Process the command-line arguments. If the command line arguments are not
 * understood, then print a usage message and return 0. Otherwise, return 1.
 * @return 1 if the arguments processed successfully, 0 otherwise.
 */
int process_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
       if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--id") == 0) {
           if (argc < i + 2) {
               fprintf(stderr, "Error: --id needs a string argument.\n");
               usage(argc, argv);
               return 0;
           }
           i++;
           printf("Federation ID at RTI: %s\n", argv[i]);
           federation_id = argv[i++];
       } else {
           fprintf(stderr, "Error: Unrecognized command-line argument: %s\n", argv[i]);
           usage(argc, argv);
           return 0;
       }
    }
    return 1;
}
