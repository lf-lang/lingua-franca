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

/** Create a server and enable listening for socket connections.
 *  @param port The port number to use.
 *  @return The socket descriptor on which to accept connections.
 */
int create_server(int port) {
    // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
    int socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_descriptor < 0) error("ERROR on creating RTI socket");

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
    if (result != 0) error("ERROR on binding RTI socket");

    // Enable listening for socket connections.
    // The second argument is the maximum number of queued socket requests,
    // which according to the Mac man page is limited to 128.
    listen(socket_descriptor, 128);

    return socket_descriptor;
}

/** Handle a message being received from a federate via the RTI.
 *  @param sending_socket The identifier for the sending socket.
 *  @param buffer The buffer to read into (the first byte is already there).
 *  @param header_size The number of bytes in the header.
 */
void handle_message(int sending_socket, unsigned char* buffer, unsigned int header_size) {
    // Read the header, minus the first byte which is already there.
    read_from_socket(sending_socket, header_size - 1, buffer + 1);
    // Extract the header information.
    unsigned short port_id;
    unsigned short federate_id;
    unsigned int length;
    // Extract information from the header.
    extract_header(buffer + 1, &port_id, &federate_id, &length);

    unsigned int total_bytes_to_read = length + header_size;
    unsigned int bytes_to_read = length;
    // Prevent a buffer overflow.
    if (bytes_to_read + header_size > BUFFER_SIZE) bytes_to_read = BUFFER_SIZE - header_size;

    read_from_socket(sending_socket, bytes_to_read, buffer + header_size);
    int bytes_read = bytes_to_read + header_size;
    // printf("DEBUG: Message received by RTI: %s.\n", buffer + header_size);

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

    // printf("DEBUG: RTI forwarding message to port %d of federate %d of length %d.\n", port_id, federate_id, length);
    // Need to make sure that the destination federate's thread has already
    // sent the starting TIMESTAMP message.
    while (federates[federate_id].state == NOT_CONNECTED) {
        // Need to wait here.
        pthread_cond_wait(&sent_start_time, &rti_mutex);
    }
    write_to_socket(destination_socket, bytes_read, buffer);

    // The message length may be longer than the buffer,
    // in which case we have to handle it in chunks.
    int total_bytes_read = bytes_read;
    while (total_bytes_read < total_bytes_to_read) {
        // printf("DEBUG: Forwarding message in chunks.\n");
        bytes_to_read = total_bytes_to_read - total_bytes_read;
        if (bytes_to_read > BUFFER_SIZE) bytes_to_read = BUFFER_SIZE;
        read_from_socket(sending_socket, bytes_to_read, buffer);
        total_bytes_read += bytes_to_read;

        write_to_socket(destination_socket, bytes_to_read, buffer);
    }
    pthread_mutex_unlock(&rti_mutex);
}

/** Send a time advance grant (TAG) message to the specified socket
 *  with the specified time.
 */
void send_time_advance_grant(federate_t* fed, instant_t time) {
    if (fed->state == NOT_CONNECTED) return;
    unsigned char buffer[9];
    buffer[0] = TIME_ADVANCE_GRANT;
    encode_ll(time, &(buffer[1]));
    write_to_socket(fed->socket, 9, buffer);
    // printf("DEBUG: RTI sent to federate %d the TAG %lld.\n", fed->id, time - start_time);
}

/** Find the earliest time at which the specified federate may
 *  experience its next event. This is the least next event time
 *  of the specified federate and (transitively) upstream federates
 *  (with delays of the connections added). For upstream federates,
 *  we assume (conservatively) that federate upstream of those
 *  may also send an event. If any upstream federate has not sent
 *  a next event message, this will return the completion time
 *  of the federate (which may be NEVER, if the federate has not
 *  yet completed a logical time).
 *  @param fed The federate.
 *  @param candidate A candidate time (for the first invocation,
 *   this should be fed->next_time).
 *  @param visited An array of booleans indicating which federates
 *   have been visited (for the first invocation, this should be
 *   an array of falses of size NUMBER_OF_FEDERATES).
 */
instant_t transitive_next_event(federate_t* fed, instant_t candidate, bool visited[]) {
    if (visited[fed->id]) return candidate;
    visited[fed->id] = true;
    instant_t result = fed->next_event;
    if (fed->state == NOT_CONNECTED) {
        // Federate has stopped executing.
        // No point in checking upstream federates.
        return candidate;
    }
    if (candidate < result) {
        result = candidate;
    }
    // Check upstream federates to see whether any of them might send
    // an event that would result in an earlier next event.
    for (int i = 0; i < fed->num_upstream; i++) {
        instant_t upstream_result = transitive_next_event(
                &federates[fed->upstream[i]], result, visited);
        if (upstream_result != NEVER) upstream_result += fed->upstream_delay[i];
        if (upstream_result < result) {
            result = upstream_result;
        }
    }
    if (result == NEVER) {
        result = fed->completed;
    }
    return result;
}

/** Determine whether the specified reactor is eligible for a time advance grant,
 *  and, if so, send it one. This first compares the next event time of the
 *  federate to the completion time of all its upstream federates (plus the
 *  delay on the connection to those federates). It finds the minimum of all
 *  these times, with a caveat. If the candidate for minimum has a sufficiently
 *  large next event time that we can be sure it will provide no event before
 *  the next smallest minimum, then that candidate is ignored and the next
 *  smallest minimum determines the time.  If the resulting time to advance
 *  to does not move time forward for the federate, then no time advance
 *  grant message is sent to the federate.
 *
 *  This should be called whenever an upstream federate either registers
 *  a next event time or completes a logical time.
 *
 *  This function assumes that the caller holds the mutex lock.
 *
 *  @return True if the TAG message is sent and false otherwise.
 */
bool send_time_advance_if_appropriate(federate_t* fed) {
    // Determine whether to send a time advance grant to the downstream reactor.
    // The first candidate is its next event time. But we may need to advance
    // it to a lesser time.

    instant_t candidate_time_advance = fed->next_event;
    // Look at its upstream federates (including this one).
    for (int j = 0; j < fed->num_upstream; j++) {
        // First, find the minimum completed time or time of the
        // next event of all federates upstream from this one.
        federate_t* upstream = &federates[fed->upstream[j]];
        // There may be a delay on the connection. Add that to the candidate.
        interval_t delay = fed->upstream_delay[j];
        instant_t upstream_completion_time = upstream->completed + delay;
        // Preserve NEVER.
        if (upstream->completed == NEVER) upstream_completion_time = NEVER;
        // If the completion time of the upstream federate
        // is less than the candidate time, then we will need to use that
        // completion time unless the (transitive) next_event time of the
        // upstream federate (plus the delay) is larger than the
        // current candidate completion time.
        if (upstream_completion_time < candidate_time_advance) {
            // To handle cycles, need to create a boolean array to keep
            // track of which upstream federates have been visited.
            bool visited[NUMBER_OF_FEDERATES];
            for (int i = 0; i < NUMBER_OF_FEDERATES; i++) visited[i] = false;

            // Find the (transitive) next event time upstream.
            instant_t upstream_next_event = transitive_next_event(
                    upstream, upstream->next_event, visited);
            if (upstream_next_event != NEVER) upstream_next_event += delay;
            if (upstream_next_event <= candidate_time_advance) {
                // Cannot advance the federate to the upstream
                // next event time because that event has presumably not yet
                // been produced.
                candidate_time_advance = upstream_completion_time;
            }
        }
    }

    // If the resulting time will advance time
    // in the federate, send it a time advance grant.
    if (candidate_time_advance > fed->completed) {
        send_time_advance_grant(fed, candidate_time_advance);
        return true;
    } else {
        // printf("DEBUG: Not sending TAG to fed %d of %lld because it is not greater than the completed %lld.\n", fed->id, candidate_time_advance - start_time, fed->completed - start_time);
    }
    return false;
}

/** Handle a logical time complete (LTC) message.
 *  @param fed The federate that has completed a logical time.
 */
void handle_logical_time_complete(federate_t* fed) {
    union {
        long long ull;
        unsigned char c[sizeof(long long)];
    } buffer;
    read_from_socket(fed->socket, sizeof(long long), (unsigned char*)&buffer.c);

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&rti_mutex);

    fed->completed = swap_bytes_if_big_endian_ll(buffer.ull);
    // printf("DEBUG: RTI received from federate %d the logical time complete %lld.\n", fed->id, fed->completed - start_time);

    // Check downstream federates to see whether they should now be granted a TAG.
    for (int i = 0; i < fed->num_downstream; i++) {
        send_time_advance_if_appropriate(&federates[fed->downstream[i]]);
    }
    pthread_mutex_unlock(&rti_mutex);
}

/** Handle a next event time (NET) message.
 *  @param fed The federate sending a NET message.
 */
void handle_next_event_time(federate_t* fed) {
    union {
        long long ull;
        unsigned char c[sizeof(long long)];
    } buffer;
    read_from_socket(fed->socket, sizeof(long long), (unsigned char*)&buffer.c);

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&rti_mutex);

    fed->next_event = swap_bytes_if_big_endian_ll(buffer.ull);
    // printf("DEBUG: RTI received from federate %d the NET %lld.\n", fed->id, fed->next_event - start_time);

    // Check to see whether we can reply now with a time advance grant.
    // If the federate has no upstream federates, then it does not wait for
    // nor expect a reply. It just proceeds to advance time.
    if (fed->num_upstream > 0) {
        send_time_advance_if_appropriate(fed);
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
    read_from_socket(fed->socket, sizeof(long long), (unsigned char*)&buffer.c);

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&rti_mutex);

    instant_t stop_time = swap_bytes_if_big_endian_ll(buffer.ull);
    // printf("DEBUG: RTI received from federate %d a STOP request with time %lld.\n", fed->id, stop_time - start_time);

    // Iterate over federates and send each a STOP message.
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        if (i != fed->id) {
            if (federates[i].state == NOT_CONNECTED) continue;
            unsigned char buffer[9];
            buffer[0] = STOP;
            encode_ll(stop_time, &(buffer[1]));
            write_to_socket(federates[i].socket, 9, buffer);
            // printf("DEBUG: RTI sent to federate %d the TAG %lld.\n", fed->id, time - start_time);
        }
    }

    pthread_mutex_unlock(&rti_mutex);
}

char* ERROR_UNRECOGNIZED_MESSAGE_TYPE = "ERROR Received from federate an unrecognized message type";

/** Thread handling communication with a federate.
 *  @param fed A pointer to an int that is the
 *   socket descriptor for the federate.
 */
void* federate(void* fed) {
    federate_t* my_fed = (federate_t*)fed;

    // Buffer for incoming messages.
    // This does not constrain the message size because messages
    // are forwarded piece by piece.
    unsigned char buffer[BUFFER_SIZE];

    // Read bytes from the socket. We need 9 bytes.
    read_from_socket(my_fed->socket, sizeof(long long) + 1, buffer);

    // First byte received is the message ID.
    if (buffer[0] != TIMESTAMP) {
        fprintf(stderr, "ERROR: RTI expected a TIMESTAMP message. Got %u (see rti.h).\n", buffer[0]);
    }

    instant_t timestamp = swap_bytes_if_big_endian_ll(*((long long*)(&(buffer[1]))));
    // printf("DEBUG: RTI received message: %llx\n", timestamp);

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
    write_to_socket(my_fed->socket, 1, &message_marker);

    // Send the timestamp.
    // Add an offset to this start time to get everyone starting together.
    start_time = max_start_time + DELAY_START;
    long long message = swap_bytes_if_big_endian_ll(start_time);
    write_to_socket(my_fed->socket, sizeof(long long), (unsigned char*)(&message));

    // Update state for the federate to indicate that the TIMESTAMP
    // message has been sent. That TIMESTAMP message grants time advance to
    // the federate to the start time.
    my_fed->state = GRANTED;
    pthread_cond_broadcast(&sent_start_time);
    pthread_mutex_unlock(&rti_mutex);

    // Listen for messages from the federate.
    while (1) {
        if (my_fed->state == NOT_CONNECTED) return NULL;
        // Read no more than one byte to get the message type.
        read_from_socket(my_fed->socket, 1, buffer);
        switch(buffer[0]) {
        case MESSAGE:
            handle_message(my_fed->socket, buffer, 9);
            break;
        case TIMED_MESSAGE:
            handle_message(my_fed->socket, buffer, 17);
            break;
        case RESIGN:
            // Nothing more to do. Close the socket and exit.
            printf("Federate %d has resigned.\n", my_fed->id);
            pthread_mutex_lock(&rti_mutex);
            my_fed->state = NOT_CONNECTED;
            close(my_fed->socket); //  from unistd.h
            pthread_mutex_unlock(&rti_mutex);
            return NULL;
            break;
        case NEXT_EVENT_TIME:
            handle_next_event_time(my_fed);
            break;
        case LOGICAL_TIME_COMPLETE:
            handle_logical_time_complete(my_fed);
            break;
        case STOP:
            handle_stop_message(my_fed);
            break;
        default:
            error(ERROR_UNRECOGNIZED_MESSAGE_TYPE);
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
        if (socket_id < 0) error("ERROR on server accept");

        // The first message from the federate should be its ID.
        // Buffer for message ID plus the federate ID.
        int length = sizeof(int) + 1;
        unsigned char buffer[length];

        // Read bytes from the socket. We need 5 bytes.
        int bytes_read = 0;
        while (bytes_read < length) {
            int more = read(socket_id, &(buffer[bytes_read]),
                    length - bytes_read);
            if (more < 0) error("ERROR on RTI reading from socket");
            // If more == 0, this is an EOF. Exit the thread.
            if (more == 0) return;
            bytes_read += more;
        }
        // printf("DEBUG: read %d bytes.\n", bytes_read);

        // First byte received is the message ID.
        if (buffer[0] != FED_ID) {
            fprintf(stderr, "ERROR: RTI expected a FED_ID message. Got %u (see rti.h).\n", buffer[0]);
        }

        ushort fed_id = extract_ushort(buffer + 1);
        // printf("DEBUG: RTI received federate ID: %d\n", fed_id);

        if (fed_id >= NUMBER_OF_FEDERATES) {
            fprintf(stderr, "ERROR: Federate ID %d is required to be between zero and %d.\n", fed_id, NUMBER_OF_FEDERATES-1);
            // FIXME: Rather harsh error handling here.
            exit(1);
        }

        if (federates[fed_id].state != NOT_CONNECTED) {
            fprintf(stderr, "Duplicate federate ID: %d.\n", fed_id);
            // FIXME: Rather harsh error handling here.
            exit(1);
        }
        federates[fed_id].socket = socket_id;

        // Create a thread to communicate with the federate.
        pthread_create(&(federates[fed_id].thread_id), NULL, federate, &(federates[fed_id]));
    }
}

/** Initialize the federate with the specified ID.
 *  @param id The federate ID.
 */
void initialize_federate(int id) {
    federates[id].id = id;
    federates[id].socket = -1;      // No socket.
    federates[id].completed = NEVER;
    federates[id].next_event = NEVER;
    federates[id].state = NOT_CONNECTED;
    federates[id].upstream = NULL;
    federates[id].upstream_delay = NULL;
    federates[id].num_upstream = 0;
    federates[id].downstream = NULL;
    federates[id].num_downstream = 0;
    federates[id].mode = REALTIME;
}

/** Launch the specified executable by forking the calling process and converting
 *  the forked process into the specified executable.
 *  If forking the process fails, this will return -1.
 *  Otherwise, it will return the process ID of the created process.
 *  @param executable The executable program.
 *  @return The PID of the created process or -1 if the fork fails.
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
        fprintf(stderr, "ERROR forking the RTI process to start the executable: %s\n", executable);
    }
    return pid;
}

/** Start the socket server for the runtime infrastructure (RTI) and
 *  return the socket descriptor.
 *  @param num_feds Number of federates.
 *  @param port The port on which to listen for socket connections.
 */
int start_rti_server(int port) {
    int socket_descriptor = create_server(port);
    printf("RTI: Listening for federates on port %d.\n", port);
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

    // Wait for federate threads to exit.
    void* thread_exit_status;
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        pthread_join(federates[i].thread_id, &thread_exit_status);
        printf("RTI: Federate thread exited.\n");
    }
    // NOTE: In all common TCP/IP stacks, there is a time period,
    // typically between 30 and 120 seconds, called the TIME_WAIT period,
    // before the port is released after this close. This is because
    // the OS is preventing another program from accidentally receiving
    // duplicated packets intended for this program.

    // Before binding the socket, the RTI sets the SO_REUSEADDR (and
    // SO_REUSEPORT if applicable) in order to circumvent the "ERROR on
    // binding:Address already in use" error, which arises if the RTI 
    // is restarted within the TIME_WAIT window.
    close(socket_descriptor);
}
