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
#include <errno.h>      // Defined perror(), errno
#include <sys/socket.h>
#include <sys/types.h>  // Provides select() function to read from multiple sockets.
#include <netinet/in.h> // Defines struct sockaddr_in
#include <unistd.h>     // Defines read(), write(), and close()
#include <netdb.h>      // Defines gethostbyname().
#include <strings.h>    // Defines bzero().
#include <pthread.h>
#include <assert.h>
#include "util.c"       // Defines error() and swap_bytes_if_big_endian().
#include "rti.h"        // Defines TIMESTAMP.
#include "reactor.h"    // Defines instant_t.

// The one and only mutex lock.
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// Condition variable used to signal receipt of all proposed start times.
pthread_cond_t received_start_times = PTHREAD_COND_INITIALIZER;

// State of a federate.
typedef enum fed_state_t {
    NOT_CONNECTED,  // The federate has not connected.
    GRANTED,        // Most recent NMR has been granted.
    PENDING         // Waiting for upstream federates.
} fed_state_t;

// Struct for a federate.
typedef struct federate_t {
    pthread_t thread_id;    // The ID of the thread handling communication with this federate.
    int socket;             // The socket descriptor for communicating with this federate.
    instant_t nmr;          // Most recent NMR tag received from each federate (or NEVER).
    fed_state_t state;      // State of the federate.
    struct federate_t* upstream;   // Array of upstream federates.
    int num_upstream;       // Size of the array of upstream federates.
    struct federate_t* downstream; // Array of downstream federates.
    int num_downstream;     // Size of the array of downstream federates.
} federate_t;

// The federates.
federate_t federates[NUMBER_OF_FEDERATES];

// Maximum start time seen so far from the federates.
instant_t max_start_time = 0LL;

// Number of federates that have proposed start times.
int num_feds_proposed_start = 0;

/** Create a server and enable listening for socket connections.
 *  @param port The port number to use.
 *  @return The socket descriptor on which to accept connections.
 */
int create_server(int port) {
    // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
    int socket_descriptor = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_descriptor < 0) error("ERROR on creating RTI socket");

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

/** Handle a message being sent from one federate to another via the RTI.
 *  @param sending_socket The identifier for the sending socket.
 *  @param buffer The message.
 *  @param bytes_read The number of bytes read from the socket and already
 *   in the buffer.
 */
void handle_message(int sending_socket, unsigned char* buffer, int bytes_read) {
    // The message is a message to forward to a federate.
    // The first byte, which has value MESSAGE, has already been read.
    // We need 8 more bytes:
    //   - two bytes with the ID of the destination port.
    //   - two bytes with the destination federate ID.
    //   - four bytes after that will be the length of the message.
    // We keep the MESSAGE byte in the buffer for forwarding.
    int min_bytes = 9;
    while (bytes_read < min_bytes) {
        int more = read(sending_socket, &(buffer[bytes_read]), min_bytes - bytes_read);
        if (more < 0) error("ERROR on RTI reading from federate socket");
        bytes_read += more;
    }
    // The next two bytes are the ID of the destination reactor.
    unsigned short port_id
            = swap_bytes_if_big_endian_ushort(
              *((unsigned short*)(buffer + 1)));
    // The next four bytes are the message length.
    // The next two bytes are the ID of the destination federate.
    unsigned short federate_id
            = swap_bytes_if_big_endian_ushort(
              *((unsigned short*)(buffer + 3)));
    // FIXME: Better error handling needed here.
    assert(federate_id < NUMBER_OF_FEDERATES);
    // The next four bytes are the message length.
    unsigned int length
            = swap_bytes_if_big_endian_int(
              *((unsigned int*)(buffer + 5)));

    printf("DEBUG: RTI forwarding message to port %d of federate %d of length %d.\n", port_id, federate_id, length);

    unsigned int bytes_to_read = length + min_bytes;
    // Prevent a buffer overflow.
    if (bytes_to_read > BUFFER_SIZE) bytes_to_read = BUFFER_SIZE;

    while (bytes_read < bytes_to_read) {
        int more = read(sending_socket, &(buffer[min_bytes]), bytes_to_read - bytes_read);
        if (more < 0) error("ERROR on RTI reading from federate socket");
        bytes_read += more;
    }
    printf("DEBUG: Message received by RTI: %s.\n", &(buffer[9]));

    // Forward the message or message chunk.
    int destination_socket = federates[federate_id].socket;
    int bytes_written = 0;
    while (bytes_written < bytes_read) {
        int more = write(destination_socket, &(buffer[bytes_written]), bytes_read - bytes_written);
        if (more < 0) error("ERROR forwarding message to federate");
        bytes_written += more;
    }

    // The message length may be longer than the buffer,
    // in which case we have to handle it in chunks.
    int total_bytes_read = bytes_read;
    while (total_bytes_read < length + min_bytes) {
        bytes_to_read = length + min_bytes;
        if (bytes_to_read > BUFFER_SIZE) bytes_to_read = BUFFER_SIZE;
        int more = read(sending_socket, buffer, bytes_to_read);
        if (more < 0) error("ERROR on RTI reading from federate socket");
        int bytes_written = 0;
        while (bytes_written < more) {
            int more_written = write(destination_socket, &(buffer[bytes_written]), more - bytes_written);
            if (more < 0) error("ERROR forwarding message to federate");
            bytes_written += more;
        }
        total_bytes_read += more;
    }
}

/** Thread for a federate.
 *  @param fed_socket_descriptor A pointer to an int that is the
 *   socket descriptor for the federate.
 */
void* federate(void* fed_socket_descriptor) {
    int fed_socket = *((int*)fed_socket_descriptor);

    // Buffer for incoming messages.
    // This does not constrain the message size because messages
    // are forwarded piece by piece.
    unsigned char buffer[BUFFER_SIZE];

    // Read bytes from the socket. We need 9 bytes.
    int bytes_read = 0;
    while (bytes_read < sizeof(long long) + 1) {
        int more = read(fed_socket, &(buffer[bytes_read]),
                sizeof(long long) + 1 - bytes_read);
        if (more < 0) error("ERROR on RTI reading from socket");
        // If more == 0, this is an EOF. Exit the thread.
        if (more == 0) return NULL;
        bytes_read += more;
    }
    /*
    printf("DEBUG: read %d bytes.\n", bytes_read);
    for (int i = 0; i < sizeof(long long) + 1; i++) {
        printf("DEBUG: received byte %d: %u\n", i, buffer[i]);
    }
    */

    // First byte received is the message ID.
    if (buffer[0] != TIMESTAMP) {
        fprintf(stderr, "ERROR: RTI expected a TIMESTAMP message. Got %u (see rti.h).\n", buffer[0]);
    }

    instant_t timestamp = swap_bytes_if_big_endian_ll(*((long long*)(&(buffer[1]))));
    // printf("DEBUG: RTI received message: %llx\n", timestamp);

    pthread_mutex_lock(&mutex);
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
            pthread_cond_wait(&received_start_times, &mutex);
        }
    }
    pthread_mutex_unlock(&mutex);

    // Send back to the federate the maximum time.
    // FIXME: Should perhaps increment this time stamp by some amount?
    // Otherwise, the start time will be late by rountrip communication time
    // compared to physical time.

    // Start by sending a timestamp marker.
    unsigned char message_marker = TIMESTAMP;
    int bytes_written = write(fed_socket, &message_marker, 1);
    // FIXME: Retry rather than exit.
    if (bytes_written < 0) error("ERROR sending message ID to federate");

    // Send the timestamp.
    long long message = swap_bytes_if_big_endian_ll(max_start_time);
    bytes_written = write(fed_socket, (void*)(&message), sizeof(long long));
    if (bytes_written < 0) error("ERROR sending start time to federate");

    // Listen for messages from the federate.
    while (1) {
        // Read no more than one byte to get the message type.
        bytes_read = read(fed_socket, &buffer, 1);
        // FIXME: Need more robust error handling. This will kill the RTI.
        if (bytes_read < 0) error("ERROR on RTI reading from federate socket");
        if (bytes_read == 0 || buffer[0] == RESIGN) {
            printf("Federate has resigned.\n");
            break;
        } else if (buffer[0] == MESSAGE) {
            handle_message(fed_socket, buffer, bytes_read);
        }
    }

    // Nothing more to do. Close the socket and exit.
    close(fed_socket); //  from unistd.h

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
        /*
        printf("DEBUG: read %d bytes.\n", bytes_read);
        for (int i = 0; i < sizeof(long long) + 1; i++) {
            printf("DEBUG: received byte %d: %u\n", i, buffer[i]);
        }
        */

        // First byte received in the message ID.
        if (buffer[0] != FED_ID) {
            fprintf(stderr, "ERROR: RTI expected a FED_ID message. Got %u (see rti.h).\n", buffer[0]);
        }

        int fed_id = swap_bytes_if_big_endian_int(*((int*)(&(buffer[1]))));
        printf("DEBUG: RTI received federate ID: %d\n", fed_id);

        if (federates[fed_id].state != NOT_CONNECTED) {
            fprintf(stderr, "Duplicate federate ID: %d.\n", fed_id);
            // FIXME: Rather harsh error handling here.
            exit(1);
        }
        // Default state is as if an NMR has been granted for the start time.
        federates[fed_id].state = GRANTED;
        federates[fed_id].socket = socket_id;

        // Create a thread to communicate with the federate.
        pthread_create(&(federates[fed_id].thread_id), NULL, federate, &(federates[fed_id].socket));
    }
}

/** Initialize the federate with the specified ID.
 *  @param id The federate ID.
 */
void initialize_federate(int id) {
    federates[id].socket = -1;      // No socket.
    federates[id].nmr = NEVER;      // No NMR.
    federates[id].state = NOT_CONNECTED;
    federates[id].upstream = NULL;
    federates[id].num_upstream = 0;
    federates[id].downstream = NULL;
    federates[id].num_downstream = 0;
}

/** Start a runtime infrastructure (RTI) for the specified number of
 *  federates that listens for socket connections on the specified port.
 *  @param num_feds Number of federates.
 *  @param port The port on which to listen for socket connections.
 */
void start_rti(int port) {
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        initialize_federate(i);
    }

    int socket_descriptor = create_server(port);

    printf("RTI: Listening for federates on port %d.\n", port);

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
    close(socket_descriptor);
}
