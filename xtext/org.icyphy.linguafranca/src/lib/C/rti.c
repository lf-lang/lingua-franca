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
#include "util.c"       // Defines error() and swap_bytes_if_big_endian().
#include "rti.h"        // Defines TIMESTAMP. Includes <pthread.h> and "reactor.h".

/** Delay the start of all federates by this amount. */
#define DELAY_START SEC(1)

// The one and only mutex lock.
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// Condition variable used to signal receipt of all proposed start times.
pthread_cond_t received_start_times = PTHREAD_COND_INITIALIZER;

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
    // printf("DEBUG: RTI forwarding message to port %d of federate %d of length %d.\n", port_id, federate_id, length);

    unsigned int total_bytes_to_read = length + header_size;
    unsigned int bytes_to_read = length;
    // Prevent a buffer overflow.
    if (bytes_to_read + header_size > BUFFER_SIZE) bytes_to_read = BUFFER_SIZE - header_size;

    read_from_socket(sending_socket, bytes_to_read, buffer + header_size);
    int bytes_read = bytes_to_read + header_size;
    // printf("DEBUG: Message received by RTI: %s.\n", buffer + header_size);

    // If the destination federate is no longer connected, issue a warning
    // and return.
    if (federates[federate_id].state == NOT_CONNECTED) {
        printf("RTI: Destination federate %d is no longer connected. Dropping message.\n",
                federate_id
        );
        return;
    }

    // Forward the message or message chunk.
    int destination_socket = federates[federate_id].socket;

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

    // Send back to the federate the maximum time plus an offset.
    // Start by sending a timestamp marker.
    unsigned char message_marker = TIMESTAMP;
    write_to_socket(my_fed->socket, 1, &message_marker);

    // Send the timestamp.
    // Add an offset to this start time to get everyone starting together.
    long long message = swap_bytes_if_big_endian_ll(max_start_time + DELAY_START);
    write_to_socket(my_fed->socket, sizeof(long long), (unsigned char*)(&message));

    // Listen for messages from the federate.
    while (1) {
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
            pthread_mutex_lock(&mutex);
            my_fed->state = NOT_CONNECTED;
            close(my_fed->socket); //  from unistd.h
            pthread_mutex_unlock(&mutex);
            return NULL;
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

        int fed_id = swap_bytes_if_big_endian_int(*((int*)(&(buffer[1]))));
        // printf("DEBUG: RTI received federate ID: %d\n", fed_id);

        if (federates[fed_id].state != NOT_CONNECTED) {
            fprintf(stderr, "Duplicate federate ID: %d.\n", fed_id);
            // FIXME: Rather harsh error handling here.
            exit(1);
        }
        // Default state is as if an NMR has been granted for the start time.
        federates[fed_id].state = GRANTED;
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
    federates[id].nmr = NEVER;      // No NMR.
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
    close(socket_descriptor);
}
