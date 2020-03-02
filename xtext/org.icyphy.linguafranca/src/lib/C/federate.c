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
 * Utility functions for a federate in a federated execution.
 */


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>      // Defined perror(), errno
#include <sys/socket.h>
#include <netinet/in.h> // Defines struct sockaddr_in
#include <unistd.h>     // Defines read(), write(), and close()
#include <netdb.h>      // Defines gethostbyname().
#include <strings.h>    // Defines bzero().
#include "util.c"       // Defines error() and swap_bytes_if_little_endian().
#include "rti.h"        // Defines TIMESTAMP.
#include "reactor.h"    // Defines instant_t.

/** Connect to the RTI at the specified host and port and return
 *  the socket descriptor for the connection. If this fails, the
 *  program exits.
 *  @param hostname A hostname, such as "localhost".
 *  @param port A port number.
 *  @return A socket descriptor.
 */
int connect_to_rti(char* hostname, int port) {
    int socket_descriptor;
    // Repeatedly try to connect, one attempt every 2 seconds, until
    // either the program is killed, the sleep is interrupted,
    // or the connection succeeds.
    int result = -1;
    while (result < 0) {
        // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
        socket_descriptor = socket(AF_INET , SOCK_STREAM , 0);
        if (socket_descriptor < 0) error("ERROR on federate creating socket to RTI");

        struct hostent *server = gethostbyname(hostname);
        if (server == NULL) {
            fprintf(stderr,"ERROR, no such host for RTI.\n");
            exit(1);
        }
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
        server_fd.sin_port = htons(port);

        result = connect(
            socket_descriptor,
            (struct sockaddr *)&server_fd,
            sizeof(server_fd));
        if (result < 0) {
            printf("Could not connect to RTI at %s, port %d. Will try again every 2 seconds.\n", hostname, port);
            // Wait two seconds.
            struct timespec wait_time = {(time_t)2, 0L};
            struct timespec remaining_time;
            if (nanosleep(&wait_time, &remaining_time) != 0) {
                // Sleep was interrupted.
                break;
            }
        }
    }
    printf("Federate: connected to RTI at %s, port %d.\n", hostname, port);
    return socket_descriptor;
}

/** Send the specified timestamp to the RTI and wait for a response.
 *  The specified timestamp should be current physical time of the
 *  federate, and the response will be the designated start time for
 *  the federate. This proceedure blocks until the response is
 *  received from the RTI.
 *  @param socket_descriptor The socket descriptor returned by
 *   connect_to_rti.
 *  @param my_physical_time The physical time at this federate.
 *  @return The designated start time for the federate.
 */
instant_t get_start_time_from_rti(int socket_descriptor, instant_t my_physical_time) {
    // Send the timestamp marker first.
    unsigned char message_marker = TIMESTAMP;
    int bytes_written = write(socket_descriptor, &message_marker, 1);
    // FIXME: Retry rather than exit.
    if (bytes_written < 0) error("ERROR sending message ID to RTI");

    // Send the timestamp.
    long long message = swap_bytes_if_little_endian_ll(my_physical_time);
    /*
    for (int i = 0; i < sizeof(long long); i++) {
        printf("DEBUG: sending %d: %u\n", i, ((unsigned char*)(&message))[i]);
    }
    */
    bytes_written = write(socket_descriptor, (void*)(&message), sizeof(long long));
    if (bytes_written < 0) error("ERROR sending message to RTI");

    // Get a reply.
    // Buffer for message ID plus timestamp.
    unsigned char buffer[sizeof(long long) + 1];

    // Read bytes from the socket. We need 9 bytes.
    int bytes_read = 0;
    while (bytes_read < 9) {
        int more = read(socket_descriptor, &(buffer[bytes_read]),
                sizeof(long long) + 1 - bytes_read);
        if (more < 0) error("ERROR on federate reading reply from RTI");
        // If more == 0, this is an EOF. Kill the federate.
        if (more == 0) {
            fprintf(stderr, "RTI sent an EOF. Exiting.");
            exit(1);
        }
        bytes_read += more;
    }
    /*
    printf("DEBUG: federate read %d bytes.\n", bytes_read);
    for (int i = 0; i < sizeof(long long) + 1; i++) {
        printf("DEBUG: federate received byte %d: %u\n", i, buffer[i]);
    }
    */

    // First byte received in the message ID.
    if (buffer[0] != TIMESTAMP) {
        fprintf(stderr,
                "ERROR: Federate expected a TIMESTAMP message from the RTI. Got %u (see rti.h).\n",
                buffer[0]);
        exit(1);
    }

    instant_t timestamp = swap_bytes_if_little_endian_ll(*((long long*)(&(buffer[1]))));
    printf("Federate: starting timestamp is: %llx\n", timestamp);

    return timestamp;
}

/** Synchronize the start with other federates via the RTI.
 *  This initiates a connection with the RTI, then
 *  sends the current logical time to the RTI and waits for the
 *  RTI to respond with a specified time.
 *  It then waits for physical time to match the specified time,
 *  sets current logical time to the time returned by the RTI,
 *  and then returns.
 *  @param hostname The name of the RTI host, such as "localhost".
 *  @param port The port used by the RTI.
 */
void synchronize_with_other_federates(char* hostname, int port) {

    // printf("DEBUG: Federate synchronizing with other federates.\n");

    // Connect to the RTI.
    int rti_socket = connect_to_rti(hostname, port);

    // Reset the start time to the coordinated start time for all federates.
    current_time = get_start_time_from_rti(rti_socket, current_time);

    start_time = current_time;

    if (duration >= 0LL) {
        // A duration has been specified. Recalculate the stop time.
        stop_time = current_time + duration;
    }
    // If --fast was not specified, wait until physical time matches
    // or exceeds the start time.
    wait_until(current_time);
}
