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
 * The main entry point is synchronize_with_other_federates().
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>      // Defined perror(), errno
#include <sys/socket.h>
#include <netinet/in.h> // Defines struct sockaddr_in
#include <arpa/inet.h>  // inet_ntop & inet_pton
#include <unistd.h>     // Defines read(), write(), and close()
#include <netdb.h>      // Defines gethostbyname().
#include <strings.h>    // Defines bzero().
#include <pthread.h>
#include <assert.h>
#include "net_util.c"   // Defines network functions.
#include "rti.h"        // Defines TIMESTAMP.
#include "reactor.h"    // Defines instant_t.
#include "clock-sync.c"      // Defines clock synchronization functions.

// Error messages.
char* ERROR_SENDING_HEADER = "ERROR sending header information to federate via RTI";
char* ERROR_SENDING_MESSAGE = "ERROR sending message to federate via RTI";

// Mutex lock held while performing socket operations.
pthread_mutex_t socket_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * The TCP socket descriptor for this federate to communicate with the RTI.
 * This is set by connect_to_rti(), which must be called before other
 * functions that communicate with the rti are called.
 */
int _lf_rti_socket_TCP = -1;

/**
 * Number of inbound physical connections to the federate.
 * This can be either physical connections, or logical connections
 * in the decentralized coordination, or both.
 */
int _lf_number_of_inbound_p2p_connections;

/**
 * Number of outbound peer-to-peer connections from the federate.
 * This can be either physical connections, or logical connections
 * in the decentralized coordination, or both.
 */
int _lf_number_of_outbound_p2p_connections;

/**
 * An array that holds the socket descriptors for inbound
 * connections from each federate. The index will be the federate
 * ID of the remote sending federate. This is initialized at startup
 * to -1 and is set to a socket ID by handle_p2p_connections_from_federates()
 * when the socket is opened.
 * 
 * @note There will not be an inbound socket unless a physical connection
 * or a p2p logical connection (by setting the coordination target property 
 * to "distributed") is specified in the Lingua Franca program where this 
 * federate is the destination. Multiple incoming p2p connections from the 
 * same remote federate will use the same socket.
 */
int _lf_federate_sockets_for_inbound_p2p_connections[NUMBER_OF_FEDERATES];

/**
 * An array that holds the socket descriptors for outbound physical
 * connections to each remote federate. The index will be the federate
 * ID of the remote receiving federate. This is initialized at startup
 * to -1 and is set to a socket ID by connect_to_federate()
 * when the socket is opened.
 * 
 * @note This federate will not open an outbound socket unless a physical
 * connection or a p2p logical connection (by setting the coordination target
 * property to "distributed") is specified in the Lingua Franca
 * program where this federate acts as the source. Multiple outgoing p2p 
 * connections to the same remote federate will use the same socket.
 */
int _lf_federate_sockets_for_outbound_p2p_connections[NUMBER_OF_FEDERATES];

/**
 * Thread ID for a thread that accepts sockets and then supervises
 * listening to those sockets for incoming P2P (physical) connections.
 */
pthread_t _lf_inbound_p2p_handling_thread_id;

/**
 * A socket descriptor for the socket server of the federate.
 * This is assigned in create_server().
 * This socket is used to listen to incoming physical connections from
 * remote federates. Once an incoming connection is accepted, the
 * opened socket will be stored in
 * _lf_federate_sockets_for_inbound_p2p_connections.
 */
int _lf_server_socket;

/**
 * The port used for the server socket 
 * to listen for messages from other federates.
 * The federate informs the RTI of this port once
 * it has created its socket server by sending
 * an ADDRESS_AD message (@see rti.h).
 */
int _lf_server_port = -1;

/** 
 * Thread that listens for inputs from other federates.
 * This thread listens for messages of type P2P_TIMED_MESSAGE 
 * from the specified peer federate and calls schedule to 
 * schedule an event. If an error occurs or an EOF is received 
 * from the peer, then this procedure returns, terminating the 
 * thread.
 * @param fed_id_ptr A pointer to a ushort containing federate ID being listened to.
 *  This procedure frees the memory pointed to before returning.
 */
void* listen_to_federates(void* args);

/** 
 * Create a server to listen to incoming physical
 * connections from remote federates. This function
 * only handles the creation of the server socket.
 * The reserved port for the server socket is then
 * sent to the RTI by sending an ADDRESS_AD message 
 * (@see rti.h). This function expects no response
 * from the RTI.
 * 
 * If a port is specified by the user, that will be used
 * as the only possibility for the server. This function
 * will fail if that port is not available. If a port is not
 * specified, the STARTING_PORT (@see rti.h) will be used.
 * The function will keep incrementing the port in this case 
 * until the number of tries reaches PORT_RANGE_LIMIT.
 * 
 * @note This function is similar to create_server(...) in rti.c.
 * However, it contains specific log messages for the peer to
 * peer connections between federates. It also additionally 
 * sends an address advertisement (ADDRESS_AD) message to the
 * RTI informing it of the port.
 * 
 * @param specified_port The specified port by the user.
 */
void create_server(int specified_port) {
    int port = specified_port;
    if (specified_port == 0) {
        // Use the default starting port.
        port = STARTING_PORT;
    }
    DEBUG_PRINT("Creating a socket server on port %d.", port);
    // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
    int socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_descriptor < 0) {
        error_print_and_exit("Failed to obtain a socket server.");
    }

    // Server file descriptor.
    struct sockaddr_in server_fd;
    // Zero out the server address structure.
    bzero((char*)&server_fd, sizeof(server_fd));

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
        DEBUG_PRINT("Failed to get port %d. Trying %d.", port, port + 1);
        port++;
        server_fd.sin_port = htons(port);
        result = bind(
                socket_descriptor,
                (struct sockaddr *) &server_fd,
                sizeof(server_fd));
    }
    if (result != 0) {
        if (specified_port == 0) {
            error_print_and_exit("Failed to bind socket. Cannot find a usable port. \
                                 Consider increasing PORT_RANGE_LIMIT in federate.c");
        } else {
            error_print_and_exit("Failed to bind socket. Specified port is not available. \
                                 Consider leaving the port unspecified");
        }
    }
    LOG_PRINT("Server for communicating with other federates started using port %d.", port);

    // Enable listening for socket connections.
    // The second argument is the maximum number of queued socket requests,
    // which according to the Mac man page is limited to 128.
    listen(socket_descriptor, 128);

    // Set the global server port
    _lf_server_port = port;

    // Send the server port number to the RTI
    // on an ADDRESS_AD message (@see rti.h).
    unsigned char buffer[sizeof(int) + 1];
    buffer[0] = ADDRESS_AD;
    encode_int(_lf_server_port, &(buffer[1]));
    write_to_socket_errexit(_lf_rti_socket_TCP, sizeof(int) + 1, (unsigned char*)buffer,
                    "Failed to send address advertisement.");
    DEBUG_PRINT("Sent port %d to the RTI.", _lf_server_port);

    // Set the global server socket
    _lf_server_socket = socket_descriptor;
}

/**
 * Send a message to another federate directly or via the RTI.
 * 
 * @note This function is similar to send_time_message() except that it
 *  does not deal with time and timed_messages.
 * 
 * @param additional_delay The offset applied to the timestamp
 *  using after. The additional delay will be greater or equal to zero
 *  if an after is used on the connection. If no after is given in the
 *  program, -1 is passed.
 * @param socket The socket to send the message on
 * @param message_type The type of the message being sent. 
 *  Currently can be TIMED_MESSAGE for messages sent via
 *  RTI or P2P_TIMED_MESSAGE for messages sent between
 *  federates.
 * @param port The ID of the destination port.
 * @param federate The ID of the destination federate.
 * @param next_destination_str The name of the next destination in string format
 * @param length The message length.
 * @param message The message.
 */
void send_message(int socket,
                  int message_type,
                  unsigned int port,
                  unsigned int federate,
                  const char next_destination_str[],
                  size_t length,
                  unsigned char* message) {
    assert(port < 65536);
    assert(federate < 65536);
    unsigned char header_buffer[1 + sizeof(ushort) + sizeof(ushort) + sizeof(int)];
    // First byte identifies this as a timed message.
    header_buffer[0] = message_type;
    // Next two bytes identify the destination port.
    // NOTE: Send messages little endian, not big endian.
    encode_ushort(port, &(header_buffer[1]));

    // Next two bytes identify the destination federate.
    encode_ushort(federate, &(header_buffer[1 + sizeof(ushort)]));

    // The next four bytes are the message length.
    encode_int(length, &(header_buffer[1 + sizeof(ushort) + sizeof(ushort)]));

    LOG_PRINT("Sending untimed message to %s.", next_destination_str);

    // Header:  message_type + port_id + federate_id + length of message + timestamp + microstep
    const int header_length = 1 + sizeof(ushort) + sizeof(ushort) + sizeof(int);
    // Use a mutex lock to prevent multiple threads from simultaneously sending.
    pthread_mutex_lock(&socket_mutex);
    write_to_socket_errexit(socket, header_length, header_buffer, "Failed to send message header to to %s.", next_destination_str);
    write_to_socket_errexit(socket, length, message, "Failed to send message body to to %s.", next_destination_str);
    pthread_mutex_unlock(&socket_mutex);
}

/** 
 * Send the specified timestamped message to the specified port in the
 * specified federate via the RTI or directly to a federate depending on
 * the given socket. The timestamp is calculated as current_logical_time +
 * additional delay which is greater than or equal to zero.
 * The port should be an input port of a reactor in 
 * the destination federate. This version does include the timestamp 
 * in the message. The caller can reuse or free the memory after this returns.
 * This method assumes that the caller does not hold the socket_mutex lock,
 * which it acquires to perform the send.
 * 
 * @note This function is similar to send_message() except that it
 *   sends timed messages and also contains logics related to time.
 * 
 * @param additional_delay The offset applied to the timestamp
 *  using after. The additional delay will be greater or equal to zero
 *  if an after is used on the connection. If no after is given in the
 *  program, -1 is passed.
 * @param socket The socket to send the message on
 * @param message_type The type of the message being sent. 
 *  Currently can be TIMED_MESSAGE for messages sent via
 *  RTI or P2P_TIMED_MESSAGE for messages sent between
 *  federates.
 * @param port The ID of the destination port.
 * @param federate The ID of the destination federate.
 * @param next_destination_str The name of the next destination in string format
 * @param length The message length.
 * @param message The message.
 */
void send_timed_message(interval_t additional_delay,
                        int socket,
                        int message_type,
                        unsigned int port,
                        unsigned int federate,
                        const char next_destination_str[],
                        size_t length,
                        unsigned char* message) {
    assert(port < 65536);
    assert(federate < 65536);
    unsigned char header_buffer[1 + sizeof(ushort) + sizeof(ushort) + sizeof(int) + sizeof(instant_t) + sizeof(microstep_t)];
    // First byte identifies this as a timed message.
    header_buffer[0] = message_type;
    // Next two bytes identify the destination port.
    // NOTE: Send messages little endian, not big endian.
    encode_ushort(port, &(header_buffer[1]));

    // Next two bytes identify the destination federate.
    encode_ushort(federate, &(header_buffer[1 + sizeof(ushort)]));

    // The next four bytes are the message length.
    encode_int(length, &(header_buffer[1 + sizeof(ushort) + sizeof(ushort)]));

    // Get current logical time
    instant_t current_message_timestamp = get_logical_time();

    // Note that we are getting the microstep here directly
    // under the assumption that it does not change while
    // this function is executing.
    microstep_t current_message_microstep = get_microstep();
    if (additional_delay == 0LL) {
        // After was specified by the user
        // on the connection with a delay of 0.
        // In this case,
        // the tag of the outgoing message
        // should be (get_logical_time(), get_microstep() + 1).
        current_message_microstep += 1;
    } else if (additional_delay > 0LL) {
        // After was specified by the user
        // on the connection with a positive delay.
        // In this case,
        // the tag of the outgoing message
        // should be (get_logical_time() + additional_delay, 0)

        current_message_timestamp += additional_delay;
        current_message_microstep = 0;
    } else if (additional_delay == -1LL) {
        // No after delay is given by the user
        // In this case,
        // the tag of the outgoing message
        // should be (get_logical_time(), get_microstep())
    }

    // Next 8 bytes are the timestamp.
    encode_ll(current_message_timestamp, &(header_buffer[1 + sizeof(ushort) + sizeof(ushort) + sizeof(int)]));
    // Next 4 bytes are the microstep.
    encode_int(current_message_microstep, &(header_buffer[1 + sizeof(ushort) + sizeof(ushort) + sizeof(int) + sizeof(instant_t)]));
    LOG_PRINT("Sending message with tag (%lld, %u) to %s.",
            current_message_timestamp - start_time, current_message_microstep, next_destination_str);

    // Header:  message_type + port_id + federate_id + length of message + timestamp + microstep
    const int header_length = 1 + sizeof(ushort) + sizeof(ushort) + sizeof(int) + sizeof(instant_t) + sizeof(microstep_t);

    if (_lf_is_tag_after_stop_tag((tag_t){.time=current_message_timestamp,.microstep=current_message_microstep})) {
        // Message tag is past the timeout time (the stop time) so it should
        // not be sent.
        return;
    }
    // Use a mutex lock to prevent multiple threads from simultaneously sending.
    pthread_mutex_lock(&socket_mutex);
    write_to_socket_errexit(socket, header_length, header_buffer,
            "Failed to send timed message header to %s.", next_destination_str);
    write_to_socket_errexit(socket, length, message,
            "Federate %d failed to send timed message body to %s.", next_destination_str);
    pthread_mutex_unlock(&socket_mutex);
}

/** 
 * Send a time to the RTI.
 * This is not synchronized.
 * It assumes the caller is.
 * @param type The message type (NEXT_EVENT_TIME or LOGICAL_TAG_COMPLETE).
 * @param time The time of this federate's next event.
 */
void send_tag(unsigned char type, instant_t time, microstep_t microstep) {
    DEBUG_PRINT("Sending tag (%lld, %u) to the RTI.", time - start_time, microstep);
    unsigned char buffer[1 + sizeof(instant_t) + sizeof(microstep_t)];
    buffer[0] = type;
    encode_ll(time, &(buffer[1]));
    encode_int(microstep, &(buffer[1 + sizeof(instant_t)]));
    write_to_socket_errexit(_lf_rti_socket_TCP, 1 + sizeof(instant_t) + sizeof(microstep_t), buffer,
            "Failed to send tag (%lld, %u) to the RTI.", time - start_time, microstep);
}

/**
 * Thread to accept connections from other federates that send this federate
 * messages directly (not through the RTI). This thread starts a thread for
 * each accepted socket connection and then waits for all those threads to exit
 * before exiting itself.
 * @param ignored No argument needed for this thread.
 */
void* handle_p2p_connections_from_federates(void* ignored) {
    int received_federates = 0;
    pthread_t thread_ids[_lf_number_of_inbound_p2p_connections];
    while (received_federates < _lf_number_of_inbound_p2p_connections) {
        // Wait for an incoming connection request.
        struct sockaddr client_fd;
        uint32_t client_length = sizeof(client_fd);
        int socket_id = accept(_lf_server_socket, &client_fd, &client_length);
        // FIXME: Error handling here is too harsh maybe?
        if (socket_id < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            error_print("A fatal error occurred while accepting a new socket. "
                        "Federate %d will not accept connections anymore.");
            return NULL;
        }
        LOG_PRINT("Accepted new connection from remote federate.");

        int header_length = 1 + sizeof(ushort) + 1;
        unsigned char buffer[header_length];
        int bytes_read = read_from_socket(socket_id, header_length, (unsigned char*)&buffer);
        if (bytes_read != header_length || buffer[0] != P2P_SENDING_FED_ID) {
            warning_print("Federate received invalid first message on P2P socket. Closing socket.");
            if (bytes_read >= 0) {
                unsigned char response[2];
                response[0] = REJECT;
                response[1] = WRONG_SERVER;
                // Ignore errors on this response.
                write_to_socket(socket_id, 2, response);
            }
            close(socket_id);
            continue;
        }

        // Get the federation ID and check it.
        unsigned char federation_id_length = buffer[header_length - 1];
        char remote_federation_id[federation_id_length];
        bytes_read = read_from_socket(socket_id, federation_id_length, (unsigned char*)remote_federation_id);
        if (bytes_read != federation_id_length
                 || (strncmp(federation_id, remote_federation_id, strnlen(federation_id, 255)) != 0)) {
            warning_print("Received invalid federation ID. Closing socket.");
            if (bytes_read >= 0) {
                unsigned char response[2];
                response[0] = REJECT;
                response[1] = FEDERATION_ID_DOES_NOT_MATCH;
                // Ignore errors on this response.
                write_to_socket(socket_id, 2, response);
            }
            close(socket_id);
            continue;
        }

        // Extract the ID of the sending federate.
        ushort remote_fed_id = extract_ushort((unsigned char*)&(buffer[1]));
        DEBUG_PRINT("Received sending federate ID %d.", remote_fed_id);
        _lf_federate_sockets_for_inbound_p2p_connections[remote_fed_id] = socket_id;

        // Send an ACK message.
        unsigned char response = ACK;
        write_to_socket_errexit(socket_id, 1, (unsigned char*)&response,
                "Failed to write ACK in response to federate %d.",
                remote_fed_id);

        // Start a thread to listen for incoming messages from other federates.
        // We cannot pass a pointer to remote_fed_id to the thread we need to create
        // because that variable is on the stack. Instead, we malloc memory.
        // The created thread is responsible for calling free().
        ushort* remote_fed_id_copy = (ushort*)malloc(sizeof(ushort));
        if (remote_fed_id_copy == NULL) {
            error_print_and_exit("malloc failed.");
        }
        *remote_fed_id_copy = remote_fed_id;
        int result = pthread_create(&thread_ids[received_federates], NULL, listen_to_federates, remote_fed_id_copy);
        if (result != 0) {
            // Failed to create a listening thread.
            close(socket_id);
            error_print_and_exit(
                    "Failed to create a thread to listen for incoming physical connection. Error code: %d.",
                    result
            );
        }

        received_federates++;
    }

    LOG_PRINT("All remote federates are connected.");

    void* thread_exit_status;
    for (int i = 0; i < _lf_number_of_inbound_p2p_connections; i++) {
        pthread_join(thread_ids[i], &thread_exit_status);
        LOG_PRINT("Thread listening for incoming messages from other federates exited.");
    }
    return NULL;
}

/**
 * Connect to the federate with the specified id. This established
 * connection will then be used in functions such as send_timed_message() 
 * to send messages directly to the specified federate. 
 * This function first sends an ADDRESS_QUERY message to the RTI to obtain 
 * the IP address and port number of the specified federate. It then attempts 
 * to establish a socket connection to the specified federate.
 * If this fails, the program exits. If it succeeds, it sets element [id] of 
 * the _lf_federate_sockets_for_outbound_p2p_connections global array to 
 * refer to the socket for communicating directly with the federate.
 * @param remote_federate_id The ID of the remote federate.
 */
void connect_to_federate(ushort remote_federate_id) {
    int result = -1;
    int count_retries = 0;

    // Ask the RTI for port number of the remote federate.
    // The buffer is used for both sending and receiving replies.
    // The size is what is needed for receiving replies.
    unsigned char buffer[sizeof(int) + INET_ADDRSTRLEN];
    int port = -1;
    struct in_addr host_ip_addr;
    int count_tries = 0;
    while (port == -1) {
        buffer[0] = ADDRESS_QUERY;
        // NOTE: Sending messages in little endian.
        encode_ushort(remote_federate_id, &(buffer[1]));

        DEBUG_PRINT("Sending address query for federate %d.", remote_federate_id);

        write_to_socket_errexit(_lf_rti_socket_TCP, sizeof(ushort) + 1, buffer,
                "Failed to send address query for federate %d to RTI.",
                remote_federate_id);

        // Read RTI's response.
        read_from_socket_errexit(_lf_rti_socket_TCP, sizeof(int), buffer,
                "Failed to read the requested port number for federate %d from RTI.",
                remote_federate_id);

        port = extract_int(buffer);

        read_from_socket_errexit(_lf_rti_socket_TCP, sizeof(host_ip_addr), (unsigned char*)&host_ip_addr,
                "Failed to read the IP address for federate %d from RTI.",
                remote_federate_id);

        // A reply of -1 for the port means that the RTI does not know
        // the port number of the remote federate, presumably because the
        // remote federate has not yet sent an ADDRESS_AD message to the RTI.
        // Sleep for some time before retrying.
        if (port == -1) {
            if (count_tries++ >= CONNECT_NUM_RETRIES) {
                error_print_and_exit("TIMEOUT obtaining IP/port for federate %d from the RTI.",
                        remote_federate_id);
            }
            struct timespec wait_time = {0L, ADDRESS_QUERY_RETRY_INTERVAL};
            struct timespec remaining_time;
            if (nanosleep(&wait_time, &remaining_time) != 0) {
                // Sleep was interrupted.
                continue;
            }
        }
    }
    assert(port < 65536);
    assert(port > 0);

#if LOG_LEVEL > 3
    // Print the received IP address in a human readable format
    // Create the human readable format of the received address.
    // This is avoided unless LOG_LEVEL is high enough to
    // subdue the overhead caused by inet_ntop().
    char hostname[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &host_ip_addr, hostname, INET_ADDRSTRLEN);
    LOG_PRINT("Received address %s port %d for federate %d from RTI.",
            hostname, port, remote_federate_id);
#endif

    while (result < 0) {
        // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
        _lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id] = socket(AF_INET, SOCK_STREAM, 0);
        if (_lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id] < 0) {
            error_print_and_exit("Failed to create socket to federate %d.", remote_federate_id);
        }

        // Server file descriptor.
        struct sockaddr_in server_fd;
        // Zero out the server_fd struct.
        bzero((char*)&server_fd, sizeof(server_fd));

        // Set up the server_fd fields.
        server_fd.sin_family = AF_INET;    // IPv4
        server_fd.sin_addr = host_ip_addr; // Received from the RTI

        // Convert the port number from host byte order to network byte order.
        server_fd.sin_port = htons(port);
        result = connect(
            _lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id],
            (struct sockaddr *)&server_fd,
            sizeof(server_fd));

        if (result != 0) {
            error_print("Failed to connect to federate %d on port %d.", remote_federate_id, port);

            // Try again after some time if the connection failed.
            // Note that this should not really happen since the remote federate should be
            // accepting socket connections. But possibly it will be busy (in process of accepting
            // another socket connection?). Hence, we retry.
            count_retries++;
            if (count_retries > CONNECT_NUM_RETRIES) {
                // If the remote federate is not accepting the connection after CONNECT_NUM_RETRIES
                // treat it as a soft error condition and return.
                error_print("Failed to connect to federate %d after %d retries. Giving up.",
                            remote_federate_id, CONNECT_NUM_RETRIES);
                return;
            }
            warning_print("Could not connect to federate %d. Will try again every %d nanoseconds.\n",
                   remote_federate_id, ADDRESS_QUERY_RETRY_INTERVAL);
            // Wait CONNECT_RETRY_INTERVAL seconds.
            struct timespec wait_time = {0L, ADDRESS_QUERY_RETRY_INTERVAL};
            struct timespec remaining_time;
            if (nanosleep(&wait_time, &remaining_time) != 0) {
                // Sleep was interrupted.
                continue;
            }
        } else {
            size_t buffer_length = 1 + sizeof(ushort) + 1;
            unsigned char buffer[buffer_length];
            buffer[0] = P2P_SENDING_FED_ID;
            if (_lf_my_fed_id > USHRT_MAX) {
                error_print_and_exit("Too many federates! More than %d.", USHRT_MAX);
            }
            encode_ushort((ushort)_lf_my_fed_id, (unsigned char*)&(buffer[1]));
            unsigned char federation_id_length = strnlen(federation_id, 255);
            buffer[sizeof(ushort) + 1] = federation_id_length;
            write_to_socket_errexit(_lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id],
                    buffer_length, buffer,
                    "Failed to send fed_id to federate %d.", remote_federate_id);
            write_to_socket_errexit(_lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id],
                    federation_id_length, (unsigned char*)federation_id,
                    "Failed to send federation id to federate %d.",
                    remote_federate_id);

            read_from_socket_errexit(_lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id], 1, (unsigned char*)buffer,
                             "Failed to read ACK from federate %d in response to sending fed_id.", remote_federate_id);
            if (buffer[0] != ACK) {
                // Get the error code.
                read_from_socket_errexit(_lf_federate_sockets_for_outbound_p2p_connections[remote_federate_id], 1, (unsigned char*)buffer,
                                 "Failed to read error code from federate %d in response to sending fed_id.", remote_federate_id);
                error_print("Received REJECT message from remote federate (%d).", buffer[0]);
                result = -1;
                continue;
            } else {
                info_print("Connected to federate %d, port %d.", remote_federate_id, port);
            }
        }
    }
}

/**
 * Connect to the RTI at the specified host and port and return
 * the socket descriptor for the connection. If this fails, the
 * program exits. If it succeeds, it sets the _lf_rti_socket_TCP global
 * variable to refer to the socket for communicating with the RTI.
 * @param hostname A hostname, such as "localhost".
 * @param port A port number.
 */
void connect_to_rti(char* hostname, int port) {
    LOG_PRINT("Connecting to the RTI.");

    // Repeatedly try to connect, one attempt every 2 seconds, until
    // either the program is killed, the sleep is interrupted,
    // or the connection succeeds.
    // If the specified port is 0, set it instead to the start of the
    // port range.
    bool specific_port_given = true;
    if (port == 0) {
        port = STARTING_PORT;
        specific_port_given = false;
    }
    int result = -1;
    int count_retries = 0;

    while (result < 0) {
        // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
        _lf_rti_socket_TCP = socket(AF_INET, SOCK_STREAM, 0);
        if (_lf_rti_socket_TCP < 0) {
            error_print_and_exit("Creating socket to RTI.");
        }

        struct hostent *server = gethostbyname(hostname);
        if (server == NULL) {
            error_print_and_exit("ERROR, no such host for RTI: %s\n", hostname);
        }
        // Server file descriptor.
        struct sockaddr_in server_fd;
        // Zero out the server_fd struct.
        bzero((char*)&server_fd, sizeof(server_fd));

        // Set up the server_fd fields.
        server_fd.sin_family = AF_INET;    // IPv4
        bcopy((char*)server->h_addr,
             (char*)&server_fd.sin_addr.s_addr,
             server->h_length);
        // Convert the port number from host byte order to network byte order.
        server_fd.sin_port = htons(port);
        result = connect(
            _lf_rti_socket_TCP,
            (struct sockaddr *)&server_fd,
            sizeof(server_fd));
        // If this failed, try more ports, unless a specific port was given.
        if (result != 0
                && !specific_port_given
                && port >= STARTING_PORT
                && port <= STARTING_PORT + PORT_RANGE_LIMIT
        ) {
            info_print("Failed to connect to RTI on port %d. Trying %d.", port, port + 1);
            port++;
            // Wait PORT_KNOCKING_RETRY_INTERVAL seconds.
            struct timespec wait_time = {0L, PORT_KNOCKING_RETRY_INTERVAL};
            struct timespec remaining_time;
            if (nanosleep(&wait_time, &remaining_time) != 0) {
                // Sleep was interrupted.
                continue;
            }
        }
        // If this still failed, try again with the original port after some time.
        if (result < 0) {
            if (!specific_port_given && port == STARTING_PORT + PORT_RANGE_LIMIT + 1) {
                port = STARTING_PORT;
            }
            count_retries++;
            if (count_retries > CONNECT_NUM_RETRIES) {
                error_print_and_exit("Failed to connect to the RTI after %d retries. Giving up.",
                                     CONNECT_NUM_RETRIES);
            }
            info_print("Could not connect to RTI at %s. Will try again every %d seconds.",
                   hostname, CONNECT_RETRY_INTERVAL);
            // Wait CONNECT_RETRY_INTERVAL seconds.
            struct timespec wait_time = {(time_t)CONNECT_RETRY_INTERVAL, 0L};
            struct timespec remaining_time;
            if (nanosleep(&wait_time, &remaining_time) != 0) {
                // Sleep was interrupted.
                continue;
            }
        } else {
            // Have connected to an RTI, but not sure it's the right RTI.
            // Send a FED_ID message and wait for a reply.
            // Notify the RTI of the ID of this federate and its federation.
            unsigned char buffer[4];

            LOG_PRINT("Connected to an RTI. Sending federation ID for authentication.");

            // Send the message type first.
            buffer[0] = FED_ID;
            // Next send the federate ID.
            if (_lf_my_fed_id > USHRT_MAX) {
                error_print_and_exit("Too many federates! More than %d.", USHRT_MAX);
            }
            encode_ushort((ushort)_lf_my_fed_id, &buffer[1]);
            // Next send the federation ID length.
            // The federation ID is limited to 255 bytes.
            size_t federation_id_length = strnlen(federation_id, 255);
            buffer[1 + sizeof(ushort)] = (unsigned char)(federation_id_length & 0xff);

            write_to_socket_errexit(_lf_rti_socket_TCP, 2 + sizeof(ushort), buffer,
                    "Failed to send federate ID to RTI.");

            // Next send the federation ID itself.
            write_to_socket_errexit(_lf_rti_socket_TCP, federation_id_length, (unsigned char*)federation_id,
                            "Failed to send federation ID to RTI.");

            // Wait for a response.
            // The response will be REJECT if the federation ID doesn't match.
            // Otherwise, it will be either ACK or UDP_PORT, where the latter
            // is used if clock synchronization will be performed.
            unsigned char response;

            DEBUG_PRINT("Waiting for response to federation ID from the RTI.");

            read_from_socket_errexit(_lf_rti_socket_TCP, 1, &response, "Failed to read response from RTI.");
            if (response == REJECT) {
                // Read one more byte to determine the cause of rejection.
                unsigned char cause;
                read_from_socket_errexit(_lf_rti_socket_TCP, 1, &cause, "Failed to read the cause of rejection by the RTI.");
                if (cause == FEDERATION_ID_DOES_NOT_MATCH || cause == WRONG_SERVER) {
                    info_print("Connected to the wrong RTI on port %d. Trying %d.", port, port + 1);
                    port++;
                    result = -1;
                    continue;
                }
                error_print_and_exit("RTI Rejected FED_ID message with response (see rti.h): "
                        "%d. Error code: %d. Federate quits.\n", response, cause);
            } else if (response == ACK) {
                LOG_PRINT("Received acknowledgment from the RTI.");
                ushort udp_port = setup_clock_synchronization_with_rti();

                // Write the returned port number to the RTI
                unsigned char UDP_port_number[1 + sizeof(ushort)];
                UDP_port_number[0] = UDP_PORT;
                encode_ushort(udp_port, &(UDP_port_number[1]));
                write_to_socket_errexit(_lf_rti_socket_TCP, 1 + sizeof(ushort), UDP_port_number,
                            "Failed to send the UDP port number to the RTI.");
            } else {
                error_print_and_exit("Received unexpected response %u from the RTI (see rti.h).",
                        response);
            }
            info_print("Connected to RTI at %s:%d.", hostname, port);
        }
    }
}

/** 
 * Send the specified timestamp to the RTI and wait for a response.
 * The specified timestamp should be current physical time of the
 * federate, and the response will be the designated start time for
 * the federate. This procedure blocks until the response is
 * received from the RTI.
 * @param my_physical_time The physical time at this federate.
 * @return The designated start time for the federate.
 */
instant_t get_start_time_from_rti(instant_t my_physical_time) {
    // Buffer for message ID plus timestamp.
    int buffer_length = sizeof(instant_t) + 1;
    // Send the timestamp marker first.
    unsigned char buffer[buffer_length];
    buffer[0] = TIMESTAMP;
    encode_ll(my_physical_time, &(buffer[1]));
    // FIXME: Retry rather than exit.
    write_to_socket_errexit(_lf_rti_socket_TCP, buffer_length, buffer,
                    "Failed to send TIMESTAMP message to RTI.");

    // Read bytes from the socket. We need 9 bytes.
    read_from_socket_errexit(_lf_rti_socket_TCP, buffer_length, buffer,
            "Failed to read TIMESTAMP message from RTI.");
    DEBUG_PRINT("Read 9 bytes.");

    // First byte received is the message ID.
    if (buffer[0] != TIMESTAMP) {
        error_print_and_exit("Expected a TIMESTAMP message from the RTI. Got %u (see rti.h).",
                             buffer[0]);
    }

    instant_t timestamp = extract_ll(&(buffer[1]));
    info_print("Starting timestamp is: %lld.", timestamp);
    LOG_PRINT("Current physical time is: %lld.", get_physical_time());

    return timestamp;
}

/**
 * Placeholder for a generated function that returns a pointer to the
 * trigger_t struct for the action corresponding to the specified port ID.
 * @param port_id The port ID.
 * @return A pointer to a trigger_t struct or null if the ID is out of range.
 */
trigger_t* __action_for_port(int port_id);

/**
 * Version of schedule_value() similar to that in reactor_common.c
 * except that it does not acquire the mutex lock and has a special
 * behavior during startup where it can inject reactions to the reaction
 * queue if execution has not started yet.
 * It is also responsible for setting the intended tag of the 
 * network message based on the calculated delay.
 * This function assumes that the caller holds the mutex lock.
 * 
 * This is used for handling incoming timed messages to a federate.
 * 
 * 
 * @param action The action or timer to be triggered.
 * @param tag The tag of the message received over the network.
 * @param value Dynamically allocated memory containing the value to send.
 * @param length The length of the array, if it is an array, or 1 for a
 *  scalar and 0 for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule_message_received_from_network_already_locked(
        trigger_t* trigger,
        tag_t tag,
        void* value,
        int length) {

    lf_token_t* token = create_token(trigger->element_size);
    // Return value of the function
    int return_value = 0;
    // Current logical time
    instant_t current_logical_time = get_logical_time();

    // Indicates whether or not the intended tag
    // of the message (timestamp, microstep) is
    // in the future relative to the tag of this
    // federate. By default, assume it is not.
    bool message_tag_is_in_the_future = compare_tags(tag, get_current_tag()) > 0;

    // Set up the token
    token->value = value;
    token->length = length;

    // Assign the intended tag
    trigger->intended_tag = tag;

    // Calculate the extra_delay required to be passed
    // to the schedule function.
    interval_t extra_delay = tag.time - current_logical_time;

    if (!_lf_execution_started && !message_tag_is_in_the_future) {
        // If execution has not started yet,
        // there could be a special case where a message has
        // arrived on a logical connection with a tag
        // (0, 0) when this federate
        // is at tag (0, 0).
        // In this case, the schedule
        // function cannot be called since it will
        // incur a microstep (i.e., it will insert an
        // event with tag (0,1)). To check, we call
        // _lf_schedule_init_reactions, which is a special kind
        // of schedule that does not incur a microstep. This function will first
        // check the appropriate conditions and if the above-mentioned
        // conditions are not met, the return value will be 0.
        return_value = _lf_schedule_init_reactions(trigger, extra_delay, token);
        DEBUG_PRINT("Startup schedule returned %d.", return_value);
    }

    // If return_value remains 0, it means that the special startup procedure
    // does not apply or the call to _lf_schedule_init_reactions() has failed.
    // Thus, call __schedule() instead.
    if (return_value == 0) {
        if (!message_tag_is_in_the_future) {
#ifdef _LF_COORD_CENTRALIZED
            // If the coordination is centralized, receiving a message
            // that does not carry a timestamp that is in the future
            // would indicate a critical condition, showing that the
            // time advance mechanism is not working correctly.
            error_print_and_exit("Received a message at tag (%lld, %u) that"
                                 " has a tag (%lld, %u) that is in the past.",
                                 current_logical_time - start_time, get_microstep(),
                                 tag.time - start_time, tag.microstep);
#else
            // Set the delay back to 0
            extra_delay = 0LL;
            DEBUG_PRINT("Calling schedule with 0 delay and intended tag (%lld, %u).",
                        trigger->intended_tag.time - start_time,
                        trigger->intended_tag.microstep);
            return_value = __schedule(trigger, extra_delay, token);
#endif
        } else if (tag.time == current_logical_time) {
            // In case the message is in the future
            // in terms of microstep, call
            // _lf_schedule_at_tag() and pass the tag
            // of the message.
            DEBUG_PRINT("Received a message that is (%lld nanoseconds, %u microsteps) "
                    "in the future.", extra_delay, tag.microstep - get_microstep());
            return_value = _lf_schedule_at_tag(trigger, tag, token);
        } else {
            // In case the message is in the future
            // in terms of logical time, call __schedule().
            DEBUG_PRINT("Received a message that is %lld nanoseconds in the future.", extra_delay);
            return_value = __schedule(trigger, extra_delay, token);
        }
        // Notify the main thread in case it is waiting for physical time to elapse.
        DEBUG_PRINT("Broadcasting notification that event queue changed.");
        pthread_cond_broadcast(&event_q_changed);
    }
    return return_value;
}

/**
 * Handle a message being received from a remote federate.
 * 
 * This function assumes the caller does not hold the mutex lock.
 * @param socket The socket to read the message from
 * @param buffer The buffer to read
 */
void handle_message(int socket, unsigned char* buffer) {
    // Read the header.
    read_from_socket_errexit(socket, sizeof(ushort) + sizeof(ushort) + sizeof(int), buffer,
            "Failed to read message header.");
    // Extract the header information.
    unsigned short port_id;
    unsigned short federate_id;
    unsigned int length;
    extract_header(buffer, &port_id, &federate_id, &length);
    // Check if the message is intended for this federate
    assert(_lf_my_fed_id == federate_id);
    DEBUG_PRINT("Receiving message to port %d of length %d.", port_id, length);

    // Get the triggering action for the corerponding port
    trigger_t* action = __action_for_port(port_id);

    // Read the payload.
    // Allocate memory for the message contents.
    unsigned char* message_contents = (unsigned char*)malloc(length);
    int bytes_read = read_from_socket(socket, length, message_contents);
    if (bytes_read < (int)length) {
        // FIXME: Need better error handling?
        error_print_and_exit("Failed to read message body.");
    }
    LOG_PRINT("Message received by federate: %s. Length: %d.", message_contents, length);

    DEBUG_PRINT("Calling schedule for message received on a physical connection.");
    _lf_schedule_value(&action, 0, message_contents, length);
}

/**
 * Handle a timed message being received from a remote federate via the RTI
 * or directly from other federates.
 * This will read the tag encoded in the header
 * and calculate an offset to pass to the schedule function.
 * This function assumes the caller does not hold the mutex lock.
 * Instead of holding the mutex lock, this function calls 
 * _lf_increment_global_tag_barrier with the tag carried in
 * the message header as an argument. This ensures that the current tag
 * will not advance to the tag of the message if it is in the future, or
 * the tag will not advance at all if the tag of the message is
 * now or in the past.
 * @param socket The socket to read the message from.
 * @param buffer The buffer to read.
 */
void handle_timed_message(int socket, unsigned char* buffer) {
    // Read the header which contains the timestamp.
    read_from_socket_errexit(socket,
            sizeof(ushort) + sizeof(ushort) + sizeof(int) + sizeof(instant_t) + sizeof(microstep_t),
            buffer,
            "Failed to read timed message header.");
    // Extract the header information.
    unsigned short port_id;
    unsigned short federate_id;
    unsigned int length;
    extract_header(buffer, &port_id, &federate_id, &length);
    // Check if the message is intended for this federate
    assert(_lf_my_fed_id == federate_id);
    DEBUG_PRINT("Receiving message to port %d of length %d.", port_id, length);

    // Get the triggering action for the corerponding port
    trigger_t* action = __action_for_port(port_id);

    if (action->is_physical) {
        // Messages sent on physical connections should be handled via handle_message().
        error_print_and_exit("Received a timed message on a physical connection.");
    }

    // Read the tag of the message.
    tag_t tag;
    tag.time = extract_ll(&(buffer[sizeof(ushort) + sizeof(ushort) + sizeof(int)]));
    tag.microstep = extract_int(&(buffer[sizeof(ushort) + sizeof(ushort) + sizeof(int) + sizeof(instant_t)]));

#ifdef _LF_COORD_DECENTRALIZED // Only applicable for federated programs with decentralized coordination
    // For logical connections in decentralized coordination,
    // increment the barrier to prevent advancement of tag beyond
    // the received tag if possible. The following function call
    // suggests that the tag barrier be raised to the tag provided
    // by the message. If this tag is in the past, the function will cause
    // the tag to freeze at the current level.
    _lf_increment_global_tag_barrier(tag);
#endif
    LOG_PRINT("Received message with tag: (%lld, %u), Current tag: (%lld, %u).", tag.time - start_time, tag.microstep, get_elapsed_logical_time(), get_microstep());

    // Read the payload.
    // Allocate memory for the message contents.
    unsigned char* message_contents = (unsigned char*)malloc(length);
    int bytes_read = read_from_socket(socket, length, message_contents);
    if (bytes_read < length) {
#ifdef _LF_COORD_DECENTRALIZED // Only applicable for federated programs with decentralized coordination
        pthread_mutex_lock(&mutex);
        // Decrement the barrier to allow advancement of tag.
        _lf_decrement_global_tag_barrier_already_locked();
        pthread_mutex_unlock(&mutex);
#endif
        // FIXME: Better error handling?
        error_print_and_exit("Failed to read timed message body.");
    }

    // If something happens, make sure to release the barrier.
    DEBUG_PRINT("Message received: %s.", message_contents);

    // NOTE: Cannot call schedule_value(), which is what we really want to call,
    // because pthreads is too incredibly stupid and deadlocks trying to acquire
    // a lock that the calling thread already holds.
    pthread_mutex_lock(&mutex);
    // Acquire the one mutex lock to prevent logical time from advancing
    // during the call to schedule().

    DEBUG_PRINT("Calling schedule with tag (%lld, %u).", tag.time - start_time, tag.microstep);
    schedule_message_received_from_network_already_locked(action, tag, message_contents,
                                                          length);

#ifdef _LF_COORD_DECENTRALIZED // Only applicable for federated programs with decentralized coordination
    // Finally, decrement the barrier to allow the execution to continue
    // past the raised barrier
    _lf_decrement_global_tag_barrier_already_locked();
#endif

    // The mutex is unlocked here after the barrier on
    // logical time has been removed to avoid
    // the need for unecessary lock and unlock
    // operations.
    pthread_mutex_unlock(&mutex);
}

/**
 * Most recent TIME_ADVANCE_GRANT received from the RTI, or NEVER if none
 * has been received.
 * This is used to communicate between the listen_to_rti_TCP thread and the
 * main federate thread.
 * This variable should only be accessed while holding the mutex lock.
 */
tag_t _lf_last_tag = {.time = NEVER, .microstep = 0u};

/**
 * Indicator of whether a NET has been sent to the RTI and no TAG
 * yet received in reply.
 * This variable should only be accessed while holding the mutex lock.
 */
bool __tag_pending = false;

/** Handle a time advance grant (TAG) message from the RTI.
 *  This function assumes the caller does not hold the mutex lock,
 *  which it acquires to interact with the main thread, which may
 *  be waiting for a TAG (this broadcasts a condition signal).
 */
void handle_tag_advance_grant() {
    unsigned char buffer[sizeof(instant_t) + sizeof(microstep_t)];
    read_from_socket_errexit(_lf_rti_socket_TCP, sizeof(instant_t) + sizeof(microstep_t), buffer,
                     "Failed to read the time advance grant from the RTI.");

    pthread_mutex_lock(&mutex);
    _lf_last_tag.time = extract_ll(buffer);
    _lf_last_tag.microstep = extract_int(&(buffer[sizeof(instant_t)]));
    __tag_pending = false;
    LOG_PRINT("Received Time Advance Grant (TAG): (%lld, %u).", _lf_last_tag.time - start_time, _lf_last_tag.microstep);
    // Notify everything that is blocked.
    pthread_cond_broadcast(&event_q_changed);
    pthread_mutex_unlock(&mutex);
}

/**
 * Used to prevent the federate from sending a REQUEST_STOP
 * message multiple times to the RTI.
 * This variable should only be accessed while holding the mutex lock.
 */
bool federate_has_already_sent_a_stop_request_to_rti = false;

/** 
 * Send a STOP_REQUEST message to the RTI.
 * 
 * This function raises a global barrier on
 * logical time at the current time.
 * 
 * This function assumes the caller holds the mutex lock.
 */
void _lf_fd_send_stop_request_to_rti() {
    if (federate_has_already_sent_a_stop_request_to_rti == true) {
        return;
    }
    LOG_PRINT("Requesting the whole program to stop.");
    // Raise a logical time barrier at the current time
    _lf_increment_global_tag_barrier_already_locked(current_tag);
    // Send a stop request with the current tag to the RTI
    unsigned char buffer[1 + sizeof(instant_t)];
    buffer[0] = STOP_REQUEST;
    encode_ll(current_tag.time, &(buffer[1]));
    write_to_socket_errexit(_lf_rti_socket_TCP, 1 + sizeof(instant_t), buffer,
            "Failed to send stop time %lld to the RTI.", current_tag.time - start_time);
    federate_has_already_sent_a_stop_request_to_rti = true;
}

/** 
 * Handle a STOP_GRANTED message from the RTI. * 
 * 
 * This function removes the global barrier on
 * logical time raised when request_stop() was
 * called.
 * 
 * This function assumes the caller does not hold
 * the mutex lock, therefore, it acquires it.
 * 
 * FIXME: It should be possible to at least handle the situation
 * where the specified stop time is larger than current time.
 * This would require implementing a shutdown action.
 */
void handle_stop_granted_message() {
    unsigned char buffer[sizeof(instant_t)];
    read_from_socket_errexit(_lf_rti_socket_TCP, sizeof(instant_t), buffer,
            "Failed to read STOP_GRANTED time from RTI.");

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&mutex);

    tag_t received_stop_tag;
    received_stop_tag.time = extract_ll(buffer);
    LOG_PRINT("Received from RTI a STOP_GRANTED message with elapsed time %lld.\n",
            received_stop_tag.time - start_time);
    // Deduce the microstep
    if (received_stop_tag.time == current_tag.time) {
        received_stop_tag.microstep = current_tag.microstep + 1;
    } else if (received_stop_tag.time > current_tag.time) {
        received_stop_tag.microstep = 0;
    } else if (current_tag.time != FOREVER) {
        // FIXME: I see no way for current_tag.time to be FOREVER, but
        // this seems to be nondeterministically happening in PingPongDistributed. How?
        error_print("Received a stop_time %lld from the RTI that is in the past. "
                    "Stopping at the next microstep (%lld, %u).",
                    received_stop_tag.time - start_time,
                    current_tag.time - start_time,
                    current_tag.microstep + 1);
        received_stop_tag = current_tag;
        received_stop_tag.microstep++;
    }

    stop_tag = received_stop_tag;
    DEBUG_PRINT("Setting the stop tag to (%lld, %u).",
                stop_tag.time - start_time,
                stop_tag.microstep);

    _lf_decrement_global_tag_barrier_already_locked();
    // In case any thread is waiting on a condition, notify all.
    pthread_cond_broadcast(&reaction_q_changed);
    // We signal instead of broadcast under the assumption that only
    // one worker thread can call wait_until at a given time because
    // the call to wait_until is protected by a mutex lock
    pthread_cond_signal(&event_q_changed);
    pthread_mutex_unlock(&mutex);
}

/**
 * Handle a STOP_REQUEST message from the RTI.
 * 
 * This function assumes the caller does not hold
 * the mutex lock, therefore, it acquires it.
 */
void handle_stop_request_message() {
    unsigned char buffer[sizeof(instant_t)];
    read_from_socket_errexit(_lf_rti_socket_TCP, sizeof(instant_t), buffer,
            "Failed to read STOP_REQUEST time from RTI.");

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    pthread_mutex_lock(&mutex);
    // Don't send a stop tag twice
    if (federate_has_already_sent_a_stop_request_to_rti == true) {
        pthread_mutex_unlock(&mutex);
        return;
    }

    instant_t stop_time = extract_ll(buffer); // Note: ignoring the payload of the incoming stop request from the RTI
    LOG_PRINT("Received from RTI a STOP_REQUEST message with time %lld.",
             stop_time - start_time);

    unsigned char outgoing_buffer[1 + sizeof(instant_t)];
    outgoing_buffer[0] = STOP_REQUEST_REPLY;
    // Encode the current logical time
    encode_ll(current_tag.time, &(outgoing_buffer[1]));
    // Send the current logical time to the RTI. This message does not have an identifying byte since
    // since the RTI is waiting for a response from this federate.
    write_to_socket_errexit(_lf_rti_socket_TCP, 1 + sizeof(instant_t), outgoing_buffer,
            "Failed to send the answer to STOP_REQUEST to RTI.");

    // Raise a barrier at current time
    // because we are sending it to the RTI
    _lf_increment_global_tag_barrier_already_locked(current_tag);

    // A subsequent call to request_stop will be a no-op.
    federate_has_already_sent_a_stop_request_to_rti = true;

    pthread_mutex_unlock(&mutex);
}

/** 
 * Thread that listens for inputs from other federates.
 * This thread listens for messages of type P2P_TIMED_MESSAGE 
 * (@see rti.h) from the specified peer federate and calls schedule to 
 * schedule an event. If an error occurs or an EOF is received 
 * from the peer, then this procedure sets the corresponding 
 * socket in _lf_federate_sockets_for_inbound_p2p_connections 
 * to -1 and returns, terminating the thread.
 * @param fed_id_ptr A pointer to a ushort containing federate ID being listened to.
 *  This procedure frees the memory pointed to before returning.
 */
void* listen_to_federates(void* fed_id_ptr) {

    ushort fed_id = *((ushort*)fed_id_ptr);

    LOG_PRINT("Listening to federate %d.", fed_id);

    int socket_id = _lf_federate_sockets_for_inbound_p2p_connections[fed_id];

    // Buffer for incoming messages.
    // This does not constrain the message size
    // because the message will be put into malloc'd memory.
    unsigned char buffer[FED_COM_BUFFER_SIZE];

    // Listen for messages from the federate.
    while (1) {
        // Read one byte to get the message type.
        DEBUG_PRINT("Waiting for a P2P message.");
        int bytes_read = read_from_socket(socket_id, 1, buffer);
        DEBUG_PRINT("Received a P2P message of type %d.", buffer[0]);
        if (bytes_read == 0) {
            // EOF occurred. This breaks the connection.
            warning_print("Received EOF from peer federate %d. Closing the socket.", fed_id);
            close(socket_id);
            _lf_federate_sockets_for_inbound_p2p_connections[fed_id] = -1;
            break;
        } else if (bytes_read < 0) {
            error_print("P2P socket to federate %d is broken.", fed_id);
            close(socket_id);
            _lf_federate_sockets_for_inbound_p2p_connections[fed_id] = -1;
            break;
        }
        switch (buffer[0]) {
            case P2P_MESSAGE:
                LOG_PRINT("Received untimed message from federate %d.", fed_id);
                handle_message(socket_id, buffer + 1);
                break;
            case P2P_TIMED_MESSAGE:
                LOG_PRINT("Received timed message from federate %d.", fed_id);
                handle_timed_message(socket_id, buffer + 1);
                break;
            default:
                error_print("Received erroneous message type: %d. Closing the socket.", buffer[0]);
                close(socket_id);
                _lf_federate_sockets_for_inbound_p2p_connections[fed_id] = -1;
                break;
        }
    }
    free(fed_id_ptr);
    return NULL;
}

/** 
 * Thread that listens for TCP inputs from the RTI.
 *  When a physical message arrives, this calls schedule.
 */
void* listen_to_rti_TCP(void* args) {
    // Buffer for incoming messages.
    // This does not constrain the message size
    // because the message will be put into malloc'd memory.
    unsigned char buffer[FED_COM_BUFFER_SIZE];

    // Listen for messages from the federate.
    while (1) {
        // Read one byte to get the message type.
        // This will exit if the read fails.
        int bytes_read = read_from_socket(_lf_rti_socket_TCP, 1, buffer);
        if (bytes_read < 0) {
            error_print_and_exit("Socket connection to the RTI has been broken.");
        } else if (bytes_read < 1) {
            // EOF received.
            info_print("Connection to the RTI closed with an EOF.");
            return NULL;
        }
        switch (buffer[0]) {
            case TIMED_MESSAGE:
                handle_timed_message(_lf_rti_socket_TCP, &(buffer[1]));
                break;
            case TIME_ADVANCE_GRANT:
                handle_tag_advance_grant();
                break;
            case STOP_REQUEST:
                handle_stop_request_message();
                break;
            case STOP_GRANTED:
                handle_stop_granted_message();
                break;
            case PHYSICAL_CLOCK_SYNC_MESSAGE_T1:
            case PHYSICAL_CLOCK_SYNC_MESSAGE_T4:
                error_print("Federate %d received unexpected clock sync message from RTI on TCP socket.",
                            _lf_my_fed_id);
                break;
            default:
                error_print_and_exit("Received from RTI an unrecognized TCP message type: %hhx.", buffer[0]);
        }
    }
    return NULL;
}

/** 
 * Synchronize the start with other federates via the RTI.
 * This assumes that a connection to the RTI is already made 
 * and _lf_rti_socket_TCP is valid. It then sends the current logical 
 * time to the RTI and waits for the RTI to respond with a specified
 * time. It starts a thread to listen for messages from the RTI.
 * It then waits for physical time to match the specified time,
 * sets current logical time to the time returned by the RTI,
 * and then returns. If --fast was specified, then this does
 * not wait for physical time to match the logical start time
 * returned by the RTI.
 */
void synchronize_with_other_federates() {

    DEBUG_PRINT("Synchronizing with other federates.");

    // Reset the start time to the coordinated start time for all federates.
    // Note that this does not grant execution to this federate. In the centralized
    // coordination, the tag (0,0) should be explicitly sent to the RTI on a Time
    // Advance Grant message to request for permission to execute. In the decentralized
    // coordination, either the after delay on the connection must be sufficiently large
    // enough or the STP offset must be set globally to an accurate value.
    current_tag.time = get_start_time_from_rti(get_physical_time());

    start_time = current_tag.time;

    if (duration >= 0LL) {
        // A duration has been specified. Recalculate the stop time.
       stop_tag = ((tag_t) {.time = current_tag.time + duration, .microstep = 0});
    }
    
    // Start two threads to listen for incoming messages from the RTI.
    // One for TCP messages:
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, listen_to_rti_TCP, NULL);

    if (create_clock_sync_thread(&thread_id)) {
        warning_print("Failed to create thread to handle clock synchronization.");
    }

    // If --fast was not specified, wait until physical time matches
    // or exceeds the start time. Microstep is ignored.
    // Need to hold the mutex lock to call wait_until().
    pthread_mutex_lock(&mutex);
    LOG_PRINT("Waiting for start time %lld.", current_tag.time);
    // Ignore interrupts to this wait. We don't want to start executing until
    // physical time matches or exceeds the logical start time.
    while (!wait_until(current_tag.time)) {}
    DEBUG_PRINT("Done waiting for start time %lld.", current_tag.time);
    DEBUG_PRINT("Physical time is ahead of current time by %lld. This should be small.",
            get_physical_time() - current_tag.time);
    pthread_mutex_unlock(&mutex);

    // Reinitialize the physical start time to match the current physical time.
    // This will be different on each federate. If --fast was given, it could
    // be very different.
    physical_start_time = get_physical_time();
}

/** Indicator of whether this federate has upstream federates.
 *  The default value of false may be overridden in __initialize_trigger_objects.
 */
bool __fed_has_upstream = false;

/** Indicator of whether this federate has downstream federates.
 *  The default value of false may be overridden in __initialize_trigger_objects.
 */
bool __fed_has_downstream = false;

/**
 * In some situations, federates can send logical_tag_complete for
 * the same tag twice or more in-a-row to the RTI. For example, when 
 * __next() returns without advancing tag. To prevent overwhelming 
 * the RTI with extra messages, record the last sent logical tag 
 * complete message and check against it in 
 * _lf_logical_tag_complete().
 * 
 * @note Here, the underlying assumption is that the TCP stack will
 *  deliver the Logical TAG Complete message to the RTI eventually
 *  if it is deliverable
 */
tag_t last_sent_logical_tag_complete = (tag_t) {.time = NEVER, .microstep = 0u};

/** 
 * Send a logical tag complete (LTC) message to the RTI.
 * This function assumes the caller holds the mutex lock.
 * 
 * @param time The time of the tag
 * @param microstep The microstep of the tag
 */
void _lf_logical_tag_complete(instant_t time, microstep_t microstep) {
    tag_t tag_to_send = (tag_t){.time=time, .microstep=microstep};
    int compare_with_last_tag = compare_tags(last_sent_logical_tag_complete, tag_to_send);
    if (compare_with_last_tag == 0) {
        // Sending this tag more than once can happen if __next() returns without
        // adding an event to the event queue (and thus not advancing tag).
        DEBUG_PRINT("Was trying to send logical tag complete (%lld, %u) twice to the RTI.", 
                           tag_to_send.time - start_time,
                           tag_to_send.microstep);
        return;
    } else if (compare_with_last_tag > 0) {
        // This is a critical error. The federate is trying to inform the RTI of
        // the completion of a tag when it has already reported a larger tag as completed.
        error_print_and_exit("Was trying to send logical tag complete (%lld, %u) out of order to the RTI "
                    "when it has already sent (%lld, %u).", 
                    tag_to_send.time - start_time,
                    tag_to_send.microstep,
                    last_sent_logical_tag_complete.time - start_time,
                    last_sent_logical_tag_complete.microstep);
        return;
    }
    LOG_PRINT("Handling the completion of logical tag (%lld, %u).", time - start_time, microstep);
    send_tag(LOGICAL_TAG_COMPLETE, time, microstep);
    last_sent_logical_tag_complete = tag_to_send;
}

/** 
 * If this federate depends on upstream federates or sends data to downstream
 * federates, then notify the RTI of the next event on the event queue.
 * If there are upstream federates, then this will block until either the
 * RTI grants the advance to the requested time or the wait for the response
 * from the RTI is interrupted by a change in the event queue (e.g., a
 * physical action triggered).  This returns either the specified time or
 * a lesser time when it is safe to advance logical time to the returned time.
 * The returned time may be less than the specified time if there are upstream
 * federates and either the RTI responds with a lesser time or
 * the wait for a response from the RTI is interrupted by a
 * change in the event queue.
 * This function assumes the caller holds the mutex lock.
 */
tag_t __next_event_tag(instant_t time, microstep_t microstep) {
    if (!__fed_has_downstream && !__fed_has_upstream) {
        // This federate is not connected (except possibly by physical links)
        // so there is no need for the RTI to get involved.

        // NOTE: If the event queue is empty, then the time argument is either
        // the timeout_time or FOREVER. If -fast is also set, then
        // it matters whether there are upstream federates connected by physical
        // connections, which do not affect __fed_has_upstream. Perhaps we
        // should not return immediately because
        // then the execution will hit its timeout_time and fail to receive any
        // messages sent by upstream federates.
        // However, -fast is really incompatible with federated execution with
        // physical connections, so I don't think we need to worry about this.
        return (tag_t){.time = time, .microstep = microstep};
    }

    // FIXME: The returned value t is a promise that, absent inputs from
    // another federate, this federate will not produce events earlier than t.
    // But if there are downstream federates and there is
    // a physical action (not counting receivers from upstream federates),
    // then we can only promise up to current physical time.
    // This will result in this federate busy waiting, looping through this code
    // and notifying the RTI with next_event_tag(current_physical_time())
    // repeatedly.

    // If there are upstream federates, then we need to wait for a
    // reply from the RTI.

    // If time advance has already been granted for this tag or a larger
    // tag, then return immediately.
    if (compare_tags(_lf_last_tag, (tag_t){.time=time, .microstep=microstep}) >= 0) {
        return (tag_t){.time = time, .microstep = microstep};
    }

    send_tag(NEXT_EVENT_TIME, time, microstep);
    LOG_PRINT("Sent next event tag (NET) (%lld, %u) to RTI.", time - start_time, microstep);

    // If there are no upstream federates, return immediately, without
    // waiting for a reply. This federate does not need to wait for
    // any other federate.
    // NOTE: If fast execution is being used, it may be necessary to
    // throttle upstream federates.
    // FIXME: As noted above, this is not correct if the time is the timeout_time.
    if (!__fed_has_upstream) {
        return (tag_t){.time = time, .microstep = microstep};
    }

    __tag_pending = true;
    while (__tag_pending) {
        // Wait until either something changes on the event queue or
        // the RTI has responded with a TAG.
        DEBUG_PRINT("Waiting for changes on the event queue.");
        if (pthread_cond_wait(&event_q_changed, &mutex) != 0) {
            error_print("Wait error.");
        }

        if (__tag_pending) {
            // The RTI has not replied, so the wait must have been
            // interrupted by activity on the event queue.
            // If there is now an earlier event on the event queue,
            // then we should return with the time of that event.
            event_t *head_event = (event_t *)pqueue_peek(event_q);
            if (head_event != NULL && head_event->time < time) {
                if (head_event->time == current_tag.time) {
                    microstep = get_microstep() + 1;
                } else {
                    microstep = 0u;
                }
                LOG_PRINT("RTI has not replied to NET, but an earlier event was detected on the event queue.");

                return (tag_t){.time = head_event->time, .microstep = microstep};
            }
            // If we get here, any activity on the event queue is not relevant.
            // Either the queue is empty or whatever appeared on it
            // has a timestamp greater than this request.
            // Keep waiting for the TAG.
        }
    }

    return _lf_last_tag;
}
