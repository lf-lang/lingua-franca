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
#include <assert.h>
#include <signal.h>     // Defines sigaction.
#include "net_util.c"   // Defines network functions.
#include "net_common.h" // Defines message types, etc.
#include "../reactor.h"    // Defines instant_t.
#include "../platform.h"
#include "../threaded/scheduler.h"
#include "clock-sync.c" // Defines clock synchronization functions.
#include "federate.h"   // Defines federate_instance_t
#include "regex.h"

// Error messages.
char* ERROR_SENDING_HEADER = "ERROR sending header information to federate via RTI";
char* ERROR_SENDING_MESSAGE = "ERROR sending message to federate via RTI";

// Mutex lock held while performing socket write and close operations.
lf_mutex_t outbound_socket_mutex;
lf_cond_t port_status_changed;

/**
 * The state of this federate instance.
 */
federate_instance_t _fed = {
        .socket_TCP_RTI = -1,
        .number_of_inbound_p2p_connections = 0,
        .inbound_socket_listeners = NULL,
        .number_of_outbound_p2p_connections = 0,
        .sockets_for_inbound_p2p_connections = { -1 },
        .sockets_for_outbound_p2p_connections = { -1 },
        .inbound_p2p_handling_thread_id = 0,
        .server_socket = -1,
        .server_port = -1,
        .last_TAG = {.time = NEVER, .microstep = 0u},
        .is_last_TAG_provisional = false,
        .waiting_for_TAG = false,
        .has_upstream = false,
        .has_downstream = false,
        .sent_a_stop_request_to_rti = false,
        .last_sent_LTC = (tag_t) {.time = NEVER, .microstep = 0u},
        .last_sent_NET = (tag_t) {.time = NEVER, .microstep = 0u},
        .min_delay_from_physical_action_to_federate_output = NEVER,
        .triggers_for_network_input_control_reactions = NULL,
        .triggers_for_network_input_control_reactions_size = 0,
        .trigger_for_network_output_control_reactions = NULL
};


federation_metadata_t federation_metadata = {
    .federation_id =  "Unidentified Federation",
    .rti_host = NULL,
    .rti_port = -1,
    .rti_user = NULL
};


/** 
 * Thread that listens for inputs from other federates.
 * This thread listens for messages of type MSG_TYPE_P2P_TAGGED_MESSAGE
 * from the specified peer federate and calls schedule to 
 * schedule an event. If an error occurs or an EOF is received 
 * from the peer, then this procedure returns, terminating the 
 * thread.
 * @param fed_id_ptr A pointer to a uint16_t containing federate ID being listened to.
 *  This procedure frees the memory pointed to before returning.
 */
void* listen_to_federates(void* args);


/**
 * Generated function that sends information about connections between this federate and
 * other federates where messages are routed through the RTI. Currently, this
 * only includes logical connections when the coordination is centralized. This
 * information is needed for the RTI to perform the centralized coordination.
 * @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h
 */
void send_neighbor_structure_to_RTI(int rti_socket);

/** 
 * Create a server to listen to incoming physical
 * connections from remote federates. This function
 * only handles the creation of the server socket.
 * The reserved port for the server socket is then
 * sent to the RTI by sending an MSG_TYPE_ADDRESS_ADVERTISEMENT message
 * (@see net_common.h). This function expects no response
 * from the RTI.
 * 
 * If a port is specified by the user, that will be used
 * as the only possibility for the server. This function
 * will fail if that port is not available. If a port is not
 * specified, the STARTING_PORT (@see net_common.h) will be used.
 * The function will keep incrementing the port in this case 
 * until the number of tries reaches PORT_RANGE_LIMIT.
 * 
 * @note This function is similar to create_server(...) in rti.c.
 * However, it contains specific log messages for the peer to
 * peer connections between federates. It also additionally 
 * sends an address advertisement (MSG_TYPE_ADDRESS_ADVERTISEMENT) message to the
 * RTI informing it of the port.
 * 
 * @param specified_port The specified port by the user.
 */
void create_server(int specified_port) {
    if (specified_port > UINT16_MAX ||
        specified_port < 0) {
        lf_print_error(
            "create_server(): The specified port (%d) is out of range."
            " Starting with %d instead.",
            specified_port,
            STARTING_PORT
        );
        specified_port = 0;
    }
    uint16_t port = (uint16_t)specified_port;
    if (specified_port == 0) {
        // Use the default starting port.
        port = STARTING_PORT;
    }
    LF_PRINT_DEBUG("Creating a socket server on port %d.", port);
    // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
    int socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_descriptor < 0) {
        lf_print_error_and_exit("Failed to obtain a socket server.");
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
        LF_PRINT_DEBUG("Failed to get port %d. Trying %d.", port, port + 1);
        port++;
        server_fd.sin_port = htons(port);
        result = bind(
                socket_descriptor,
                (struct sockaddr *) &server_fd,
                sizeof(server_fd));
    }
    if (result != 0) {
        if (specified_port == 0) {
            lf_print_error_and_exit("Failed to bind socket. Cannot find a usable port. \
                                 Consider increasing PORT_RANGE_LIMIT in federate.c");
        } else {
            lf_print_error_and_exit("Failed to bind socket. Specified port is not available. \
                                 Consider leaving the port unspecified");
        }
    }
    LF_PRINT_LOG("Server for communicating with other federates started using port %d.", port);

    // Enable listening for socket connections.
    // The second argument is the maximum number of queued socket requests,
    // which according to the Mac man page is limited to 128.
    listen(socket_descriptor, 128);

    // Set the global server port
    _fed.server_port = port;

    // Send the server port number to the RTI
    // on an MSG_TYPE_ADDRESS_ADVERTISEMENT message (@see net_common.h).
    unsigned char buffer[sizeof(int32_t) + 1];
    buffer[0] = MSG_TYPE_ADDRESS_ADVERTISEMENT;
    encode_int32(_fed.server_port, &(buffer[1]));
    write_to_socket_errexit(_fed.socket_TCP_RTI, sizeof(int32_t) + 1, (unsigned char*)buffer,
                    "Failed to send address advertisement.");
    LF_PRINT_DEBUG("Sent port %d to the RTI.", _fed.server_port);

    // Set the global server socket
    _fed.server_socket = socket_descriptor;
}

/**
 * Send a message to another federate directly or via the RTI.
 * This method assumes that the caller does not hold the outbound_socket_mutex lock,
 * which it acquires to perform the send.
 *
 * If the socket connection to the remote federate or the RTI has been broken,
 * then this returns 0 without sending. Otherwise, it returns 1.
 * 
 * @note This function is similar to send_timed_message() except that it
 *  does not deal with time and timed_messages.
 * 
 * @param message_type The type of the message being sent. 
 *  Currently can be MSG_TYPE_TAGGED_MESSAGE for messages sent via
 *  RTI or MSG_TYPE_P2P_TAGGED_MESSAGE for messages sent between
 *  federates.
 * @param port The ID of the destination port.
 * @param federate The ID of the destination federate.
 * @param next_destination_str The name of the next destination in string format
 * @param length The message length.
 * @param message The message.
 * @return 1 if the message has been sent, 0 otherwise.
 */
int send_message(int message_type,
                  unsigned short port,
                  unsigned short federate,
                  const char* next_destination_str,
                  size_t length,
                  unsigned char* message) {
    unsigned char header_buffer[1 + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(int32_t)];
    // First byte identifies this as a timed message.
    if (message_type != MSG_TYPE_MESSAGE &&
        message_type != MSG_TYPE_P2P_MESSAGE
    ) {
        lf_print_error(
            "send_message() was called with an invalid message type (%d).",
            message_type
        );
        return 0;
    }
    header_buffer[0] = (unsigned char)message_type;
    // Next two bytes identify the destination port.
    // NOTE: Send messages little endian, not big endian.
    encode_uint16(port, &(header_buffer[1]));

    // Next two bytes identify the destination federate.
    encode_uint16(federate, &(header_buffer[1 + sizeof(uint16_t)]));

    // The next four bytes are the message length.
    encode_int32((int32_t)length, &(header_buffer[1 + sizeof(uint16_t) + sizeof(uint16_t)]));

    LF_PRINT_LOG("Sending untimed message to %s.", next_destination_str);

    // Header:  message_type + port_id + federate_id + length of message + timestamp + microstep
    const int header_length = 1 + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(int32_t);
    // Use a mutex lock to prevent multiple threads from simultaneously sending.
    lf_mutex_lock(&outbound_socket_mutex);
    // First, check that the socket is still connected. This must done
    // while holding the mutex lock.
    int socket = -1;
    if (message_type == MSG_TYPE_P2P_MESSAGE || message_type == MSG_TYPE_P2P_TAGGED_MESSAGE) {
        socket = _fed.sockets_for_outbound_p2p_connections[federate];
    } else {
        socket = _fed.socket_TCP_RTI;
    }
    if (socket < 0) {
        lf_print_warning("Socket is no longer connected. Dropping message.");
        lf_mutex_unlock(&outbound_socket_mutex);
        return 0;
    }
    write_to_socket_errexit_with_mutex(socket, header_length, header_buffer, &outbound_socket_mutex,
            "Failed to send message header to to %s.", next_destination_str);
    write_to_socket_errexit_with_mutex(socket, length, message, &outbound_socket_mutex,
            "Failed to send message body to to %s.", next_destination_str);
    lf_mutex_unlock(&outbound_socket_mutex);
    return 1;
}

/** 
 * Send the specified timestamped message to the specified port in the
 * specified federate via the RTI or directly to a federate depending on
 * the given socket. The timestamp is calculated as current_logical_time +
 * additional delay which is greater than or equal to zero.
 * The port should be an input port of a reactor in 
 * the destination federate. This version does include the timestamp 
 * in the message. The caller can reuse or free the memory after this returns.
 *
 * If the socket connection to the remote federate or the RTI has been broken,
 * then this returns 0 without sending. Otherwise, it returns 1.
 *
 * This method assumes that the caller does not hold the outbound_socket_mutex lock,
 * which it acquires to perform the send.
 * 
 * @note This function is similar to send_message() except that it
 *   sends timed messages and also contains logics related to time.
 * 
 * @param additional_delay The offset applied to the timestamp
 *  using after. The additional delay will be greater or equal to zero
 *  if an after is used on the connection. If no after is given in the
 *  program, -1 is passed.
 * @param message_type The type of the message being sent. 
 *  Currently can be MSG_TYPE_TAGGED_MESSAGE for messages sent via
 *  RTI or MSG_TYPE_P2P_TAGGED_MESSAGE for messages sent between
 *  federates.
 * @param port The ID of the destination port.
 * @param federate The ID of the destination federate.
 * @param next_destination_str The next destination in string format (RTI or federate)
 *  (used for reporting errors).
 * @param length The message length.
 * @param message The message.
 * @return 1 if the message has been sent, 0 otherwise.
 */
int send_timed_message(interval_t additional_delay,
                        int message_type,
                        unsigned short port,
                        unsigned short federate,
                        const char* next_destination_str,
                        size_t length,
                        unsigned char* message) {
    unsigned char header_buffer[1 + sizeof(uint16_t) + sizeof(uint16_t)
             + sizeof(int32_t) + sizeof(instant_t) + sizeof(microstep_t)];
    // First byte identifies this as a timed message.
    if (message_type != MSG_TYPE_TAGGED_MESSAGE &&
        message_type != MSG_TYPE_P2P_TAGGED_MESSAGE
    ) {
        lf_print_error(
            "send_message() was called with an invalid message type (%d).",
            message_type
        );
        return 0;
    }
    size_t buffer_head = 0;
    header_buffer[buffer_head] = (unsigned char)message_type;
    buffer_head += sizeof(unsigned char);
    // Next two bytes identify the destination port.
    // NOTE: Send messages little endian, not big endian.
    encode_uint16(port, &(header_buffer[buffer_head]));
    buffer_head += sizeof(uint16_t);

    // Next two bytes identify the destination federate.
    encode_uint16(federate, &(header_buffer[buffer_head]));
    buffer_head += sizeof(uint16_t);

    // The next four bytes are the message length.
    encode_int32((int32_t)length, &(header_buffer[buffer_head]));
    buffer_head += sizeof(int32_t);

    // Apply the additional delay to the current tag and use that as the intended
    // tag of the outgoing message
    tag_t current_message_intended_tag = _lf_delay_tag(lf_tag(),
                                                    additional_delay);

    // Next 8 + 4 will be the tag (timestamp, microstep)
    encode_tag(
        &(header_buffer[buffer_head]), 
        current_message_intended_tag
    );
    buffer_head += sizeof(int64_t) + sizeof(uint32_t);

    LF_PRINT_LOG("Sending message with tag " PRINTF_TAG " to %s.",
            current_message_intended_tag.time - start_time, current_message_intended_tag.microstep, next_destination_str);

    // Header:  message_type + port_id + federate_id + length of message + timestamp + microstep
    size_t header_length = buffer_head;

    if (_lf_is_tag_after_stop_tag(current_message_intended_tag)) {
        // Message tag is past the timeout time (the stop time) so it should
        // not be sent.
        return 0;
    }

    // Use a mutex lock to prevent multiple threads from simultaneously sending.
    lf_mutex_lock(&outbound_socket_mutex);
    // First, check that the socket is still connected. This must done
    // while holding the mutex lock.
    int socket = -1;
    if (message_type == MSG_TYPE_P2P_MESSAGE || message_type == MSG_TYPE_P2P_TAGGED_MESSAGE) {
        socket = _fed.sockets_for_outbound_p2p_connections[federate];
    } else {
        socket = _fed.socket_TCP_RTI;
    }
    if (socket < 0) {
        lf_print_warning("Socket is no longer connected. Dropping message.");
        lf_mutex_unlock(&outbound_socket_mutex);
        return 0;
    }
    write_to_socket_errexit_with_mutex(socket, header_length, header_buffer, &outbound_socket_mutex,
            "Failed to send timed message header to %s.", next_destination_str);
    write_to_socket_errexit_with_mutex(socket, length, message, &outbound_socket_mutex,
            "Failed to send timed message body to %s.", next_destination_str);
    lf_mutex_unlock(&outbound_socket_mutex);
    return 1;
}

/** 
 * Send a time to the RTI.
 * This is not synchronized.
 * It assumes the caller is.
 * @param type The message type (MSG_TYPE_TIMESTAMP or MSG_TYPE_TIME_ADVANCE_NOTICE).
 * @param time The time.
 * @param exit_on_error If set to true, exit the program if sending 'time' fails.
 *  Print a soft error message otherwise
 */
void _lf_send_time(unsigned char type, instant_t time, bool exit_on_error) {
    LF_PRINT_DEBUG("Sending time " PRINTF_TIME " to the RTI.", time);
    size_t bytes_to_write = 1 + sizeof(instant_t);
    unsigned char buffer[bytes_to_write];
    buffer[0] = type;
    encode_int64(time, &(buffer[1]));
    lf_mutex_lock(&outbound_socket_mutex);
    if (_fed.socket_TCP_RTI < 0) {
        lf_print_warning("Socket is no longer connected. Dropping message.");
        lf_mutex_unlock(&outbound_socket_mutex);
        return;
    }
    ssize_t bytes_written = write_to_socket(_fed.socket_TCP_RTI, bytes_to_write, buffer);
    if (bytes_written < (ssize_t)bytes_to_write) {
        if (!exit_on_error) {
            lf_print_error("Failed to send time " PRINTF_TIME " to the RTI."
                            " Error code %d: %s",
                            time - start_time,
                            errno,
                            strerror(errno)
                        );

        } else if (errno == ENOTCONN) {
            // FIXME: Shutdown is probably not working properly because the socket gets disconnected.
            lf_print_error("Socket to the RTI is no longer connected. Considering this a soft error.");
        } else {
            lf_print_error_and_exit("Failed to send time " PRINTF_TIME " to the RTI."
                                    " Error code %d: %s",
                                    time - start_time,
                                    errno,
                                    strerror(errno)
                                );
        }
    }
    lf_mutex_unlock(&outbound_socket_mutex);
}

/**
 * Send a tag to the RTI.
 * This is not synchronized.
 * It assumes the caller is.
 * @param type The message type (MSG_TYPE_NEXT_EVENT_TAG or MSG_TYPE_LOGICAL_TAG_COMPLETE).
 * @param tag The tag.
 * @param exit_on_error If set to true, exit the program if sending 'tag' fails.
 *  Print a soft error message otherwise
 */
void _lf_send_tag(unsigned char type, tag_t tag, bool exit_on_error) {
    LF_PRINT_DEBUG("Sending tag " PRINTF_TAG " to the RTI.", tag.time - start_time, tag.microstep);
    size_t bytes_to_write = 1 + sizeof(instant_t) + sizeof(microstep_t);
    unsigned char buffer[bytes_to_write];
    buffer[0] = type;
    encode_tag(&(buffer[1]), tag);

    lf_mutex_lock(&outbound_socket_mutex);
    if (_fed.socket_TCP_RTI < 0) {
        lf_print_warning("Socket is no longer connected. Dropping message.");
        lf_mutex_unlock(&outbound_socket_mutex);
        return;
    }
    ssize_t bytes_written = write_to_socket(_fed.socket_TCP_RTI, bytes_to_write, buffer);
    if (bytes_written < (ssize_t)bytes_to_write) {
        if (!exit_on_error) {
            lf_print_error("Failed to send tag " PRINTF_TAG " to the RTI."
                            " Error code %d: %s",
                            tag.time - start_time, 
                            tag.microstep,
                            errno,
                            strerror(errno)
                        );

        } else if (errno == ENOTCONN) {
            lf_print_error("Socket to the RTI is no longer connected. Considering this a soft error.");
        } else {
            lf_mutex_unlock(&outbound_socket_mutex);
            lf_print_error_and_exit("Failed to send tag " PRINTF_TAG " to the RTI."
                                    " Error code %d: %s",
                                    tag.time - start_time, 
                                    tag.microstep,
                                    errno,
                                    strerror(errno)
                                );
        }
    }

    lf_mutex_unlock(&outbound_socket_mutex);
}

/**
 * Thread to accept connections from other federates that send this federate
 * messages directly (not through the RTI). This thread starts a thread for
 * each accepted socket connection and, once it has opened all expected
 * sockets, exits.
 * @param ignored No argument needed for this thread.
 */
void* handle_p2p_connections_from_federates(void* ignored) {
    int received_federates = 0;
    // Allocate memory to store thread IDs.
    _fed.inbound_socket_listeners = (lf_thread_t*)calloc(_fed.number_of_inbound_p2p_connections, sizeof(lf_thread_t));
    while (received_federates < _fed.number_of_inbound_p2p_connections) {
        // Wait for an incoming connection request.
        struct sockaddr client_fd;
        uint32_t client_length = sizeof(client_fd);
        int socket_id = accept(_fed.server_socket, &client_fd, &client_length);
        // FIXME: Error handling here is too harsh maybe?
        if (socket_id < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            lf_print_error("A fatal error occurred while accepting a new socket. "
                        "Federate will not accept connections anymore.");
            return NULL;
        }
        LF_PRINT_LOG("Accepted new connection from remote federate.");

        size_t header_length = 1 + sizeof(uint16_t) + 1;
        unsigned char buffer[header_length];
        ssize_t bytes_read = read_from_socket(socket_id, header_length, (unsigned char*)&buffer);
        if (bytes_read != (ssize_t)header_length || buffer[0] != MSG_TYPE_P2P_SENDING_FED_ID) {
            lf_print_warning("Federate received invalid first message on P2P socket. Closing socket.");
            if (bytes_read >= 0) {
                unsigned char response[2];
                response[0] = MSG_TYPE_REJECT;
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
                 || (strncmp(federation_metadata.federation_id, remote_federation_id, strnlen(federation_metadata.federation_id, 255)) != 0)) {
            lf_print_warning("Received invalid federation ID. Closing socket.");
            if (bytes_read >= 0) {
                unsigned char response[2];
                response[0] = MSG_TYPE_REJECT;
                response[1] = FEDERATION_ID_DOES_NOT_MATCH;
                // Ignore errors on this response.
                write_to_socket(socket_id, 2, response);
            }
            close(socket_id);
            continue;
        }

        // Extract the ID of the sending federate.
        uint16_t remote_fed_id = extract_uint16((unsigned char*)&(buffer[1]));
        LF_PRINT_DEBUG("Received sending federate ID %d.", remote_fed_id);

        // Once we record the socket_id here, all future calls to close() on
        // the socket should be done while holding a mutex, and this array
        // element should be reset to -1 during that critical section.
        // Otherwise, there can be race condition where, during termination,
        // two threads attempt to simultaneously access the socket.
        _fed.sockets_for_inbound_p2p_connections[remote_fed_id] = socket_id;

        // Send an MSG_TYPE_ACK message.
        unsigned char response = MSG_TYPE_ACK;
        write_to_socket_errexit(socket_id, 1, (unsigned char*)&response,
                "Failed to write MSG_TYPE_ACK in response to federate %d.",
                remote_fed_id);

        // Start a thread to listen for incoming messages from other federates.
        // We cannot pass a pointer to remote_fed_id to the thread we need to create
        // because that variable is on the stack. Instead, we malloc memory.
        // The created thread is responsible for calling free().
        uint16_t* remote_fed_id_copy = (uint16_t*)malloc(sizeof(uint16_t));
        if (remote_fed_id_copy == NULL) {
            lf_print_error_and_exit("malloc failed.");
        }
        *remote_fed_id_copy = remote_fed_id;
        int result = lf_thread_create(
                &_fed.inbound_socket_listeners[received_federates],
                listen_to_federates,
                remote_fed_id_copy);
        if (result != 0) {
            // Failed to create a listening thread.
            close(socket_id);
            _fed.sockets_for_inbound_p2p_connections[remote_fed_id] = -1;
            lf_print_error_and_exit(
                    "Failed to create a thread to listen for incoming physical connection. Error code: %d.",
                    result
            );
        }

        received_federates++;
    }

    LF_PRINT_LOG("All remote federates are connected.");
    return NULL;
}

/**
 * Close the socket that sends outgoing messages to the
 * specified federate ID. This function assumes the caller holds
 * the outbound_socket_mutex mutex lock.
 * @param The ID of the peer federate receiving messages from this
 *  federate, or -1 if the RTI (centralized coordination).
 */
void _lf_close_outbound_socket(int fed_id) {
    assert (fed_id >= 0 && fed_id < NUMBER_OF_FEDERATES);
    if (_fed.sockets_for_outbound_p2p_connections[fed_id] >= 0) {
        shutdown(_fed.sockets_for_outbound_p2p_connections[fed_id], SHUT_RDWR);
        close(_fed.sockets_for_outbound_p2p_connections[fed_id]);
        _fed.sockets_for_outbound_p2p_connections[fed_id] = -1;
    }
}

/**
 * For each incoming message socket, we create this thread that listens
 * for upstream messages. Currently, the only possible upstream message
 * is MSG_TYPE_CLOSE_REQUEST.  If this thread receives that message, then closes
 * the socket.  The idea here is that a peer-to-peer socket connection
 * is always closed from the sending end, never from the receiving end.
 * This way, any sends in progress complete before the socket is actually
 * closed.
 */
void* listen_for_upstream_messages_from_downstream_federates(void* fed_id_ptr) {
    uint16_t fed_id = *((uint16_t*)fed_id_ptr);
    unsigned char message;

    lf_mutex_lock(&outbound_socket_mutex);
    while(_fed.sockets_for_outbound_p2p_connections[fed_id] >= 0) {
        // Unlock the mutex before performing a blocking read.
        // Note that there is a race condition here, but the read will return
        // a failure if the socket gets closed.
        lf_mutex_unlock(&outbound_socket_mutex);

        LF_PRINT_DEBUG("Thread listening for MSG_TYPE_CLOSE_REQUEST from federate %d", fed_id);
        ssize_t bytes_read = read_from_socket(
                _fed.sockets_for_outbound_p2p_connections[fed_id], 1, &message);
        // Reacquire the mutex lock before closing or reading the socket again.
        lf_mutex_lock(&outbound_socket_mutex);

        if (bytes_read == 1 && message == MSG_TYPE_CLOSE_REQUEST) {
            // Received a request to close the socket.
            LF_PRINT_DEBUG("Received MSG_TYPE_CLOSE_REQUEST from federate %d.", fed_id);
            _lf_close_outbound_socket(fed_id);
        }
    }
    lf_mutex_unlock(&outbound_socket_mutex);
    return NULL;
}

/**
 * Connect to the federate with the specified id. This established
 * connection will then be used in functions such as send_timed_message() 
 * to send messages directly to the specified federate. 
 * This function first sends an MSG_TYPE_ADDRESS_QUERY message to the RTI to obtain
 * the IP address and port number of the specified federate. It then attempts 
 * to establish a socket connection to the specified federate.
 * If this fails, the program exits. If it succeeds, it sets element [id] of 
 * the _fed.sockets_for_outbound_p2p_connections global array to
 * refer to the socket for communicating directly with the federate.
 * @param remote_federate_id The ID of the remote federate.
 */
void connect_to_federate(uint16_t remote_federate_id) {
    int result = -1;
    int count_retries = 0;

    // Ask the RTI for port number of the remote federate.
    // The buffer is used for both sending and receiving replies.
    // The size is what is needed for receiving replies.
    unsigned char buffer[sizeof(int32_t) + INET_ADDRSTRLEN];
    int port = -1;
    struct in_addr host_ip_addr;
    int count_tries = 0;
    while (port == -1) {
        buffer[0] = MSG_TYPE_ADDRESS_QUERY;
        // NOTE: Sending messages in little endian.
        encode_uint16(remote_federate_id, &(buffer[1]));

        LF_PRINT_DEBUG("Sending address query for federate %d.", remote_federate_id);

        write_to_socket_errexit(_fed.socket_TCP_RTI, sizeof(uint16_t) + 1, buffer,
                "Failed to send address query for federate %d to RTI.",
                remote_federate_id);

        // Read RTI's response.
        read_from_socket_errexit(_fed.socket_TCP_RTI, sizeof(int32_t), buffer,
                "Failed to read the requested port number for federate %d from RTI.",
                remote_federate_id);

        port = extract_int32(buffer);

        read_from_socket_errexit(_fed.socket_TCP_RTI, sizeof(host_ip_addr), (unsigned char*)&host_ip_addr,
                "Failed to read the IP address for federate %d from RTI.",
                remote_federate_id);

        // A reply of -1 for the port means that the RTI does not know
        // the port number of the remote federate, presumably because the
        // remote federate has not yet sent an MSG_TYPE_ADDRESS_ADVERTISEMENT message to the RTI.
        // Sleep for some time before retrying.
        if (port == -1) {
            if (count_tries++ >= CONNECT_NUM_RETRIES) {
                lf_print_error_and_exit("TIMEOUT obtaining IP/port for federate %d from the RTI.",
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
    uint16_t uport = (uint16_t)port;

#if LOG_LEVEL > 3
    // Print the received IP address in a human readable format
    // Create the human readable format of the received address.
    // This is avoided unless LOG_LEVEL is high enough to
    // subdue the overhead caused by inet_ntop().
    char hostname[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &host_ip_addr, hostname, INET_ADDRSTRLEN);
    LF_PRINT_LOG("Received address %s port %d for federate %d from RTI.",
            hostname, uport, remote_federate_id);
#endif

    // Iterate until we either successfully connect or exceed the number of
    // attempts given by CONNECT_NUM_RETRIES.
    int socket_id = -1;
    while (result < 0) {
        // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
        socket_id = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_id < 0) {
            lf_print_error_and_exit("Failed to create socket to federate %d.", remote_federate_id);
        }

        // Server file descriptor.
        struct sockaddr_in server_fd;
        // Zero out the server_fd struct.
        bzero((char*)&server_fd, sizeof(server_fd));

        // Set up the server_fd fields.
        server_fd.sin_family = AF_INET;    // IPv4
        server_fd.sin_addr = host_ip_addr; // Received from the RTI

        // Convert the port number from host byte order to network byte order.
        server_fd.sin_port = htons(uport);
        result = connect(
            socket_id,
            (struct sockaddr *)&server_fd,
            sizeof(server_fd));

        if (result != 0) {
            lf_print_error("Failed to connect to federate %d on port %d.", remote_federate_id, uport);

            // Try again after some time if the connection failed.
            // Note that this should not really happen since the remote federate should be
            // accepting socket connections. But possibly it will be busy (in process of accepting
            // another socket connection?). Hence, we retry.
            count_retries++;
            if (count_retries > CONNECT_NUM_RETRIES) {
                // If the remote federate is not accepting the connection after CONNECT_NUM_RETRIES
                // treat it as a soft error condition and return.
                lf_print_error("Failed to connect to federate %d after %d retries. Giving up.",
                            remote_federate_id, CONNECT_NUM_RETRIES);
                return;
            }
            lf_print_warning("Could not connect to federate %d. Will try again every " PRINTF_TIME " nanoseconds.\n",
                   remote_federate_id, ADDRESS_QUERY_RETRY_INTERVAL);
            // Wait CONNECT_RETRY_INTERVAL seconds.
            struct timespec wait_time = {0L, ADDRESS_QUERY_RETRY_INTERVAL};
            struct timespec remaining_time;
            if (nanosleep(&wait_time, &remaining_time) != 0) {
                // Sleep was interrupted.
                continue;
            }
        } else {
            // Connect was successful.
            size_t buffer_length = 1 + sizeof(uint16_t) + 1;
            unsigned char buffer[buffer_length];
            buffer[0] = MSG_TYPE_P2P_SENDING_FED_ID;
            if (_lf_my_fed_id > UINT16_MAX) {
                // This error is very unlikely to occur.
                lf_print_error_and_exit("Too many federates! More than %d.", UINT16_MAX);
            }
            encode_uint16((uint16_t)_lf_my_fed_id, (unsigned char*)&(buffer[1]));
            unsigned char federation_id_length = (unsigned char)strnlen(federation_metadata.federation_id, 255);
            buffer[sizeof(uint16_t) + 1] = federation_id_length;
            write_to_socket_errexit(socket_id,
                    buffer_length, buffer,
                    "Failed to send fed_id to federate %d.", remote_federate_id);
            write_to_socket_errexit(socket_id,
                    federation_id_length, (unsigned char*)federation_metadata.federation_id,
                    "Failed to send federation id to federate %d.",
                    remote_federate_id);

            read_from_socket_errexit(socket_id, 1, (unsigned char*)buffer,
                    "Failed to read MSG_TYPE_ACK from federate %d in response to sending fed_id.",
                    remote_federate_id);
            if (buffer[0] != MSG_TYPE_ACK) {
                // Get the error code.
                read_from_socket_errexit(socket_id, 1, (unsigned char*)buffer,
                        "Failed to read error code from federate %d in response to sending fed_id.", remote_federate_id);
                lf_print_error("Received MSG_TYPE_REJECT message from remote federate (%d).", buffer[0]);
                result = -1;
                continue;
            } else {
                lf_print("Connected to federate %d, port %d.", remote_federate_id, port);
            }
        }
    }
    // Once we set this variable, then all future calls to close() on this
    // socket ID should reset it to -1 within a critical section.
    _fed.sockets_for_outbound_p2p_connections[remote_federate_id] = socket_id;

    // Start a thread to listen for upstream messages (MSG_TYPE_CLOSE_REQUEST) from
    // this downstream federate.
    uint16_t* remote_fed_id_copy = (uint16_t*)malloc(sizeof(uint16_t));
    if (remote_fed_id_copy == NULL) {
        lf_print_error_and_exit("malloc failed.");
    }
    *remote_fed_id_copy = remote_federate_id;
    lf_thread_t thread_id;
    result = lf_thread_create(
            &thread_id,
            listen_for_upstream_messages_from_downstream_federates,
            remote_fed_id_copy);
    if (result != 0) {
        // Failed to create a listening thread.
        lf_print_error_and_exit(
                "Failed to create a thread to listen for upstream message. Error code: %d.",
                result
        );
    }
}

/**
 * Connect to the RTI at the specified host and port and return
 * the socket descriptor for the connection. If this fails, the
 * program exits. If it succeeds, it sets the _fed.socket_TCP_RTI global
 * variable to refer to the socket for communicating with the RTI.
 * @param hostname A hostname, such as "localhost".
 * @param port_number A port number.
 */
void connect_to_rti(char* hostname, int port) {
    LF_PRINT_LOG("Connecting to the RTI.");

    // override passed hostname and port if passed as runtime arguments
    hostname = federation_metadata.rti_host ? federation_metadata.rti_host : hostname;
    port = federation_metadata.rti_port >= 0 ? federation_metadata.rti_port : port;

    uint16_t uport = 0;
    if (port < 0 ||
            port > INT16_MAX) {
        lf_print_error(
            "connect_to_rti(): Specified port (%d) is out of range,"
            " using zero instead.",
            port
        );
    } else {
        uport = (uint16_t)port;
    }

    // Repeatedly try to connect, one attempt every 2 seconds, until
    // either the program is killed, the sleep is interrupted,
    // or the connection succeeds.
    // If the specified port is 0, set it instead to the start of the
    // port range.
    bool specific_port_given = true;
    if (uport == 0) {
        uport = STARTING_PORT;
        specific_port_given = false;
    }
    int result = -1;
    int count_retries = 0;

    while (result < 0) {
        // Create an IPv4 socket for TCP (not UDP) communication over IP (0).
        _fed.socket_TCP_RTI = socket(AF_INET, SOCK_STREAM, 0);
        if (_fed.socket_TCP_RTI < 0) {
            lf_print_error_and_exit("Creating socket to RTI.");
        }

        struct hostent *server = gethostbyname(hostname);
        if (server == NULL) {
            lf_print_error_and_exit("ERROR, no such host for RTI: %s\n", hostname);
        }
        // Server file descriptor.
        struct sockaddr_in server_fd;
        // Zero out the server_fd struct.
        bzero((char*)&server_fd, sizeof(server_fd));

        // Set up the server_fd fields.
        server_fd.sin_family = AF_INET;    // IPv4
        bcopy((char*)server->h_addr,
             (char*)&server_fd.sin_addr.s_addr,
             (size_t)server->h_length);
        // Convert the port number from host byte order to network byte order.
        server_fd.sin_port = htons(uport);
        result = connect(
            _fed.socket_TCP_RTI,
            (struct sockaddr *)&server_fd,
            sizeof(server_fd));
        // If this failed, try more ports, unless a specific port was given.
        if (result != 0
                && !specific_port_given
                && uport >= STARTING_PORT
                && uport <= STARTING_PORT + PORT_RANGE_LIMIT
        ) {
            lf_print("Failed to connect to RTI on port %d. Trying %d.", uport, uport + 1);
            uport++;
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
            if (!specific_port_given && uport == STARTING_PORT + PORT_RANGE_LIMIT + 1) {
                uport = STARTING_PORT;
            }
            count_retries++;
            if (count_retries > CONNECT_NUM_RETRIES) {
                lf_print_error_and_exit("Failed to connect to the RTI after %d retries. Giving up.",
                                     CONNECT_NUM_RETRIES);
            }
            lf_print("Could not connect to RTI at %s. Will try again every %d seconds.",
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
            // Send a MSG_TYPE_FED_IDS message and wait for a reply.
            // Notify the RTI of the ID of this federate and its federation.
            unsigned char buffer[4];

            LF_PRINT_LOG("Connected to an RTI. Sending federation ID for authentication.");

            // Send the message type first.
            buffer[0] = MSG_TYPE_FED_IDS;
            // Next send the federate ID.
            if (_lf_my_fed_id > UINT16_MAX) {
                lf_print_error_and_exit("Too many federates! More than %d.", UINT16_MAX);
            }
            encode_uint16((uint16_t)_lf_my_fed_id, &buffer[1]);
            // Next send the federation ID length.
            // The federation ID is limited to 255 bytes.
            size_t federation_id_length = strnlen(federation_metadata.federation_id, 255);
            buffer[1 + sizeof(uint16_t)] = (unsigned char)(federation_id_length & 0xff);

            write_to_socket_errexit(_fed.socket_TCP_RTI, 2 + sizeof(uint16_t), buffer,
                    "Failed to send federate ID to RTI.");

            // Next send the federation ID itself.
            write_to_socket_errexit(_fed.socket_TCP_RTI, federation_id_length, (unsigned char*)federation_metadata.federation_id,
                            "Failed to send federation ID to RTI.");

            // Wait for a response.
            // The response will be MSG_TYPE_REJECT if the federation ID doesn't match.
            // Otherwise, it will be either MSG_TYPE_ACK or MSG_TYPE_UDP_PORT, where the latter
            // is used if clock synchronization will be performed.
            unsigned char response;

            LF_PRINT_DEBUG("Waiting for response to federation ID from the RTI.");

            read_from_socket_errexit(_fed.socket_TCP_RTI, 1, &response, "Failed to read response from RTI.");
            if (response == MSG_TYPE_REJECT) {
                // Read one more byte to determine the cause of rejection.
                unsigned char cause;
                read_from_socket_errexit(_fed.socket_TCP_RTI, 1, &cause, "Failed to read the cause of rejection by the RTI.");
                if (cause == FEDERATION_ID_DOES_NOT_MATCH || cause == WRONG_SERVER) {
                    lf_print("Connected to the wrong RTI on port %d. Trying %d.", uport, uport + 1);
                    uport++;
                    result = -1;
                    continue;
                }
                lf_print_error_and_exit("RTI Rejected MSG_TYPE_FED_IDS message with response (see net_common.h): "
                        "%d. Error code: %d. Federate quits.\n", response, cause);
            } else if (response == MSG_TYPE_ACK) {
                LF_PRINT_LOG("Received acknowledgment from the RTI.");

                // Call a generated (external) function that sends information
                // about connections between this federate and other federates
                // where messages are routed through the RTI.
                // @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h
                send_neighbor_structure_to_RTI(_fed.socket_TCP_RTI);

                uint16_t udp_port = setup_clock_synchronization_with_rti();

                // Write the returned port number to the RTI
                unsigned char UDP_port_number[1 + sizeof(uint16_t)];
                UDP_port_number[0] = MSG_TYPE_UDP_PORT;
                encode_uint16(udp_port, &(UDP_port_number[1]));
                write_to_socket_errexit(_fed.socket_TCP_RTI, 1 + sizeof(uint16_t), UDP_port_number,
                            "Failed to send the UDP port number to the RTI.");
            } else {
                lf_print_error_and_exit("Received unexpected response %u from the RTI (see net_common.h).",
                        response);
            }
            lf_print("Connected to RTI at %s:%d.", hostname, uport);
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
    // Send the timestamp marker first.
    _lf_send_time(MSG_TYPE_TIMESTAMP, my_physical_time, true);

    // Read bytes from the socket. We need 9 bytes.
    // Buffer for message ID plus timestamp.
    size_t buffer_length = 1 + sizeof(instant_t);
    unsigned char buffer[buffer_length];

    read_from_socket_errexit(_fed.socket_TCP_RTI, buffer_length, buffer,
            "Failed to read MSG_TYPE_TIMESTAMP message from RTI.");
    LF_PRINT_DEBUG("Read 9 bytes.");

    // First byte received is the message ID.
    if (buffer[0] != MSG_TYPE_TIMESTAMP) {
        lf_print_error_and_exit("Expected a MSG_TYPE_TIMESTAMP message from the RTI. Got %u (see net_common.h).",
                             buffer[0]);
    }

    instant_t timestamp = extract_int64(&(buffer[1]));
    lf_print("Starting timestamp is: " PRINTF_TIME ".", timestamp);
    LF_PRINT_LOG("Current physical time is: " PRINTF_TIME ".", lf_time_physical());

    return timestamp;
}

////////////////////////////////Port Status Handling///////////////////////////////////////

/**
 * Placeholder for a generated function that returns a pointer to the
 * trigger_t struct for the action corresponding to the specified port ID.
 * @param port_id The port ID.
 * @return A pointer to a trigger_t struct or null if the ID is out of range.
 */
trigger_t* _lf_action_for_port(int port_id);


/**
 * Set the status of network port with id portID.
 * 
 * @param portID The network port ID
 * @param status The network port status (port_status_t)
 */
void set_network_port_status(int portID, port_status_t status) {
    trigger_t* network_input_port_action = _lf_action_for_port(portID);
    network_input_port_action->status = status;
}


/**
 * Mark all status fields of unknown network input ports as absent.
 */
void mark_all_unknown_ports_as_absent() {
    for (int i = 0; i < _fed.triggers_for_network_input_control_reactions_size; i++) {
        trigger_t* input_port_action = _lf_action_for_port(i);
        if (input_port_action->status == unknown) {
            set_network_port_status(i, absent);
        }
    }
}

/**
 * Return true if there is an input control reaction blocked waiting for input.
 * This assumes the caller holds the mutex.
 */
bool is_input_control_reaction_blocked() {
    for (int i = 0; i < _fed.triggers_for_network_input_control_reactions_size; i++) {
        trigger_t* input_port_action = _lf_action_for_port(i);
        if (input_port_action->is_a_control_reaction_waiting) {
            return true;
        }
    }
    return false;
}

/**
 * Update the last known status tag of all network input ports
 * to the value of `tag`, unless that the provided `tag` is less
 * than the last_known_status_tag of the port. This is called when
 * all inputs to network ports with tags up to an including `tag`
 * have been received by those ports. If any update occurs and if
 * there are control reactions blocked, then this broadcasts a
 * signal to potentially unblock those control reactions.
 * 
 * This assumes the caller holds the mutex.
 *
 * @param tag The tag on which the latest status of network input
 *  ports is known.
 */
void update_last_known_status_on_input_ports(tag_t tag) {
    bool notify = false;
    for (int i = 0; i < _fed.triggers_for_network_input_control_reactions_size; i++) {
        trigger_t* input_port_action = _lf_action_for_port(i);
        // This is called when a TAG is received.
        // But it is possible for an input port to have received already
        // a message with a larger tag (if there is an after delay on the
        // connection), in which case, the last known status tag of the port
        // is in the future and should not be rolled back. So in that case,
        // we do not update the last known status tag.
        if (lf_tag_compare(tag,
                input_port_action->last_known_status_tag) >= 0) {
            LF_PRINT_DEBUG(
                "Updating the last known status tag of port %d to " PRINTF_TAG ".",
                i,
                tag.time - lf_time_start(), 
                tag.microstep
            );
            input_port_action->last_known_status_tag = tag;
            if (input_port_action->is_a_control_reaction_waiting) {
                notify = true;
            }
        }
    }
    // Then, check if any control reaction is waiting.
    // If so, notify them.
    // FIXME: We could put a condition variable into the trigger_t
    // struct for each network input port, in which case this won't
    // be a broadcast but rather a targetted signal.
    if (notify) {
        // Notify network input control reactions
        lf_cond_broadcast(&port_status_changed);
    }
}


/**
 * Update the last known status tag of a network input port
 * to the value of "tag". This is the largest tag at which the status
 * (present or absent) of the port was known.
 *
 * This function assumes the caller holds the mutex, and, if the tag
 * actually increases, it notifies the waiting control reaction if there is one.
 * 
 * @param tag The tag on which the latest status of network input
 *  ports is known.
 * @param portID The port ID
 */
void update_last_known_status_on_input_port(tag_t tag, int port_id) {
    trigger_t* input_port_action = _lf_action_for_port(port_id);
    if (lf_tag_compare(tag,
            input_port_action->last_known_status_tag) >= 0) {
                if (lf_tag_compare(tag,
                        input_port_action->last_known_status_tag) == 0) {
                    // If the intended tag for an input port is equal to the last known status, we need
                    // to increment the microstep. This is a direct result of the behavior of the _lf_delay_tag()
                    // semantics in tag.h.
                    tag.microstep++;
                }
        LF_PRINT_DEBUG(
            "Updating the last known status tag of port %d to " PRINTF_TAG ".",
            port_id,
            tag.time - lf_time_start(), 
            tag.microstep
        );
        input_port_action->last_known_status_tag = tag;
        // If any control reaction is waiting, notify them that the status has changed
        if (input_port_action->is_a_control_reaction_waiting) {
            // The last known status tag of the port has changed. Notify any waiting threads.
            lf_cond_broadcast(&port_status_changed);
        }
    } else {
        lf_print_warning("Attempt to update the last known status tag "
               "of network input port %d to an earlier tag was ignored.", port_id);
    }
}

/**
 * Reset the status fields on network input ports to unknown.
 * 
 * @note This function must be called at the beginning of each
 *  logical time.
 */
void reset_status_fields_on_input_port_triggers() {
    for (int i = 0; i < _fed.triggers_for_network_input_control_reactions_size; i++) {
        set_network_port_status(i, unknown);
    }
}

/**
 * Mark the trigger associated with the specified port to
 * indicate whether a control reaction is waiting.
 */
void mark_control_reaction_waiting(int portID, bool waiting) {
    trigger_t* network_input_port_action = _lf_action_for_port(portID);
    network_input_port_action->is_a_control_reaction_waiting = waiting;
}

/**
 * Return the status of the port at the current tag.
 *
 * This assumes that the caller holds the mutex.
 *
 * @param portID the ID of the port to determine status for
 */
port_status_t get_current_port_status(int portID) {
    // Check whether the status of the port is known at the current tag.
    trigger_t* network_input_port_action = _lf_action_for_port(portID);
    if (network_input_port_action->status == present) {
        // The status of the trigger is present.
        return present;
    } else if (network_input_port_action->status == absent) {
        // The status of the trigger is absent.
        return absent;
    } else if (network_input_port_action->status == unknown
            && lf_tag_compare(network_input_port_action->last_known_status_tag, lf_tag()) >= 0) {
        // We have a known status for this port in a future tag. Therefore, no event is going
        // to be present for this port at the current tag.
        set_network_port_status(portID, absent);
        return absent;
    } else if (_fed.is_last_TAG_provisional
            && lf_tag_compare(_fed.last_TAG, lf_tag()) > 0) {
        // In this case, a PTAG has been received with a larger tag than the current tag,
        // which means that the input port is known to be absent.
        set_network_port_status(portID, absent);
        return absent;
    }
    return unknown;
}

/**
 * Enqueue network input control reactions that determine if the trigger for a
 * given network input port is going to be present at the current logical time
 * or absent.
 */
void enqueue_network_input_control_reactions() {
#ifdef FEDERATED_CENTRALIZED
    if (!_fed.has_upstream) {
        // This federate is not connected to any upstream federates via a
        // logical connection. No need to trigger network input control
        // reactions.
        return;
    }
#endif
    for (int i = 0; i < _fed.triggers_for_network_input_control_reactions_size; i++) {
        // Reaction 0 should always be the network input control reaction
        if (get_current_port_status(i) == unknown) {
            reaction_t *reaction = _fed.triggers_for_network_input_control_reactions[i]->reactions[0];
            if (reaction->status == inactive) {
                reaction->is_a_control_reaction = true;
                LF_PRINT_DEBUG("Inserting network input control reaction on reaction queue.");
                lf_sched_trigger_reaction(reaction, -1);
                mark_control_reaction_waiting(i, true);
            }
        }
    }
}

/**
 * Enqueue network output control reactions that will send a MSG_TYPE_PORT_ABSENT
 * message to downstream federates if a given network output port is not present.
 */
void enqueue_network_output_control_reactions(){
#ifdef FEDERATED_CENTRALIZED
    if (!_fed.has_downstream) {
        // This federate is not connected to any downstream federates via a
        // logical connection. No need to trigger network output control
        // reactions.
        return;
    }
#endif
    LF_PRINT_DEBUG("Enqueueing output control reactions.");
    if (_fed.trigger_for_network_output_control_reactions == NULL) {
        // There are no network output control reactions
        LF_PRINT_DEBUG("No output control reactions.");
        return;
    }
    for (int i = 0; i < _fed.trigger_for_network_output_control_reactions->number_of_reactions; i++) {
        reaction_t* reaction = _fed.trigger_for_network_output_control_reactions->reactions[i];
        if (reaction->status == inactive) {
            reaction->is_a_control_reaction = true;
            LF_PRINT_DEBUG("Inserting network output control reaction on reaction queue.");
            lf_sched_trigger_reaction(reaction, -1);
        }
    }
}


/**
 * Enqueue network control reactions.
 */
void enqueue_network_control_reactions() {
    enqueue_network_output_control_reactions();
#ifdef FEDERATED_CENTRALIZED
    // If the granted tag is not provisional, there is no
    // need for network input control reactions
    if (lf_tag_compare(_fed.last_TAG, lf_tag()) != 0
            || _fed.is_last_TAG_provisional == false) {
        return;
    }
#endif
    enqueue_network_input_control_reactions();
}

/**
 * Send a port absent message to federate with fed_ID, informing the
 * remote federate that the current federate will not produce an event
 * on this network port at the current logical time.
 * 
 * @param additional_delay The offset applied to the timestamp
 *  using after. The additional delay will be greater or equal to zero
 *  if an after is used on the connection. If no after is given in the
 *  program, -1 is passed.
 * @param port_ID The ID of the receiving port.
 * @param fed_ID The fed ID of the receiving federate.
 */
void send_port_absent_to_federate(interval_t additional_delay,
                                    unsigned short port_ID, 
                                  unsigned short fed_ID) {
    // Construct the message
    size_t message_length = 1 + sizeof(port_ID) + sizeof(fed_ID) + sizeof(instant_t) + sizeof(microstep_t);
    unsigned char buffer[message_length];

    // Apply the additional delay to the current tag and use that as the intended
    // tag of the outgoing message
    tag_t current_message_intended_tag = _lf_delay_tag(lf_tag(),
                                                    additional_delay);

    LF_PRINT_LOG("Sending port "
            "absent for tag " PRINTF_TAG " for port %d to federate %d.",
            current_message_intended_tag.time - start_time,
            current_message_intended_tag.microstep,
            port_ID, fed_ID);

    buffer[0] = MSG_TYPE_PORT_ABSENT;
    encode_uint16(port_ID, &(buffer[1]));
    encode_uint16(fed_ID, &(buffer[1+sizeof(port_ID)]));
    encode_tag(&(buffer[1+sizeof(port_ID)+sizeof(fed_ID)]), current_message_intended_tag);
    
    lf_mutex_lock(&outbound_socket_mutex);
#ifdef FEDERATED_CENTRALIZED
    // Send the absent message through the RTI
    int socket = _fed.socket_TCP_RTI;
#else
    // Send the absent message directly to the federate
    int socket = _fed.sockets_for_outbound_p2p_connections[fed_ID];
#endif
    // Do not write if the socket is closed.
    if (socket >= 0) {
        write_to_socket_errexit_with_mutex(socket, message_length, buffer, &outbound_socket_mutex,
                "Failed to send port absent message for port %hu to federate %hu.",
                port_ID, fed_ID);
    }
    lf_mutex_unlock(&outbound_socket_mutex);
}

/**
 * Wait until the status of network port "port_ID" is known.
 * 
 * In decentralized coordination mode, the wait time is capped by STAA + STA,
 * after which the status of the port is presumed to be absent.
 * 
 * This function assumes the holder does not hold a mutex.
 * 
 * @param port_ID The ID of the network port
 * @param STAA The safe-to-assume-absent threshold for the port
 */
void wait_until_port_status_known(int port_ID, interval_t STAA) {            
    // Need to lock the mutex to prevent
    // a race condition with the network
    // receiver logic.
    lf_mutex_lock(&mutex);

    // See if the port status is already known.
    if (get_current_port_status(port_ID) != unknown) {
        // The status of the trigger is known. No need to wait.
        LF_PRINT_LOG("------ Not waiting for network input port %d: "
                    "Status of the port is known already.", port_ID);
        mark_control_reaction_waiting(port_ID, false);
        lf_mutex_unlock(&mutex);
        return;
    }

    // Determine the wait time.
    // In centralized coordination, the wait time is until
    // the RTI can determine the port status and send a TAG
    // replacing the PTAG it sent earlier or until a port absent
    // message has been sent by an upstream federate for this port
    // with a tag greater than the current tag. The federate will 
    // block here FOREVER, until one of the aforementioned 
    // conditions is met.
    interval_t wait_until_time = FOREVER;
#ifdef FEDERATED_DECENTRALIZED // Only applies to decentralized coordination
    // The wait time for port status in the decentralized 
    // coordination is capped by the STAA offset assigned 
    // to the port plus the global STA offset for this federate.
    wait_until_time = current_tag.time + STAA + _lf_fed_STA_offset;
#endif

    // Perform the wait, unless the STAA is zero.
    if (wait_until_time != current_tag.time) {
        LF_PRINT_LOG("------ Waiting until time " PRINTF_TIME "ns for network input port %d at tag (%llu, %d).",
                wait_until_time,
                port_ID,
                current_tag.time - start_time,
                current_tag.microstep);
        while(!wait_until(wait_until_time, &port_status_changed)) {
            // Interrupted
            LF_PRINT_DEBUG("------ Wait for network input port %d interrupted.", port_ID);
            // Check if the status of the port is known
            if (get_current_port_status(port_ID) != unknown) {
                // The status of the trigger is known. No need to wait.
                LF_PRINT_LOG("------ Done waiting for network input port %d: "
                            "Status of the port has changed.", port_ID);
                mark_control_reaction_waiting(port_ID, false);
                lf_mutex_unlock(&mutex);
                return;
            }
        }
    }
    // NOTE: In centralized coordination, cannot reach this point because
    // the wait_until is called with FOREVER, so the while loop above exits
    // only when the port becomes known.

#ifdef FEDERATED_DECENTRALIZED // Only applies in decentralized coordination
    // The wait has timed out. However, a message header
    // for the current tag could have been received in time 
    // but not the the body of the message.
    // Wait on the tag barrier based on the current tag. 
    _lf_wait_on_global_tag_barrier(lf_tag());

    // Done waiting
    // If the status of the port is still unknown, assume it is absent.
    if (get_current_port_status(port_ID) == unknown) {
        // Port will not be triggered at the
        // current logical time. Set the absent
        // value of the trigger accordingly
        // so that the receiving logic cannot
        // insert any further reaction
        set_network_port_status(port_ID, absent);
    }
    mark_control_reaction_waiting(port_ID, false);
    lf_mutex_unlock(&mutex);
    LF_PRINT_LOG("------ Done waiting for network input port %d: "
                "Wait timed out without a port status change.", port_ID);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////

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
 * @param action The action or timer to be triggered.
 * @param tag The tag of the message received over the network.
 * @param value Dynamically allocated memory containing the value to send.
 * @param length The length of the array, if it is an array, or 1 for a
 *  scalar and 0 for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
trigger_handle_t schedule_message_received_from_network_already_locked(
        trigger_t* trigger,
        tag_t tag,
        lf_token_t* token) {
    // Return value of the function
    int return_value = 0;

    // Indicates whether or not the intended tag
    // of the message (timestamp, microstep) is
    // in the future relative to the tag of this
    // federate. By default, assume it is not.
    bool message_tag_is_in_the_future = lf_tag_compare(tag, current_tag) > 0;

    // Assign the intended tag
    trigger->intended_tag = tag;

    // Calculate the extra_delay required to be passed
    // to the schedule function.
    interval_t extra_delay = tag.time - current_tag.time;
    if (!message_tag_is_in_the_future) {
#ifdef FEDERATED_CENTRALIZED
        // If the coordination is centralized, receiving a message
        // that does not carry a timestamp that is in the future
        // would indicate a critical condition, showing that the
        // time advance mechanism is not working correctly.
        lf_print_error_and_exit("Received a message at tag " PRINTF_TAG " that"
                                " has a tag " PRINTF_TAG " that has violated the STP offset. "
                                "Centralized coordination should not have these types of messages.",
                                current_tag.time - start_time, lf_tag().microstep,
                                tag.time - start_time, tag.microstep);
#else
        // Set the delay back to 0
        extra_delay = 0LL;
        LF_PRINT_LOG("Calling schedule with 0 delay and intended tag " PRINTF_TAG ".",
                    trigger->intended_tag.time - start_time,
                    trigger->intended_tag.microstep);
        return_value = _lf_schedule(trigger, extra_delay, token);
#endif
    } else {
        // In case the message is in the future, call
        // _lf_schedule_at_tag() so that the microstep is respected.
        LF_PRINT_LOG("Received a message that is (" PRINTF_TIME " nanoseconds, " PRINTF_MICROSTEP " microsteps) "
                "in the future.", extra_delay, tag.microstep - lf_tag().microstep);
        return_value = _lf_schedule_at_tag(trigger, tag, token);
    }
    // Notify the main thread in case it is waiting for physical time to elapse.
    LF_PRINT_DEBUG("Broadcasting notification that event queue changed.");
    lf_cond_broadcast(&event_q_changed);
    return return_value;
}

/**
 * Request to close the socket that receives incoming messages from the
 * specified federate ID. This sends a message to the upstream federate
 * requesting that it close the socket. If the message is sent successfully,
 * this returns 1. Otherwise it returns 0, which presumably means that the
 * socket is already closed.
 *
 * @param The ID of the peer federate sending messages to this federate.
 *
 * @return 1 if the MSG_TYPE_CLOSE_REQUEST message is sent successfully, 0 otherwise.
 */
int _lf_request_close_inbound_socket(int fed_id) {
    assert(fed_id >= 0 && fed_id < NUMBER_OF_FEDERATES);

     if (_fed.sockets_for_inbound_p2p_connections[fed_id] < 1) return 0;

       // Send a MSG_TYPE_CLOSE_REQUEST message.
    unsigned char message_marker = MSG_TYPE_CLOSE_REQUEST;
       LF_PRINT_LOG("Sending MSG_TYPE_CLOSE_REQUEST message to upstream federate.");
    ssize_t written = write_to_socket(
             _fed.sockets_for_inbound_p2p_connections[fed_id],
            1, &message_marker);
    _fed.sockets_for_inbound_p2p_connections[fed_id] = -1;
    if (written == 1) {
           LF_PRINT_LOG("Sent MSG_TYPE_CLOSE_REQUEST message to upstream federate.");
           return 1;
    } else {
        return 0;
    }
}

/**
 * Close the socket that receives incoming messages from the
 * specified federate ID or RTI. This function should be called when a read
 * of incoming socket fails or when an EOF is received.
 *
 * @param The ID of the peer federate sending messages to this
 *  federate, or -1 if the RTI.
 */
void _lf_close_inbound_socket(int fed_id) {
    if (fed_id < 0) {
        // socket connection is to the RTI.
        int socket = _fed.socket_TCP_RTI;
        // First, set the global socket to -1.
        _fed.socket_TCP_RTI = -1;
        // Then shutdown and close the socket.
        shutdown(socket, SHUT_RDWR);
        close(socket);
    } else if (_fed.sockets_for_inbound_p2p_connections[fed_id] >= 0) {
        shutdown(_fed.sockets_for_inbound_p2p_connections[fed_id], SHUT_RDWR);
        close(_fed.sockets_for_inbound_p2p_connections[fed_id]);
        _fed.sockets_for_inbound_p2p_connections[fed_id] = -1;
    }
}

/** 
 * Handle a port absent message received from a remote federate.
 * This just sets the last known status tag of the port specified
 * in the message.
 *
 * This assumes the caller does not hold the mutex, which it acquires.
 *
 * @param socket The socket to read the message from
 * @param buffer The buffer to read
 * @param fed_id The sending federate ID or -1 if the centralized coordination.
 */
void handle_port_absent_message(int socket, int fed_id) {
    size_t bytes_to_read = sizeof(uint16_t) + sizeof(uint16_t) + sizeof(instant_t) + sizeof(microstep_t);
    unsigned char buffer[bytes_to_read];
    read_from_socket_errexit(socket, bytes_to_read, buffer,
            "Failed to read port absent message.");

    // Extract the header information.
    unsigned short port_id = extract_uint16(buffer);
    // The next part of the message is the federate_id, but we don't need it.
    // unsigned short federate_id = extract_uint16(&(buffer[sizeof(uint16_t)]));
    tag_t intended_tag = extract_tag(&(buffer[sizeof(uint16_t)+sizeof(uint16_t)]));

    LF_PRINT_LOG("Handling port absent for tag " PRINTF_TAG " for port %hu of fed %d.",
            intended_tag.time - lf_time_start(),
            intended_tag.microstep,
            port_id, 
            fed_id
    );

    lf_mutex_lock(&mutex);
#ifdef FEDERATED_DECENTRALIZED
    trigger_t* network_input_port_action = _lf_action_for_port(port_id);
    if (lf_tag_compare(intended_tag,
            network_input_port_action->last_known_status_tag) < 0) {
        lf_mutex_unlock(&mutex);
        lf_print_error_and_exit("The following contract was violated for port absent messages: In-order "
                             "delivery of messages over a TCP socket. Had status for " PRINTF_TAG ", got "
                             "port absent with intended tag " PRINTF_TAG ".",
                             network_input_port_action->last_known_status_tag.time - start_time,
                             network_input_port_action->last_known_status_tag.microstep,
                             intended_tag.time - start_time,
                             intended_tag.microstep);
    }
#endif // In centralized coordination, a TAG message from the RTI 
       // can set the last_known_status_tag to a future tag where messages
       // have not arrived yet.
    // Set the mutex status as absent
    update_last_known_status_on_input_port(intended_tag, port_id);
    lf_mutex_unlock(&mutex);
}

/**
 * Handle a message being received from a remote federate.
 * 
 * This function assumes the caller does not hold the mutex lock.
 * @param socket The socket to read the message from
 * @param buffer The buffer to read
 * @param fed_id The sending federate ID or -1 if the centralized coordination.
 */
void handle_message(int socket, int fed_id) {
    // FIXME: Need better error handling?
    // Read the header.
    size_t bytes_to_read = sizeof(uint16_t) + sizeof(uint16_t) + sizeof(int32_t);
    unsigned char buffer[bytes_to_read];
    read_from_socket_errexit(socket, bytes_to_read, buffer,
            "Failed to read message header.");

    // Extract the header information.
    unsigned short port_id;
    unsigned short federate_id;
    size_t length;
    extract_header(buffer, &port_id, &federate_id, &length);
    // Check if the message is intended for this federate
    assert(_lf_my_fed_id == federate_id);
    LF_PRINT_DEBUG("Receiving message to port %d of length %zu.", port_id, length);

    // Get the triggering action for the corerponding port
    trigger_t* action = _lf_action_for_port(port_id);

    // Read the payload.
    // Allocate memory for the message contents.
    unsigned char* message_contents = (unsigned char*)malloc(length);
    read_from_socket_errexit(socket, length, message_contents,
            "Failed to read message body.");

    LF_PRINT_LOG("Message received by federate: %s. Length: %zu.", message_contents, length);

    LF_PRINT_DEBUG("Calling schedule for message received on a physical connection.");
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
 * @param fed_id The sending federate ID or -1 if the centralized coordination.
 */
void handle_tagged_message(int socket, int fed_id) {
    // FIXME: Need better error handling?
    // Read the header which contains the timestamp.
    size_t bytes_to_read = sizeof(uint16_t) + sizeof(uint16_t) + sizeof(int32_t)
            + sizeof(instant_t) + sizeof(microstep_t);
    unsigned char buffer[bytes_to_read];
    read_from_socket_errexit(socket, bytes_to_read, buffer,
            "Failed to read timed message header");

    // Extract the header information.
    unsigned short port_id;
    unsigned short federate_id;
    size_t length;
    tag_t intended_tag;
    extract_timed_header(buffer, &port_id, &federate_id, &length, &intended_tag);
    // Check if the message is intended for this federate
    assert(_lf_my_fed_id == federate_id);
    LF_PRINT_DEBUG("Receiving message to port %d of length %zu.", port_id, length);

    // Get the triggering action for the corresponding port
    trigger_t* action = _lf_action_for_port(port_id);

    // Record the physical time of arrival of the message
    action->physical_time_of_arrival = lf_time_physical();

    if (action->is_physical) {
        // Messages sent on physical connections should be handled via handle_message().
        lf_print_error_and_exit("Received a timed message on a physical connection.");
    }

#ifdef FEDERATED_DECENTRALIZED
    // Only applicable for federated programs with decentralized coordination:
    // For logical connections in decentralized coordination,
    // increment the barrier to prevent advancement of tag beyond
    // the received tag if possible. The following function call
    // suggests that the tag barrier be raised to the tag provided
    // by the message. If this tag is in the past, the function will cause
    // the tag to freeze at the current level.
    // If something happens, make sure to release the barrier.
    _lf_increment_global_tag_barrier(intended_tag);
#endif
    LF_PRINT_LOG("Received message with tag: " PRINTF_TAG ", Current tag: " PRINTF_TAG ".",
            intended_tag.time - start_time, intended_tag.microstep,
            lf_time_logical_elapsed(), lf_tag().microstep);

    // Read the payload.
    // Allocate memory for the message contents.
    unsigned char* message_contents = (unsigned char*)malloc(length);
    read_from_socket_errexit(socket, length, message_contents,
            "Failed to read message body.");

    // The following is only valid for string messages.
    // LF_PRINT_DEBUG("Message received: %s.", message_contents);

    lf_mutex_lock(&mutex);

    // Create a token for the message
    lf_token_t* message_token = create_token(action->element_size);
    // Set up the token

    message_token->value = message_contents;
    message_token->length = length;

    // Sanity checks
#ifdef FEDERATED_DECENTRALIZED
    if (lf_tag_compare(intended_tag,
            action->last_known_status_tag) < 0) {        
        lf_print_error_and_exit("The following contract was violated for a timed message: In-order "
                             "delivery of messages over a TCP socket. Had status for " PRINTF_TAG ", got "
                             "timed message with intended tag " PRINTF_TAG ".",
                             action->last_known_status_tag.time - start_time,
                             action->last_known_status_tag.microstep,
                             intended_tag.time - start_time,
                             intended_tag.microstep);
    }
#endif // In centralized coordination, a TAG message from the RTI 
       // can set the last_known_status_tag to a future tag where messages
       // have not arrived yet.

    // FIXME: It might be enough to just check this field and not the status at all
    update_last_known_status_on_input_port(intended_tag, port_id);

    // Check whether reactions need to be inserted directly into the reaction
    // queue or a call to schedule is needed. This checks if the intended
    // tag of the message is for the current tag or a tag that is already
    // passed and if any control reaction is waiting on this port (or the
    // execution hasn't even started).
    // If the tag is intended for a tag that is passed, the control reactions
    // would need to exit because only one message can be processed per tag,
    // and that message is going to be a tardy message. The actual tardiness
    // handling is done inside _lf_insert_reactions_for_trigger.
    // To prevent multiple processing of messages per tag,
    // we also need to check the port status.
    // For example, there could be a case where current tag is 
    // 10 with a control reaction waiting, and a message has arrived with intended_tag 8.
    // This message will eventually cause the control reaction to exit, but before that,
    // a message with intended_tag of 9 could arrive before the control reaction has had a chance
    // to exit. The port status is on the other hand changed in this thread, and thus,
    // can be checked in this scenario without this race condition. The message with 
    // intended_tag of 9 in this case needs to wait one microstep to be processed.
    if (lf_tag_compare(intended_tag, lf_tag()) <= 0 && // The event is meant for the current or a previous tag.                          
            ((action->is_a_control_reaction_waiting && // Check if a control reaction is waiting and
             action->status == unknown) ||             // if the status of the port is still unknown.
             (_lf_execution_started == false))         // Or, execution hasn't even started, so it's safe to handle this event.
    ) {
        // Since the message is intended for the current tag and a control reaction
        // was waiting for the message, trigger the corresponding reactions for this
        // message.
        LF_PRINT_LOG(
            "Inserting reactions directly at tag " PRINTF_TAG ". "
            "Intended tag: " PRINTF_TAG ".",
            lf_tag().time - lf_time_start(),
            lf_tag().microstep, 
            intended_tag.time - lf_time_start(), 
            intended_tag.microstep
        );
        action->intended_tag = intended_tag;
        _lf_insert_reactions_for_trigger(action, message_token);

        // Set the status of the port as present here to inform the network input
        // control reactions know that they no longer need to block. The reason for
        // that is because the network receiver reaction is now in the reaction queue
        // keeping the precedence order intact.
        set_network_port_status(port_id, present);        
        // Port is now present. Therefore, notify the network input control reactions to
        // stop waiting and re-check the port status.
        lf_cond_broadcast(&port_status_changed);
    } else {
        // If no control reaction is waiting for this message, or if the intended
        // tag is in the future, use schedule functions to process the message.

        // Before that, if the current time >= stop time, discard the message.
        // But only if the stop time is not equal to the start time!
        if (lf_tag_compare(lf_tag(), stop_tag) >= 0) {
            lf_mutex_unlock(&mutex);
            lf_print_error("Received message too late. Already at stop tag.\n"
            		"Current tag is " PRINTF_TAG " and intended tag is " PRINTF_TAG ".\n"
            		"Discarding message.",
					lf_tag().time - start_time, lf_tag().microstep,
					intended_tag.time - start_time, intended_tag.microstep);
            return;
        }
        
        LF_PRINT_LOG("Calling schedule with tag " PRINTF_TAG ".", intended_tag.time - start_time, intended_tag.microstep);
        schedule_message_received_from_network_already_locked(action, intended_tag, message_token);
    }


#ifdef FEDERATED_DECENTRALIZED // Only applicable for federated programs with decentralized coordination
    // Finally, decrement the barrier to allow the execution to continue
    // past the raised barrier
    _lf_decrement_global_tag_barrier_locked();
#endif

    // The mutex is unlocked here after the barrier on
    // logical time has been removed to avoid
    // the need for unecessary lock and unlock
    // operations.
    lf_mutex_unlock(&mutex);
}

/**
 * Handle a time advance grant (TAG) message from the RTI.
 * This updates the last known status tag for each network input
 * port, and broadcasts a signal, which may cause a blocking
 * control reaction to unblock.
 *
 * In addition, this updates the last known TAG/PTAG and broadcasts
 * a notification of this update, which may unblock whichever worker
 * thread is trying to advance time.
 *
 * This function assumes the caller does not hold the mutex lock,
 * which it acquires.
 * 
 * @note This function is very similar to handle_provisinal_tag_advance_grant() except that
 *  it sets last_TAG_was_provisional to false.
 */
void handle_tag_advance_grant() {
    size_t bytes_to_read = sizeof(instant_t) + sizeof(microstep_t);
    unsigned char buffer[bytes_to_read];
    read_from_socket_errexit(_fed.socket_TCP_RTI, bytes_to_read, buffer,
            "Failed to read tag advance grant from RTI.");
    tag_t TAG = extract_tag(buffer);

    lf_mutex_lock(&mutex);

    // Update the last known status tag of all network input ports
    // to the TAG received from the RTI. Here we assume that the RTI
    // knows the status of network ports up to and including the granted tag,
    // so by extension, we assume that the federate can safely rely
    // on the RTI to handle port statuses up until the granted tag.
    update_last_known_status_on_input_ports(TAG);

    // It is possible for this federate to have received a PTAG
    // earlier with the same tag as this TAG.
    if (lf_tag_compare(TAG, _fed.last_TAG) >= 0) {
        _fed.last_TAG.time = TAG.time;
        _fed.last_TAG.microstep = TAG.microstep;
        _fed.is_last_TAG_provisional = false;
        LF_PRINT_LOG("Received Time Advance Grant (TAG): " PRINTF_TAG ".",
                _fed.last_TAG.time - start_time, _fed.last_TAG.microstep);
    } else {
        lf_mutex_unlock(&mutex);
        lf_print_error("Received a TAG " PRINTF_TAG " that wasn't larger "
                "than the previous TAG or PTAG " PRINTF_TAG ". Ignoring the TAG.",
                TAG.time - start_time, TAG.microstep,
                _fed.last_TAG.time - start_time, _fed.last_TAG.microstep);
    }

    _fed.waiting_for_TAG = false;
    // Notify everything that is blocked.
    lf_cond_broadcast(&event_q_changed);

    lf_mutex_unlock(&mutex);
}

/**
 * Send a logical tag complete (LTC) message to the RTI
 * unless an equal or later LTC has previously been sent.
 * This function assumes the caller holds the mutex lock.
 *
 * @param tag_to_send The tag to send.
 */
void _lf_logical_tag_complete(tag_t tag_to_send) {
    int compare_with_last_tag = lf_tag_compare(_fed.last_sent_LTC, tag_to_send);
    if (compare_with_last_tag >= 0) {
        return;
    }
    LF_PRINT_LOG("Sending Logical Time Complete (LTC) " PRINTF_TAG " to the RTI.",
            tag_to_send.time - start_time,
            tag_to_send.microstep);
    _lf_send_tag(MSG_TYPE_LOGICAL_TAG_COMPLETE, tag_to_send, true);
    _fed.last_sent_LTC = tag_to_send;
}

/**
 * Handle a provisional tag advance grant (PTAG) message from the RTI.
 * This updates the last known TAG/PTAG and broadcasts
 * a notification of this update, which may unblock whichever worker
 * thread is trying to advance time.
 * If current_time is less than the specified PTAG, then this will
 * also insert into the event_q a dummy event with the specified tag.
 * This will ensure that the federate advances time to the specified
 * tag and, for centralized coordination, inserts blocking reactions
 * and null-message-sending output reactions at that tag.
 *
 * This function assumes the caller does not hold the mutex lock,
 * which it acquires.
 * 
 * @note This function is similar to handle_tag_advance_grant() except that
 *  it sets last_TAG_was_provisional to true and also it does not update the
 *  last known tag for input ports.
 */
void handle_provisional_tag_advance_grant() {
    size_t bytes_to_read = sizeof(instant_t) + sizeof(microstep_t);
    unsigned char buffer[bytes_to_read];    
    read_from_socket_errexit(_fed.socket_TCP_RTI, bytes_to_read, buffer,
            "Failed to read provisional tag advance grant from RTI.");
    tag_t PTAG = extract_tag(buffer);

    // Note: it is important that last_known_status_tag of ports does not
    // get updated to a PTAG value because a PTAG does not indicate that
    // the RTI knows about the status of all ports up to and _including_
    // the value of PTAG. Only a TAG message indicates that.
    lf_mutex_lock(&mutex);

    // Sanity check
    if (lf_tag_compare(PTAG, _fed.last_TAG) < 0
            || (lf_tag_compare(PTAG, _fed.last_TAG) == 0 && !_fed.is_last_TAG_provisional)) {
        lf_mutex_unlock(&mutex);
        lf_print_error_and_exit("Received a PTAG " PRINTF_TAG " that is equal or earlier "
                "than an already received TAG " PRINTF_TAG ".",
                PTAG.time, PTAG.microstep,
                _fed.last_TAG.time, _fed.last_TAG.microstep);
    }

    _fed.last_TAG = PTAG;
    _fed.waiting_for_TAG = false;
    _fed.is_last_TAG_provisional = true;
    LF_PRINT_LOG("At tag " PRINTF_TAG ", received Provisional Tag Advance Grant (PTAG): " PRINTF_TAG ".",
            current_tag.time - start_time, current_tag.microstep,
            _fed.last_TAG.time - start_time, _fed.last_TAG.microstep);

    // Even if we don't modify the event queue, we need to broadcast a change
    // because we do not need to continue to wait for a TAG.
    lf_cond_broadcast(&event_q_changed);
    // Notify control reactions that are blocked.
    // Check here whether there is any control reaction waiting
    // before broadcasting to avoid an unnecessary broadcast.
    // This also avoids problems waking up threads before execution
    // has started (while they are waiting for the start time).
    if (is_input_control_reaction_blocked()) {
        lf_cond_broadcast(&port_status_changed);
    }

    // Possibly insert a dummy event into the event queue if current time is behind
    // (which it should be). Do not do this if the federate has not fully
    // started yet.

    instant_t dummy_event_time = PTAG.time;
    microstep_t dummy_event_relative_microstep = PTAG.microstep;

    if (lf_tag_compare(current_tag, PTAG) == 0) {
        // The current tag can equal the PTAG if we are at the start time
        // or if this federate has been able to advance time to the current
        // tag (e.g., it has no upstream federates). In either case, either
        // it is already treating the current tag as PTAG cycle (e.g. at the
        // start time) or it will be completing the current cycle and sending
        // a LTC message shortly. In either case, there is nothing more to do.
           lf_mutex_unlock(&mutex);
        return;
    } else if (lf_tag_compare(current_tag, PTAG) > 0) {
        // Current tag is greater than the PTAG.
        // It could be that we have sent an LTC that crossed with the incoming
        // PTAG or that we have advanced to a tag greater than the PTAG.
        // In the former case, there is nothing more to do.
        // In the latter case, we may be blocked processing a PTAG cycle at
        // a greater tag or we may be in the middle of processing a regular
        // TAG. In either case, we know that at the PTAG tag, all outputs
        // have either been sent or are absent, so we can send an LTC.
        // Send an LTC to indicate absent outputs.
        _lf_logical_tag_complete(PTAG);
        // Nothing more to do.
           lf_mutex_unlock(&mutex);
        return;
    } else if (PTAG.time == current_tag.time) {
        // We now know current_tag < PTAG, but the times are equal.
        // Adjust the microstep for scheduling the dummy event.
        dummy_event_relative_microstep -= current_tag.microstep;
    }
    // We now know current_tag < PTAG.

    if (dummy_event_time != FOREVER) {
        // Schedule a dummy event at the specified time and (relative) microstep.
        LF_PRINT_DEBUG("At tag " PRINTF_TAG ", inserting into the event queue a dummy event "
               "with time " PRINTF_TIME " and (relative) microstep " PRINTF_MICROSTEP ".",
        current_tag.time - start_time, current_tag.microstep,
        dummy_event_time - start_time, dummy_event_relative_microstep);
        // Dummy event points to a NULL trigger and NULL real event.
        event_t* dummy = _lf_create_dummy_events(
                NULL, dummy_event_time, NULL, dummy_event_relative_microstep);
        pqueue_insert(event_q, dummy);
    }

    lf_mutex_unlock(&mutex);
}

/** 
 * Send a MSG_TYPE_STOP_REQUEST message to the RTI with payload equal
 * to the current tag plus one microstep.
 * 
 * This function raises a global barrier on
 * logical tag at the current tag.
 * 
 * This function assumes the caller holds the mutex lock.
 */
void _lf_fd_send_stop_request_to_rti() {
    // Do not send a stop request twice.
    if (_fed.sent_a_stop_request_to_rti == true) {
        return;
    }
    LF_PRINT_LOG("Requesting the whole program to stop.");
    // Raise a logical time barrier at the current tag.
    _lf_increment_global_tag_barrier_already_locked(current_tag);

    // Send a stop request with the current tag to the RTI
    unsigned char buffer[MSG_TYPE_STOP_REQUEST_LENGTH];
    // Stop at the next microstep
    ENCODE_STOP_REQUEST(buffer, current_tag.time, current_tag.microstep + 1);

    lf_mutex_lock(&outbound_socket_mutex);
    if (_fed.socket_TCP_RTI < 0) {
        lf_print_warning("Socket is no longer connected. Dropping message.");
        lf_mutex_unlock(&outbound_socket_mutex);
        return;
    }
    write_to_socket_errexit_with_mutex(_fed.socket_TCP_RTI, MSG_TYPE_STOP_REQUEST_LENGTH, 
            buffer, &outbound_socket_mutex,
            "Failed to send stop time " PRINTF_TIME " to the RTI.", current_tag.time - start_time);
    lf_mutex_unlock(&outbound_socket_mutex);
    _fed.sent_a_stop_request_to_rti = true;
}

/** 
 * Handle a MSG_TYPE_STOP_GRANTED message from the RTI.
 * 
 * This function removes the global barrier on
 * logical time raised when lf_request_stop() was
 * called.
 * 
 * This function assumes the caller does not hold
 * the mutex lock, therefore, it acquires it.
 */
void handle_stop_granted_message() {
    size_t bytes_to_read = MSG_TYPE_STOP_GRANTED_LENGTH - 1;
    unsigned char buffer[bytes_to_read];    
    read_from_socket_errexit(_fed.socket_TCP_RTI, bytes_to_read, buffer,
            "Failed to read stop granted from RTI.");

    // Acquire a mutex lock to ensure that this state does change while a
    // message is transport or being used to determine a TAG.
    lf_mutex_lock(&mutex);

    tag_t received_stop_tag = extract_tag(buffer);

    LF_PRINT_LOG("Received from RTI a MSG_TYPE_STOP_GRANTED message with elapsed tag " PRINTF_TAG ".",
            received_stop_tag.time - start_time, received_stop_tag.microstep);
    
    // Sanity check.
    tag_t current_tag = lf_tag();
    if (lf_tag_compare(received_stop_tag, current_tag) <= 0) {
        lf_print_error("RTI granted a MSG_TYPE_STOP_GRANTED tag that is equal to or less than this federate's current tag " PRINTF_TAG ". "
                "Stopping at the next microstep instead.",
                current_tag.time - start_time, current_tag.microstep);
        received_stop_tag = current_tag;
        received_stop_tag.microstep++;
    }

    stop_tag = received_stop_tag;
    LF_PRINT_DEBUG("Setting the stop tag to " PRINTF_TAG ".",
                stop_tag.time - start_time,
                stop_tag.microstep);

    _lf_decrement_global_tag_barrier_locked();
    // We signal instead of broadcast under the assumption that only
    // one worker thread can call wait_until at a given time because
    // the call to wait_until is protected by a mutex lock
    lf_cond_signal(&event_q_changed);
    lf_mutex_unlock(&mutex);
}

/**
 * Handle a MSG_TYPE_STOP_REQUEST message from the RTI.
 * 
 * This function assumes the caller does not hold
 * the mutex lock, therefore, it acquires it.
 */
void handle_stop_request_message() {
    size_t bytes_to_read = MSG_TYPE_STOP_REQUEST_LENGTH - 1;
    unsigned char buffer[bytes_to_read];    
    read_from_socket_errexit(_fed.socket_TCP_RTI, bytes_to_read, buffer,
            "Failed to read stop request from RTI.");

    // Acquire a mutex lock to ensure that this state does change while a
    // message is being used to determine a TAG.
    lf_mutex_lock(&mutex);
    // Ignore the message if this federate originated a request.
    // The federate is already blocked is awaiting a MSG_TYPE_STOP_GRANTED message.
    if (_fed.sent_a_stop_request_to_rti == true) {
        lf_mutex_unlock(&mutex);
        return;
    }

    tag_t tag_to_stop = extract_tag(buffer);

    LF_PRINT_LOG("Received from RTI a MSG_TYPE_STOP_REQUEST message with tag " PRINTF_TAG ".",
             tag_to_stop.time - start_time,
             tag_to_stop.microstep);

    // Encode the current logical time plus one microstep
    // or the requested tag_to_stop, whichever is bigger.
    if (lf_tag_compare(tag_to_stop, current_tag) <= 0) {
        // Can't stop at the requested tag. Make a counteroffer.
        tag_to_stop = current_tag;
        tag_to_stop.microstep++;
    }

    unsigned char outgoing_buffer[MSG_TYPE_STOP_REQUEST_REPLY_LENGTH];
    ENCODE_STOP_REQUEST_REPLY(outgoing_buffer, tag_to_stop.time, tag_to_stop.microstep);

    lf_mutex_lock(&outbound_socket_mutex);
    if (_fed.socket_TCP_RTI < 0) {
        lf_print_warning("Socket is no longer connected. Dropping message.");
        lf_mutex_unlock(&outbound_socket_mutex);
        lf_mutex_unlock(&mutex);
        return;
    }
    // Send the current logical time to the RTI. This message does not have an identifying byte since
    // since the RTI is waiting for a response from this federate.
    write_to_socket_errexit_with_mutex(
            _fed.socket_TCP_RTI, MSG_TYPE_STOP_REQUEST_REPLY_LENGTH, outgoing_buffer, &outbound_socket_mutex,
            "Failed to send the answer to MSG_TYPE_STOP_REQUEST to RTI.");
    lf_mutex_unlock(&outbound_socket_mutex);

    // Raise a barrier at current tag
    // because we are sending it to the RTI
    _lf_increment_global_tag_barrier_already_locked(tag_to_stop);

    // A subsequent call to lf_request_stop will be a no-op.
    _fed.sent_a_stop_request_to_rti = true;

    lf_mutex_unlock(&mutex);
}

/**
 * Close sockets used to communicate with other federates, if they are open,
 * and send a MSG_TYPE_RESIGN message to the RTI. This implements the function
 * defined in reactor.h. For unfederated execution, the code generator
 * generates an empty implementation.
 */
void terminate_execution() {
    // Check for all outgoing physical connections in
    // _fed.sockets_for_outbound_p2p_connections and
    // if the socket ID is not -1, the connection is still open.
    // Send an EOF by closing the socket here.
    // NOTE: It is dangerous to acquire a mutex in a termination
    // process because it can block program exit if a deadlock occurs.
    // Hence, it is paramount that these mutexes not allow for any
    // possibility of deadlock. To ensure this, this
    // function should NEVER be called while holding any mutex lock.
    lf_mutex_lock(&outbound_socket_mutex);
    for (int i=0; i < NUMBER_OF_FEDERATES; i++) {
        // Close outbound connections, in case they have not closed themselves.
        // This will result in EOF being sent to the remote federate, I think.
        _lf_close_outbound_socket(i);
    }
    // Resign the federation, which will close the socket to the RTI.
       if (_fed.socket_TCP_RTI >= 0) {
        unsigned char message_marker = MSG_TYPE_RESIGN;
        ssize_t written = write_to_socket(_fed.socket_TCP_RTI, 1, &message_marker);
        if (written == 1) {
            LF_PRINT_LOG("Resigned.");
        }
    }
    lf_mutex_unlock(&outbound_socket_mutex);

    LF_PRINT_DEBUG("Requesting closing of incoming P2P sockets.");
    // Request closing the incoming P2P sockets.
    for (int i=0; i < NUMBER_OF_FEDERATES; i++) {
        if (_lf_request_close_inbound_socket(i) == 0) {
            // Sending the close request failed. Mark the socket closed.
            _fed.sockets_for_inbound_p2p_connections[i] = -1;
        }
    }

    LF_PRINT_DEBUG("Waiting for inbound p2p socket listener threads.");
    // Wait for each inbound socket listener thread to close.
    if (_fed.number_of_inbound_p2p_connections > 0) {
        LF_PRINT_LOG("Waiting for %zu threads listening for incoming messages to exit.",
                _fed.number_of_inbound_p2p_connections);
        for (int i=0; i < _fed.number_of_inbound_p2p_connections; i++) {
            // Ignoring errors here.
            lf_thread_join(_fed.inbound_socket_listeners[i], NULL);
        }
    }

    LF_PRINT_DEBUG("Waiting for RTI's socket listener threads.");
    // Wait for the thread listening for messages from the RTI to close.
    lf_thread_join(_fed.RTI_socket_listener, NULL);

    LF_PRINT_DEBUG("Freeing memory occupied by the federate.");
    free(_fed.inbound_socket_listeners);
    free(federation_metadata.rti_host);
    free(federation_metadata.rti_user);
}

/** 
 * Thread that listens for inputs from other federates.
 * This thread listens for messages of type MSG_TYPE_P2P_MESSAGE,
 * MSG_TYPE_P2P_TAGGED_MESSAGE, or MSG_TYPE_PORT_ABSENT (@see net_common.h) from the specified
 * peer federate and calls the appropriate handling function for
 * each message type. If an error occurs or an EOF is received
 * from the peer, then this procedure sets the corresponding 
 * socket in _fed.sockets_for_inbound_p2p_connections
 * to -1 and returns, terminating the thread.
 * @param fed_id_ptr A pointer to a uint16_t containing federate ID being listened to.
 *  This procedure frees the memory pointed to before returning.
 */
void* listen_to_federates(void* fed_id_ptr) {

    uint16_t fed_id = *((uint16_t*)fed_id_ptr);

    LF_PRINT_LOG("Listening to federate %d.", fed_id);

    int socket_id = _fed.sockets_for_inbound_p2p_connections[fed_id];

    // Buffer for incoming messages.
    // This does not constrain the message size
    // because the message will be put into malloc'd memory.
    unsigned char buffer[FED_COM_BUFFER_SIZE];

    // Listen for messages from the federate.
    while (1) {
        // Read one byte to get the message type.
        LF_PRINT_DEBUG("Waiting for a P2P message on socket %d.", socket_id);
        ssize_t bytes_read = read_from_socket(socket_id, 1, buffer);
        if (bytes_read == 0) {
            // EOF occurred. This breaks the connection.
            lf_print("Received EOF from peer federate %d. Closing the socket.", fed_id);
            _lf_close_inbound_socket(fed_id);
            break;
        } else if (bytes_read < 0) {
            lf_print_error("P2P socket to federate %d is broken.", fed_id);
            _lf_close_inbound_socket(fed_id);
            break;
        }
        LF_PRINT_DEBUG("Received a P2P message on socket %d of type %d.",
                socket_id, buffer[0]);
        bool bad_message = false;
        switch (buffer[0]) {
            case MSG_TYPE_P2P_MESSAGE:
                LF_PRINT_LOG("Received untimed message from federate %d.", fed_id);
                handle_message(socket_id, fed_id);
                break;
            case MSG_TYPE_P2P_TAGGED_MESSAGE:
                LF_PRINT_LOG("Received timed message from federate %d.", fed_id);
                handle_tagged_message(socket_id, fed_id);
                break;
            case MSG_TYPE_PORT_ABSENT:
                LF_PRINT_LOG("Received port absent message from federate %d.", fed_id);
                handle_port_absent_message(socket_id, fed_id);
                break;
            default:
                bad_message = true;
        }
        if (bad_message) {
            // FIXME: Better error handling needed.
            lf_print_error("Received erroneous message type: %d. Closing the socket.", buffer[0]);
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
        // Check whether the RTI socket is still valid
        if (_fed.socket_TCP_RTI < 0) {
            lf_print_warning("Socket to the RTI unexpectedly closed.");
            return NULL;
        }
        // Read one byte to get the message type.
        // This will exit if the read fails.
        ssize_t bytes_read = read_from_socket(_fed.socket_TCP_RTI, 1, buffer);
        if (bytes_read < 0) {
            if (errno == ECONNRESET) {
                lf_print_error("Socket connection to the RTI was closed by the RTI without"
                            " properly sending an EOF first. Considering this a soft error.");
                // FIXME: If this happens, possibly a new RTI must be elected.
                _fed.socket_TCP_RTI = -1;
                return NULL;
            } else {
                lf_print_error("Socket connection to the RTI has been broken" 
                                    " with error %d: %s. The RTI should"
                                    " close connections with an EOF first."
                                    " Considering this a soft error.", 
                                    errno, 
                                    strerror(errno));
                // FIXME: If this happens, possibly a new RTI must be elected.
                _fed.socket_TCP_RTI = -1;
                return NULL;                   
            }
        } else if (bytes_read == 0) {
            // EOF received.
            lf_print("Connection to the RTI closed with an EOF.");
            _fed.socket_TCP_RTI = -1;
            return NULL;
        }
        switch (buffer[0]) {
            case MSG_TYPE_TAGGED_MESSAGE:
                handle_tagged_message(_fed.socket_TCP_RTI, -1);
                break;
            case MSG_TYPE_TAG_ADVANCE_GRANT:
                handle_tag_advance_grant();
                break;
            case MSG_TYPE_PROVISIONAL_TAG_ADVANCE_GRANT:
                handle_provisional_tag_advance_grant();
                break;
            case MSG_TYPE_STOP_REQUEST:
                handle_stop_request_message();
                break;
            case MSG_TYPE_STOP_GRANTED:
                handle_stop_granted_message();
                break;
            case MSG_TYPE_PORT_ABSENT:
                handle_port_absent_message(_fed.socket_TCP_RTI, -1);
                break;
            case MSG_TYPE_CLOCK_SYNC_T1:
            case MSG_TYPE_CLOCK_SYNC_T4:
                lf_print_error("Federate %d received unexpected clock sync message from RTI on TCP socket.",
                            _lf_my_fed_id);
                break;
            default:
                lf_print_error_and_exit("Received from RTI an unrecognized TCP message type: %hhx.", buffer[0]);
        }
    }
    return NULL;
}

/** 
 * Synchronize the start with other federates via the RTI.
 * This assumes that a connection to the RTI is already made 
 * and _fed.socket_TCP_RTI is valid. It then sends the current logical
 * time to the RTI and waits for the RTI to respond with a specified
 * time. It starts a thread to listen for messages from the RTI.
 * It then waits for physical time to match the specified time,
 * sets current logical time to the time returned by the RTI,
 * and then returns. If --fast was specified, then this does
 * not wait for physical time to match the logical start time
 * returned by the RTI.
 * 
 * FIXME: Possibly should be renamed
 */
void synchronize_with_other_federates() {

    LF_PRINT_DEBUG("Synchronizing with other federates.");

    // Reset the start time to the coordinated start time for all federates.
    // Note that this does not grant execution to this federate. In the centralized
    // coordination, the tag (0,0) should be explicitly sent to the RTI on a Time
    // Advance Grant message to request for permission to execute. In the decentralized
    // coordination, either the after delay on the connection must be sufficiently large
    // enough or the STP offset must be set globally to an accurate value.
    start_time = get_start_time_from_rti(lf_time_physical());

    if (duration >= 0LL) {
        // A duration has been specified. Recalculate the stop time.
       stop_tag = ((tag_t) {.time = start_time + duration, .microstep = 0});
    }
    
    // Start a thread to listen for incoming TCP messages from the RTI.
    // @note Up until this point, the federate has been listening for messages
    //  from the RTI in a sequential manner in the main thread. From now on, a
    //  separate thread is created to allow for asynchronous communication.
    lf_thread_create(&_fed.RTI_socket_listener, listen_to_rti_TCP, NULL);

    lf_thread_t thread_id;
    if (create_clock_sync_thread(&thread_id)) {
        lf_print_warning("Failed to create thread to handle clock synchronization.");
    }
}

/**
 * Modify the specified tag, if necessary, to be an earlier tag based
 * on the current physical time. The earlier tag is necessary if this federate
 * has downstream federates and also has physical actions that may trigger
 * outputs.  In that case, the earlier tag will be the current physical time
 * plus the minimum delay on all such physical actions plus any other delays
 * along the path from the triggering physical action to the output port
 * minus one nanosecond. The modified tag is assured of being less than any
 * output tag that might later be produced.
 * @param tag A pointer to the proposed NET.
 * @return True if this federate requires this modification and the tag was
 *  modified.
 */
bool _lf_bounded_NET(tag_t* tag) {
    // The tag sent by this function is a promise that, absent
    // inputs from another federate, this federate will not produce events
    // earlier than t. But if there are downstream federates and there is
    // a physical action (not counting receivers from upstream federates),
    // then we can only promise up to current physical time (plus the minimum
    // of all minimum delays on the physical actions).
    // In this case, we send a NET message with the current physical time
    // to permit downstream federates to advance. To avoid
    // overwhelming the network, this NET message should be sent periodically
    // at specified intervals controlled by the target parameter
    // coordination-options: {advance-message-interval: time units}.
    // The larger the interval, the more downstream federates will lag
    // behind real time, but the less network traffic. If this option is
    // missing, we issue a warning message suggesting that a redesign
    // might be in order so that outputs don't depend on physical actions.
    LF_PRINT_DEBUG("Checking NET to see whether it should be bounded by physical time."
            " Min delay from physical action: " PRINTF_TIME ".",
            _fed.min_delay_from_physical_action_to_federate_output);
    if (_fed.min_delay_from_physical_action_to_federate_output >= 0LL
            && _fed.has_downstream
    ) {
        // There is a physical action upstream of some output from this
        // federate, and there is at least one downstream federate.
        // Compare the tag to the current physical time.
        instant_t physical_time = lf_time_physical();
        if (physical_time + _fed.min_delay_from_physical_action_to_federate_output < tag->time) {
            // Can only promise up and not including this new time:
            tag->time = physical_time + _fed.min_delay_from_physical_action_to_federate_output - 1L;
            tag->microstep = 0;
            LF_PRINT_LOG("Has physical actions that bound NET to " PRINTF_TAG ".",
                    tag->time - start_time, tag->microstep);
            return true;
        }
    }
    return false;
}

/** 
 * If this federate depends on upstream federates or sends data to downstream
 * federates, then send to the RTI a NET, which will give the tag of the
 * earliest event on the event queue, or, if the queue is empty, the timeout
 * time, or, if there is no timeout, FOREVER.
 * 
 * If there are network outputs that
 * depend on physical actions, then insert a dummy event to ensure this federate
 * advances its tag so that downstream federates can make progress.
 *
 * A NET is a promise saying that, absent network inputs, this federate will
 * not produce an output message with tag earlier than the NET value.
 *
 * If there are upstream federates, then after sending a NET, this will block
 * until either the RTI grants the advance to the requested time or the wait
 * for the response from the RTI is interrupted by a change in the event queue
 * (e.g., a physical action triggered or a network message arrived).
 * If there are no upstream federates, then it will not wait for a TAG
 * (which won't be forthcoming anyway) and returns the earliest tag on the event queue.
 *
 * If the federate has neither upstream nor downstream federates, then this
 * returns the specified tag immediately without sending anything to the RTI.
 *
 * If there is at least one physical action somewhere in the federate that can
 * trigger an output to a downstream federate, then the NET is required to be
 * less than the current physical time. If physical time is less than the
 * earliest event in the event queue (or the event queue is empty), then this
 * function will insert a dummy event with a tag equal to the current physical
 * time (and a microstep of 0). This will enforce advancement of tag for this
 * federate and causes a NET message to be sent repeatedly as physical time
 * advances with the time interval between messages controlled by the target
 * parameter coordination-options: {advance-message-interval timevalue}. It will
 * stop creating dummy events if and when its event queue has an event with a
 * timestamp less than physical time.
 *
 * If wait_for_reply is false, then this function will simply send the
 * specified tag and return that tag immediately. This is useful when a
 * federate is shutting down and will not be sending any more messages at all.
 *
 * In all cases, this returns either the specified tag or
 * another tag when it is safe to advance logical time to the returned tag.
 * The returned tag may be less than the specified tag if there are upstream
 * federates and either the RTI responds with a lesser tag or
 * the wait for a response from the RTI is interrupted by a
 * change in the event queue.
 *
 * This function is used in centralized coordination only.
 *
 * This function assumes the caller holds the mutex lock.
 *
 * @param tag The tag.
 * @param wait_for_reply If true, wait for a reply.
 */
tag_t _lf_send_next_event_tag(tag_t tag, bool wait_for_reply) {
    while (true) {
        if (!_fed.has_downstream && !_fed.has_upstream) {
            // This federate is not connected (except possibly by physical links)
            // so there is no need for the RTI to get involved.

            // NOTE: If the event queue is empty, then the time argument is either
            // the timeout_time or FOREVER. If -fast is also set, then
            // it matters whether there are upstream federates connected by physical
            // connections, which do not affect _fed.has_upstream. Perhaps we
            // should not return immediately because
            // then the execution will hit its timeout_time and fail to receive any
            // messages sent by upstream federates.
            // However, -fast is really incompatible with federated execution with
            // physical connections, so I don't think we need to worry about this.
            LF_PRINT_DEBUG("Granted tag " PRINTF_TAG " because the federate has neither "
                    "upstream nor downstream federates.",
                    tag.time - start_time, tag.microstep);
            return tag;
        }

        // If time advance (TAG or PTAG) has already been granted for this tag
        // or a larger tag, then return immediately.
        if (lf_tag_compare(_fed.last_TAG, tag) >= 0) {
            LF_PRINT_DEBUG("Granted tag " PRINTF_TAG " because TAG or PTAG has been received.",
                    _fed.last_TAG.time - start_time, _fed.last_TAG.microstep);
            return _fed.last_TAG;
        }

        // Copy the tag because _lf_bounded_NET() may modify it.
        tag_t original_tag = tag;

        // A NET sent by this function is a promise that, absent
        // inputs from another federate, this federate will not produce events
        // earlier than t. But if there are downstream federates and there is
        // a physical action (not counting receivers from upstream federates),
        // then we can only promise up to current physical time (plus the minimum
        // of all minimum delays on the physical actions).
        // If wait_for_reply is false, leave the tag alone.
        bool tag_bounded_by_physical_time = wait_for_reply ?
                _lf_bounded_NET(&tag)
                : false;

        // What we do next depends on whether the NET has been bounded by
        // physical time or by an event on the event queue.
        if (!tag_bounded_by_physical_time) {
            // This if statement does not fall through but rather returns.
            // NET is not bounded by physical time or has no downstream federates.
            // Normal case.
            _lf_send_tag(MSG_TYPE_NEXT_EVENT_TAG, tag, wait_for_reply);
            _fed.last_sent_NET = tag;
            LF_PRINT_LOG("Sent next event tag (NET) " PRINTF_TAG " to RTI.",
                    tag.time - start_time, tag.microstep);

            if (!wait_for_reply) {
                LF_PRINT_LOG("Not waiting for reply to NET.");
                return tag;
            }

            // If there are no upstream federates, return immediately, without
            // waiting for a reply. This federate does not need to wait for
            // any other federate.
            // NOTE: If fast execution is being used, it may be necessary to
            // throttle upstream federates.
            if (!_fed.has_upstream) {
                LF_PRINT_DEBUG("Not waiting for reply to NET " PRINTF_TAG " because I "
                        "have no upstream federates.",
                        tag.time - start_time, tag.microstep);
                return tag;
            }
            // Fed has upstream federates. Have to wait for a TAG or PTAG.
            _fed.waiting_for_TAG = true;

            // Wait until a TAG is received from the RTI.
            while (true) {
                // Wait until either something changes on the event queue or
                // the RTI has responded with a TAG.
                LF_PRINT_DEBUG("Waiting for a TAG from the RTI.");
                if (lf_cond_wait(&event_q_changed, &mutex) != 0) {
                    lf_print_error("Wait error.");
                }
                // Either a TAG or PTAG arrived or something appeared on the event queue.
                if (!_fed.waiting_for_TAG) {
                    // _fed.last_TAG will have been set by the thread receiving the TAG message that
                    // set _fed.waiting_for_TAG to false.
                    return _fed.last_TAG;
                }
                // Check whether the new event on the event queue requires sending a new NET.
                tag_t next_tag = get_next_event_tag();
                if (lf_tag_compare(next_tag, tag) != 0) {
                    _lf_send_tag(MSG_TYPE_NEXT_EVENT_TAG, next_tag, wait_for_reply);
                    _fed.last_sent_NET = next_tag;
                    LF_PRINT_LOG("Sent next event tag (NET) " PRINTF_TAG " to RTI.",
                        next_tag.time - lf_time_start(), next_tag.microstep);
                }
            }
        }
        
        if (tag.time != FOREVER) {
            // Create a dummy event that will force this federate to advance time and subsequently enable progress for
            // downstream federates.
            event_t* dummy = _lf_create_dummy_events(NULL, tag.time, NULL, 0);
            pqueue_insert(event_q, dummy);
        }

        LF_PRINT_DEBUG("Inserted a dummy event for logical time " PRINTF_TIME ".",
                tag.time - lf_time_start());

        if (!wait_for_reply) {
            LF_PRINT_LOG("Not waiting physical time to advance further.");
            return tag;
        }

        // This federate should repeatedly advance its tag to ensure downstream federates can make progress.
        // Before advancing to the next tag, we need to wait some time so that we don't overwhelm the network and the
        // RTI. That amount of time will be no greater than ADVANCE_MESSAGE_INTERVAL in the future.
        LF_PRINT_DEBUG("Waiting for physical time to elapse or an event on the event queue.");

        // The above call to _lf_bounded_NET called lf_time_physical()
        // set _lf_last_reported_unadjusted_physical_time_ns, the
        // time obtained using CLOCK_REALTIME before adjustment for
        // clock synchronization. Since that is the clock used by
        // lf_cond_timedwait, this is the clock we want to use.
        instant_t wait_until_time_ns =
                _lf_last_reported_unadjusted_physical_time_ns + ADVANCE_MESSAGE_INTERVAL;

        // Regardless of the ADVANCE_MESSAGE_INTERVAL, do not let this
        // wait exceed the time of the next tag.
        if (wait_until_time_ns > original_tag.time) {
            wait_until_time_ns = original_tag.time;
        }

        lf_cond_timedwait(&event_q_changed, &mutex, wait_until_time_ns);

        LF_PRINT_DEBUG("Wait finished or interrupted.");

        // Either the timeout expired or the wait was interrupted by an event being
        // put onto the event queue. In either case, we can just loop around.
        // The next iteration will determine whether another
        // NET should be sent or not.
        tag = get_next_event_tag();
    }
}

/**
 * Parse the address of the RTI and store them into the global federation_metadata struct.
 * @return a parse_rti_code_t indicating the result of the parse.
 */
parse_rti_code_t parse_rti_addr(char* rti_addr) {
    bool has_host = false, has_port = false, has_user = false;
    rti_addr_info_t rti_addr_info = {0};
    extract_rti_addr_info(rti_addr, &rti_addr_info);
    if (!rti_addr_info.has_host && !rti_addr_info.has_port && !rti_addr_info.has_user) {
        return FAILED_TO_PARSE;
    }
    if (rti_addr_info.has_host) {
        if (validate_host(rti_addr_info.rti_host_str)) {
            char* rti_host = (char*) calloc(256, sizeof(char));
            strncpy(rti_host, rti_addr_info.rti_host_str, 255);
            federation_metadata.rti_host = rti_host;
        } else {
            return INVALID_HOST;
        }
    }
    if (rti_addr_info.has_port) {
        if (validate_port(rti_addr_info.rti_port_str)) {
            federation_metadata.rti_port = atoi(rti_addr_info.rti_port_str);
        } else {
            return INVALID_PORT;
        }
    }
    if (rti_addr_info.has_user) {
        if (validate_user(rti_addr_info.rti_user_str)) {
            char* rti_user = (char*) calloc(256, sizeof(char));
            strncpy(rti_user, rti_addr_info.rti_user_str, 255);
            federation_metadata.rti_user = rti_user;
        } else {
            return INVALID_USER;
        }
    }
    return SUCCESS;
}

/**
 * Sets the federation_id of this federate to fid.
 */
void set_federation_id(char* fid) {
    federation_metadata.federation_id = fid;
}
