/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
 * @author Soroush Bateni (soroush@utdallas.edu)
 *
 * @section LICENSE
Copyright (c) 2020-21, The University of California at Berkeley.

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
 * Utility functions for clock synchronization.
 */

#include "clock-sync.h"
#include "rti.h"

/** 
 * Keep a record of connection statistics
 * and the remote physical clock of the RTI.
 */
socket_stat_t _lf_rti_socket_stat = {
    .remote_physical_clock_snapshot_T1 = NEVER,
    .local_physical_clock_snapshot_T2 = NEVER,
    .local_delay = 0LL,
    .received_T4_messages_in_current_sync_window = 0,
    .history = 0LL,
    .network_stat_round_trip_delay_max = 0LL,
    .network_stat_sample_index = 0,
    .clock_synchronization_error_bound = 0LL
};

/**
 * Records the physical time at which the clock of this federate was
 * synchronized with the RTI. Used to calculate the drift.
 */
instant_t _lf_last_clock_sync_instant = 0LL;

/**
 * The UDP socket descriptor for this federate to communicate with the RTI.
 * This is set by setup_clock_synchronization_with_rti() in connect_to_rti() 
 * in federate.c, which must be called before other
 * functions that communicate with the rti are called.
 */
int _lf_rti_socket_UDP = -1;

#ifdef _LF_CLOCK_SYNC_COLLECT_STATS
/**
 * Update statistic on the socket based on the newly calculated network delay 
 * and clock synchronization error
 * 
 * @param socket_stat The socket_stat_t struct that  keeps track of stats for a given connection
 * @param network_round_trip_delay The newly calculated round trip delay to the remote federate/RTI
 * @param clock_synchronization_error The newly calculated clock synchronization error relative to
 *  the remote federate/RTI
 */
void update_socket_stat(socket_stat_t* socket_stat, 
                        long long network_round_trip_delay,
                        long long clock_synchronization_error) {
    // Add the data point
    socket_stat->network_stat_samples[socket_stat->network_stat_sample_index] = network_round_trip_delay;
    socket_stat->network_stat_sample_index++;
    
    // Calculate maximums
    if (socket_stat->network_stat_round_trip_delay_max < network_round_trip_delay) {
        socket_stat->network_stat_round_trip_delay_max = network_round_trip_delay;
    }

    if (socket_stat->clock_synchronization_error_bound < clock_synchronization_error) {
        socket_stat->clock_synchronization_error_bound = clock_synchronization_error;
    }
}

/**
 * Calculate statistics of the socket.
 * The releavent information is returned as a lf_stat struct.
 * 
 * @param socket_stat The socket_stat_t struct that  keeps track of stats for a given connection
 */
lf_stat_ll calculate_socket_stat(struct socket_stat_t* socket_stat) {
    // Initialize the stat struct
    lf_stat_ll stats = {0, 0, 0, 0};
    // Calculate the average and max
    for (int i = 0; i < socket_stat->network_stat_sample_index; i++) {
        if (socket_stat->network_stat_samples[i] > stats.max) {
            stats.max = socket_stat->network_stat_samples[i];
        }
        stats.average += socket_stat->network_stat_samples[i] / socket_stat->network_stat_sample_index;
    }
    for (int i = 0; i < socket_stat->network_stat_sample_index; i++) {
        long long delta = socket_stat->network_stat_samples[i] - stats.average;
        stats.variance += powl(delta, 2);
    }
    stats.variance /= socket_stat->network_stat_sample_index;
    stats.standard_deviation = sqrtl(stats.variance);

    return stats;
}
#endif

/**
 * Reset statistics on the socket.
 * 
 * @param socket_stat The socket_stat_t struct that  keeps track of stats for a given connection
 */
void reset_socket_stat(struct socket_stat_t* socket_stat) {
    socket_stat->received_T4_messages_in_current_sync_window = 0;
    socket_stat->history = 0LL;
    socket_stat->network_stat_sample_index = 0;
}

/**
 * Setup necessary functionalities to synchronize clock with the RTI.
 * 
 * @return port number to be sent to the RTI
 *  If clock synchronization is off compeltely, USHRT_MAX is returned.
 *  If clock synchronization is set to initial, 0 is sent.
 *  If clock synchronization is set to on, a reserved UDP port number
 *   will be sent. 
 */
ushort setup_clock_synchronization_with_rti() {
    ushort port_to_return = USHRT_MAX;
#ifdef _LF_CLOCK_SYNC_ON
    // Initialize the UDP socket
    _lf_rti_socket_UDP = socket(AF_INET, SOCK_DGRAM, 0);
    // Initialize the necessary information for the UDP address
    struct sockaddr_in federate_UDP_addr;
    federate_UDP_addr.sin_family = AF_INET;
    federate_UDP_addr.sin_port = htons(0u); // Port 0 indicates to bind that
                                            // it can assign any port to this
                                            // socket. This is okay because
                                            // the port number is then sent
                                            // to the RTI.
    federate_UDP_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(
        _lf_rti_socket_UDP,
        (struct sockaddr *) &federate_UDP_addr,
        sizeof(federate_UDP_addr)) < 0) {
            error_print_and_exit("Failed to bind its UDP socket: %s.",
                                    strerror(errno));
    }
    // Retrieve the port number that was assigned by the operating system
    socklen_t addr_length = sizeof(federate_UDP_addr);
    if (getsockname(_lf_rti_socket_UDP, (struct sockaddr *)&federate_UDP_addr, &addr_length) == -1) {
        // FIXME: Send 0 UDP_PORT message instead of exiting.
        // That will disable clock synchronization.
        error_print_and_exit("Failed to retrieve UDP port: %s.",
                                strerror(errno));
    }
    DEBUG_PRINT("Assigned UDP port number %u to its socket.", ntohs(federate_UDP_addr.sin_port));

    port_to_return = ntohs(federate_UDP_addr.sin_port);

    // Set the option for this socket to reuse the same address 
    if (setsockopt(_lf_rti_socket_UDP, SOL_SOCKET, SO_REUSEADDR, &(int){1}, sizeof(int)) < 0) {
        error_print("Failed to set SO_REUSEADDR option on the socket: %s.", strerror(errno));
    }
    // Set the timeout on the UDP socket so that read and write operations don't block for too long
    struct timeval timeout_time = {.tv_sec = UDP_TIMEOUT_TIME / BILLION, .tv_usec = (UDP_TIMEOUT_TIME % BILLION) / 1000}; 
    if (setsockopt(_lf_rti_socket_UDP, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_time, sizeof(timeout_time)) < 0) {
        error_print("Failed to set SO_RCVTIMEO option on the socket: %s.", strerror(errno));
    }
    if (setsockopt(_lf_rti_socket_UDP, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout_time, sizeof(timeout_time)) < 0) {
        error_print("Failed to set SO_SNDTIMEO option on the socket: %s.", strerror(errno));
    }
#else // No runtime clock synchronization. Send port -1 or 0 instead.
#ifdef _LF_CLOCK_SYNC_INITIAL
    port_to_return = 0u;
#endif
#endif // _LF_CLOCK_SYNC_ON
    return port_to_return;
}

/**
 * Synchronize the initial physical clock with the RTI.
 * A call to this function is inserted into the startup
 * sequence by the code generator if initial clock synchronization
 * is required.
 *
 * This is a blocking function that expects
 * to read a PHYSICAL_CLOCK_SYNC_MESSAGE_T1 from the RTI TCP socket.
 * It will then follow the PTP protocol to synchronize the local
 * physical clock with the RTI.
 * Failing to complete this protocol is treated as a catastrophic
 * error that causes the federate to exit.
 * 
 * @param rti_socket_TCP The rti's socket
 */
void synchronize_initial_physical_clock_with_rti(int rti_socket_TCP) {
    DEBUG_PRINT("Waiting for initial clock synchronization messages from the RTI.");

    size_t message_size = 1 + sizeof(instant_t);
    unsigned char buffer[message_size];

    for (int i=0; i < _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL; i++) {
        // The first message expected from the RTI is PHYSICAL_CLOCK_SYNC_MESSAGE_T1
        read_from_socket_errexit(rti_socket_TCP, message_size, buffer,
                "Federate %d did not get the initial clock synchronization message T1 from the RTI.",
                _lf_my_fed_id);

        // Get local physical time before doing anything else.
        instant_t receive_time = get_physical_time();

        // Check that this is the T1 message.
        if (buffer[0] != PHYSICAL_CLOCK_SYNC_MESSAGE_T1) {
            error_print_and_exit("Initial clock sync: Expected T1 message from RTI. Got %x.", buffer[0]);
        }
        // Handle the message and send a reply T3 message.
        // NOTE: No need to acquire the mutex lock during initialization because only
        // one thread is running.
        if (handle_T1_clock_sync_message(buffer, rti_socket_TCP, receive_time) != 0) {
            error_print_and_exit("Initial clock sync: Failed to send T3 reply to RTI.");
        }

        // Next message from the RTI is required to be PHYSICAL_CLOCK_SYNC_MESSAGE_T4
        read_from_socket_errexit(rti_socket_TCP, message_size, buffer,
                "Federate %d did not get the clock synchronization message T4 from the RTI.",
                _lf_my_fed_id);

        // Check that this is the T4 message.
        if (buffer[0] != PHYSICAL_CLOCK_SYNC_MESSAGE_T4) {
            error_print_and_exit("Federate %d expected T4 message from RTI. Got %x.", _lf_my_fed_id, buffer[0]);
        }

        // Handle the message.
        handle_T4_clock_sync_message(buffer, rti_socket_TCP, receive_time);
    }

    LOG_PRINT("Finished initial clock synchronization with the RTI.");
}

/**
 * Handle a clock synchroninzation message T1 coming from the RTI.
 * T1 is the first message in a PTP exchange.
 * This replies to the RTI with a T3 message.
 * It also measures the time it takes between when the method is
 * called and the reply has been sent.
 * @param buffer The buffer containing the message, including the message type.
 * @param socket The socket (either _lf_rti_socket_TCP or _lf_rti_socket_UDP).
 * @param t2 The physical time at which the T1 message was received.
 * @return 0 if T3 reply is successfully sent, -1 otherwise.
 */
int handle_T1_clock_sync_message(unsigned char* buffer, int socket, instant_t t2) {
    // Extract the payload
    instant_t t1 = extract_ll(&(buffer[1]));

    DEBUG_PRINT("Received T1 message with time payload %lld from RTI at local time %lld.",
                t1, t2);

    // Store snapshots of remote (master) and local physical clock
    _lf_rti_socket_stat.remote_physical_clock_snapshot_T1 = t1;
    _lf_rti_socket_stat.local_physical_clock_snapshot_T2  = t2;
    // Send a message to the RTI and calculate the local delay
    // T3-T2 between receiving the T1 message and replying.

    // Reply will have the federate ID as a payload.
    unsigned char reply_buffer[1 + sizeof(int)];
    reply_buffer[0] = PHYSICAL_CLOCK_SYNC_MESSAGE_T3;
    encode_int(_lf_my_fed_id, &(reply_buffer[1]));

    // Write the reply to the socket.
    DEBUG_PRINT("Sending T3 message to RTI.");
    if (write_to_socket(socket, 1 + sizeof(int), reply_buffer) != 1 + sizeof(int)) {
        error_print("Clock sync: Failed to send T3 message to RTI.");
        return -1;
    }

    // Measure the time _after_ the write on the assumption that the read
    // from the socket, which occurs before this function is called, takes
    // about the same amount of time as the write of the reply.
    _lf_rti_socket_stat.local_delay = get_physical_time() - t2;
    return 0;
}

/**
 * Handle a clock synchronization message T4 coming from the RTI.
 * If the socket is _lf_rti_socket_TCP, then assume we are in the
 * initial clock synchronization phase and set the clock offset
 * based on the estimated clock synchronization error.
 * Otherwise, if the socket is _lf_rti_socket_UDP, then this looks also for a
 * subsequent "coded probe" message on the socket. If the delay between
 * the T4 and the coded probe message is not as expected, then reject
 * this clock synchronization round. If it is not rejected, then make
 * an adjustment to the clock offset based on the estimated error.
 * This function does not acquire the socket_mutex lock.
 * The caller should acquire it unless it is sure there is only one thread running.
 * @param buffer The buffer containing the message, including the message type.
 * @param socket The socket (either _lf_rti_socket_TCP or _lf_rti_socket_UDP).
 * @param r4 The physical time at which this T4 message was received.
 */
void handle_T4_clock_sync_message(unsigned char* buffer, int socket, instant_t r4) {
    // Increment the number of received T4 messages
    _lf_rti_socket_stat.received_T4_messages_in_current_sync_window++;

    // Extract the payload
    instant_t t4 = extract_ll(&(buffer[1]));

    DEBUG_PRINT("Clock sync: Received T4 message with time payload %lld from RTI at local time %lld. "
            "(difference %lld)",
            t4, r4, r4 - t4);

    // Calculate the round trip delay from T1 to T4:
    // (T4 - T1) - (T3 - T2)
    interval_t network_round_trip_delay = (t4
            - _lf_rti_socket_stat.remote_physical_clock_snapshot_T1)
            - _lf_rti_socket_stat.local_delay;
    

    // Estimate the clock synchronization error based on the assumption
    // that the channel delay is symmetric:
    // one_way_channel_delay - (T2 - T1).
    // This number is positive if the clock at the federate (T2) is
    // behind the clock at the RTI (T1).
    interval_t estimated_clock_error =
            network_round_trip_delay/2
            - (_lf_rti_socket_stat.local_physical_clock_snapshot_T2
            - _lf_rti_socket_stat.remote_physical_clock_snapshot_T1);
    DEBUG_PRINT("Clock sync: Estimated clock error: %lld.", estimated_clock_error);

    // The adjustment to the clock offset (to be calculated)
    interval_t adjustment = 0;
    // If the socket is _lf_rti_socket_UDP, then
    // after sending T4, the RTI sends a "coded probe" message,
    // which can be used to filter out noise.
    if (socket == _lf_rti_socket_UDP) {
        // Read the coded probe message.
        // We can reuse the same buffer.
        int bytes_read = read_from_socket(socket, 1 + sizeof(instant_t), buffer);

        instant_t r5 = get_physical_time();

        if (bytes_read < (int)(1 + sizeof(instant_t))
                || buffer[0] != PHYSICAL_CLOCK_SYNC_MESSAGE_T4_CODED_PROBE) {
            warning_print("Clock sync: Did not get the expected coded probe message from the RTI. "
                    "Skipping clock synchronization round.");
            return;
        }
        // Filter out noise.
        instant_t t5 = extract_ll(&(buffer[1]));  // Time at the RTI of sending the coded probe.

        // Compare the difference in time at the RTI between sending T4 and the coded probe
        // against the difference in time at this federate of receiving these two message.
        interval_t coded_probe_distance = llabs((r5 - r4) - (t5 - t4));

        DEBUG_PRINT("Clock sync: Received code probe that reveals a time discrepancy between "
                "messages of %lld.",
                coded_probe_distance);

        // Check against the guard band.
        if (coded_probe_distance >= CLOCK_SYNC_GUARD_BAND) {
            // Discard this clock sync cycle
            LOG_PRINT("Clock sync: Skipping the current clock synchronization cycle "
                    "due to impure coded probes.");
            LOG_PRINT("Clock sync: Coded probe packet stats: "
                    "Distance: %lld. r5 - r4 = %lld. t5 - t4 = %lld.",
                    coded_probe_distance,
                    r5 - r4,
                    t5 - t4);
            _lf_rti_socket_stat.received_T4_messages_in_current_sync_window--;
            return;
        }
        // Apply a jitter attenuator to the estimated clock error to prevent
        // large jumps in the underlying clock.
        // Note that estimated_clock_error is calculated using get_physical_time() which includes
        // the _lf_global_physical_clock_offset adjustment.
        adjustment = estimated_clock_error / _LF_CLOCK_SYNC_ATTENUATION;

        // FIXME: Adjust drift.
        // _lf_global_physical_clock_drift = ((r1 - t1) -
        //                                    (_lf_rti_socket_stat.local_physical_clock_snapshot_T2 -
        //                                    _lf_rti_socket_stat.remote_physical_clock_snapshot_T1)) /
        //                                    (t1 - _lf_rti_socket_stat.remote_physical_clock_snapshot_T1);
    } else {
        // Use of TCP socket means we are in the startup phase, so
        // rather than adjust the clock offset, we simply set it to the
        // estimated error.
        adjustment =  estimated_clock_error;
    }

#ifdef _LF_CLOCK_SYNC_COLLECT_STATS // Enabled by default 
    // Update RTI's socket stats
    update_socket_stat(&_lf_rti_socket_stat, network_round_trip_delay, estimated_clock_error);
#endif
    
    // FIXME: Enable alternative regression mechanism here.
    DEBUG_PRINT("Clock sync: Adjusting clock offset running average by %lld.",
            adjustment/_LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL);
    // Calculate the running average
    _lf_rti_socket_stat.history += adjustment/_LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL;
    
    if (_lf_rti_socket_stat.received_T4_messages_in_current_sync_window >=
            _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL) {
        
        lf_stat_ll stats = {0, 0, 0, 0};
#ifdef _LF_CLOCK_SYNC_COLLECT_STATS // Enabled by default 
        stats = calculate_socket_stat(&_lf_rti_socket_stat);
        // Issue a warning if standard deviation is high in data
        if (stats.standard_deviation >= CLOCK_SYNC_GUARD_BAND) {
            // Reset the stats
            LOG_PRINT("Clock sync: Large standard deviation detected in network delays (%lld) for the current period."
                        " Clock synchronization offset might not be accurate.",
                        stats.standard_deviation);
            reset_socket_stat(&_lf_rti_socket_stat);
            return;
        }
#endif
        // The number of received T4 messages has reached _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL
        // which means we can now adjust the clock offset.
        // For the AVG algorithm, history is a running average and can be directly
        // applied                                                 
        _lf_global_physical_clock_offset += _lf_rti_socket_stat.history;
        // @note AVG and SD will be zero if collect-stats is set to false
        LOG_PRINT("Clock sync:"
                    " New offset: %lld."
                    " Round trip delay to RTI (now): %lld."
                    " (AVG): %lld."
                    " (SD): %lld."
                    " Local round trip delay: %lld."
                    " Test offset: %lld.",
                    _lf_global_physical_clock_offset,
                    network_round_trip_delay,
                    stats.average,
                    stats.standard_deviation,
                    _lf_rti_socket_stat.local_delay,
                    _lf_global_test_physical_clock_offset);
        // Reset the stats
        reset_socket_stat(&_lf_rti_socket_stat);
        // Set the last instant at which the clocks were synchronized
        _lf_last_clock_sync_instant = r4;
    }
}

/** 
 * Thread that listens for UDP inputs from the RTI.
 */
void* listen_to_rti_UDP_thread(void* args) {
    // Listen for UDP messages from the RTI.
    // The only expected messages are T1 and T4, which have
    // a payload of a time value.
    int message_size = 1 + sizeof(instant_t);
    unsigned char buffer[message_size];
    // This thread will be either waiting for T1 or waiting
    // for T4. Track the mode with this variable:
    bool waiting_for_T1 = true;
    // Even though UDP messages are connectionless, we need to call connect()
    // at least once to record the address of the RTI's UDP port. The RTI
    // uses bind() to reserve that address, so recording it once is sufficient.
    bool connected = false;
    while (1) {
        struct sockaddr_in RTI_UDP_addr;
        socklen_t RTI_UDP_addr_length = sizeof(RTI_UDP_addr);
        int bytes_read = 0;
        // Read from the UDP socket
        do {
            int bytes = recvfrom(_lf_rti_socket_UDP,                    // The UDP socket
                                    &buffer[bytes_read],                // The buffer to read into
                                    message_size - bytes_read,          // Number of bytes to read
                                    MSG_WAITALL,                        // Read the entire datagram
                                    (struct sockaddr*)&RTI_UDP_addr,    // Record the RTI's address
                                    &RTI_UDP_addr_length);              // The RTI's address length
            // Try reading again if errno indicates the need to try again and there are more
            // bytes to read.
            if (bytes > 0) {
                bytes_read += bytes;
            }
        } while ((errno == EAGAIN || errno == EWOULDBLOCK) && bytes_read < message_size);

        // Get local physical time before doing anything else.
        instant_t receive_time = get_physical_time();

        if (bytes_read < message_size) {
            // Either the socket has closed or the RTI has sent EOF.
            // Exit the thread to halt clock synchronization.
            error_print("Clock sync: UDP socket to RTI is broken: %s. Clock sync is now disabled.",
                    strerror(errno));
            break;
        }
        DEBUG_PRINT("Clock sync: Received UDP message %u from RTI on port %u.",
                buffer[0], ntohs(RTI_UDP_addr.sin_port));

        // Handle the message
        if (waiting_for_T1) {
            if (buffer[0] == PHYSICAL_CLOCK_SYNC_MESSAGE_T1) {
                waiting_for_T1 = false;
                // The reply (or return) address is given in RTI_UDP_addr.
                // We utilize the connect() function to set the default address
                // of the _lf_rti_socket_UDP socket to RTI_UDP_addr. This is convenient
                // because subsequent calls to write_to_socket do not need this address.
                // Note that this only needs to be done for handle_T1_clock_sync_message()
                // because it is the only function that needs to reply to the RTI.
                if (!connected
                        && connect(_lf_rti_socket_UDP,
                        (struct sockaddr*)&RTI_UDP_addr,
                        RTI_UDP_addr_length) < 0) {
                    error_print("Clock sync: Federate %d failed to register RTI's UDP reply address. "
                            "Clock synchronization has stopped.",
                            _lf_my_fed_id);
                    break;
                }
                connected = true;
                if (handle_T1_clock_sync_message(buffer, _lf_rti_socket_UDP, receive_time) != 0) {
                    // Failed to send T3 reply. Wait for the next T1.
                    waiting_for_T1 = true;
                    continue;
                }
            } else {
                // Waiting for a T1 message, but received something else. Discard message.
                warning_print("Clock sync: Received %u message from RTI, but waiting for %u (T1). "
                        "Discarding the message.",
                        buffer[0],
                        PHYSICAL_CLOCK_SYNC_MESSAGE_T1);
                continue;
            }
        } else if (buffer[0] == PHYSICAL_CLOCK_SYNC_MESSAGE_T4) {
            handle_T4_clock_sync_message(buffer, _lf_rti_socket_UDP, receive_time);
            waiting_for_T1 = true;
        } else {
            warning_print("Clock sync: Received from RTI an unexpected UDP message type: %u. "
                    "Discarding the message and skipping this round.",
                    buffer[0]);
            // Ignore further clock sync messages until we get a T1.
            waiting_for_T1 = true;
        }
    }
    return NULL;
}

/**
 * Create the thread responsible for handling clock synchronization
 * with the RTI if (runtime) clock synchronization is on.
 * Otherwise, do nothing an return 0.
 * 
 * @return On success, returns 0; On error, it returns an error number.
 */
int create_clock_sync_thread(lf_thread_t* thread_id) {
#ifdef _LF_CLOCK_SYNC_ON
    // One for UDP messages if clock synchronization is enabled for this federate
    return lf_thread_create(thread_id, listen_to_rti_UDP_thread, NULL);
#endif // _LF_CLOCK_SYNC_ON
    return 0;
}
