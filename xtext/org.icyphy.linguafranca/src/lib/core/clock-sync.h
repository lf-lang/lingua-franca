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

#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

/**
 * Number of required clock sync T4 messages per synchronization
 * interval. The offset to the clock will not be adjusted until 
 * this number of T4 clock synchronization messages have been received.
 */
#ifndef _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL
#define _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL 10
#endif

/** Runtime clock offset updates will be divided by this number. */
#ifndef _LF_CLOCK_SYNC_ATTENUATION
#define _LF_CLOCK_SYNC_ATTENUATION 10
#endif

/**
 * Define a guard band to filter clock synchronization
 * messages based on discrepancies in the network delay.
 * @see Coded probes in Geng, Yilong, et al.
 * "Exploiting a natural network effect for scalable, fine-grained clock synchronization."
 */
#define CLOCK_SYNC_GUARD_BAND USEC(100)

/**
 * Statistics for a given socket.
 * The RTI initiates a clock synchronization action by sending its
 * current physical time T1 to a federate.  The federate records
 * the local time T2 that it receives T1. It sends a reply at
 * local time T3, which the RTI receives at its time T4. The RTI
 * sends back T4.  The round trip delay on the socket is therefore
 * estimated as:  (T4 - T1) - (T3 - T2).
 */
typedef struct socket_stat_t {
    instant_t remote_physical_clock_snapshot_T1;  // T1 in PTP. The first snapshot of the physical
                                                  // clock of the remote device (the RTI).
    instant_t local_physical_clock_snapshot_T2;   // T2 in PTP. The first snapshot of the physical
                                                  // clock of the local device (the federate).
    interval_t local_delay;                       // T3 - T2. Estimated delay between a consecutive
                                                  // receive and send on the socket for one byte.
    int received_T4_messages_in_current_sync_window; // Checked against _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL
                                                     // Must be reset to 0 every time it reaches the threshold. 
    interval_t history;                              // A history of clock synchronization data. For AVG
                                                     // strategy, this is a running partially computed average.
    
    /***** The following stats can be used to calculate an automated STP offset **************/
    /** FIXME: TODO: A federate should create a socket_stat_t for every federate it is connected to and keep record
                     of the following stats **/
    /*** Network stats ****/
    interval_t network_stat_round_trip_delay_max; // Maximum estimated delay between the local socket and the
                                                  // remote socket.
    int network_stat_sample_index;                // Current index of network_stat_samples
    /*** Clock sync stats ***/
    interval_t clock_synchronization_error_bound; // A bound on the differences between this federate's clock and
                                                  // the remote clock.
    // Note: The following array should come last because g++ will not allow 
    // designated initialization (e.g., .network_stat_sample_index = 0) out of 
    // order and we do not want to (and cannot) initialize this array statically
    interval_t network_stat_samples[_LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL];   // Samples gathered during a clock sync 
                                                                              // period
} socket_stat_t;


#ifdef _LF_CLOCK_SYNC_COLLECT_STATS
/**
 * To hold statistics
 */
struct lf_stat_ll;

/**
 * Update statistics on the socket based on the newly calculated network delay 
 * and clock synchronization error
 * 
 * @param socket_stat The socket_stat_t struct that  keeps track of stats for a given connection
 * @param network_round_trip_delay The newly calculated round trip delay to the remote federate/RTI
 * @param clock_synchronization_error The newly calculated clock synchronization error relative to
 *  the remote federate/RTI
 */
void update_socket_stat(struct socket_stat_t* socket_stat, long long network_delay, long long clock_synchronization_error);

/**
 * Calculate statistics of the socket.
 * The releavent information is returned as a lf_stat_ll struct.
 * 
 * @param socket_stat The socket_stat_t struct that  keeps track of stats for a given connection
 * @return An lf_stat_ll struct with relevant information.
 */
struct lf_stat_ll calculate_socket_stat(struct socket_stat_t* socket_stat);
#endif

/**
 * Reset statistics on the socket.
 *  @param socket_stat The socket_stat_t struct that  keeps track of stats for a given connection
 */
void reset_socket_stat(struct socket_stat_t* socket_stat);

/**
 * Setup necessary functionalities to synchronize clock with the RTI.
 * 
 * @return port number to be sent to the RTI
 */
unsigned short int setup_clock_synchronization_with_rti();

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
void synchronize_initial_physical_clock_with_rti(int rti_socket_TCP);

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
int handle_T1_clock_sync_message(unsigned char* buffer, int socket, instant_t t2);

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
void handle_T4_clock_sync_message(unsigned char* buffer, int socket, instant_t r4);

/** 
 * Thread that listens for UDP inputs from the RTI.
 */
void* listen_to_rti_UDP_thread(void* args);

/**
 * Create the thread responsible for handling clock synchronization
 * with the RTI if (runtime) clock synchronization is on.
 * Otherwise, do nothing an return 0.
 *
 * @return On success, returns 0; On error, it returns an error number.
 */
int create_clock_sync_thread(lf_thread_t* thread_id);

#endif // CLOCK_SYNC_H
