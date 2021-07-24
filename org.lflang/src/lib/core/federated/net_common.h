/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
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
/**
 * @section DESCRIPTION
 * Header file for common message types and definitions for federated Lingua Franca programs.
 *
 * This file defines the message types for the federate to communicate with the RTI.
 * Each message type has a unique one-byte ID.
 *
 * The startup sequence is as follows:
 *
 * Each federate attempts to connect with an RTI at the IP address
 * put into its code by the code generator (i.e., it attempts to
 * open a TCP connection). It starts by trying the
 * port number given by STARTING_PORT and increments the port number
 * from there until it successfully connects. The maximum port number
 * it will try before giving up is STARTING_PORT + PORT_RANGE_LIMIT.
 *
 * FIXME: What if a port is specified in the "at" of the federated statement?
 *
 * When it has successfully opened a TCP connection, the first message it sends
 * to the RTI is a MSG_TYPE_FED_IDS message, which contains the ID of this federate
 * within the federation, contained in the global variable _lf_my_fed_id
 * in the federate code
 * (which is initialized by the code generator) and the unique ID of
 * the federation, a GUID that is created at run time by the generated script
 * that launches the federation.
 * If you launch the federates and the RTI manually, rather than using the script,
 * then the federation ID is a string that is optionally given to the federate
 * on the command line when it is launched. The federate will connect
 * successfully only to an RTI that is given the same federation ID on
 * its command line. If no ID is given on the command line, then the
 * default ID "Unidentified Federation" will be used.
 *
 * The RTI will respond with a MSG_TYPE_REJECT message if the federation IDs
 * do not match and close the connection. At this point the federate
 * will increment the port number and try again to find an RTI that matches.
 *
 * When the federation IDs match, the RTI will respond with an
 * MSG_TYPE_ACK.
 * 
 * The next message to the RTI will be a MSG_TYPE_NEIGHBOR_STRUCTURE message
 * that informs the RTI about connections between this federate and other
 * federates where messages are routed through the RTI. Currently, this only
 * includes logical connections when the coordination is centralized. This
 * information is needed for the RTI to perform the centralized coordination.
 * The burden is on the federates to inform the RTI about relevant connections.
 *
 * The next message to the RTI will be a MSG_TYPE_UDP_PORT message, which has
 * payload USHRT_MAX if clock synchronization is disabled altogether, 0 if
 * only initial clock synchronization is enabled, and a port number for
 * UDP communication if runtime clock synchronization is enabled.
 * By default, if the federate host is identical to that of the RTI
 * (either no "at" clause is given for either or they both have exactly
 * the same string), then clock synchronization is disabled.
 * Otherwise, the default is that initial clock synchronization is enabled.
 * To turn turn off clock synchronization altogether, set the clock-sync
 * property of the target to off. To turn on runtime clock synchronization,
 * set it to on. The default value is initial.
 *
 * If initial clock sync is enabled, the next step is to perform the initial
 * clock synchronization (using the TCP connection), which attempts
 * to find an initial offset to the physical clock of the federate to make it
 * better match the physical clock at the RTI.
 *
 * Clock synchronization is initiated by the RTI by sending a message
 * of type MSG_TYPE_CLOCK_SYNC_T1, the payload of which is the
 * current physical clock reading at the RTI. The federate records
 * the physical time when it receives this message (T2) and sends
 * a reply message of type MSG_TYPE_CLOCK_SYNC_T3 to the RTI.
 * It records the time (T3) at which this message has gone out.
 * The payload of the MSG_TYPE_CLOCK_SYNC_T3 message is the
 * federate ID.  The RTI responds to the T3 message with a message
 * of type MSG_TYPE_CLOCK_SYNC_T4, which has as a payload
 * the physical time at which that response was sent. This cycle will happen
 * _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL times at startup to account for network delay variations
 * (see below).
 *
 * The times T1 and T4 are taken from the physical clock at the RTI,
 * whereas the times T2 and T3 are taken from the physical clock at
 * the federate.  The round trip latency on the connection to the RTI
 * is therefore measured as (T4 - T1) - (T3 - T2). Half this quantity
 * is an estimate L of the one-way latency.  The estimated clock error
 * E is therefore L - (T2 - T1). Over several cycles, the average value of E
 * becomes the initial offset for the
 * clock at the federate. Henceforth, when get_physical_time() is
 * called, the offset will be added to whatever the physical clock says.
 *
 * If clock synchronization is enabled, then the federate will also
 * start a thread to listen for incoming UDP messages from the RTI.
 * With period given by the `-c on period <n>` command-line argument, the RTI
 * will initiate a clock synchronization round by sending to the
 * federate a MSG_TYPE_CLOCK_SYNC_T1 message. A similar
 * protocol to that above is followed to estimate the average clock
 * synchronization error E, with two exceptions. First, a fraction
 * of E (given by _LF_CLOCK_SYNC_ATTENUATION) is used to adjust the
 * offset up or down rather than just setting the offset equal to E.
 * Second, after MSG_TYPE_CLOCK_SYNC_T4, the RTI immediately
 * sends a following message of type MSG_TYPE_CLOCK_SYNC_CODED_PROBE.
 * The federate measures the time difference between its receipt of
 * T4 and this code probe and compares that time difference against
 * the time difference at the RTI (the difference between the two
 * payloads). If that difference is larger than CLOCK_SYNC_GUARD_BAND
 * in magnitude, then the clock synchronization round is skipped
 * and no adjustment is made. The round will also be skipped if
 * any of the expected UDP messages fails to arrive.
 *
 * FIXME: Citation needed here.
 *
 * The next step depends on the coordination mode. If the coordination
 * parameter of the target is "decentralized" and the federate has
 * inbound connections from other federates, then it starts a socket
 * server to listen for incoming connections from those federates.
 * It attempts to create the server at the port given by STARTING_PORT,
 * and if this fails, increments the port number from there until a
 * port is available. It then sends to the RTI an MSG_TYPE_ADDRESS_ADVERTISEMENT message
 * with the port number as a payload. The federate then creates a thread
 * to listen for incoming socket connections and messages.
 *
 * If the federate has outbound connections to other federates, then it
 * establishes a socket connection to those federates.  It does this by
 * first sending to the RTI an MSG_TYPE_ADDRESS_QUERY message with the payload
 * being the ID of the federate it wishes to connect to. If the RTI
 * responds with a -1, then the RTI does not (yet) know the remote federate's
 * port number and IP address, so the local federate will try again
 * after waiting ADDRESS_QUERY_RETRY_INTERVAL. When it gets a valid port
 * number and IP address in reply, it will establish a socket connection
 * to that remote federate.
 *
 * Physical connections also use the above P2P sockets between
 * federates even if the coordination is centralized.
 *
 * Peer-to-peer sockets can be closed by the downstream federate.
 * For example, when a downstream federate reaches its stop time, then
 * it will stop accepting physical messages. To achieve an orderly shutdown,
 * the downstream federate sends a MSG_TYPE_CLOSE_REQUEST message to the upstream
 * one and the upstream federate handles closing the socket. This way, any
 * messages that are in the middle of being sent while the downstream
 * federate shuts down will successfully traverse the socket, even if
 * only to be ignored by the downstream federate.  It is valid to ignore
 * such messages if the connection is physical or if the coordination is
 * decentralized and the messages arrive after the STP offset of the
 * downstream federate (i.e., they are "tardy").
 *
 * FIXME: What happens after this?
 *
 */

#ifndef NET_COMMON_H
#define NET_COMMON_H

#include <pthread.h>

/**
 * The timeout time in ns for TCP operations.
 * Default value is 10 secs.
 */
#define TCP_TIMEOUT_TIME SEC(10)

/**
 * The timeout time in ns for UDP operations.
 * Default value is 1 sec.
 */
#define UDP_TIMEOUT_TIME SEC(1)


/**
 * Size of the buffer used for messages sent between federates.
 * This is used by both the federates and the rti, so message lengths
 * should generally match.
 */
#define FED_COM_BUFFER_SIZE 256u

/**
 * Number of seconds that elapse between a federate's attempts
 * to connect to the RTI.
 */
#define CONNECT_RETRY_INTERVAL 2

/**
 * Bound on the number of retries to connect to the RTI.
 * A federate will retry every CONNECT_RETRY_INTERVAL seconds
 * this many times before giving up. E.g., 500 retries every
 * 2 seconds results in retrying for about 16 minutes.
 */
#define CONNECT_NUM_RETRIES 500

/**
 * Number of nanoseconds that a federate waits before asking
 * the RTI again for the port and IP address of a federate
 * (an MSG_TYPE_ADDRESS_QUERY message) after the RTI responds that it
 * does not know.
 */
#define ADDRESS_QUERY_RETRY_INTERVAL 100000000LL

/**
 * Number of nanoseconds that a federate waits before trying
 * another port for the RTI. This is to avoid overwhelming
 * the OS and the socket with too many calls.
 * FIXME: Is this too small?
 */
#define PORT_KNOCKING_RETRY_INTERVAL 10000LL

/**
 * Default starting port number for the RTI and federates' socket server.
 * Unless a specific port has been specified by the LF program in the "at"
 * for the RTI, when the federates start up, they will attempt
 * to open a socket server
 * on this port, and, if this fails, increment the port number and
 * try again. The number of increments is limited by PORT_RANGE_LIMIT.
 * FIXME: Clarify what happens if a specific port has been given in "at".
 */
#define STARTING_PORT 15045u

/**
 * Number of ports to try to connect to. Unless the LF program specifies
 * a specific port number to use, the RTI or federates will attempt to start
 * a socket server on port STARTING_PORT. If that port is not available (e.g.,
 * another RTI is running or has recently exited), then it will try the
 * next port, STARTING_PORT+1, and keep incrementing the port number up to this
 * limit. If no port between STARTING_PORT and STARTING_PORT + PORT_RANGE_LIMIT
 * is available, then the RTI or the federate will fail to start. This number, therefore,
 * limits the number of RTIs and federates that can be simultaneously
 * running on any given machine without assigning specific port numbers.
 */
#define PORT_RANGE_LIMIT 1024

/**
 * Delay the start of all federates by this amount.
 * FIXME: More.
 * FIXME: Should use the latency estimates that were
 * acquired during initial clock synchronization.
 */
#define DELAY_START SEC(1)

////////////////////////////////////////////
//// Message types

// These message types will be encoded in an unsigned char,
// so the magnitude must not exceed 255. Note that these are
// listed in increasing numerical order starting from 0 interleaved
// with decreasing numerical order starting from 255 (so that they
// can be listed in a logical order here even as the design evolves).

/**
 * Byte identifying a rejection of the previously received message.
 * The reason for the rejection is included as an additional byte
 * (uchar) (see below for encodings of rejection reasons).
 */
#define MSG_TYPE_REJECT 0

/**
 * Byte identifying an acknowledgment of the previously received message.
 * This message carries no payload.
 */
#define MSG_TYPE_ACK 255

/**
 * Byte identifying an acknowledgment of the previously received MSG_TYPE_FED_IDS message
 * sent by the RTI to the federate
 * with a payload indicating the UDP port to use for clock synchronization.
 * The next four bytes will be the port number for the UDP server, or
 * 0 or USHRT_MAX if there is no UDP server.  0 means that initial clock synchronization
 * is enabled, whereas USHRT_MAX mean that no synchronization should be performed at all.
 */
#define MSG_TYPE_UDP_PORT 254

/** Byte identifying a message from a federate to an RTI containing
 *  the federation ID and the federate ID. The message contains, in
 *  this order:
 *  * One byte equal to MSG_TYPE_FED_IDS.
 *  * Two bytes (ushort) giving the federate ID.
 *  * One byte (uchar) giving the length N of the federation ID.
 *  * N bytes containing the federation ID.
 *  Each federate needs to have a unique ID between 0 and
 *  NUMBER_OF_FEDERATES-1.
 *  Each federate, when starting up, should send this message
 *  to the RTI. This is its first message to the RTI.
 *  The RTI will respond with either MSG_TYPE_REJECT, MSG_TYPE_ACK, or MSG_TYPE_UDP_PORT.
 *  If the federate is a C target LF program, the generated federate
 *  code does this by calling synchronize_with_other_federates(),
 *  passing to it its federate ID.
 */
#define MSG_TYPE_FED_IDS 1

/**
 * Byte identifying a timestamp message, which is 64 bits long.
 * Each federate sends its starting physical time as a message of this
 * type, and the RTI broadcasts to all the federates the starting logical
 * time as a message of this type.
 s*/
#define MSG_TYPE_TIMESTAMP 2
#define MSG_TYPE_TIMESTAMP_LENGTH (1 + sizeof(int64_t))

/** Byte identifying a message to forward to another federate.
 *  The next two bytes will be the ID of the destination port.
 *  The next two bytes are the destination federate ID.
 *  The four bytes after that will be the length of the message.
 *  The remaining bytes are the message.
 *  NOTE: This is currently not used. All messages are tagged, even
 *  on physical connections, because if "after" is used, the message
 *  may preserve the logical timestamp rather than using the physical time.
 */
#define MSG_TYPE_MESSAGE 3

/** Byte identifying that the federate is ending its execution. */
#define MSG_TYPE_RESIGN 4

/** Byte identifying a timestamped message to forward to another federate.
 *  The next two bytes will be the ID of the destination reactor port.
 *  The next two bytes are the destination federate ID.
 *  The four bytes after that will be the length of the message.
 *  The next eight bytes will be the timestamp of the message.
 *  The next four bytes will be the microstep of the message.
 *  The remaining bytes are the message.
 *
 *  With centralized coordination, all such messages flow through the RTI.
 *  With decentralized coordination, tagged messages are sent peer-to-peer
 *  between federates and are marked with MSG_TYPE_P2P_TAGGED_MESSAGE.
 */
#define MSG_TYPE_TAGGED_MESSAGE 5

/** 
 * Byte identifying a next event tag (NET) message sent from a federate
 * in centralized coordination.
 * The next eight bytes will be the timestamp.
 * The next four bytes will be the microstep.
 * This message from a federate tells the RTI the tag of the earliest event 
 * on that federate's event queue. In other words, absent any further inputs 
 * from other federates, this will be the least tag of the next set of
 * reactions on that federate. If the event queue is empty and a timeout
 * time has been specified, then the timeout time will be sent. If there is
 * no timeout time, then FOREVER will be sent. Note that this message should
 * not be sent if there are physical actions and the earliest event on the event
 * queue has a tag that is ahead of physical time (or the queue is empty).
 * In that case, send TAN instead.
 */
#define MSG_TYPE_NEXT_EVENT_TAG 6

/**
 * Byte identifying a time advance notice (TAN) message sent from
 * a federate in centralized coordination.  This message is used by
 * a federate that has outputs that are directly or indirectly
 * triggered by a physical action to notify the RTI that its physical
 * time has advanced and that it will produce no outputs with
 * timestamps less than the specified timestamp.
 * The next eight bytes will be the timestamp.
 */
#define MSG_TYPE_TIME_ADVANCE_NOTICE 253

/** 
 * Byte identifying a time advance grant (TAG) sent by the RTI to a federate
 * in centralized coordination. This message is a promise by the RTI to the federate
 * that no later message sent to the federate will have a tag earlier than or
 * equal to the tag carried by this TAG message.
 * The next eight bytes will be the timestamp.
 * The next four bytes will be the microstep.
 */
#define MSG_TYPE_TAG_ADVANCE_GRANT 7

/** 
 * Byte identifying a provisional time advance grant (PTAG) sent by the RTI to a federate
 * in centralized coordination. This message is a promise by the RTI to the federate
 * that no later message sent to the federate will have a tag earlier than the tag
 * carried by this PTAG message.
 * The next eight bytes will be the timestamp.
 * The next four bytes will be the microstep.
 */
#define MSG_TYPE_PROVISIONAL_TAG_ADVANCE_GRANT 8

/** 
 * Byte identifying a logical tag complete (LTC) message sent by a federate
 * to the RTI.
 * The next eight bytes will be the timestep of the completed tag.
 * The next four bytes will be the microsteps of the completed tag.
 */
#define MSG_TYPE_LOGICAL_TAG_COMPLETE 9

/////////// Messages used in request_stop() ///////////////
//// Overview of the algorithm:
////  When any federate calls request_stop(), it will
////  send a MSG_TYPE_STOP_REQUEST message to the RTI, which will then
////  forward a MSG_TYPE_STOP_REQUEST message
////  to any federate that has not yet provided a stop time to the RTI. The federates will reply
////  with a MSG_TYPE_STOP_REQUEST_REPLY and a stop tag (which shall be the
////  maximum of their current logical tag
////  at the time they receive the MSG_TYPE_STOP_REQUEST and the tag of the stop
////  request). When the RTI has gathered all the stop tags
////  from federates (that are still connected), it will decide on a common stop tag
////  which is the maximum of the seen stop tag and answer with a MSG_TYPE_STOP_GRANTED. The federate
////  sending the MSG_TYPE_STOP_REQUEST and federates sending the MSG_TYPE_STOP_REQUEST_REPLY will freeze
////  the advancement of tag until they receive the MSG_TYPE_STOP_GRANTED message, in which
////  case they might continue their execution until the stop tag has been reached.

/**
 * Byte identifying a stop request. This message is first sent to the RTI by a federate
 * that would like to stop execution at the specified tag. The RTI will forward
 * the MSG_TYPE_STOP_REQUEST to all other federates. Those federates will either agree to
 * the requested tag or propose a larger tag. The RTI will collect all proposed
 * tags and broadcast the largest of those to all federates. All federates
 * will then be expected to stop at the granted tag.
 *
 * The next 8 bytes will be the timestamp.
 * The next 4 bytes will be the microstep.
 *
 * NOTE: The RTI may reply with a larger tag than the one specified in this message.
 * It has to be that way because if any federate can send a MSG_TYPE_STOP_REQUEST message
 * that specifies the stop time on all other federates, then every federate
 * depends on every other federate and time cannot be advanced.
 * Hence, the actual stop time may be nondeterministic.
 * 
 * If, on the other hand, the federate requesting the stop is upstream of every
 * other federate, then it should be possible to respect its requested stop tag.
 */
#define MSG_TYPE_STOP_REQUEST 10
#define MSG_TYPE_STOP_REQUEST_LENGTH (1 + sizeof(instant_t) + sizeof(microstep_t))
#define ENCODE_STOP_REQUEST(buffer, time, microstep) do { \
    buffer[0] = MSG_TYPE_STOP_REQUEST; \
    encode_int64(time, &(buffer[1])); \
    assert(microstep >= 0); \
    encode_int32((int32_t)microstep, &(buffer[1 + sizeof(instant_t)])); \
} while(0)

/**
 * Byte indicating a federate's reply to a MSG_TYPE_STOP_REQUEST that was sent
 * by the RTI. The payload is a proposed stop tag that is at least as large
 * as the one sent to the federate in a MSG_TYPE_STOP_REQUEST message.
 *
 * The next 8 bytes will be the timestamp.
 * The next 4 bytes will be the microstep.
 */
#define MSG_TYPE_STOP_REQUEST_REPLY 11
#define MSG_TYPE_STOP_REQUEST_REPLY_LENGTH (1 + sizeof(instant_t) + sizeof(microstep_t))
#define ENCODE_STOP_REQUEST_REPLY(buffer, time, microstep) do { \
    buffer[0] = MSG_TYPE_STOP_REQUEST_REPLY; \
    encode_int64(time, &(buffer[1])); \
    assert(microstep >= 0); \
    encode_int32((int32_t)microstep, &(buffer[1 + sizeof(instant_t)])); \
} while(0)

/**
 * Byte sent by the RTI indicating that the stop request from some federate
 * has been granted. The payload is the tag at which all federates have
 * agreed that they can stop.
 * The next 8 bytes will be the time at which the federates will stop. * 
 * The next 4 bytes will be the microstep at which the federates will stop..
 */
#define MSG_TYPE_STOP_GRANTED 12
#define MSG_TYPE_STOP_GRANTED_LENGTH (1 + sizeof(instant_t) + sizeof(microstep_t))
#define ENCODE_STOP_GRANTED(buffer, time, microstep) do { \
    buffer[0] = MSG_TYPE_STOP_GRANTED; \
    encode_int64(time, &(buffer[1])); \
    assert(microstep >= 0); \
    encode_int32((int32_t)microstep, &(buffer[1 + sizeof(instant_t)])); \
} while(0)

/////////// End of request_stop() messages ////////////////

/**
 * Byte identifying a address query message, sent by a federate to RTI
 * to ask for another federate's address and port number.
 * The next two bytes are the other federate's ID.
 * The reply from the RTI will a port number (an int32_t), which is -1
 * if the RTI does not know yet (it has not received MSG_TYPE_ADDRESS_ADVERTISEMENT from
 * the other federate), followed by the IP address of the other
 * federate (an IPV4 address, which has length INET_ADDRSTRLEN).
 */
#define MSG_TYPE_ADDRESS_QUERY 13

/**
 * Byte identifying a message advertising the port for the TCP connection server
 * of a federate. This is utilized in decentralized coordination as well as for physical
 * connections in centralized coordination.
 * The next four bytes (or sizeof(int32_t)) will be the port number.
 * The sending federate will not wait for a response from the RTI and assumes its
 * request will be processed eventually by the RTI.
 */
#define MSG_TYPE_ADDRESS_ADVERTISEMENT 14

/**
 * Byte identifying a first message that is sent by a federate directly to another federate
 * after establishing a socket connection to send messages directly to the federate. This
 * first message contains two bytes identifying the sending federate (its ID), a byte
 * giving the length of the federation ID, followed by the federation ID (a string).
 * The response from the remote federate is expected to be MSG_TYPE_ACK, but if the remote
 * federate does not expect this federate or federation to connect, it will respond
 * instead with MSG_TYPE_REJECT.
 */
#define MSG_TYPE_P2P_SENDING_FED_ID 15

/**
 * Byte identifying a message to send directly to another federate.
 * 
 * The next two bytes will be the ID of the destination port.
 * The next two bytes are the destination federate ID. This is checked against
 * the _lf_my_fed_id of the receiving federate to ensure the message was intended for
 * The four bytes after will be the length of the message.
 * The ramaining bytes are the message.
 */
#define MSG_TYPE_P2P_MESSAGE 16

/**
 * Byte identifying a timestamped message to send directly to another federate.
 * This is a variant of @see MSG_TYPE_TAGGED_MESSAGE that is used in P2P connections between
 * federates. Having a separate message type for P2P connections between federates
 * will be useful in preventing crosstalk.
 * 
 * The next two bytes will be the ID of the destination port.
 * The next two bytes are the destination federate ID. This is checked against
 * the _lf_my_fed_id of the receiving federate to ensure the message was intended for
 * the correct federate.
 * The four bytes after will be the length of the message.
 * The next eight bytes will be the timestamp.
 * The next four bytes will be the microstep of the sender.
 * The ramaining bytes are the message.
 */
#define MSG_TYPE_P2P_TAGGED_MESSAGE 17

/**
 * Byte identifying a message that a downstream federate sends to its
 * upstream counterpart to request that the socket connection be closed.
 * This is the only message that should flow upstream on such socket
 * connections.
 */
#define MSG_TYPE_CLOSE_REQUEST 18

////////////////////////////////////////////////
/**
 * Physical clock synchronization messages according to PTP.
 */

/*
 * The next 8 bytes will be a timestamp sent according to
 * PTP.
 */
#define MSG_TYPE_CLOCK_SYNC_T1 19

/*
 * Prompts the master to send a T4.
 * The next four bytes will be the sendin federate's id
 */
#define MSG_TYPE_CLOCK_SYNC_T3 20

/*
 * The next 8 bytes will be a timestamp sent according to
 * PTP.
 */
#define MSG_TYPE_CLOCK_SYNC_T4 21

/**
 * Coded probe message.
 * This messages is sent by the server (master)
 * right after MSG_TYPE_CLOCK_SYNC_T4(t1) with a new physical clock snapshot t2.
 * At the receiver, the previous MSG_TYPE_CLOCK_SYNC_T4 message and this message
 * are assigned a receive timestamp r1 and r2. If |(r2 - r1) - (t2 - t1)| < GUARD_BAND,
 * then the current clock sync cycle is considered pure and can be processed.
 * @see Geng, Yilong, et al.
 * "Exploiting a natural network effect for scalable, fine-grained clock synchronization."
 */
#define MSG_TYPE_CLOCK_SYNC_CODED_PROBE 22


/**
 * A port absent message, informing the receiver that a given port
 * will not have event for the current logical time.
 * 
 * The next 2 bytes is the port id.
 * The next 2 bytes will be the federate id of the destination federate.
 *  This is needed for the centralized coordination so that the RTI knows where
 *  to forward the message.
 * The next 8 bytes are the intended time of the absent message
 * The next 4 bytes are the intended microstep of the absent message
 */
#define MSG_TYPE_PORT_ABSENT 23



/**
 * A message that informs the RTI about connections between this federate and
 * other federates where messages are routed through the RTI. Currently, this
 * only includes logical connections when the coordination is centralized. This
 * information is needed for the RTI to perform the centralized coordination.
 * 
 * @note Only information about the immediate neighbors is required. The RTI can
 * transitively obtain the structure of the federation based on each federate's
 * immediate neighbor information.
 *
 * The next 4 bytes is the number of upstream federates. 
 * The next 4 bytes is the number of downstream federates.
 * 
 * Depending on the first four bytes, the next bytes are pairs of (fed ID (2
 * bytes), delay (8 bytes)) for this federate's connection to upstream federates
 * (by direct connection). The delay is the minimum "after" delay of all
 * connections from the upstream federate.
 *
 * Depending on the second four bytes, the next bytes are fed IDs (2
 * bytes each), of this federate's downstream federates (by direct connection).
 *
 * @note The upstream and downstream connections are transmitted on the same
 *  message to prevent (at least to some degree) the scenario where the RTI has
 *  information about one, but not the other (which is a critical error).
 */
#define MSG_TYPE_NEIGHBOR_STRUCTURE 24
#define MSG_TYPE_NEIGHBOR_STRUCTURE_HEADER_SIZE 9

/////////////////////////////////////////////
//// Rejection codes

/**
 * These codes are sent in a MSG_TYPE_REJECT message.
 * They are limited to one byte (uchar).
 */

/** Federation ID does not match. */
#define FEDERATION_ID_DOES_NOT_MATCH 1

/** Federate with the specified ID has already joined. */
#define FEDERATE_ID_IN_USE 2

/** Federate ID out of range. */
#define FEDERATE_ID_OUT_OF_RANGE 3

/** Incoming message is not expected. */
#define UNEXPECTED_MESSAGE 4

/** Connected to the wrong server. */
#define WRONG_SERVER 5

#endif /* NET_COMMON_H */
