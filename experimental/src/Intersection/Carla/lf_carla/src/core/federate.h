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
 * Data structures and functions used and defined in federate.c.
 */

#ifndef FEDERATE_H
#define FEDERATE_H

#ifndef ADVANCE_MESSAGE_INTERVAL
#define ADVANCE_MESSAGE_INTERVAL MSEC(10)
#endif

/**
 * Structure that a federate instance uses to keep track of its own state.
 */
typedef struct federate_instance_t {
	/**
	 * The TCP socket descriptor for this federate to communicate with the RTI.
	 * This is set by connect_to_rti(), which must be called before other
	 * functions that communicate with the rti are called.
	 */
	int socket_TCP_RTI;

	/**
	 * Thread listening for incoming TCP messages from the RTI.
	 */
	lf_thread_t RTI_socket_listener;

	/**
	 * Number of inbound physical connections to the federate.
	 * This can be either physical connections, or logical connections
	 * in the decentralized coordination, or both.
	 */
	int number_of_inbound_p2p_connections;

	/**
	 * Array of thread IDs for threads that listen for incoming messages.
	 * This is NULL if there are none and otherwise has size given by
	 * number_of_inbound_p2p_connections.
	 */
	lf_thread_t *inbound_socket_listeners;

	/**
	 * Number of outbound peer-to-peer connections from the federate.
	 * This can be either physical connections, or logical connections
	 * in the decentralized coordination, or both.
	 */
	int number_of_outbound_p2p_connections;

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
	int sockets_for_inbound_p2p_connections[NUMBER_OF_FEDERATES];

	/**
	 * An array that holds the socket descriptors for outbound direct
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
	int sockets_for_outbound_p2p_connections[NUMBER_OF_FEDERATES];

	/**
	 * Thread ID for a thread that accepts sockets and then supervises
	 * listening to those sockets for incoming P2P (physical) connections.
	 */
	lf_thread_t inbound_p2p_handling_thread_id;

	/**
	 * A socket descriptor for the socket server of the federate.
	 * This is assigned in create_server().
	 * This socket is used to listen to incoming physical connections from
	 * remote federates. Once an incoming connection is accepted, the
	 * opened socket will be stored in
	 * federate_sockets_for_inbound_p2p_connections.
	 */
	int server_socket;

	/**
	 * The port used for the server socket
	 * to listen for messages from other federates.
	 * The federate informs the RTI of this port once
	 * it has created its socket server by sending
	 * an ADDRESS_AD message (@see rti.h).
	 */
	int server_port;

	/**
	 * Most recent TIME_ADVANCE_GRANT received from the RTI, or NEVER if none
	 * has been received.
	 * This is used to communicate between the listen_to_rti_TCP thread and the
	 * main federate thread.
	 * This variable should only be accessed while holding the mutex lock.
	 */
	tag_t last_TAG;

	/**
	 * Indicates whether the last TAG received is provisional or an ordinary
	 * TAG. 
	 * If the last TAG has been provisional, network control reactions must be inserted.
	 * This variable should only be accessed while holding the mutex lock.
	 */
	bool is_last_TAG_provisional;

	/**
	 * Indicator of whether a NET has been sent to the RTI and no TAG
	 * yet received in reply.
	 * This variable should only be accessed while holding the mutex lock.
	 */
	bool waiting_for_TAG;

	/**
	 * Indicator of whether this federate has upstream federates.
	 * The default value of false may be overridden in __initialize_trigger_objects.
	 */
	bool has_upstream;

	/**
	 * Indicator of whether this federate has downstream federates.
	 * The default value of false may be overridden in __initialize_trigger_objects.
	 */
	bool has_downstream;

	/**
	 * Used to prevent the federate from sending a REQUEST_STOP
	 * message multiple times to the RTI.
	 * This variable should only be accessed while holding the mutex lock.
	 */
	bool sent_a_stop_request_to_rti;

	/**
	 * A record of the most recently sent LTC (logical tag complete) message.
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
	tag_t last_sent_LTC;

	/**
	 * A record of the most recently sent NET (next event tag) message.
	 */
	tag_t last_sent_NET;

	/**
	 * For use in federates with centralized coordination, the minimum
	 * time delay between a physical action within this federate and an
	 * output from this federate.  This is NEVER if there is causal
	 * path from a physical action to any output.
	 */
	instant_t min_delay_from_physical_action_to_federate_output;

	/**
	 * This list is also used to determine the status of a given network 
	 * input port at a given logical time. The status of the port (trigger->status) can be: 
	 * present, absent, or unknown. To determine the status of that port, for a given trigger 
	 * 't' in this list, a (number of) network input control reactions are inserted into the 
	 * reaction queue, which is are special kind of reaction that wait long enough until the 
	 * status of the port becomes known. In the centralized coordination, this wait is until 
	 * the RTI informs the reaction of the status of the port. In the decentralized coordination,
	 * this wait is until the STP offset expires (or the status is somehow becomes known sooner).
	 */

	/**
	 * List of triggers for network input control reactions, used
	 * to trigger these reaction at the beginning of every tag.
	 */
	trigger_t** triggers_for_network_input_control_reactions;
	int triggers_for_network_input_control_reactions_size;


	/**
	 * The triggers for all network output control reactions. 
	 * 
	 * This is used to trigger network output
	 * control reactions that will potentially send an ABSENT 
	 * message to any downstream federate that might be blocking 
	 * on the network port. The ABSENT message will only be sent if
	 * the output is not present.
	 */
	trigger_t* trigger_for_network_output_control_reactions;

} federate_instance_t;

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
void synchronize_with_other_federates();

/**
 * Wait until the status of network port "portID" can be determined.
 * 
 * In decentralized coordination mode, the wait time is capped by "STP",
 * after which the status of the port is presumed to be absent.
 * 
 * @param portID The ID of the network port
 * @param STP The STP offset of the port
 */
void wait_until_port_status_known(int portID, interval_t STP);

#endif // FEDERATE_H
