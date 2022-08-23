#include "../federated/federate.c"
trigger_t* _lf_action_table[0];
trigger_t* _lf_action_for_port(int port_id) {
    if ((port_id < 0) && (port_id >= 0)) return _lf_action_table[port_id];
    else return NULL;
}
void _lf_executable_preamble() {
    // ***** Start initializing the federated execution. */
    // Initialize the socket mutex
    lf_mutex_init(&outbound_socket_mutex);
    lf_cond_init(&port_status_changed);
    _fed.has_downstream = true;
    _lf_my_fed_id = 0;
    _fed.number_of_inbound_p2p_connections = 0;
    _fed.number_of_outbound_p2p_connections = 0;
    // Connect to the RTI. This sets _fed.socket_TCP_RTI and _lf_rti_socket_UDP.
    connect_to_rti("localhost", 0);
}
#define initialize_triggers_for_federate() \
do { \
    _fed.trigger_for_network_output_control_reactions=&s_main_self[0]->_lf__outputControlReactionTrigger; \
} \
while (0)
/**
* Generated function that sends information about connections between this federate and
* other federates where messages are routed through the RTI. Currently, this
* only includes logical connections when the coordination is centralized. This
* information is needed for the RTI to perform the centralized coordination.
* @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h
*/
void send_neighbor_structure_to_RTI(int rti_socket) {
    interval_t candidate_tmp;
    size_t buffer_size = 1 + 8 + 
                    0 * ( sizeof(uint16_t) + sizeof(int64_t) ) +
                    1 * sizeof(uint16_t);
    unsigned char buffer_to_send[buffer_size];
    
    size_t message_head = 0;
    buffer_to_send[message_head] = MSG_TYPE_NEIGHBOR_STRUCTURE;
    message_head++;
    encode_int32((int32_t)0, &(buffer_to_send[message_head]));
    message_head+=sizeof(int32_t);
    encode_int32((int32_t)1, &(buffer_to_send[message_head]));
    message_head+=sizeof(int32_t);
    encode_uint16(1, &(buffer_to_send[message_head]));
    message_head += sizeof(uint16_t);
    write_to_socket_errexit(
        rti_socket, 
        buffer_size,
        buffer_to_send,
        "Failed to send the neighbor structure message to the RTI."
    );
}
