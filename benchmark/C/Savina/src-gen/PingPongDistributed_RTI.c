#define LOG_LEVEL 2
#define _LF_CLOCK_SYNC_INITIAL
#define _LF_CLOCK_SYNC_PERIOD_NS 5000000
#define _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL 10
#ifdef NUMBER_OF_FEDERATES
#undefine NUMBER_OF_FEDERATES
#endif
#define NUMBER_OF_FEDERATES 2
#include "core/rti.c"
int main(int argc, char* argv[]) {
    process_args(argc, argv);
    printf("Starting RTI for %d federates in federation ID %s\n", NUMBER_OF_FEDERATES, federation_id);
    for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
        initialize_federate(i);
    }
    interval_t candidate_tmp;
    int socket_descriptor = start_rti_server(0);
    wait_for_federates(socket_descriptor);
    printf("RTI is exiting.\n");
    return 0;
}
