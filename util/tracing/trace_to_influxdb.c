/**
 * @file
 * @author Edward A. Lee
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley

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
 * FIXME: I was unable to get this to work. InfluxDB documentation is inadequate.
 * I have tried both basic authentication with user/password and token authentication
 * in the HTTP POST header. Neither works.
 * 
 * Standalone program to send a Lingua Franca trace file to InfluxDB.
 * InfluxDB is a database server to which data can be posted using HTTP
 * or sent as a UDP datagram.
 * 
 * To set up InfluxDB, see:
 * 
 * [https://docs.influxdata.com/influxdb/v2.0/get-started/](https://docs.influxdata.com/influxdb/v2.0/get-started/)
 * 
 * To start the server on port 8087, do this:
 * ```shell
 *     influxd --http-bind-address localhost:8087 --reporting-disabled
 * ```
 * Then point your browser to [http://localhost:8087](http://localhost:8087).
 * Here, you can create a user, password, and bucket into which to dump data.
 * 
 * FIXME: How to use this program.
 */
#define LINGUA_FRANCA_TRACE
#include "reactor.h"
#include "trace.h"
#include "trace_util.h"
#include "influxdb.h"

#define MAX_NUM_REACTIONS 64  // Maximum number of reactions reported in summary stats.
#define MAX_NUM_WORKERS 64

/** Struct identifying the influx client. */
influx_client_t influx_client;
influx_v2_client_t influx_v2_client;
/**
 * Print a usage message.
 */
void usage() {
    printf("\nUsage: trace_to_influxdb [options] trace_file\n\n");
    /* No options yet:
    printf("\nOptions: \n\n");
    printf("  -f, --fast [true | false]\n");
    printf("   Whether to wait for physical time to match logical time.\n\n");
    printf("\n\n");
    */
}

/** Largest timestamp seen. */
instant_t latest_time = 0LL;

/**
 * Read a trace in the specified file and write it to the specified CSV file.
 * @return The number of records read or 0 upon seeing an EOF.
 */
size_t read_and_write_trace() {
    int trace_length = read_trace();
    if (trace_length == 0) return 0;
    // Write each line.
    for (int i = 0; i < trace_length; i++) {
        char* reaction_name = "none";
        if (trace[i].reaction_number >= 0) {
            reaction_name = (char*)malloc(4);
            snprintf(reaction_name, 4, "%d", trace[i].reaction_number);
        }
        // printf("DEBUG: reactor self struct pointer: %p\n", trace[i].pointer);
        int object_instance = -1;
        char* reactor_name = get_object_description(trace[i].pointer, &object_instance);
        if (reactor_name == NULL) {
            reactor_name = "NO REACTOR";
        }
        int trigger_instance = -1;
        char* trigger_name = get_trigger_name(trace[i].trigger, &trigger_instance);
        if (trigger_name == NULL) {
            trigger_name = "NO TRIGGER";
        }
        // FIXME: Treating physical time as the timestamp.
        // Do we want this to optionally be logical time?
        // FIXME: What is the difference between a TAG and F_STR (presumably, Field String)?
        // Presumably, the HTTP post is formatted as a "line protocol" command. See:
        // https://docs.influxdata.com/influxdb/v2.0/reference/syntax/line-protocol/
        int response_code = post_curl(&influx_v2_client,
            INFLUX_MEAS(trace_event_names[trace[i].event_type]),
            INFLUX_TAG("Reactor", reactor_name),
            INFLUX_TAG("Reaction", reaction_name),
            INFLUX_F_INT("Worker", trace[i].worker),
            INFLUX_F_INT("Logical Time", trace[i].logical_time),
            INFLUX_F_INT("Microstep", trace[i].microstep),
            INFLUX_F_STR("Trigger Name", trigger_name),
            INFLUX_F_INT("Extra Delay", trace[i].extra_delay),
            INFLUX_TS(trace[i].physical_time),
            INFLUX_END
        );
        if (response_code != 0) {
            fprintf(stderr, "****** response code: %d\n", response_code);
            return 0;
        }
    }
    return trace_length;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        usage();
        exit(0);
    }
    // FIXME: Get from command line.
    // Change these paramerets as per your environment
    influx_v2_client.host = "localhost";
    influx_v2_client.port = 8088;
    influx_v2_client.org = "icyphy"; 
    influx_v2_client.bucket = "tracing";
    influx_v2_client.token = "ra0gassNhZoC0V1ABxVHT6-34thskx5HFgMEivd2WFfHuNXyspaYj9SB992YFKTCtne0_pb80OSKundUa7KLGQ==";
    
    //influx_client.db = "test";
    //influx_client.usr = "eal";
    //influx_client.pwd = "changeme";

    open_files(argv[1], NULL);

    if (read_header() >= 0) {
        size_t num_records = 0, result;
        while ((result = read_and_write_trace()) != 0) {
            num_records = result;
        };
        printf("***** %zu records written to InfluxDB.\n", num_records);
        // File closing is handled by termination function.
    }
}