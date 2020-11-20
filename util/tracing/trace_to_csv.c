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
 * Standalone program to convert a Lingua Franca trace file to a comma-separated values
 * text file.
 */
#define LINGUA_FRANCA_TRACE
#include "reactor.h"
#include "trace.h"
#include "trace_util.h"

/**
 * Print a usage message.
 */
void usage() {
    printf("\nUsage: trace_to_csv [options] trace_file_root (without .lft extension)\n\n");
    /* No options yet:
    printf("\nOptions: \n\n");
    printf("  -f, --fast [true | false]\n");
    printf("   Whether to wait for physical time to match logical time.\n\n");
    printf("\n\n");
    */
}

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
        char* reactor_name = get_object_description(trace[i].pointer, NULL);
        if (reactor_name == NULL) {
            reactor_name = "NO REACTOR";
        }
        char* trigger_name = get_trigger_name(trace[i].trigger, NULL);
        if (trigger_name == NULL) {
            trigger_name = "NO TRIGGER";
        }
        fprintf(output_file, "%s, %s, %s, %d, %lld, %d, %lld, %s, %lld\n",
                trace_event_names[trace[i].event_type],
                reactor_name,
                reaction_name,
                trace[i].worker,
                trace[i].logical_time - start_time,
                trace[i].microstep,
                trace[i].physical_time - start_time,
                trigger_name,
                trace[i].extra_delay
        );
    }
    return trace_length;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        usage();
        exit(0);
    }
    open_files(argv[1], "csv");

    if (read_header() >= 0) {
        // Write a header line into the CSV file.
        fprintf(output_file, "Event, Reactor, Reaction, Worker, Elapsed Logical Time, Microstep, Elapsed Physical Time, Trigger, Extra Delay\n");
        while (read_and_write_trace() != 0) {};
    }
}
