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
 * @param trace_file The file to read.
 * @param csv_file The file to write.
 * @return The number of records read or 0 upon seeing an EOF.
 */
size_t read_and_write_trace(FILE* trace_file, FILE* csv_file) {
    int trace_length = read_trace(trace_file);
    if (trace_length == 0) return 0;
    // Write each line.
    for (int i = 0; i < trace_length; i++) {
        char* reaction_name = "none";
        if (trace[i].reaction_number >= 0) {
            reaction_name = (char*)malloc(4);
            snprintf(reaction_name, 4, "%d", trace[i].reaction_number);
        }
        // printf("DEBUG: object pointer: %p\n", trace[i].object);
        fprintf(csv_file, "%s, %s, %s, %d, %lld, %d, %lld, %lld\n",
                trace_event_names[trace[i].event_type],
                get_description(trace[i].object),
                reaction_name,
                trace[i].worker,
                trace[i].logical_time - start_time,
                trace[i].microstep,
                trace[i].physical_time - start_time,
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
    // Open the input file for reading.
    char trace_file_name[strlen(argv[1]) + 4];
    strcpy(trace_file_name, argv[1]);
    strcat(trace_file_name, ".lft");
    FILE* trace_file = fopen(trace_file_name, "r");
    if (trace_file == NULL) {
        fprintf(stderr, "No trace file named %s.\n", trace_file_name);
        usage();
        exit(2);
    }

    // Open the output file for writing.
    char csv_file_name[strlen(argv[1]) + 4];
    strcpy(csv_file_name, argv[1]);
    strcat(csv_file_name, ".csv");
    FILE* csv_file = fopen(csv_file_name, "w");
    if (csv_file == NULL) {
        fprintf(stderr, "Could not create output file named %s.\n", csv_file_name);
        usage();
        exit(2);
    }

    if (read_header(trace_file) >= 0) {
        // Write a header line into the CSV file.
        fprintf(csv_file, "Event, Object, Reaction, Worker, Elapsed Logical Time, Microstep, Elapsed Physical Time, Extra Delay\n");
        while (read_and_write_trace(trace_file, csv_file) != 0) {};
    }
    // Free memory in object description table.
    for (int i = 0; i < object_descriptions_size; i++) {
        free(object_descriptions[i].description);
    }
    fclose(trace_file);
    fclose(csv_file);
}
