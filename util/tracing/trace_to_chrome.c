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
 * Standalone program to convert a Lingua Franca trace file to a JSON file suitable
 * for viewing in Chrome's event visualizer. To visualize the resulting file,
 * point your chrome browser to chrome://tracing/ and the load the .json file.
 */
#define LINGUA_FRANCA_TRACE
#include "reactor.h"
#include "trace.h"
#include "trace_util.h"

/** Buffer for reading trace records. */
trace_record_t trace[TRACE_BUFFER_CAPACITY];

/** Maximum thread ID seen. */
int max_thread_id = 0;

/**
 * Print a usage message.
 */
void usage() {
    printf("\nUsage: trace_to_chrome [options] trace_file_root (without .lft extension)\n\n");
    /* No options yet:
    printf("\nOptions: \n\n");
    printf("  -f, --fast [true | false]\n");
    printf("   Whether to wait for physical time to match logical time.\n\n");
    printf("\n\n");
    */
}

/**
 * Read a trace in the specified file and write it to the specified json file.
 * @param trace_file The file to read.
 * @param json_file The file to write.
 * @return The number of records read or 0 upon seeing an EOF.
 */
size_t read_and_write_trace(FILE* trace_file, FILE* json_file) {
    int trace_length = read_trace(trace_file);
    if (trace_length == 0) return 0;
    // Write each line.
    for (int i = 0; i < trace_length; i++) {
        char* reaction_name = "none";
        if (trace[i].reaction_number >= 0) {
            reaction_name = (char*)malloc(4);
            snprintf(reaction_name, 4, "%d", trace[i].reaction_number);
        }
        char* phase;
        int pid;
        switch(trace[i].event_type) {
            case reaction_starts:
                phase = "B";
                pid = 0; // Process 0 will be named "Execution"
                break;
            case reaction_ends:
                phase = "E";
                pid = 0; // Process 0 will be named "Execution"
                break;
            default:
                phase = "i";
        }
        // printf("DEBUG: self_struct pointer: %p\n", trace[i].self_struct);
        fprintf(json_file, "{"
                    "\"name\": \"%s\", "   // name is the reactor name.
                    "\"cat\": \"%s\", "    // category is the type of event.
                    "\"ph\": \"%s\", "     // phase is "B" (begin), "E" (end), or "X" (complete).
                    "\"tid\": %d, "        // thread ID.
                    "\"pid\": %d, "        // process ID is required.
                    "\"ts\": %lld, "       // timestamp is physical time in microseconds
                    "\"args\": {"
                        "\"reaction\": %s,"          // reaction number.
                        "\"logical time\": %lld,"    // logical time.
                        "\"microstep\": %d"          // microstep.
                    "}},\n",
                get_description(trace[i].self_struct),
                trace_event_names[trace[i].event_type],
                phase,
                trace[i].worker,
                pid,
                (trace[i].physical_time - start_time)/1000,
                reaction_name,
                (trace[i].logical_time - start_time)/1000,
                trace[i].microstep
        );
        if (trace[i].worker > max_thread_id) {
            max_thread_id = trace[i].worker;
        }
    }
    return trace_length;
}

/**
 * Write metadata events, which provide names in the renderer.
 */
void write_metadata_events(FILE* json_file) {
    // Thread 0 is the main thread.
    fprintf(json_file, "{"
            "\"name\": \"thread_name\", "
            "\"ph\": \"M\", "      // mark as metadata.
            "\"pid\": 0, "
            "\"tid\": 0, "
            "\"args\": {"
                "\"name\": \"Main thread\"" 
                    "}},\n"
        );

    // Name the worker threads.
    for (int i = 1; i <= max_thread_id; i++) {
        fprintf(json_file, "{"
                    "\"name\": \"thread_name\", "
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": 0, "
                    "\"tid\": %d, "
                    "\"args\": {"
                        "\"name\": \"Worker %d\""
                    "}},\n",
                i, i
        );
    }
    // Name the "process" for "Execution"
    // Last metadata entry lacks a comma.
    fprintf(json_file, "{"
                    "\"name\": \"process_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": 0, "         // the "process" to label "Execution".
                    "\"args\": {"
                        "\"name\": \"Execution of %s\"" 
                    "}}\n",
                top_level);
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
    char json_file_name[strlen(argv[1]) + 4];
    strcpy(json_file_name, argv[1]);
    strcat(json_file_name, ".json");
    FILE* json_file = fopen(json_file_name, "w");
    if (json_file == NULL) {
        fprintf(stderr, "Could not create output file named %s.\n", json_file_name);
        usage();
        exit(2);
    }

    if (read_header(trace_file) >= 0) {
        // Write the opening bracket into the json file.
        fprintf(json_file, "{ \"traceEvents\": [\n");
        while (read_and_write_trace(trace_file, json_file) != 0) {};
        write_metadata_events(json_file);
        fprintf(json_file, "]}\n");
   }
    // Free memory in object description table.
    for (int i = 0; i < object_descriptions_size; i++) {
        free(object_descriptions[i].description);
    }
    fclose(trace_file);
    fclose(json_file);
}
