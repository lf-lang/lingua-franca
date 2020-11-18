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
 * @return The number of records read or 0 upon seeing an EOF.
 */
size_t read_and_write_trace() {
    int trace_length = read_trace(trace_file);
    if (trace_length == 0) return 0;
    // Write each line.
    for (int i = 0; i < trace_length; i++) {
        char* reaction_name = "none";
        if (trace[i].reaction_number >= 0) {
            reaction_name = (char*)malloc(4);
            snprintf(reaction_name, 4, "%d", trace[i].reaction_number);
        }
        // printf("DEBUG: Reactor's self struct pointer: %p\n", trace[i].reactor);
        int reactor_index;
        char* reactor_name = get_reactor_name(trace[i].reactor, &reactor_index);
        if (reactor_name == NULL) {
            reactor_name = "NO REACTOR";
        }
        // Default name is the reactor name.
        char* name = reactor_name;

        int trigger_index;
        char* trigger_name = get_trigger_name(trace[i].trigger, &trigger_index);
        if (trigger_name == NULL) {
            trigger_name = "NONE";
        }
        // By default, the timestamp used in the trace is the elapsed
        // physical time in microseconds.  But for schedule_called events,
        // it will instead be the logical time at which the action or timer
        // is to be scheduled.
        interval_t elapsed_physical_time = (trace[i].physical_time - start_time)/1000;
        interval_t timestamp = elapsed_physical_time;
        interval_t elapsed_logical_time = (trace[i].logical_time - start_time)/1000;

        // Default thread id is the worker number.
        int thread_id = trace[i].worker;

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
            case schedule_called:
                phase = "i";
                pid = reactor_index + 1; // One pid per reactor.
                timestamp = elapsed_logical_time + trace[i].extra_delay/1000;
                thread_id = trigger_index;
                name = trigger_name;
            default:
                phase = "i";
        }
        fprintf(output_file, "{"
                    "\"name\": \"%s\", "   // name is the reactor or trigger name.
                    "\"cat\": \"%s\", "    // category is the type of event.
                    "\"ph\": \"%s\", "     // phase is "B" (begin), "E" (end), or "X" (complete).
                    "\"tid\": %d, "        // thread ID.
                    "\"pid\": %d, "        // process ID is required.
                    "\"ts\": %lld, "       // timestamp in microseconds
                    "\"args\": {"
                        "\"reaction\": %s,"          // reaction number.
                        "\"logical time\": %lld,"    // logical time.
                        "\"physical time\": %lld,"   // physical time.
                        "\"microstep\": %d"          // microstep.
                    "}},\n",
                name,
                trace_event_names[trace[i].event_type],
                phase,
                thread_id,
                pid,
                timestamp,
                reaction_name,
                elapsed_logical_time,
                elapsed_physical_time,
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
void write_metadata_events() {
    // Thread 0 is the main thread.
    fprintf(output_file, "{"
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
        fprintf(output_file, "{"
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
     // Write the reactor names for the logical timelines.
    for (int i = 0; i < object_table_size; i++) {
        if (object_table[i].type == trace_trigger) {
            // We need the reactor index (not the name) to set the pid.
            int reactor_index;
            get_reactor_name(object_table[i].reactor, &reactor_index);
            fprintf(output_file, "{"
                    "\"name\": \"thread_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "       // mark as metadata.
                    "\"pid\": %d, "         // the "process" to identify by reactor.
                    "\"tid\": %d,"          // The "thread" to label with action or timer name.
                    "\"args\": {"
                        "\"name\": \"Trigger %s\"" 
                    "}},\n",
                reactor_index + 1, // Offset of 1 prevents collision with Execution.
                i,  
                object_table[i].description);

        } else if (object_table[i].type == trace_reactor) {
            fprintf(output_file, "{"
                    "\"name\": \"process_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "         // the "process" to label as reactor.
                    "\"args\": {"
                        "\"name\": \"Reactor %s actions and timers in logical time\"" 
                    "}},\n",
                i + 1,  // Offset of 1 prevents collision with Execution.
                object_table[i].description);
        }
    }
   // Name the "process" for "Execution"
    // Last metadata entry lacks a comma.
    fprintf(output_file, "{"
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
    open_files(argv[1], "json");

    if (read_header(trace_file) >= 0) {
        // Write the opening bracket into the json file.
        fprintf(output_file, "{ \"traceEvents\": [\n");
        while (read_and_write_trace() != 0) {};
        write_metadata_events();
        fprintf(output_file, "]}\n");
   }
}
