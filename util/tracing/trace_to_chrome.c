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
#define LF_TRACE
#include "reactor.h"
#include "trace.h"
#include "trace_util.h"

#define PID_FOR_USER_EVENT 1000000 // Assumes no more than a million reactors.
#define PID_FOR_WORKER_WAIT 0  // Use 1000001 to show in separate trace.
#define PID_FOR_WORKER_ADVANCING_TIME 0 // Use 1000002 to show in separate trace.
#define PID_FOR_UNKNOWN_EVENT 2000000

/** Maximum thread ID seen. */
int max_thread_id = 0;

/**
 * Print a usage message.
 */
void usage() {
    printf("\nUsage: trace_to_chrome [options] trace_file (with or without .lft extension)\n");
    printf("Options: \n");
    printf("  -p, --physical\n");
    printf("   Use only physical time, not logical time, for all horizontal axes.\n");
    printf("\n");
}

/** Maximum reaction number encountered. */
int max_reaction_number = 0;

/** Indicator to plot vs. physical time only. */
bool physical_time_only = false;

/**
 * Read a trace in the specified file and write it to the specified json file.
 * @return The number of records read or 0 upon seeing an EOF.
 */
size_t read_and_write_trace() {
    int trace_length = read_trace(trace_file);
    if (trace_length == 0) return 0;
    // Write each line.
    for (int i = 0; i < trace_length; i++) {
        char* reaction_name = "\"UNKNOWN\"";
        if (trace[i].reaction_number >= 0) {
            reaction_name = (char*)malloc(4);
            snprintf(reaction_name, 4, "%d", trace[i].reaction_number);
        }
        // printf("DEBUG: Reactor's self struct pointer: %p\n", trace[i].pointer);
        int reactor_index;
        char* reactor_name = get_object_description(trace[i].pointer, &reactor_index);
        if (reactor_name == NULL) {
            if (trace[i].event_type == worker_wait_starts || trace[i].event_type == worker_wait_ends) {
                reactor_name = "WAIT";
            } else if (trace[i].event_type == scheduler_advancing_time_starts 
                    || trace[i].event_type == scheduler_advancing_time_starts) {
                reactor_name = "ADVANCE TIME";
            } else {
                reactor_name = "NO REACTOR";
            }
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

        if (elapsed_physical_time < 0) {
            fprintf(stderr, "WARNING: Negative elapsed physical time %lld. Skipping trace entry.\n", elapsed_physical_time);
            continue;
        }
        if (elapsed_logical_time < 0) {
            fprintf(stderr, "WARNING: Negative elapsed logical time %lld. Skipping trace entry.\n", elapsed_logical_time);
            continue;
        }

        // Default thread id is the worker number.
        int thread_id = trace[i].worker;

        char* args;
        asprintf(&args, "{"
                        "\"reaction\": %s,"          // reaction number.
                        "\"logical time\": %lld,"    // logical time.
                        "\"physical time\": %lld,"   // physical time.
                        "\"microstep\": %d"          // microstep.
                    "}",
                reaction_name,
                elapsed_logical_time,
                elapsed_physical_time,
                trace[i].microstep
        );
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
                if (!physical_time_only) {
                    timestamp = elapsed_logical_time + trace[i].extra_delay/1000;
                }
                thread_id = trigger_index;
                name = trigger_name;
                break;
            case user_event:
                pid = PID_FOR_USER_EVENT;
                phase= "i";
                if (!physical_time_only) {
                    timestamp = elapsed_logical_time;
                }
                thread_id = reactor_index;
                break;
            case user_value:
                pid = PID_FOR_USER_EVENT;
                phase= "C";
                if (!physical_time_only) {
                    timestamp = elapsed_logical_time;
                }
                thread_id = reactor_index;
                free(args);
                asprintf(&args, "{\"value\": %lld}", trace[i].extra_delay);
                break;
            case worker_wait_starts:
                pid = PID_FOR_WORKER_WAIT;
                phase = "B";
                break;
            case worker_wait_ends:
                pid = PID_FOR_WORKER_WAIT;
                phase = "E";
                break;
            case scheduler_advancing_time_starts:
                pid = PID_FOR_WORKER_ADVANCING_TIME;
                phase = "B";
                break;
            case scheduler_advancing_time_ends:
                pid = PID_FOR_WORKER_ADVANCING_TIME;
                phase = "E";
                break;
            default:
                fprintf(stderr, "WARNING: Unrecognized event type %d: %s", 
                        trace[i].event_type, trace_event_names[trace[i].event_type]);
                pid = PID_FOR_UNKNOWN_EVENT;
                phase = "i";
        }
        fprintf(output_file, "{"
                    "\"name\": \"%s\", "   // name is the reactor or trigger name.
                    "\"cat\": \"%s\", "    // category is the type of event.
                    "\"ph\": \"%s\", "     // phase is "B" (begin), "E" (end), or "X" (complete).
                    "\"tid\": %d, "        // thread ID.
                    "\"pid\": %d, "        // process ID is required.
                    "\"ts\": %lld, "       // timestamp in microseconds
                    "\"args\": %s"         // additional arguments from above.
                    "},\n",
                name,
                trace_event_names[trace[i].event_type],
                phase,
                thread_id,
                pid,
                timestamp,
                args
        );
        free(args);

        if (trace[i].worker > max_thread_id) {
            max_thread_id = trace[i].worker;
        }
        // If the event is reaction_starts and physical_time_only is not set,
        // then also generate an instantaneous
        // event to be shown in the reactor's section, along with timers and actions.
        if (trace[i].event_type == reaction_starts && !physical_time_only) {
            phase = "i";
            pid = reactor_index + 1;
            reaction_name = (char*)malloc(4);
            char name[13];
            snprintf(name, 13, "reaction %d", trace[i].reaction_number);

            // NOTE: If the reactor has more than 1024 timers and actions, then
            // there will be a collision of thread IDs here.
            thread_id = 1024 + trace[i].reaction_number;
            if (trace[i].reaction_number > max_reaction_number) {
                max_reaction_number = trace[i].reaction_number;
            }

            fprintf(output_file, "{"
                    "\"name\": \"%s\", "   // name is the reactor or trigger name.
                    "\"cat\": \"%s\", "    // category is the type of event.
                    "\"ph\": \"%s\", "     // phase is "B" (begin), "E" (end), or "X" (complete).
                    "\"tid\": %d, "        // thread ID.
                    "\"pid\": %d, "        // process ID is required.
                    "\"ts\": %lld, "       // timestamp in microseconds
                    "\"args\": {"
                        "\"microstep\": %d, "       // microstep.
                        "\"physical time\": %lld"   // physical time.
                    "}},\n",
                name,
                "Reaction",
                phase,
                thread_id,
                pid,
                elapsed_logical_time,
                trace[i].microstep,
                elapsed_physical_time
            );
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
        fprintf(output_file, "{"
                    "\"name\": \"thread_name\", "
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "
                    "\"tid\": %d, "
                    "\"args\": {"
                        "\"name\": \"Worker %d\""
                    "}},\n",
                PID_FOR_WORKER_WAIT, i, i
        );
        fprintf(output_file, "{"
                    "\"name\": \"thread_name\", "
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "
                    "\"tid\": %d, "
                    "\"args\": {"
                        "\"name\": \"Worker %d\""
                    "}},\n",
                PID_FOR_WORKER_ADVANCING_TIME, i, i
        );
    }

    // Name reactions for each reactor.
    for (int reactor_index = 1; reactor_index <= object_table_size; reactor_index++) {
        for (int reaction_number = 0; reaction_number <= max_reaction_number; reaction_number++) {
            fprintf(output_file, "{"
                    "\"name\": \"thread_name\", "
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "
                    "\"tid\": %d, "
                    "\"args\": {"
                        "\"name\": \"Reaction %d\""
                    "}},\n",
                reactor_index, reaction_number + 1024, reaction_number
            );
        }
    }
    
    // Write the reactor names for the logical timelines.
    for (int i = 0; i < object_table_size; i++) {
        if (object_table[i].type == trace_trigger) {
            // We need the reactor index (not the name) to set the pid.
            int reactor_index;
            get_object_description(object_table[i].pointer, &reactor_index);
            fprintf(output_file, "{"
                    "\"name\": \"thread_name\", "   // metadata for thread name.
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
                        "\"name\": \"Reactor %s reactions, actions, and timers in logical time\"" 
                    "}},\n",
                i + 1,  // Offset of 1 prevents collision with Execution.
                object_table[i].description);
        } else if (object_table[i].type == trace_user) {
            fprintf(output_file, "{"
                    "\"name\": \"thread_name\", "   // metadata for thread name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "         // the "process" to label as reactor.
                    "\"tid\": %d,"          // The "thread" to label with action or timer name.
                    "\"args\": {"
                        "\"name\": \"%s\"" 
                    "}},\n",
                PID_FOR_USER_EVENT,
                i, // This is the index in the object table.
                object_table[i].description);
        }
    }
    // Name the "process" for "Execution"
    fprintf(output_file, "{"
                    "\"name\": \"process_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": 0, "         // the "process" to label "Execution".
                    "\"args\": {"
                        "\"name\": \"Execution of %s\"" 
                    "}},\n",
                top_level);
    // Name the "process" for "Worker Waiting" if the PID is not the main execution one.
    if (PID_FOR_WORKER_WAIT > 0) {
        fprintf(output_file, "{"
                    "\"name\": \"process_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "        // the "process" to label "Workers waiting for reaction queue".
                    "\"args\": {"
                        "\"name\": \"Workers waiting for reaction queue\"" 
                    "}},\n",
                PID_FOR_WORKER_WAIT);
    }
    // Name the "process" for "Worker advancing time" if the PID is not the main execution one.
    if (PID_FOR_WORKER_ADVANCING_TIME > 0) {
        fprintf(output_file, "{"
                    "\"name\": \"process_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "        // the "process" to label "Workers waiting for reaction queue".
                    "\"args\": {"
                        "\"name\": \"Workers advancing time\"" 
                    "}},\n",
                PID_FOR_WORKER_ADVANCING_TIME);
    }
    // Name the "process" for "User Events"
    // Last metadata entry lacks a comma.
    fprintf(output_file, "{"
                    "\"name\": \"process_name\", "   // metadata for process name.
                    "\"ph\": \"M\", "      // mark as metadata.
                    "\"pid\": %d, "        // the "process" to label "User events".
                    "\"args\": {"
                        "\"name\": \"User events in %s, shown in physical time:\"" 
                    "}}\n",
                PID_FOR_USER_EVENT, top_level);
}

int main(int argc, char* argv[]) {
    char* filename = NULL;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "-p", 2) == 0 || strncmp(argv[i], "--physical", 10) == 0) {
            physical_time_only = true;
        } else if (argv[i][0] == '-') {
            usage();
            return(1);
        } else {
           filename = argv[i];
        }
    }
    if (filename == NULL) {
        usage();
        exit(0);
    }
    open_files(filename, "json");

    if (read_header(trace_file) >= 0) {
        // Write the opening bracket into the json file.
        fprintf(output_file, "{ \"traceEvents\": [\n");
        while (read_and_write_trace() != 0) {};
        write_metadata_events();
        fprintf(output_file, "]}\n");
   }
}
