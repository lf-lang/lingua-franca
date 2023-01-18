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
#define LF_TRACE
#include "reactor.h"
#include "trace.h"
#include "trace_util.h"

/** Buffer for reading object descriptions. Size limit is BUFFER_SIZE bytes. */
char buffer[BUFFER_SIZE];

/** File containing the trace binary data. */
FILE* trace_file = NULL;

/** File for writing the output data. */
FILE* output_file = NULL;

/** File for writing summary statistics. */
FILE* summary_file = NULL;

/** Buffer for reading trace records. */
trace_record_t trace[TRACE_BUFFER_CAPACITY];

/** The start time read from the trace file. */
instant_t start_time;

/** Name of the top-level reactor (first entry in symbol table). */
char* top_level = NULL;

/** Table of pointers to the self struct of a reactor. */
// FIXME: Replace with hash table implementation.
object_description_t* object_table;
int object_table_size = 0;

/**
 * Function to be invoked upon exiting.
 */
void termination() {
    // Free memory in object description table.
    for (int i = 0; i < object_table_size; i++) {
        free(object_table[i].description);
    }
    if (trace_file != NULL) {
        fclose(trace_file);
    }
    if (output_file != NULL) {
        fclose(output_file);
    }
    if (summary_file != NULL) {
        fclose(summary_file);
    }
    printf("Done!\n");
}

/**
 * Open the trace file and the output file using the given filename.
 * This leaves the FILE* pointers in the global variables trace_file and output_file.
 * If the extension if "csv", then it also opens a summary_file.
 * The filename argument can include path information.
 * It can include the ".lft" extension or not.
 * The output file will have the same path and name except that the
 * extension will be given by the second argument.
 * The summary_file, if opened, will have the filename with "_summary.csv" appended.
 * @param filename The file name.
 * @param output_file_extension The extension to put on the output file name (e.g. "csv").
 * @return A pointer to the file.
 */
void open_files(char* filename, char* output_file_extension) {
    // Open the input file for reading.
    size_t length = strlen(filename);
    if (length > 4 && strcmp(&filename[length - 4], ".lft") == 0) {
        // The filename includes the .lft extension.
        length -= 4;
    }
    char trace_file_name[length + 4];
    strncpy(trace_file_name, filename, length);
    trace_file_name[length] = 0;
    strcat(trace_file_name, ".lft");
    trace_file = fopen(trace_file_name, "r");
    if (trace_file == NULL) {
        fprintf(stderr, "No trace file named %s.\n", trace_file_name);
        usage();
        exit(2);
    }

    // Open the output file for writing.
    if (output_file_extension) {
        char output_file_name[length + strlen(output_file_extension) + 1];
        strncpy(output_file_name, filename, length);
        output_file_name[length] = 0;
        strcat(output_file_name, ".");
        strcat(output_file_name, output_file_extension);
        output_file = fopen(output_file_name, "w");
        if (output_file == NULL) {
            fprintf(stderr, "Could not create output file named %s.\n", output_file_name);
            usage();
            exit(2);
        }

        if (strcmp("csv", output_file_extension) == 0) {
            // Also open a summary_file.
            char *suffix = "_summary.csv";
            char summary_file_name[length + strlen(suffix) + 1];
            strncpy(summary_file_name, filename, length);
            summary_file_name[length] = 0;
            strcat(summary_file_name, suffix);
            summary_file = fopen(summary_file_name, "w");
            if (summary_file == NULL) {
                fprintf(stderr, "Could not create summary file named %s.\n", summary_file_name);
                usage();
                exit(2);
            }
        }
    }

    if (atexit(termination) != 0) {
        fprintf(stderr, "WARNING: Failed to register termination function!");
    }
}

/**
 * Get the description of the object pointed to by the specified pointer.
 * For example, this can be the name of a reactor (pointer points to
 * the self struct) or a user-define string.
 * If there is no such pointer in the symbol table, return NULL.
 * If the index argument is non-null, then put the index
 * of the entry in the table into the int pointed to
 * or -1 if none was found.
 * @param pointer The pointer to to an object, e.g. a self struct.
 * @param index An optional pointer into which to write the index.
 */
char* get_object_description(void* pointer, int* index) {
    // FIXME: Replace with a hash table implementation.
    for (int i = 0; i < object_table_size; i++) {
        if (object_table[i].pointer == pointer) {
            if (index != NULL) {
                *index = i;
            }
            return object_table[i].description;
        }
    }
    if (index != NULL) {
        *index = 0;
    }
    return NULL;
}

/**
 * Get the trigger name for the specified pointer.
 * If there is no such trigger, return NULL.
 * If the index argument is non-null, then put the index
 * of the trigger in the table into the int pointed to
 * or -1 if none was found.
 * @param reactor The pointer to a self struct.
 * @param index An optional pointer into which to write the index.
 */
char* get_trigger_name(void* trigger, int* index) {
    // FIXME: Replace with a hash table implementation.
    for (int i = 0; i < object_table_size; i++) {
        if (object_table[i].trigger == trigger && object_table[i].type == trace_trigger) {
            if (index != NULL) {
                *index = i;
            }
            return object_table[i].description;
        }
    }
    if (index != NULL) {
        *index = 0;
    }
    return NULL;
}

/**
 * Print the object to description table.
 */
void print_table() {
    printf("------- objects traced:\n");
    for (int i = 0; i < object_table_size; i++) {
        char* type;
        if (object_table[i].type == trace_reactor) {
            type = "reactor";
        } else if (object_table[i].type == trace_trigger) {
            type = "trigger";
        } else if (object_table[i].type == trace_user) {
            type = "user-defined";
        } else {
            type = "unknown type";
        }
        printf("pointer = %p, trigger = %p, type = %s: %s\n", 
            object_table[i].pointer,
            object_table[i].trigger,
            type,
            object_table[i].description);
    } 
    printf("-------\n");
}

/**
 * Read header information.
 * @return The number of objects in the object table or -1 for failure.
 */
size_t read_header() {
    // Read the start time.
    int items_read = fread(&start_time, sizeof(instant_t), 1, trace_file);
    if (items_read != 1) _LF_TRACE_FAILURE(trace_file);

    printf("Start time is %lld.\n", start_time);

    // Read the table mapping pointers to descriptions.
    // First read its length.
    items_read = fread(&object_table_size, sizeof(int), 1, trace_file);
    if (items_read != 1) _LF_TRACE_FAILURE(trace_file);

    printf("There are %d objects traced.\n", object_table_size);

    object_table = calloc(object_table_size, sizeof(trace_record_t));
    if (object_table == NULL) {
        fprintf(stderr, "ERROR: Memory allocation failure %d.\n", errno);
        return -1;
    }

    // Next, read each table entry.
    for (int i = 0; i < object_table_size; i++) {
        void* reactor;
        items_read = fread(&reactor, sizeof(void*), 1, trace_file);
        if (items_read != 1) _LF_TRACE_FAILURE(trace_file);
        object_table[i].pointer = reactor;

        void* trigger;
        items_read = fread(&trigger, sizeof(trigger_t*), 1, trace_file);
        if (items_read != 1) _LF_TRACE_FAILURE(trace_file);
        object_table[i].trigger = trigger;

        // Next, read the type.
        _lf_trace_object_t trace_type;
        items_read = fread(&trace_type, sizeof(_lf_trace_object_t), 1, trace_file);
        if (items_read != 1) _LF_TRACE_FAILURE(trace_file);
        object_table[i].type = trace_type;

        // Next, read the string description into the buffer.
        int description_length = 0;
        char character;
        items_read = fread(&character, sizeof(char), 1, trace_file);
        if (items_read != 1) _LF_TRACE_FAILURE(trace_file);
        while(character != 0 && description_length < BUFFER_SIZE - 1) {
            buffer[description_length++] = character;
            items_read = fread(&character, sizeof(char), 1, trace_file);
            if (items_read != 1) _LF_TRACE_FAILURE(trace_file);
        }
        // Terminate with null.
        buffer[description_length++] = 0;

        // Allocate memory to store the description.
        object_table[i].description = malloc(description_length);
        strcpy(object_table[i].description, buffer);

        if (top_level == NULL) {
            top_level = object_table[i].description;
        }
    }
    print_table();
    return object_table_size;
}

/**
 * Read the trace from the specified file and put it in the trace global
 * variable. Return the length of the trace.
 * @return The number of trace record read or 0 upon seeing an EOF.
 */
int read_trace() {
    // Read first the int giving the length of the trace.
    int trace_length;
    int items_read = fread(&trace_length, sizeof(int), 1, trace_file);
    if (items_read != 1) {
        if (feof(trace_file)) return 0;
        fprintf(stderr, "Failed to read trace length.\n");
        exit(3);
    }
    if (trace_length > TRACE_BUFFER_CAPACITY) {
        fprintf(stderr, "ERROR: Trace length %d exceeds capacity. File is garbled.\n", trace_length);
        exit(4);
    }
    // printf("DEBUG: Trace of length %d being converted.\n", trace_length);

    items_read = fread(&trace, sizeof(trace_record_t), trace_length, trace_file);
    if (items_read != trace_length) {
        fprintf(stderr, "Failed to read trace of length %d.\n", trace_length);
        exit(5);
    }
    return trace_length;
}
