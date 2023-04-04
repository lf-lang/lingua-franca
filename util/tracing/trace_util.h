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
 * Header file for common utilities used in converting Lingua Franca trace files
 * into other formats.
 */
#define LF_TRACE
#include "reactor.h"
#include "trace.h"

/** Macro to use when access to trace file fails. */
#define _LF_TRACE_FAILURE(trace_file) \
    do { \
        fprintf(stderr, "WARNING: Access to trace file failed.\n"); \
        fclose(trace_file); \
        trace_file = NULL; \
        return -1; \
    } while(0)

/** Buffer for reading object descriptions. Size limit is BUFFER_SIZE bytes. */
#define BUFFER_SIZE 1024

/** Buffer for reading trace records. */
extern trace_record_t trace[];

/** File containing the trace binary data. */
extern FILE* trace_file;

/** File for writing the output data. */
extern FILE* output_file;

/** File for writing summary statistics. */
extern FILE* summary_file;

/**
 * Print a usage message.
 */
void usage();

/** The start time read from the trace file. */
extern instant_t start_time;

/** Table of pointers to a description of the object. */
extern object_description_t* object_table;
extern int object_table_size;

/** Name of the top-level reactor (first entry in symbol table). */
extern char* top_level;

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
 */
void open_files(char* filename, char* output_file_extension);

/**
 * Get the description of the object pointed to by the specified pointer.
 * For example, this can be the name of a reactor (pointer points to
 * the self struct) or a user-defined string.
 * If there is no such pointer in the symbol table, return NULL.
 * If the index argument is non-null, then put the index
 * of the entry in the table into the int pointed to
 * or -1 if none was found.
 * @param pointer The pointer to to an object, e.g. a self struct.
 * @param index An optional pointer into which to write the index.
 */
char* get_object_description(void* reactor, int* index);

/**
 * Get the trigger name for the specified pointer.
 * If there is no such trigger, return NULL.
 * If the index argument is non-null, then put the index
 * of the trigger in the table into the int pointed to
 * or -1 if none was found.
 * @param reactor The pointer to a self struct.
 * @param index An optional pointer into which to write the index.
 */
char* get_trigger_name(void* trigger, int* index);

/**
 * Print the object to description table.
 */
void print_table();

/**
 * Read header information.
 * @return The number of objects in the object table or -1 for failure.
 */
size_t read_header();

/**
 * Read the trace from the specified file and put it in the trace global
 * variable. Return the length of the trace.
 * @return The number of trace record read or 0 upon seeing an EOF.
 */
int read_trace();
