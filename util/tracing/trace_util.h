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
#define LINGUA_FRANCA_TRACE
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

/**
 * Print a usage message.
 */
void usage();

/** The start time read from the trace file. */
extern instant_t start_time;

/** Table of pointers to a description of the object. */
extern object_description_t* object_descriptions;
extern int object_descriptions_size;

/** Name of the top-level reactor (first entry in symbol table). */
extern char* top_level;

/**
 * Get the object description corresponding to the specified pointer.
 * If there is no such object, return "NO DESCRIPTION FOUND".
 * @param object The pointer.
 */
char* get_description(void* object);

/**
 * Print the object to description table.
 */
void print_table();

/**
 * Read header information.
 * @return The number of objects in the object table or -1 for failure.
 */
size_t read_header(FILE* trace_file);

/**
 * Read the trace from the specified file and put it in the trace global
 * variable. Return the length of the trace.
 * @param trace_file The file to read.
 * @return The number of trace record read or 0 upon seeing an EOF.
 */
int read_trace(FILE* trace_file);