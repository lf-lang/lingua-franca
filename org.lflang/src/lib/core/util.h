/**
 * @file
 * @author Edward A. Lee
 * @author Soroush Bateni
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley.

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
 * Header file for utility types and functions for Lingua Franca programs.
 */

#ifndef UTIL_H
#define UTIL_H

#include <stdarg.h>   // Defines va_list

/**
 * Holds generic statistical data
 */
typedef struct lf_stat_ll {
    long long average;
    long long standard_deviation;
    long long variance;
    long long max;
} lf_stat_ll;

/**
 * A handy macro that can concatenate three strings.
 * Useful in the DEBUG_PRINT macro and error_print
 * functions that want to concatenate a "DEBUG: " or
 * "ERROR: " to the beginning of the message and a 
 * new line format \n at the end.
 */
#define CONCATENATE_THREE_STRINGS(__string1, __string2, __string3) __string1 __string2 __string3

/**
 * LOG_LEVEL is set in generated code to 0 through 4 if the target
 * logging property is error, warning, info, log, or debug.
 * The default level is info (2). Currently, 0, 1, and 2 are
 * treated identically and error_print, warning_print, and info_print
 * all result in printed output.
 * If log is set (3), then LOG_DEBUG messages
 * will be printed as well.
 * If debug is set (4), the DEBUG_PRINT messages will
 * be printed as well.
 */
#define LOG_LEVEL_ERROR 0
#define LOG_LEVEL_WARNING 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_LOG 3
#define LOG_LEVEL_DEBUG 4
#define LOG_LEVEL_ALL 255

/** Default log level. */
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

/**
 * The ID of this federate. For a non-federated execution, this will
 * be -1.  For a federated execution, it will be assigned when the generated function
 * __initialize_trigger_objects() is called.
 * @see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend.
 */
extern int _lf_my_fed_id;

/**
 * Return the federate ID or -1 if this program is not part of a federation.
 */
int get_fed_id();

/**
 * Report an informational message on stdout with
 * a newline appended at the end.
 * If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void info_print(char* format, ...);

/**
 * Report an log message on stdout with the prefix
 * "LOG: " and a newline appended
 * at the end. If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void log_print(char* format, ...);

/**
 * A macro used to print useful logging information. It can be enabled
 * by setting the target property 'logging' to 'LOG' or
 * by defining LOG_LEVEL to LOG_LEVEL_LOG or
 * LOG_LEVEL_DEBUG in the top-level preamble.
 * The input to this macro is exactly like printf: (format, ...).
 * "LOG: " is prepended to the beginning of the message
 * and a newline is appended to the end of the message.
 *
 * @note This macro is non-empty even if LOG_LEVEL is not defined in
 * user-code. This is to ensure that the compiler will still parse
 * the predicate inside (...) to prevent LOG_PRINT statements
 * to fall out of sync with the rest of the code. This should have
 * a negligible impact on performance if compiler optimization
 * (e.g., -O2 for gcc) is used as long as the arguments passed to
 * it do not themselves incur significant overhead to evaluate.
 */
#define LOG_PRINT(format, ...) \
            do { if(LOG_LEVEL >= LOG_LEVEL_LOG) { \
                    log_print(format, ##__VA_ARGS__); \
                } } while (0)

/**
 * Report an debug message on stdout with the prefix
 * "DEBUG: " and a newline appended
 * at the end. If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void debug_print(char* format, ...);

/**
 * A macro used to print useful debug information. It can be enabled
 * by setting the target property 'logging' to 'DEBUG' or
 * by defining LOG_LEVEL to 2 in the top-level preamble.
 * The input to this macro is exactly like printf: (format, ...).
 * "DEBUG: " is prepended to the beginning of the message
 * and a newline is appended to the end of the message.
 *
 * @note This macro is non-empty even if LOG_LEVEL is not defined in
 * user-code. This is to ensure that the compiler will still parse
 * the predicate inside (...) to prevent DEBUG_PRINT statements
 * to fall out of sync with the rest of the code. This should have
 * a negligible impact on performance if compiler optimization
 * (e.g., -O2 for gcc) is used as long as the arguments passed to
 * it do not themselves incur significant overhead to evaluate.
 */
#define DEBUG_PRINT(format, ...) \
            do { if(LOG_LEVEL >= LOG_LEVEL_DEBUG) { \
                    debug_print(format, ##__VA_ARGS__); \
                } } while (0)

/**
 * Print the error defined by the errno variable with the
 * specified message as a prefix, then exit with error code 1.
 * @param msg The prefix to the message.
 */
void error(char *msg);

/**
 * Report an error with the prefix "ERROR: " and a newline appended
 * at the end.  The arguments are just like printf().
 */
void error_print(char* format, ...);

/**
 * Report a warning with the prefix "WARNING: " and a newline appended
 * at the end.  The arguments are just like printf().
 */
void warning_print(char* format, ...);

/**
 * Report an error with the prefix "ERROR: " and a newline appended
 * at the end, then exit with the failure code EXIT_FAILURE.
 * The arguments are just like printf().
 */
void error_print_and_exit(char* format, ...);

/**
 * Message print function type. The arguments passed to one of
 * these print functions are a printf-style format string followed
 * by a printf-style argument list collected into a va_list
 * (variable argument list).
 */
typedef void(print_message_function_t)(char*, va_list);

/**
 * Register a function to display messages. After calling this,
 * all messages passed to the above print functions will be
 * printed using the specified function rather than printf
 * if their log level is greater than the specified level.
 * The level should be one of LOG_LEVEL_ERROR, LOG_LEVEL_WARNING,
 * LOG_LEVEL_INFO, LOG_LEVEL_LOG, or LOG_LEVEL_DEBUG.
 *
 * @param function The print message function or NULL to revert
 *  to using printf.
 * @param log_level The level of messages to redirect.
 */
void register_print_function(print_message_function_t* function, int log_level);

#endif /* UTIL_H */
