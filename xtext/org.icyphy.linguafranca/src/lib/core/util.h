/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
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
 * Header file for utility functions for Lingua Franca programs.
 */

#ifndef UTIL_H
#define UTIL_H

#define HOST_LITTLE_ENDIAN 1
#define HOST_BIG_ENDIAN 2

/**
 * A handy macro that can concatenate three strings.
 * Useful in the DEBUG_PRINT macro and error_print
 * functions that want to concatenate a "DEBUG: " or
 * "ERROR: " to the beginning of the message and a 
 * new line format \n at the end.
 */
#define CONCATENATE_THREE_STRINGS(__string1, __string2, __string3) __string1 __string2 __string3

/**
 * VERBOSE can be defined in user-code in a 
 * top-level preamble
 * (FIXME: or as a target property in the future).
 * If defined, set DEBUG to level 1.
 */
#ifdef VERBOSE
#define DEBUG 1
#else
#define DEBUG 0
#endif

/**
 * A macro used to print useful debug information. It can be enabled
 * by defining VERBOSE in the top-level preamble.
 * The input to this macro is exactly like printf: (format, ...).
 * A "DEBUG: " moniker is appended to the beginning of the message
 * as well a new line to the end of the message.
 * 
 * @note This macro is generated even if VERBOSE is not defined in
 * user-code. This is to ensure that the compiler will still parse
 * the predicate inside (...) to prevent DEBUG_PRINT statements
 * to fall out of sync with the rest of the code. This should have
 * a negligible impact on performance if compiler optimization
 * (e.g., -O2 for gcc) is used.
 */ 
#define DEBUG_PRINT(format, ...) \
            do { if(DEBUG) { \
                    fprintf(stderr, CONCATENATE_THREE_STRINGS("DEBUG: ",format,"\n"), ##__VA_ARGS__); \
                } } while (0)

/**
 * A function that can be used in lieu of fprintf(stderr, ...).
 * The input to this function is exactly like printf: (format, ...).
 * An "ERROR: " moniker is appended to the beginning of the error message
 * using strcpy and the format and a new line are appended at the
 * end of the printed message using strcat.
 * The size of the error message depends on the size of the input format, which
 * should be a null-terminated string.
 * 
 * FIXME: This function could be slow.
 */
void error_print(char* format, ...);

/**
 * A function that can be used in lieu of fprintf(stderr, ...) that also exits
 * the program. The input to this function is exactly like printf: (format, ...).
 * An "ERROR: " moniker is appended to the beginning of the error message
 * using strcpy and the format and a new line are appended at the
 * end of the printed message using strcat.
 * The size of the error message depends on the size of the input format, which
 * should be a null-terminated string.
 * 
 * FIXME: This function could be slow.
 */
void error_print_and_exit(char* format, ...);

/** Print the error defined by the errno variable with the
 *  specified message as a prefix, then exit with error code 1.
 *  @param msg The prefix to the message.
 */
void error(char *msg);

/** Return true (1) if the host is big endian. Otherwise,
 *  return false.
 */
int host_is_big_endian();

/** 
 * Read the specified number of bytes from the specified socket into the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, report an error and exit. This function
 * takes a formatted string and additional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 *  
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to read.
 * @param buffer The buffer into which to put the bytes.
 */
void read_from_socket(int socket, int num_bytes, unsigned char* buffer, char* format, ...);

/** 
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, report an error and exit. This function
 * takes a formatted string and additional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 * 
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to write.
 * @param buffer The buffer from which to get the bytes.
 */
void write_to_socket(int socket, int num_bytes, unsigned char* buffer, char* format, ...);

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first).
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_ll(long long data, unsigned char* buffer);

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first).
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_int(int data, unsigned char* buffer);

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first).
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_ushort(unsigned short data, unsigned char* buffer);

/** If this host is little endian, then reverse the order of
 *  the bytes of the argument. Otherwise, return the argument
 *  unchanged. This can be used to convert the argument to
 *  network order (big endian) and then back again.
 *  Network transmissions, by convention, are big endian,
 *  meaning that the high-order byte is sent first.
 *  But many platforms, including my Mac, are little endian,
 *  meaning that the low-order byte is first in memory.
 *  @param src The argument to convert.
 */
int swap_bytes_if_big_endian_int(int src);

/** If this host is little endian, then reverse the order of
 *  the bytes of the argument. Otherwise, return the argument
 *  unchanged. This can be used to convert the argument to
 *  network order (big endian) and then back again.
 *  Network transmissions, by convention, are big endian,
 *  meaning that the high-order byte is sent first.
 *  But many platforms, including my Mac, are little endian,
 *  meaning that the low-order byte is first in memory.
 *  @param src The argument to convert.
 */
long long swap_bytes_if_big_endian_ll(long long src);

/** If this host is little endian, then reverse the order of
 *  the bytes of the argument. Otherwise, return the argument
 *  unchanged. This can be used to convert the argument to
 *  network order (big endian) and then back again.
 *  Network transmissions, by convention, are big endian,
 *  meaning that the high-order byte is sent first.
 *  But many platforms, including my Mac, are little endian,
 *  meaning that the low-order byte is first in memory.
 *  @param src The argument to convert.
 */
int swap_bytes_if_big_endian_ushort(unsigned short src);

/** Extract an int from the specified byte sequence.
 *  This will swap the order of the bytes if this machine is big endian.
 *  @param bytes The address of the start of the sequence of bytes.
 */
int extract_int(unsigned char* bytes);

/** Extract a long long from the specified byte sequence.
 *  This will swap the order of the bytes if this machine is big endian.
 *  @param bytes The address of the start of the sequence of bytes.
 */
long long extract_ll(unsigned char* bytes);

/** Extract an unsigned short from the specified byte sequence.
 *  This will swap the order of the bytes if this machine is big endian.
 *  @param bytes The address of the start of the sequence of bytes.
 */
unsigned short extract_ushort(unsigned char* bytes);

/** Extract the core header information that all messages between
 *  federates share. The core header information is two bytes with
 *  the ID of the destination port, two bytes with the ID of the destination
 *  federate, and four bytes with the length of the message.
 *  @param buffer The buffer to read from.
 *  @param port_id The place to put the port ID.
 *  @param federate_id The place to put the federate ID.
 *  @param length The place to put the length.
 */
void extract_header(
        unsigned char* buffer,
        unsigned short* port_id,
        unsigned short* federate_id,
        unsigned int* length
);

#endif /* UTIL_H */
