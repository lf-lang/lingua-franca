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
 * Header file for network utility functions for Lingua Franca programs.
 * Note that these functions do not acquire any mutexes. To use them,
 * you must ensure either that only one thread ever sends on each socket
 * and one thread receives on each socket (these two can be the same thread)
 * or that the caller handles mutual exclusion to prevent more than one thread
 * from accessing the socket at a time.
 */

#ifndef NET_UTIL_H
#define NET_UTIL_H

#include "platform.h"  // defines lf_mutex_t
#include "tag.h"       // Defines tag_t

#define HOST_LITTLE_ENDIAN 1
#define HOST_BIG_ENDIAN 2

/** Return true (1) if the host is big endian. Otherwise,
 *  return false.
 */
int host_is_big_endian();

/**
 * Read the specified number of bytes from the specified socket into the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, then if format is non-null, report an error and exit.
 * If format is null, then report the error, but do not exit.
 * This function takes a formatted
 * string and additional optional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to read.
 * @param buffer The buffer into which to put the bytes.
 * @param format A printf-style format string, followed by arguments to
 *  fill the string, or NULL to not exit with an error message.
 * @return The number of bytes read, or 0 if an EOF is received, or
 *  a negative number for an error.
 */
int read_from_socket_errexit(
		int socket,
		int num_bytes,
		unsigned char* buffer,
		char* format, ...);

/**
 * Read the specified number of bytes from the specified socket into the
 * specified buffer. If a disconnect occurs during this
 * reading, return a negative number. If an EOF occurs during this
 * reading, return 0. Otherwise, return the number of bytes read.
 * This is a version of read_from_socket_errexit() that does not error out.
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to read.
 * @param buffer The buffer into which to put the bytes.
 * @return The number of bytes read or 0 when EOF is received or negative for an error.
 */
int read_from_socket(int socket, int num_bytes, unsigned char* buffer);

/**
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, report an error and exit, unless the format string is NULL,
 * in which case, report an error and return. This function takes a formatted
 * string and additional optional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to write.
 * @param buffer The buffer from which to get the bytes.
 * @param mutex If non-NULL, the mutex to unlock before exiting.
 * @param format A format string for error messages, followed by any number of
 *  fields that will be used to fill the format string as in printf, or NULL
 *  to prevent exit on error.
 * @return The number of bytes written, or 0 if an EOF was received, or a negative
 *  number if an error occurred.
 */
int write_to_socket_errexit_with_mutex(
		int socket,
		int num_bytes,
		unsigned char* buffer,
		lf_mutex_t* mutex,
		char* format, ...);

/**
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, report an error and exit, unless the format string is NULL,
 * in which case, report an error and return. This function takes a formatted
 * string and additional optional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to write.
 * @param buffer The buffer from which to get the bytes.
 * @param mutex If non-NULL, the mutex to unlock before exiting.
 * @param format A format string for error messages, followed by any number of
 *  fields that will be used to fill the format string as in printf, or NULL
 *  to prevent exit on error.
 * @return The number of bytes written, or 0 if an EOF was received, or a negative
 *  number if an error occurred.
 */
int write_to_socket_errexit(
		int socket,
		int num_bytes,
		unsigned char* buffer,
		char* format, ...);

/**
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, return a negative number or 0 respectively. Otherwise,
 * return the number of bytes written.
 * This is a version of write_to_socket() that does not error out.
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to write.
 * @param buffer The buffer from which to get the bytes.
 * @return The number of bytes written, or 0 if an EOF was received, or a negative
 *  number if an error occurred.
 */
int write_to_socket2(int socket, int num_bytes, unsigned char* buffer);

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first).
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_ll(long long data, unsigned char* buffer);

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first). This works for either int or
 *  unsigned int.
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

/**
 * Extract the core header information that all messages between
 * federates share. The core header information is two bytes with
 * the ID of the destination port, two bytes with the ID of the destination
 * federate, and four bytes with the length of the message.
 * @param buffer The buffer to read from.
 * @param port_id The place to put the port ID.
 * @param federate_id The place to put the federate ID.
 * @param length The place to put the length.
 */
void extract_header(
        unsigned char* buffer,
        unsigned short* port_id,
        unsigned short* federate_id,
        unsigned int* length
);

/**
 * Extract the timed header information for timed messages between
 * federates. This is two bytes with the ID of the destination port,
 * two bytes with the ID of the destination
 * federate, four bytes with the length of the message,
 * eight bytes with a timestamp, and four bytes with a microstep.
 * @param buffer The buffer to read from.
 * @param port_id The place to put the port ID.
 * @param federate_id The place to put the federate ID.
 * @param length The place to put the length.
 * @param tag The place to put the tag.
 */
void extract_timed_header(
        unsigned char* buffer,
        unsigned short* port_id,
        unsigned short* federate_id,
        unsigned int* length,
		tag_t* tag
);

#endif /* NET_UTIL_H */
