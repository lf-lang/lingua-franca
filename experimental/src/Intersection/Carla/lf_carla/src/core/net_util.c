/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
 * @author Soroush Bateni (soroush@utdallas.edu)
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
 * Utility functions for a federate in a federated execution.
 */

#include "util.h"
#include "net_util.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     // Defines read(), write(), and close()
#include <assert.h>
#include <string.h>     // Defines memcpy()
#include <stdarg.h>     // Defines va_list
#include <time.h>       // Defines nanosleep()
#include <math.h>       // For sqrtl() and powl

#ifndef NUMBER_OF_FEDERATES
#define NUMBER_OF_FEDERATES 1
#endif

/** Number of nanoseconds to sleep before retrying a socket read. */
#define SOCKET_READ_RETRY_INTERVAL 1000000

/** Return true (1) if the host is big endian. Otherwise,
 *  return false.
 */
int host_is_big_endian() {
    static int host = 0;
    union {
        int uint;
        unsigned char c[4];
    } x;
    if (host == 0) {
        // Determine the endianness of the host by setting the low-order bit.
        x.uint = 0x01;
        host = (x.c[3] == 0x01) ? HOST_BIG_ENDIAN : HOST_LITTLE_ENDIAN;
    }
    return (host == HOST_BIG_ENDIAN);
}

/**
 * Read the specified number of bytes from the specified socket into the
 * specified buffer. If an error or an EOF occurs during this
 * reading, then if format is non-null, close the socket,
 * report an error and exit.
 * If format is NULL, then just return 0 for EOF and a negative number
 * for any other error.
 *
 * This function takes a formatted
 * string and additional optional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 *
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
		char* format, ...) {
    va_list args;
	if (socket < 0 && format != NULL) {
		error_print("Socket is no longer open.");
        error_print_and_exit(format, args);
	}
    int bytes_read = 0;
    while (bytes_read < num_bytes) {
        int more = read(socket, buffer + bytes_read, num_bytes - bytes_read);
        if(more <= 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // The error code set by the socket indicates
            // that we should try again (@see man errno).
            DEBUG_PRINT("Reading from socket was blocked. Will try again.");
            continue;
        } else if (more <= 0) {
            if (format != NULL) {
                shutdown(socket, SHUT_RDWR);
                close(socket);
                error_print("Read %d bytes, but expected %d.",
                		more + bytes_read, num_bytes);
                error_print_and_exit(format, args);
            } else if (more == 0) {
                // According to this: https://stackoverflow.com/questions/4160347/close-vs-shutdown-socket,
            	// upon receiving a zero length packet or an error, we can close the socket.
            	// If there are any pending outgoing messages, this will attempt to send those
            	// followed by an EOF.
            	close(socket);
            }
            return more;
        }
        bytes_read += more;
    }
    return bytes_read;
}

/**
 * Read the specified number of bytes from the specified socket into the
 * specified buffer. If a disconnect occurs during this
 * reading, return a negative number. If an EOF occurs during this
 * reading, return 0. Otherwise, return the number of bytes read.
 * This is a version of read_from_socket_errexit() that neither
 * closes the socket nor errors out.
 *
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to read.
 * @param buffer The buffer into which to put the bytes.
 * @return The number of bytes read or 0 when EOF is received or negative for an error.
 */
int read_from_socket(int socket, int num_bytes, unsigned char* buffer) {
    return read_from_socket_errexit(socket, num_bytes, buffer, NULL);
}

/**
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If an error or an EOF occurs during this
 * reading, then if the format string is non-null, close the socket,
 * report an error, and exit. If the format string is null,
 * report an error or EOF and return.
 *
 * This function takes a formatted
 * string and additional optional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 *
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
		char* format, ...) {
    int bytes_written = 0;
    va_list args;
    while (bytes_written < num_bytes) {
        int more = write(socket, buffer + bytes_written, num_bytes - bytes_written);
        if (more <= 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    // The error code set by the socket indicates
                    // that we should try again (@see man errno).
            DEBUG_PRINT("Writing to socket was blocked. Will try again.");
            continue;
        } else if (more <= 0) {
            if (format != NULL) {
                shutdown(socket, SHUT_RDWR);
            	close(socket);
            	if (mutex != NULL) {
            		lf_mutex_unlock(mutex);
            	}
                error_print(format, args);
                error_print_and_exit("Code %d: %s.", errno, strerror(errno));
            }
            return more;
        }
        bytes_written += more;
    }
    return bytes_written;
}

/**
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If an error or an EOF occurs during this
 * reading, then if the format string is non-null, close the socket,
 * report an error, and exit. If the format string is null,
 * report an error or EOF and return.
 *
 * This function takes a formatted
 * string and additional optional arguments similar to printf(format, ...)
 * that is appended to the error messages.
 *
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to write.
 * @param buffer The buffer from which to get the bytes.
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
		char* format, ...) {
	return write_to_socket_errexit_with_mutex(socket, num_bytes, buffer, NULL, format);
}

/**
 * Write the specified number of bytes to the specified socket from the
 * specified buffer. If a disconnect or an EOF occurs during this
 * reading, return a negative number or 0 respectively. Otherwise,
 * return the number of bytes written.
 * This is a version of write_to_socket_errexit() that neither closes
 * the socket nor errors out.
 *
 * @param socket The socket ID.
 * @param num_bytes The number of bytes to write.
 * @param buffer The buffer from which to get the bytes.
 * @return The number of bytes written, or 0 if an EOF was received, or a negative
 *  number if an error occurred.
 */
int write_to_socket(int socket, int num_bytes, unsigned char* buffer) {
    return write_to_socket_errexit_with_mutex(socket, num_bytes, buffer, NULL, NULL);
}

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first).
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_ll(long long data, unsigned char* buffer) {
    // This strategy is fairly brute force, but it avoids potential
    // alignment problems.
    int shift = 0;
    for(size_t i = 0; i < sizeof(long long); i++) {
        buffer[i] = (data & (0xffLL << shift)) >> shift;
        shift += 8;
    }
}

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first). This works for either int or
 *  unsigned int.
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_int(int data, unsigned char* buffer) {
    // This strategy is fairly brute force, but it avoids potential
    // alignment problems.  Note that this assumes an int is four bytes.
    buffer[0] = data & 0xff;
    buffer[1] = (data & 0xff00) >> 8;
    buffer[2] = (data & 0xff0000) >> 16;
    buffer[3] = (data & 0xff000000) >> 24;
}

/** Write the specified data as a sequence of bytes starting
 *  at the specified address. This encodes the data in little-endian
 *  order (lowest order byte first).
 *  @param data The data to write.
 *  @param buffer The location to start writing.
 */
void encode_ushort(unsigned short data, unsigned char* buffer) {
    // This strategy is fairly brute force, but it avoids potential
    // alignment problems. Note that this assumes a short is two bytes.
    buffer[0] = data & 0xff;
    buffer[1] = (data & 0xff00) >> 8;
}

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
int swap_bytes_if_big_endian_int(int src) {
    union {
        int uint;
        unsigned char c[4];
    } x;
    if (!host_is_big_endian()) return src;
    // printf("DEBUG: Host is little endian.\n");
    x.uint = src;
    // printf("DEBUG: Before swapping bytes: %lld.\n", x.ull);
    unsigned char c;
    // Swap bytes.
    c = x.c[0]; x.c[0] = x.c[3]; x.c[3] = c;
    c = x.c[1]; x.c[1] = x.c[2]; x.c[2] = c;
    // printf("DEBUG: After swapping bytes: %lld.\n", x.ull);
    return x.uint;
}

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
long long swap_bytes_if_big_endian_ll(long long src) {
    union {
        long long ull;
        unsigned char c[8];
    } x;
    if (!host_is_big_endian()) return src;
    // printf("DEBUG: Host is little endian.\n");
    x.ull = src;
    // printf("DEBUG: Before swapping bytes: %lld.\n", x.ull);
    unsigned char c;
    // Swap bytes.
    c = x.c[0]; x.c[0] = x.c[7]; x.c[7] = c;
    c = x.c[1]; x.c[1] = x.c[6]; x.c[6] = c;
    c = x.c[2]; x.c[2] = x.c[5]; x.c[5] = c;
    c = x.c[3]; x.c[3] = x.c[4]; x.c[4] = c;
    // printf("DEBUG: After swapping bytes: %lld.\n", x.ull);
    return x.ull;
}

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
int swap_bytes_if_big_endian_ushort(unsigned short src) {
    union {
        unsigned short uint;
        unsigned char c[2];
    } x;
    if (!host_is_big_endian()) return src;
    // printf("DEBUG: Host is little endian.\n");
    x.uint = src;
    // printf("DEBUG: Before swapping bytes: %lld.\n", x.ull);
    unsigned char c;
    // Swap bytes.
    c = x.c[0]; x.c[0] = x.c[1]; x.c[1] = c;
    // printf("DEBUG: After swapping bytes: %lld.\n", x.ull);
    return x.uint;
}

/** Extract an int from the specified byte sequence.
 *  This will swap the order of the bytes if this machine is big endian.
 *  @param bytes The address of the start of the sequence of bytes.
 */
int extract_int(unsigned char* bytes) {
    // Use memcpy to prevent possible alignment problems on some processors.
    union {
        int uint;
        unsigned char c[sizeof(int)];
    } result;
    memcpy(&result.c, bytes, sizeof(int));
    return swap_bytes_if_big_endian_int(result.uint);
}

/** Extract a long long from the specified byte sequence.
 *  This will swap the order of the bytes if this machine is big endian.
 *  @param bytes The address of the start of the sequence of bytes.
 */
long long extract_ll(unsigned char* bytes) {
    // Use memcpy to prevent possible alignment problems on some processors.
    union {
        long long ull;
        unsigned char c[sizeof(long long)];
    } result;
    memcpy(&result.c, bytes, sizeof(long long));
    return swap_bytes_if_big_endian_ll(result.ull);
}

/** Extract an unsigned short from the specified byte sequence.
 *  This will swap the order of the bytes if this machine is big endian.
 *  @param bytes The address of the start of the sequence of bytes.
 */
unsigned short extract_ushort(unsigned char* bytes) {
    // Use memcpy to prevent possible alignment problems on some processors.
    union {
        unsigned short ushort;
        unsigned char c[sizeof(unsigned short)];
    } result;
    memcpy(&result.c, bytes, sizeof(unsigned short));
    return swap_bytes_if_big_endian_ushort(result.ushort);
}

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
) {
    // The first two bytes are the ID of the destination reactor.
    *port_id = extract_ushort(buffer);
    
    // The next two bytes are the ID of the destination federate.
    *federate_id = extract_ushort(&(buffer[sizeof(unsigned short)]));

    // printf("DEBUG: Message for port %d of federate %d.\n", *port_id, *federate_id);
    // FIXME: Better error handling needed here.
    assert(*federate_id < NUMBER_OF_FEDERATES);
    // The next four bytes are the message length.
    *length = extract_int(&(buffer[sizeof(unsigned short) + sizeof(unsigned short)]));

    // printf("DEBUG: Federate receiving message to port %d to federate %d of length %d.\n", port_id, federate_id, length);
}

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
) {
	extract_header(buffer, port_id, federate_id, length);

    tag->time = extract_ll(&(buffer[sizeof(ushort) + sizeof(ushort) + sizeof(int)]));
    tag->microstep = extract_int(&(buffer[sizeof(ushort) + sizeof(ushort) + sizeof(int) + sizeof(instant_t)]));
}
