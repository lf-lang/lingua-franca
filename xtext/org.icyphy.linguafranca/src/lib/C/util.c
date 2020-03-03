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
 * Utility functions for a federate in a federated execution.
 */

/** Print the error defined by the errno variable with the
 *  specified message as a prefix, then exit with error code 1.
 *  @param msg The prefix to the message.
 */
void error(char *msg) {
    perror(msg);
    exit(1);
}

#define HOST_LITTLE_ENDIAN 1
#define HOST_BIG_ENDIAN 2

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
int swap_bytes_if_little_endian_int(int src) {
    union {
        int uint;
        unsigned char c[4];
    } x;
    if (host_is_big_endian()) return src;
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
long long swap_bytes_if_little_endian_ll(long long src) {
    union {
        long long ull;
        unsigned char c[8];
    } x;
    if (host_is_big_endian()) return src;
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
