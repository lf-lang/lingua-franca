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
 * Header file for the runtime infrastructure for distributed Lingua Franca programs.
 */

#ifndef RTI_H
#define RTI_H

// Size of the buffer used for messages sent between federates.
// This is used by both the federates and the rti, so message lengths
// should generally match.
#define BUFFER_SIZE 256

////////////////////////////////////////////
//// Message types

// These message types will be encoded in an unsigned char,
// so the magnitude must not exceed 255.

/** Byte identifying a federate ID message, which is 32 bits long. */
#define FED_ID 1

/** Byte identifying a timestamp message, which is 64 bits long. */
#define TIMESTAMP 2

/** Byte identifying a message to forward to another federate.
 *  The next two bytes will be the ID of the destination port.
 *  The next two bytes are the destination federate ID.
 *  The four bytes after that will be the length of the message.
 *  The remaining bytes are the message.
 */
#define MESSAGE 3

/** Byte identifying that the federate is ending its execution. */
#define RESIGN 4

/** Byte identifying a timestamped message to forward to another federate.
 *  The next two bytes will be the ID of the destination port.
 *  The next two bytes are the destination federate ID.
 *  The four bytes after that will be the length of the message.
 *  The next eight bytes will be the timestamp.
 *  The remaining bytes are the message.
 */
#define TIMED_MESSAGE 5


#endif /* RTI_H */
