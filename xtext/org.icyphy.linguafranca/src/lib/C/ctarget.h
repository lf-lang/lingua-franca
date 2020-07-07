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
 * Target-specific runtime functions for the C target language.
 * This API layer can be used in conjunction with:
 *     target C;
 * 
 * Note for target language developers. Every target language has a runtime translation
 * layer that works with the common core, as well as a source generator. 
 * This file should act as a template for future runtime developement for target languages.
 * For source generation, see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend.
 */


#ifndef CTARGET_H
#define CTARGET_H

#include "core/reactor.h"
#include "core/pqueue.c" // FIXME: Ideally this should be hidden

////////////////////////////////////////////////////////////
//// Macros for producing outputs.

// NOTE: According to the "Swallowing the Semicolon" section on this page:
//    https://gcc.gnu.org/onlinedocs/gcc-3.0.1/cpp_3.html
// the following macros should use an odd do-while construct to avoid
// problems with if ... else statements that do not use braces around the
// two branches.

/**
 * Set the specified output (or input of a contained reactor)
 * to the specified value.
 *
 * This version is used for primitive types such as int,
 * double, etc. as well as the built-in types bool and string.
 * The value is copied and therefore the variable carrying the
 * value can be subsequently modified without changing the output.
 * This can also be used for structs with a type defined by a typedef
 * so that the type designating string does not end in '*'.
 * @param out The output port (by name) or input of a contained
 *  reactor in form input_name.port_name.
 * @param value The value to insert into the self struct.
 */
#define SET(out, val) __LF_SET(out, val)

/**
 * Version of set for output types given as 'type[]' where you
 * want to send a previously dynamically allocated array.
 *
 * The deallocation is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * It also sets the corresponding _is_present variable in the self
 * struct to true (which causes the object message to be sent).
 * @param out The output port (by name).
 * @param val The array to send (a pointer to the first element).
 * @param length The length of the array to send.
 * @see token_t
 */
#define SET_ARRAY(out, val, element_size, length) __LF_SET_ARRAY(out, val, element_size, length)

/**
 * Version of set() for output types given as 'type*' that
 * allocates a new object of the type of the specified output port.
 *
 * This macro dynamically allocates enough memory to contain one
 * instance of the output datatype and sets the variable named
 * by the argument to point to the newly allocated memory.
 * The user code can then populate it with whatever value it
 * wishes to send.
 *
 * This macro also sets the corresponding _is_present variable in the self
 * struct to true (which causes the object message to be sent),
 * @param out The output port (by name).
 */
#define SET_NEW(out) __LF_SET_NEW(out)

/**
 * Version of set() for output types given as 'type[]'.
 *
 * This allocates a new array of the specified length,
 * sets the corresponding _is_present variable in the self struct to true
 * (which causes the array message to be sent), and sets the variable
 * given by the first argument to point to the new array so that the
 * user code can populate the array. The freeing of the dynamically
 * allocated array will be handled automatically
 * when the last downstream reader of the message has finished.
 * @param out The output port (by name).
 * @param length The length of the array to be sent.
 */
#define SET_NEW_ARRAY(out, length) __LF_SET_NEW_ARRAY(out, length)

/**
 * Version of set() for output types given as 'type[number]'.
 *
 * This sets the _is_present variable corresponding to the specified output
 * to true (which causes the array message to be sent). The values in the
 * output are normally written directly to the array or struct before or
 * after this is called.
 * @param out The output port (by name).
 */
#define SET_PRESENT(out) __LF_SET_PRESENT(out)

/**
 * Version of set() for output types given as 'type*' or 'type[]' where you want
 * to forward an input or action without copying it.
 *
 * The deallocation of memory is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * @param out The output port (by name).
 * @param token A pointer to token obtained from an input or action.
 */
#define SET_TOKEN(out, newtoken) __LF_SET_TOKEN(out, newtoken)

#endif // CTARGET_H
