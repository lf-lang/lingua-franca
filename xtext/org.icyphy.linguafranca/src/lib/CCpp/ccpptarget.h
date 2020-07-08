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
 * Target-specific runtime functions for the C++ target language.
 * This API layer can be used in conjunction with:
 *     target CCpp;
 * 
 * Note for target language developers. Every target language has a runtime translation
 * layer that works with the common core, as well as a source generator. 
 * This file should act as a template for future runtime developement for target languages.
 * For source generation, see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CCppGenerator.xtend.
 */

#ifndef CPP_TARGET_H
#define CPP_TARGET_H

#include <iostream>
#include "core/pqueue.c"
#include "core/reactor.h"
#include "core/reactor_threaded.c"

/**
 * A template struct for an instance of each port
 * in Lingua Franca. This template is used 
 * in the CCppGenerator instead of redefining
 * a struct for each port.
 * This template can be used for both primitive types
 * and statically allocated arrays (e.g., int x[3];).
 * T value: the value of the port with type T
 * is_present: indicates if the value of the port is present
 *     at the current logcal time
 * num_destinations: used for reference counting the number of
 *     connections to destinations.
 **/
template <class T>
struct template_port_instance_struct {
    T value;
    bool is_present;
    int num_destinations;
};

/**
 * Special version of the template_input_output_port_struct
 * for dynamic arrays
 **/
template <class T>
struct template_port_instance_with_token_struct {
    T value;
    bool is_present;
    int num_destinations;
    token_t* token;
    int length;
};


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
 * @param val The value to insert into the self struct.
 */
template <class T>
void SET(template_port_instance_struct<T>* out, T val)
{
    __LF_SET(out, static_cast<T>(val));
}


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
template <class T>
void SET(template_port_instance_with_token_struct<T>* out, T val, int element_size, int length)
{
    __LF_SET_ARRAY(out, val, element_size, length);
}

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
template <class T>
void SET(template_port_instance_with_token_struct<T>* out, int length)
{
    __LF_SET_NEW_ARRAY(out, length);
}

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
template <class T>
void SET(template_port_instance_with_token_struct<T>* out)
{
    SET(out, 1);
}

/**
 * Version of set() for output types given as 'type[number]'.
 *
 * This sets the _is_present variable corresponding to the specified output
 * to true (which causes the array message to be sent). The values in the
 * output are normally written directly to the array or struct before or
 * after this is called.
 * @param out The output port (by name).
 */
template <class T>
void SET(template_port_instance_struct<T>* out)
{
    __LF_SET_PRESENT(out);
}

/**
 * Version of set() for output types given as 'type*' or 'type[]' where you want
 * to forward an input or action without copying it.
 *
 * The deallocation of memory is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * @param out The output port (by name).
 * @param token A pointer to token obtained from an input or action.
 */
template <class T>
void SET(template_port_instance_with_token_struct<T>* out, token_t* newtoken)
{
    __LF_SET_TOKEN(out, newtoken);
}

/////////////////////////////////////////////////////
/////////////  Schedule Functions
 /* Note. The schedule functions will also be 
 * optional for C/C++ because of the commonality
 * but need to be reimplemented for other target
 * languages such as Python.
 * /
 
/**
 * Schedule an action to occur with the specified value and time offset
 * with no payload (no value conveyed).
 * See schedule_token(), which this uses, for details.
 * @param action Pointer to an action on the self struct.
 * @param offset The time offset over and above that in the action.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
handle_t schedule(void* action, interval_t offset) {
    return __lf_schedule_token(action, offset, NULL);
}

/**
 * Variant of schedule_value when the value is an integer.
 * See reactor.h for documentation.
 * @param action Pointer to an action on the self struct.
 */
handle_t schedule(void* action, interval_t extra_delay, int value)
{
    return __lf_schedule_int(action, extra_delay, value);
}

/**
 * Schedule the specified trigger at current_time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
handle_t schedule(void* action, interval_t extra_delay, token_t* token) {
    return __lf_schedule_token(action, extra_delay, token);
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
handle_t schedule(void* action, interval_t offset, void* value, int length) {
    return __lf_schedule_copy(action, offset, value, length);
}


/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
handle_t schedule_value(void* action, interval_t extra_delay, void* value, int length) {
    return __lf_schedule_value(action, extra_delay, value, length);
}

/**
 * TODO: Implement GET functions.
 */


/**
 * Compatibility layer with the C runtime (optional).
 * These functions are here to enable universal Lingua Franca programs that
 * can have the shape:
 * 
 *     target Universal;
 *     reactor UniversalReactor {
 *         input in:int;
 *         output out:int;
 *         reaction(in) -> out {=
 *             SET(out, GET(in))  
 *         =} 
 *     }
 */

template <class T>
void SET_ARRAY(template_port_instance_with_token_struct<T>* out, T val, int element_size, int length)
{
    SET(out, val, element_size, length);
}


template <class T>
void SET_NEW(template_port_instance_with_token_struct<T>* out)
{
    SET(out);
}

template <class T>
void SET_NEW_ARRAY(template_port_instance_with_token_struct<T>* out, int length)
{
    SET(out, length);
}

template <class T>
void SET_PRESENT(template_port_instance_struct<T>* out)
{
    SET(out);
}

template <class T>
void SET_TOKEN(template_port_instance_with_token_struct<T>* out, token_t* newtoken)
{
    SET(out, newtoken);
}

/* Schedule compatiblity layer */
handle_t schedule_int(void* action, interval_t extra_delay, int value)
{
    return schedule(action, extra_delay, value);
}

handle_t schedule_token(void* action, interval_t extra_delay, token_t* token) {
    return schedule(action, extra_delay, token);
}

handle_t schedule_copy(void* action, interval_t offset, void* value, int length) {
    return schedule(action, offset, value, length);
}

#endif // CPP_TARGET_H