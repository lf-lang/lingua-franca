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
#define SET(out, val) \
do { \
    out->value = val; \
    out->is_present = true; \
} while(0)

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
#define SET_ARRAY(out, val, element_size, length) \
do { \
    out->is_present = true; \
    token_t* token = __initialize_token_with_value(out->token, val, length); \
    token->ref_count = out->num_destinations; \
    out->token = token; \
    out->value = token->value; \
} while(0)

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
#define SET_NEW(out) \
do { \
    out->is_present = true; \
    token_t* token = __set_new_array_impl(out->token, 1, out->num_destinations); \
    out->value = token->value; \
    out->token = token; \
} while(0)

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
#define SET_NEW_ARRAY(out, length) \
do { \
    out->is_present = true; \
    token_t* token = __set_new_array_impl(out->token, length, out->num_destinations); \
    out->value = token->value; \
    out->token = token; \
} while(0)

/**
 * Version of set() for output types given as 'type[number]'.
 *
 * This sets the _is_present variable corresponding to the specified output
 * to true (which causes the array message to be sent). The values in the
 * output are normally written directly to the array or struct before or
 * after this is called.
 * @param out The output port (by name).
 */
#define SET_PRESENT(out) \
do { \
    out->is_present = true; \
} while(0)

/**
 * Version of set() for output types given as 'type*' or 'type[]' where you want
 * to forward an input or action without copying it.
 *
 * The deallocation of memory is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * @param out The output port (by name).
 * @param token A pointer to token obtained from an input or action.
 */
#define SET_TOKEN(out, newtoken) \
do { \
    out->is_present = true; \
    out->value = newtoken->value; \
    out->token = newtoken; \
    newtoken->ref_count += out->num_destinations; \
    out->is_present = true; \
} while(0)

#endif // CTARGET_H
