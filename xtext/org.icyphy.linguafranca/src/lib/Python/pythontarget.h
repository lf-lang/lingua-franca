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
 * Target-specific runtime functions for the Python target language.
 * This API layer can be used in conjunction with:
 *     target Python;
 * 
 * Note for target language developers. This is one way of developing a target language where 
 * the C core runtime is adopted. This file is a translation layer that implements Lingua Franca 
 * APIs which interact with the internal _lf_SET and _lf_schedule APIs. This file can act as a 
 * template for future runtime developement for target languages.
 * For source generation, see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/PythonGenerator.xtend.
 */

#ifndef PYTHON_TARGET_H
#define PYTHON_TARGET_H


#define PY_SSIZE_T_CLEAN

#include <Python.h>
#include <structmember.h>
#include "ctarget.h"

#ifdef _MSC_VER
#include <../include/limits.h>
#include <windows.h>
#ifndef PATH_MAX
#define PATH_MAX MAX_PATH
#endif
#else
#include <limits.h>
#endif

// MODULE_NAME is expected to be defined in the main file of the generated code
#ifndef MODULE_NAME
#error "MODULE_NAME is undefined"
#endif

#define CONCAT(x,y) x##y
#define GEN_NAME(x,y) CONCAT(x,y)
#define STRINGIFY(X) #X
#define TOSTRING(x) STRINGIFY(x)

/**
 * The struct used to instantiate a port
 * in Lingua Franca. This template is used 
 * in the PythonGenerator instead of redefining
 * a struct for each port.
 * This template can be used for both primitive types
 * and statically allocated arrays (e.g., int x[3];).
 * T value: the value of the port with type T
 * is_present: indicates if the value of the port is present
 *     at the current logcal time
 * num_destinations: used for reference counting the number of
 *     connections to destinations.
 **/
typedef struct {
    PyObject_HEAD
    PyObject* value;
    bool is_present;
    int num_destinations;
} generic_port_instance_struct;

/**
 * Special version of the template_input_output_port_struct
 * for dynamic arrays
 **/
typedef struct {
    PyObject_HEAD
    PyObject* value;
    bool is_present;
    int num_destinations;
    token_t* token;
    int length;
} generic_port_instance_with_token_struct;


/**
 * The struct used to hold an action
 * that is sent to a Python reaction.
 * 
 * The "action" field holds a PyCapsule of the
 * void * pointer to an action.
 * 
 * The "value" field holds the action value
 * if anything is given. This value is copied over
 * from action->value each time an action is passed
 * to a Python reaction.
 * 
 * The "is_present" field is copied over
 * from action->value each time an action is passed
 * to a Python reaction.
 **/
typedef struct {
    PyObject_HEAD
    PyObject* action; // Hold the void* pointer to a C action instance. However, passing void* directly
                      // to Python is considered unsafe practice. Instead, this void* pointer to the C action
                      // will be stored in a PyCapsule. @see https://docs.python.org/3/c-api/capsule.html
    PyObject* value; // This value will be copied from the C action->value
    bool is_present; // Same as value, is_present will be copied from the C action->is_present
} generic_action_capsule_struct;

//////////////////////////////////////////////////////////////
/////////////  SET Functions (to produce an output)

/**
 * Set the value and is_present field of self which is of type
 * LinguaFranca.port_instance (a.k.a. generic_port_instance_struct*).
 * 
 * This function can be used to set any type of PyObject ranging from
 * primitive types to complex lists and tuples. Moreover, this function
 * is callable from Python target code by using port_name.out(value)
 * 
 * Some examples include
 *  port_name.out("Hello")
 *  port_name.out(5)
 *  port_name.out(["Hello", 5 , (2.8, "X")])
 * 
 * The port type given in the Lingua Franca is only used as a "suggestion"
 * as per Python's duck typing principles. The end-user is responsible for
 * appropriately handling types on the recieveing end of this port.
 * @param self The output port (by name) or input of a contained
 *                 reactor in form input_name.port_name.
 * @param args contains:
 *      @param val The value to insert into the port struct.
 */
static PyObject* py_SET(PyObject *self, PyObject *args);

//////////////////////////////////////////////////////////////
/////////////  schedule Functions (to schedule an action)
/**
 * Schedule an action to occur with the specified value and time offset
 * with no payload (no value conveyed).
 * See schedule_token(), which this uses, for details.
 * @param self Pointer to the calling object.
 * @param args contains:
 *      @param action Pointer to an action on the self struct.
 *      @param offset The time offset over and above that in the action.
 **/
static PyObject* py_schedule(PyObject *self, PyObject *args);

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
static PyObject* py_schedule_copy(PyObject *self, PyObject *args);


//////////////////////////////////////////////////////////////
/////////////  Python Helper Functions (called from Python code)
/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_elapsed_logical_time(PyObject *self, PyObject *args);

/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_logical_time(PyObject *self, PyObject *args);

/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_physical_time(PyObject *self, PyObject *args);

/** 
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_elapsed_physical_time();
static PyObject* py_get_elapsed_physical_time(PyObject *self, PyObject *args);



//////////////////////////////////////////////////////////////
///////////// Main function callable from Python code
static PyObject* py_main(PyObject *self, PyObject *args);

//////////////////////////////////////////////////////////////
/////////////  Python Helper Functions

/**
 * A helper function to convert C actions to Python action capsules
 * @see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend for details about C actions
 * Python actions have the following fields (for more informatino @see generic_action_capsule_struct):
 *   PyObject_HEAD
 *   PyObject* action;   
 *   PyObject* value; 
 *   bool is_present; 
 *   
 * The input to this function is a pointer to a C action, which might or 
 * might not contain a value and an is_present field. To simplify the assumptions
 * made by this function, the "value" and "is_present" are passed to the function
 * instead of expecting them to exist.
 * 
 * The void* pointer to the C action instance is encapsulated in a PyCapsule instead of passing an exposed pointer through
 * Python. @see https://docs.python.org/3/c-api/capsule.html
 * This encapsulation is done by calling PyCapsule_New(action, "name_of_the_container_in_the_capsule", NULL), 
 * where "name_of_the_container_in_the_capsule" is an agreed-upon container name inside the capsule. This 
 * capsule can then be treated as a PyObject* and safely passed through Python code. On the other end 
 * (which is in schedule functions), PyCapsule_GetPointer(recieved_action,"action") can be called to retrieve 
 * the void* pointer into recieved_action.
 **/
PyObject* convert_C_action_to_py(void* action);

/**
 * A helper function to generate a mutable list of input ports to be sent 
 * to a Python reaction. Therefore, an array port_array[3] is converted to a 
 * Python list [port_array[0], port_array[1], port_array[2]].
 * 
 * This function recieves an array of ports of type port_instance_t in "port_array" with the given "size".
 * The ports in port_array will follow the template laid out in @see generic_port_instance_struct.
 * The port_array is structured as follows:
 *  generic_port_instance_struct* * *
 *                              ↑ ↑ ↑
 *                              Heap-allocated port that follows the generic_port_instance_struct template
 *                                | |
 *                                An array of ports in generic_port_instance_struct*[] format
 *                                  |
 *                                  Passed by reference
 * A Python list (which is mutable) of size "size" is created by calling PyList_New(size)
 * and each member in the generic_port_instance_struct*[] is added to this newly created list
 * by calling PyList_SET_ITEM(list, position, *input_by_reference).
 * 
 * @param port_array An array of input ports passed by reference
 * @param size size of the port_array
 * @return list A Python list of the input ports
 */
PyObject* make_input_port_list(generic_port_instance_struct*** port_array, Py_ssize_t size);

/**
 * A helper function to generate a mutable list of output ports to be sent 
 * to a Python reaction. Therefore, an array port_array[3] is converted to a 
 * Python list [port_array[0], port_array[1], port_array[2]].
 * 
 * This function recieves an array of ports of type port_instance_t in "port_array" with the given "size".
 * The ports in port_array will follow the template laid out in @see generic_port_instance_struct.
 * The port_array is structured as follows:
 *  generic_port_instance_struct* * 
 *                              ↑ ↑ 
 *                              Heap-allocated port that follows the generic_port_instance_struct template
 *                                | |
 *                                An array of ports in generic_port_instance_struct*[] format
 * 
 * A Python list (which is mutable) of size "size" is created by calling PyList_New(size)
 * and each member in the generic_port_instance_struct*[] is added to this newly created list
 * by calling PyList_SET_ITEM(list, position, input).
 * 
 * @param port_array An array of output ports passed by reference
 * @param size size of the port_array
 * @return list A Python list of the output ports
 */
PyObject* make_output_port_list(generic_port_instance_struct** port_array, Py_ssize_t size);

/**
 * A helper function to generate an immutable tuple of input ports to be sent 
 * to a Python reaction. Therefore, an array port_array[3] is converted to a 
 * Python tuple (port_array[0], port_array[1], port_array[2]). The contents of this tuple
 * cannot change at runtime.
 * 
 * This function recieves an array of ports of type port_instance_t in "port_array" with the given "size".
 * The ports in port_array will follow the template laid out in @see generic_port_instance_struct.
 * The port_array is structured as follows:
 *  generic_port_instance_struct* * *
 *                              ↑ ↑ ↑
 *                              Heap-allocated port that follows the generic_port_instance_struct template
 *                                | |
 *                                An array of ports in generic_port_instance_struct*[] format
 *                                  |
 *                                  Passed by reference
 * A Python tuple (which is mutable) of size "size" is created by calling PyTuple_New(size)
 * and each member in the generic_port_instance_struct*[] is added to this newly created tuple
 * by calling PyTuple_SET_ITEM(tuple, position, *input_by_reference).
 * 
 * @param port_array An array of input ports passed by reference
 * @param size size of the port_array
 * @return tuple A Python tuple of the input ports
 */
PyObject* make_input_port_tuple(generic_port_instance_struct*** port_array, Py_ssize_t size);

/**
 * A helper function to generate a mutable tuple of output ports to be sent 
 * to a Python reaction. Therefore, an array port_array[3] is converted to a 
 * Python tuple (port_array[0], port_array[1], port_array[2]). The contents of this tuple
 * cannot change at runtime.
 * 
 * This function recieves an array of ports of type port_instance_t in "port_array" with the given "size".
 * The ports in port_array will follow the template laid out in @see generic_port_instance_struct.
 * The port_array is structured as follows:
 *  generic_port_instance_struct* * 
 *                              ↑ ↑ 
 *                              Heap-allocated port that follows the generic_port_instance_struct template
 *                                | |
 *                                An array of ports in generic_port_instance_struct*[] format
 * 
 * A Python tuple (which is mutable) of size "size" is created by calling PyTuple_New(size)
 * and each member in the generic_port_instance_struct*[] is added to this newly created tuple
 * by calling PyTuple_SET_ITEM(tuple, position, input).
 * 
 * @param port_array An array of output ports passed by reference
 * @param size size of the port_array
 * @return tuple A Python tuple of the output ports
 */
PyObject* make_output_port_tuple(generic_port_instance_struct** port_array, Py_ssize_t size);

/** 
 * Invoke a Python func in class[instance_id] from module.
 * Class instances in generated Python code are always instantiated in a 
 * list of template classs[_class(params), _class(params), ...] (note the extra s) regardless 
 * of whether a bank is used or not. If there is no bank, or a bank of width 1, the list will be
 * instantiated as classs[_class(params)].
 * 
 * This function would thus call classs[0] to access the first instance in a bank and so on.
 * 
 * Possible optimizations include: - Not loading the module each time (by storing it in global memory),
 *                                 - Keeping a persistent argument table
 * @param module The Python module to load the function from. In embedded mode, it should
 *               be set to "__main__"
 * @param class The name of the list of classes in the generated Python code
 * @param instance_id The element number in the list of classes. class[instance_id] points to a class instance
 * @param func The reaction functino to be called
 * @param pArgs the PyList of arguments to be sent to function func()
 */
PyObject*
invoke_python_function(string module, string class, int instance_id, string func, PyObject* pArgs);



/*
 * The Python runtime will call this to initialize the module.
 * The name of this function is dynamically generated to follow
 * the requirement of PyInit_MODULE_NAME. Since the MODULE_NAME is not
 * known prior to compile time, the GEN_NAME macro is used.
 */
PyMODINIT_FUNC
GEN_NAME(PyInit_,MODULE_NAME)(void);

#endif // PYTHON_TARGET_H