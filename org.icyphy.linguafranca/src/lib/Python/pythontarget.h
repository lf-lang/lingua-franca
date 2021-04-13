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


////////////// Global variables ///////////////
// The global Python object that holds the .py module that the
// C runtime interacts with
PyObject *globalPythonModule = NULL;

// The dictionary of the Python module that is used to load
// class objects from
PyObject *globalPythonModuleDict = NULL;


/**
 * The struct used to instantiate a port
 * in Lingua Franca. This template is used 
 * in the PythonGenerator instead of redefining
 * a struct for each port.
 * This template can be used for any Python object,
 * including lists and tuples.
 * PyObject* value: the value of the port with the generic Python type
 * is_present: indicates if the value of the port is present
 *             at the current logical time
 * num_destinations: used for reference counting the number of
 *                   connections to destinations.
 **/
typedef struct {
    PyObject* value;
    bool is_present;
    int num_destinations;
} generic_port_instance_struct;

/**
 * The struct used to represent ports in Python 
 * This template is used as a blueprint to create
 * Python objects that follow the same structure.
 * The resulting Python object will have the type 
 * port_capsule_t in C (LinguaFranca.port_capsule in Python).
 * 
 * port: A PyCapsule (https://docs.python.org/3/c-api/capsule.html)
 *       that safely holds a C void* inside a Python object. This capsule
 *       is passed through the Python code and is extracted in C functions
 *       like set and __getitem__.
 * value: The value of the port at the time of invocation of @see convert_C_port_to_py.
 *        The value and is_present are copied from the port if it is not a multiport and can be accessed as
 *        port.value. For multiports, is_present will be false and value will be None. The value of each individual
 *        port can be accessed as port[idx].value (@see port_capsule_get_item). 
 *        Subsequent calls to set will also need to update the value and is_present fields so that they are reflected
 *        in Python code.
 * is_present: Indicates if the value of the singular port is present
 *             at the current logical time
 * width: Indicates the width of the multiport. This is set to -2 for non-multiports.
 * current_index: Used to facilitate iterative functions (@see port_iter)
 **/
typedef struct {
    PyObject_HEAD
    PyObject* port;
    PyObject* value;
    bool is_present;
    int width;
    int current_index;
} generic_port_capsule_struct;

/**
 * Special version of the template_input_output_port_struct
 * for dynamic arrays.
 * FIXME: This is currently kept for compatibility reasons
 * and should be removed soon.
 **/
typedef struct {
    PyObject_HEAD
    PyObject* value;
    bool is_present;
    int num_destinations;
    lf_token_t* token;
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
 * LinguaFranca.port_capsule
 * 
 * Each LinguaFranca.port_capsule includes a void* pointer of 
 * the C port (a.k.a. generic_port_instance_struct*).
 * @see generic_port_capsule_struct in pythontarget.h
 * 
 * This function calls the underlying _LF_SET API.
 * @see xtext/org.icyphy.linguafranca/src/lib/core/reactor.h
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
 * appropriately handling types on the receiving end of this port.
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

/**
 * Stop execution at the conclusion of the current logical time.
 */
static PyObject* py_stop(PyObject *self);

//////////////////////////////////////////////////////////////
///////////// Main function callable from Python code
static PyObject* py_main(PyObject *self, PyObject *args);

//////////////////////////////////////////////////////////////
/////////////  Python Helper Functions

/**
 * A function that is called any time a Python reaction is called with
 * ports as inputs and outputs. This function converts ports that are
 * either a multiport or a non-multiport into a port_capsule.
 * 
 * First, the void* pointer is stored in a PyCapsule. If the port is not
 * a multiport, the value and is_present fields are copied verbatim. These
 * feilds then can be accessed from the Python code as port.value and
 * port.is_present.
 * If the value is absent, it will be set to None.
 * 
 * For multiports, the value of the port_capsule (i.e., port.value) is always
 * set to None and is_present is set to false.
 * Individual ports can then later be accessed in Python code as port[idx].
 */
PyObject* convert_C_port_to_py(void* port, int width);

/**
 * A helper function to convert C actions to Python action capsules
 * @see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend for details about C actions
 * Python actions have the following fields (for more information @see generic_action_capsule_struct):
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
get_python_function(string module, string class, int instance_id, string func);



/*
 * The Python runtime will call this function to initialize the module.
 * The name of this function is dynamically generated to follow
 * the requirement of PyInit_MODULE_NAME. Since the MODULE_NAME is not
 * known prior to compile time, the GEN_NAME macro is used.
 * The generated function will have the name PyInit_MODULE_NAME.
 * For example for a module named LinguaFrancaFoo, this function
 * will be called PyInit_LinguaFrancaFoo
 */
PyMODINIT_FUNC
GEN_NAME(PyInit_,MODULE_NAME)(void);

#endif // PYTHON_TARGET_H
