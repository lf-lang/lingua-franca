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
#define MODULE_NAME LinguaFranca_Default
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
 **/
typedef struct {
    PyObject_HEAD
    PyObject* action;
    PyObject* value;
} generic_action_capsule_struct;

//////////////////////////////////////////////////////////////
/////////////  SET Functions (to produce an output)

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
 * @param self Pointer to the calling object.
 * @param args contains: 
 *      @param out The output port (by name) or input of a contained
 *                 reactor in form input_name.port_name.
 *      @param val The value to insert into the port struct.
 */
static PyObject* py_SET(PyObject *self, PyObject *args)
{
    generic_port_instance_struct* port;
    PyObject* val;

    if (!PyArg_ParseTuple(args, "OO" ,&port, &val))
        return NULL;
    
    _LF_SET(port, val);

    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with no payload (no value conveyed).
 * See schedule_token(), which this uses, for details.
 * @param self Pointer to the calling object.
 * @param args contains:
 *      @param action Pointer to an action on the self struct.
 *      @param offset The time offset over and above that in the action.
 **/
static PyObject* py_schedule(PyObject *self, PyObject *args)
{
    generic_action_capsule_struct* act;
    long long offset;

    if (!PyArg_ParseTuple(args, "OL" ,&act, &offset))
        return NULL;
    
    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL)
    {
        fprintf(stderr, "Null pointer recieved.\n");
        exit(1);
    }

    _lf_schedule_token(action, offset, NULL);

    // FIXME: handle is not passed to the Python side

    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Variant of schedule_value when the value is an integer.
 * See reactor.h for documentation.
 * @param action Pointer to an action on the self struct.
 */
static PyObject* py_schedule_int(PyObject *self, PyObject *args)
{
    generic_action_capsule_struct* act;
    long long offset;
    int value;

    if (!PyArg_ParseTuple(args, "OLi" ,&act, &offset, &value))
        return NULL;


    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL)
    {
        fprintf(stderr, "Null pointer recieved.\n");
        exit(1);
    }
    
    _lf_schedule_int(action, offset, value);

    // FIXME: handle is not passed to the Python side

    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
/**
 * Variant of schedule_value when the value is an integer.
 * See reactor.h for documentation.
 * @param action Pointer to an action on the self struct.
 */
static PyObject* py_schedule_value(PyObject *self, PyObject *args)
{
    generic_action_capsule_struct* act;
    long long offset;
    PyObject* value;
    int length;

    if (!PyArg_ParseTuple(args, "OLOi" ,&act, &offset, &value, &length))
        return NULL;

    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL)
    {
        fprintf(stderr, "Null pointer recieved.\n");
        exit(1);
    }
    
    _lf_schedule_value(action, offset, value, length);

    // FIXME: handle is not passed to the Python side

    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
static PyObject* py_schedule_copy(PyObject *self, PyObject *args)
{
    generic_action_capsule_struct* act;
    long long offset;
    PyObject* value;
    int length;

    if (!PyArg_ParseTuple(args, "OLOi" ,&act, &offset, &value, &length))
        return NULL;

    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL)
    {
        fprintf(stderr, "Null pointer recieved.\n");
        exit(1);
    }
    
    _lf_schedule_copy(action, offset, value, length);

    // FIXME: handle is not passed to the Python side

    Py_INCREF(Py_None);
    return Py_None;
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_elapsed_logical_time();
static PyObject* py_get_elapsed_logical_time(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(get_elapsed_logical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_logical_time();
static PyObject* py_get_logical_time(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(get_logical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_physical_time();
static PyObject* py_get_physical_time(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(get_physical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_elapsed_physical_time();
static PyObject* py_get_elapsed_physical_time(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(get_elapsed_physical_time());
}



//////////////////////////////////////////////////////////////
///////////// Main function callable from Python code
static PyObject* py_main(PyObject *self, PyObject *args)
{
    main(1, NULL);

    Py_INCREF(Py_None);
    return Py_None;
}

//////////////////////////////////////////////////////////////
/////////////  Python Helper Structs

/*
 * The members of a port_instance, used to define
 * a native Python type.
 */
static PyMemberDef port_instance_members[] = {
    {"value", T_OBJECT, offsetof(generic_port_instance_struct, value), 0, "Value of the port"},
    {"is_present", T_BOOL, offsetof(generic_port_instance_struct, is_present), 0, "Check if value is present at current logical time"},
    {"num_destinations", T_INT, offsetof(generic_port_instance_struct, num_destinations), 0, "Number of destinations"},
    {NULL}  /* Sentinel */
};

/*
 *
 */
static void
port_instance_dealloc(generic_port_instance_struct *self)
{
    Py_XDECREF(self->value);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

static PyObject *
port_intance_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    generic_port_instance_struct *self;
    self = (generic_port_instance_struct *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->value = NULL;
        self->is_present = false;
        self->num_destinations = 0;
    }
    
    return (PyObject *) self;
}

static int
port_instance_init(generic_port_instance_struct *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"value", "is_present", "num_destinations", NULL};
    PyObject *value = NULL, *tmp;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|Opi", kwlist,
                                     &value, &self->is_present,
                                     &self->num_destinations))
    {
        return -1;
    }

    if (value){
        tmp = self->value;
        Py_INCREF(value);
        self->value = value;
        Py_XDECREF(tmp);
    }

    return 0;
}

/*
 * The definition of port_instance type object.
 * Used to describe how port_instance behaves.
 */
static PyTypeObject port_instance_t = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.port_instance",
    .tp_doc = "port_instance objects",
    .tp_basicsize = sizeof(generic_port_instance_struct),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = port_intance_new,
    .tp_init = (initproc) port_instance_init,
    .tp_dealloc = (destructor) port_instance_dealloc,
    .tp_members = port_instance_members,
};

/*
 * The members of a port_instance, used to define
 * a native Python type.
 */
static PyMemberDef port_instance_token_members[] = {
    {"value", T_OBJECT_EX, offsetof(generic_port_instance_with_token_struct, value), 0, "Value of the port"},
    {"is_present", T_BOOL, offsetof(generic_port_instance_with_token_struct, is_present), 0, "Check if value is present at current logical time"},
    {"num_destinations", T_INT, offsetof(generic_port_instance_with_token_struct, num_destinations), 0, "Number of destinations"},
    {NULL}  /* Sentinel */
};

/*
 * The definition of port_instance type object.
 * Used to describe how port_instance behaves.
 */
static PyTypeObject port_instance_token_t = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.port_instance",
    .tp_doc = "port_instance objects",
    .tp_basicsize = sizeof(generic_port_instance_with_token_struct),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_members = port_instance_token_members,
};



//// Actions /////
/*
 * The members of a action_capsule that are accessible from a Python program, used to define
 * a native Python type.
 */
static PyMemberDef action_capsule_members[] = {
    {"action", T_OBJECT, offsetof(generic_action_capsule_struct, action), 0, "The pointer to the C action struct"},
    {"value", T_OBJECT, offsetof(generic_action_capsule_struct, value), 0, "Value of the action"},
    {NULL}  /* Sentinel */
};

/*
 *
 */
static void
action_capsule_dealloc(generic_action_capsule_struct *self)
{
    Py_XDECREF(self->action);
    Py_XDECREF(self->value);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

static PyObject *
action_capsule_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    generic_action_capsule_struct *self;
    self = (generic_action_capsule_struct *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->action = NULL;
        self->value = NULL;
    }
    
    return (PyObject *) self;
}

static int
action_capsule_init(generic_action_capsule_struct *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"action", "value", NULL};
    PyObject *action = NULL, *value = NULL, *tmp;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OO", kwlist,
                                     &action, &value))
    {
        return -1;
    }
    
    if (action){
        tmp = self->action;
        Py_INCREF(action);
        self->action = action;
        Py_XDECREF(tmp);
    }


    if (value){
        tmp = self->value;
        Py_INCREF(value);
        self->value = value;
        Py_XDECREF(tmp);
    }

    return 0;
}

/*
 * The definition of action_capsule type object.
 * Used to describe how an action_capsule behaves.
 */
static PyTypeObject action_capsule_t = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.action_instance",
    .tp_doc = "action_instance object",
    .tp_basicsize = sizeof(generic_action_capsule_struct),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = action_capsule_new,
    .tp_init = (initproc) action_capsule_init,
    .tp_dealloc = (destructor) action_capsule_dealloc,
    .tp_members = action_capsule_members,
};

///

/*
 * Bind Python function names to our C functions
 */
static PyMethodDef GEN_NAME(MODULE_NAME,_methods)[] = {
  {"start", py_main, METH_VARARGS, NULL},
  {"SET", py_SET, METH_VARARGS, NULL},
  {"schedule", py_schedule, METH_VARARGS, NULL},
  {"schedule_copy", py_schedule_copy, METH_VARARGS, NULL},
  {"schedule_int", py_schedule_int, METH_VARARGS, NULL},
  {"schedule_value", py_schedule_value, METH_VARARGS, NULL},
  {"get_elapsed_logical_time", py_get_elapsed_logical_time, METH_NOARGS, NULL},
  {"get_logical_time", py_get_logical_time, METH_NOARGS, NULL},
  {"get_physical_time", py_get_physical_time, METH_NOARGS, NULL},
  {"get_elapsed_physical_time", py_get_elapsed_physical_time, METH_NOARGS, NULL},
  {NULL, NULL, 0, NULL}
};

static PyModuleDef MODULE_NAME = {
    PyModuleDef_HEAD_INIT,
    TOSTRING(MODULE_NAME),
    "LinguaFranca Python Module",
    -1,
    GEN_NAME(MODULE_NAME,_methods)
};

//////////////////////////////////////////////////////////////
/////////////  Python Helper 
/**
 * A function that destroys action capsules
 **/
void destroy_action_capsule(PyObject* capsule)
{
    free(PyCapsule_GetPointer(capsule, "action"));
}

/**
 * A helper function to convert C actions to Python action capsules
 **/
PyObject* convert_C_action_to_py(void* action, PyObject* value)
{
    PyObject* cap = Py_BuildValue("OO", PyCapsule_New(action, "action", destroy_action_capsule), value);
    if(cap == NULL)
    {
        fprintf(stderr, "Failed to convert action.\n");
        exit(1);
    }

    return cap;
}

/**
 * A helper function to generate a mutable list of input ports to be sent to a Python reaction.
 */
PyObject* make_input_port_list(generic_port_instance_struct*** port_array, Py_ssize_t size)
{
#ifdef VERBOSE
    printf("Port width is %d.\n", size);
#endif
    PyObject *list = PyList_New(size);
    if(list == NULL)
    {
        fprintf(stderr, "Couldn't create a list!");
        return NULL;
    }
    for (Py_ssize_t i = 0; i != size; i++) {
        if(PyList_SET_ITEM(list, i, (PyObject*)*port_array[i]) == NULL)
        {
            fprintf(stderr, "Error setting list value.\n");
        }
    }

    return list;
}

/**
 * A helper function to generate a mutable  list of output ports to be sent to a Python reaction.
 */
PyObject* make_output_port_list(generic_port_instance_struct** port_array, Py_ssize_t size)
{
#ifdef VERBOSE
    printf("Port width is %d.\n", size);
#endif
    PyObject *list = PyList_New(size);
    if(list == NULL)
    {
        fprintf(stderr, "Couldn't create a list!");
        return NULL;
    }
    for (Py_ssize_t i = 0; i != size; i++) {
        if(PyList_SET_ITEM(list, i, (PyObject*)port_array[i]) == NULL)
        {
            fprintf(stderr, "Error setting list value.\n");
        }
    }

    return list;
}

/**
 * A helper function to generate an immutable tuple of input ports to be sent to a Python reaction.
 */
PyObject* make_input_port_tuple(generic_port_instance_struct*** port_array, Py_ssize_t size)
{
#ifdef VERBOSE
    printf("Port width is %d.\n", size);
#endif
    PyObject *list = PyTuple_New(size);
    if(list == NULL)
    {
        fprintf(stderr, "Couldn't create a tuple!");
        return NULL;
    }
    for (Py_ssize_t i = 0; i != size; i++) {
        if(PyTuple_SET_ITEM(list, i, (PyObject*)*port_array[i]) == NULL)
        {
            fprintf(stderr, "Error setting list value.\n");
        }
    }

    return list;
}

/**
 * A helper function to generate an immutable tuple of output ports to be sent to a Python reaction.
 */
PyObject* make_output_port_tuple(generic_port_instance_struct** port_array, Py_ssize_t size)
{
#ifdef VERBOSE
    printf("Port width is %d.\n", size);
#endif
    PyObject *list = PyTuple_New(size);
    if(list == NULL)
    {
        fprintf(stderr, "Couldn't create a tuple!");
        return NULL;
    }
    for (Py_ssize_t i = 0; i != size; i++) {
        if(PyTuple_SET_ITEM(list, i, (PyObject*)port_array[i]) == NULL)
        {
            fprintf(stderr, "Error setting list value.\n");
        }
    }

    return list;
}

/** 
 * Invoke a Python func in class[instance_id] from module.
 * Class instances in generate Python code are always in a list.
 * @param module The Python module to load the function from. In embedded mode, it should
 *               be set to "__main__"
 * @param class The name of the list of classes in the generated Python code
 * @param instance_id The element number in the list of classes. class[instance_id] points to a class instance
 * @param func The reaction functino to be called
 * @param pArgs the PyList of arguments to be sent to function func()
 */
static PyObject*
invoke_python_function(string module, string class, int instance_id, string func, PyObject* pArgs)
{

    // Set if the interpreter is already initialized
    int is_initialized = 0;

    if(Py_IsInitialized())
    {
        is_initialized = 1;
    }

#ifdef VERBOSE
    printf("Starting the function start()\n");
#endif

    // Necessary PyObject variables to load the react() function from test.py
    PyObject *pFileName, *pModule, *pDict, *pClasses, *pClass, *pFunc;

    PyObject *rValue;

    // Initialize the Python interpreter
    Py_Initialize();

#ifdef VERBOSE
    printf("Initialized the Python interpreter.\n");
#endif
    
    // Decode the MODULE name into a filesystem compatible string
    pFileName = PyUnicode_DecodeFSDefault(module);
    
    // Set the Python search path to be the current working directory
    char cwd[PATH_MAX];
    if( getcwd(cwd, sizeof(cwd)) == NULL)
    {
        fprintf(stderr, "Failed to get the current working directory.\n");
        exit(0);
    }

    wchar_t wcwd[PATH_MAX];

    mbstowcs(wcwd, cwd, PATH_MAX);

    Py_SetPath(wcwd);

#ifdef VERBOSE
    printf("Loading module %s in %s.\n", module, cwd);
#endif

    pModule = PyImport_Import(pFileName);

#ifdef VERBOSE
    printf("Loaded module %p.\n", pModule);
#endif

    // Free the memory occupied by pFileName
    Py_DECREF(pFileName);

    // Check if the module was correctly loaded
    if (pModule != NULL) {
        // Get contents of module. pDict is a borrowed reference.
        pDict = PyModule_GetDict(pModule);
        if(pDict == NULL)
        {
            PyErr_Print();
            fprintf(stderr, "Failed to load contents of module %s.\n", module);
            return 1;
        }

        // Convert the class name to a PyObject
        PyObject* list_name = PyUnicode_DecodeFSDefault(class);

        // Get the class list
        Py_INCREF(pDict);
        pClasses = PyDict_GetItem(pDict, list_name);
        if(pClasses == NULL){
            PyErr_Print();
            fprintf(stderr, "Failed to load class list \"%s\" in module %s.\n", class, module);
            return 1;
        }

        Py_DECREF(pDict);

        pClass = PyList_GetItem(pClasses, instance_id);
        if(pClass == NULL){
            PyErr_Print();
            fprintf(stderr, "Failed to load class \"%s[%d]\" in module %s.\n", class, instance_id, module);
            return 1;
        }

#ifdef VERBOSE
        printf("Loading function %s.\n", func);
#endif
        // Get the function react from test.py
        pFunc = PyObject_GetAttrString(pClass, func);

#ifdef VERBOSE
        printf("Loaded function %p.\n", pFunc);
#endif


        // Check if the funciton is loaded properly
        // and if it is callable
        if (pFunc && PyCallable_Check(pFunc))
        {
#ifdef VERBOSE
            printf("Attempting to call function %s from class %s[%d].\n", func , class, instance_id);
#endif


            // Call the func() function with arguments pArgs
            // The output will be returned to rValue
            rValue = PyObject_CallObject(pFunc, pArgs);

#ifdef VERBOSE
                printf("Finished calling %s.\n", func);
#endif

            // Check if the function is executed correctly
            if (rValue != NULL)
            {
#ifdef VERBOSE
                printf("I called %s and got %ld.\n", func , PyLong_AsLong(rValue));
#endif
                Py_DECREF(rValue);
            }
            else {
                // If the function call fails, print an error
                // message and get rid of the PyObjects
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                Py_DECREF(pClass);
                PyErr_Print();
                fprintf(stderr, "Calling react failed.\n");
                exit(0);
            }


            // Free pArgs (or rather decrement its reference count)
            //Py_DECREF(pArgs);
        }
        else
        {
            // Function is not found or it is not callable
            if (PyErr_Occurred())
            {
                PyErr_Print();
            }
            fprintf(stderr, "Function %s was not found or is not callable.\n", func);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule); 
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", module);
    }
    
#ifdef VERBOSE
    printf("Done with start()\n");
#endif

    if(is_initialized == 0)
    {
        /* We are the first to initilize the Pyton interpreter. Destroy it when done. */
        Py_FinalizeEx();
    }

    Py_INCREF(Py_None);
    return Py_None;
}

/*
 * Python calls this to let us initialize our module
 */
PyMODINIT_FUNC
GEN_NAME(PyInit_,MODULE_NAME)(void)
{
    PyObject *m;

    // Initialize the port_instance type
    if (PyType_Ready(&port_instance_t) < 0)
        return NULL;

    // Initialize the port_instance_token type
    if (PyType_Ready(&port_instance_token_t) < 0)
        return NULL;

    m = PyModule_Create(&MODULE_NAME);

    if (m == NULL)
        return NULL;

    // Add the port_instance type to the module's dictionary
    Py_INCREF(&port_instance_t);
    if (PyModule_AddObject(m, "port_instance", (PyObject *) &port_instance_t) < 0) {
        Py_DECREF(&port_instance_t);
        Py_DECREF(m);
        return NULL;
    }

    // Add the port_instance_token type to the module's dictionary
    Py_INCREF(&port_instance_token_t);
    if (PyModule_AddObject(m, "port_instance_token", (PyObject *) &port_instance_token_t) < 0) {
        Py_DECREF(&port_instance_token_t);
        Py_DECREF(m);
        return NULL;
    }

    // Add the action_capsule type to the module's dictionary
    Py_INCREF(&action_capsule_t);
    if (PyModule_AddObject(m, "action_capsule_t", (PyObject *) &action_capsule_t) < 0) {
        Py_DECREF(&action_capsule_t);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}

#endif // PYTHON_TARGET_H