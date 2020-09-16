/**
 * @file
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
 * Implementation of functions defined in @see pythontarget.h
 */

#include "pythontarget.h"

//////////// set Function(s) /////////////
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
static PyObject* py_SET(PyObject *self, PyObject *args)
{
    generic_port_instance_struct* port = (generic_port_instance_struct*)self;
    PyObject* val, *tmp;

    if (!PyArg_ParseTuple(args, "O", &val))
    {
        PyErr_SetString(PyExc_TypeError, "Could not set objects.");
        return NULL;
    }
    
    if(val)
    {   
        tmp = port->value;
        Py_INCREF(val);
        _LF_SET(port, val);
        //Py_XDECREF(tmp); // Since value is allocated in Python, the Python garbage collector will manage and free this memory
    }

    Py_INCREF(Py_None);
    return Py_None;
}

//////////// schedule Function(s) /////////////
/**
 * Prototype for the internal API. @see reactor_common.c
 **/
token_t* __initialize_token_with_value(token_t* token, void* value, int length);

/**
 * Prototype for API function. @see lib/core/reactor_common.c
 **/
trigger_t* _lf_action_to_trigger(void* action);

/**
 * Schedule an action to occur with the specified time offset
 * with no payload (no value conveyed). This function is callable
 * in Python by calling action_name.schedule(offset).
 * Some examples include:
 *  action_name.schedule(5)
 *  action_name.schedule(NSEC(5))
 * See schedule_token(), which this uses, for details.
 * @param self Pointer to the calling object.
 * @param args contains:
 *      @param action Pointer to an action on the self struct.
 *      @param offset The time offset over and above that in the action.
 **/
static PyObject* py_schedule(PyObject *self, PyObject *args)
{
    generic_action_capsule_struct* act = (generic_action_capsule_struct*)self;
    long long offset;
    PyObject* value;

    if (!PyArg_ParseTuple(args, "L|O", &offset, &value))
        return NULL;
    
    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL)
    {
        fprintf(stderr, "Null pointer recieved.\n");
        exit(1);
    }

    trigger_t* trigger = _lf_action_to_trigger(action);
    token_t* t = NULL;

    // Check to see if value exists and token is not NULL
    if(value && (trigger->token != NULL))
    {
        // DEBUG: adjust the element_size (might not be necessary)
        trigger->token->element_size = sizeof(PyObject*);
        trigger->element_size = sizeof(PyObject*);
        t = __initialize_token_with_value(trigger->token, value, 1);
    }

    
    // Pass the token along
    _lf_schedule_token(action, offset, t);

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


///////// Time-keeping functions //////////
/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_elapsed_logical_time(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(get_elapsed_logical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_logical_time(PyObject *self, PyObject *args)
{
    return PyLong_FromLong(get_logical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
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


///////////////// Functions used in port creation, initialization, deletion, and copying /////////////
/**
 * In order to implement mutable inputs, we need to deepcopy a port_instance.
 * For this purpose, Python needs to be able to "pickle" the objects (convert
 * them to the fomrat objectID(objmember1, ....))
 * @param self A port with type port_instance_t with a template structure akin to
 *             a generic_port_instance_struct* (therefore, self is castable to generic_port_instance_struct*)
 * @return tuple A tuple of format objectID(port->value, port->is_present, port->num_destinations)
 */
static PyObject *
port_instance_reduce(PyObject *self)
{
    PyObject *attr;
    PyObject *obj;
    PyObject *tuple;
    generic_port_instance_struct* port;

    obj = (PyObject*)self;
    port = (generic_port_instance_struct*)obj;

	attr = PyObject_GetAttrString(obj, "__class__");

#ifdef VERBOSE
    printf("Reduce called.\n");
#endif

    tuple = Py_BuildValue("O(Oii)", attr, port->value , port->is_present, port->num_destinations);
    return tuple;
}


/**
 * Called when a port_instance had to be deallocated (generally by the Python
 * garbage collector).
 * @param self An instance of generic_port_instance_struct*
 */
static void
port_instance_dealloc(generic_port_instance_struct *self)
{
    Py_XDECREF(self->value);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

/**
 * Create a new port_instance. Note that a LinguaFranca.port_instance PyObject
 * follows the same structure as the @see generic_port_instance_struct.
 * 
 * To initialize the port_instance, this function first initializes a 
 * generic_port_instance_struct* self using the tp_alloc property of 
 * port_instance (@see port_isntance_t) and then assigns the members
 * of self with default values of value = NULL, is_present = false,
 * and num_destination = 0.
 * @param type The Python type object. In this case, port_instance_t
 * @param args The optional arguments that are:
 *      @param value value of the port
 *      @param is_present An indication of whether or not the value of the port
 *                      is present at the current logical time.
 *      @param num_destination Used for reference-keeping inside the C runtime
 * @param kwds Keywords (@see Python keywords)
 */
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

/**
 * Initialize the port instance self with the given optional values for
 * value, is_present, and num_destinations. If any of these arguments
 * are missing, the default values are assigned
 * @see port_intance_new 
 * @param self The port_instance PyObject that follows
 *              the generic_port_instance_struct* internal structure
 * @param args The optional arguments that are:
 *      @param value value of the port
 *      @param is_present An indication of whether or not the value of the port
 *                      is present at the current logical time.
 *      @param num_destination Used for reference-keeping inside the C runtime
 */
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


///////////////// Functions used in action creation, initialization and deletion /////////////
/**
 * Called when an action in Python is deallocated (generally
 * called by the Python grabage collector).
 * @param self
 */
static void
action_capsule_dealloc(generic_action_capsule_struct *self)
{
    Py_XDECREF(self->action);
    Py_XDECREF(self->value);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

/**
 * Called when an action in Python is to be created. Note that LinguaFranca.action_capsule
 * follows the same structure as the @see generic_action_capsule_struct.
 * 
 * To initialize the action_capsule, this function first calls the tp_alloc
 * method of type action_capsule_t and then assign default values of NULL, NULL, 0
 * to the members of the generic_action_capsule_struct.
 */
static PyObject *
action_capsule_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    generic_action_capsule_struct *self;
    self = (generic_action_capsule_struct *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->action = NULL;
        self->value = NULL;
        self->is_present = 0;
    }
    
    return (PyObject *) self;
}

/**
 * Initialize the action capsule "self" with the given optional values for
 * action (void *), value (PyObject*), and is_present (bool). If any of these arguments
 * are missing, the default values are assigned.
 * 
 * @see port_intance_new 
 * @param self The port_instance PyObject that follows
 *              the generic_port_instance_struct* internal structure
 * @param args The optional arguments that are:
 *      @param action The void * pointer to a C action instance struct
 *      @param value value of the port
 *      @param is_present An indication of whether or not the value of the port
 *                      is present at the current logical time.
 *      @param num_destination Used for reference-keeping inside the C runtime
 */
static int
action_capsule_init(generic_action_capsule_struct *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"action", "value", "is_present", NULL};
    PyObject *action = NULL, *value = NULL, *tmp;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOi", kwlist,
                                     &action, &value, &self->is_present))
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

//////////////////////////////////////////////////////////////
/////////////  Python Structs

////// Ports //////
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
 * The function members of port_instance
 */
static PyMethodDef port_instance_methods[3] = {    
    {"__reduce__", (PyCFunction)port_instance_reduce, METH_NOARGS, "Necessary for pickling objects"},
    {"set", (PyCFunction)py_SET, METH_VARARGS, "Set value of the port as well as the is_present field"},
    {NULL}  /* Sentinel */
};


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
    .tp_methods = port_instance_methods,
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
    {"is_present", T_BOOL, offsetof(generic_action_capsule_struct, value), 0, "Check that shows if action is present"},
    {NULL}  /* Sentinel */
};


/**
 * The function members of action capsule
 */
static PyMethodDef action_capsule_methods[] = {
    {"schedule", (PyCFunction)py_schedule, METH_VARARGS, "Schedule the action with the given offset"},
    {NULL}  /* Sentinel */
};

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
    .tp_methods = action_capsule_methods,
};


///// Python Module Built-ins
/*
 * Bind Python function names to our C functions
 */
static PyMethodDef GEN_NAME(MODULE_NAME,_methods)[] = {
  {"start", py_main, METH_VARARGS, NULL},
  {"schedule_copy", py_schedule_copy, METH_VARARGS, NULL},
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
/////////////  Module Initialization
/*
 * The Python runtime will call this to initialize the module.
 * The name of this function is dynamically generated to follow
 * the requirement of PyInit_MODULE_NAME. Since the MODULE_NAME is not
 * known prior to compile time, the GEN_NAME macro is used.
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

    // Initialize the action_capsule type
    if (PyType_Ready(&action_capsule_t) < 0)
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



//////////////////////////////////////////////////////////////
/////////////  Python Helper Functions
/// These functions are called in generated C code for various reasons.
/// Their main purpose is to facilitate C runtime's communication with
/// Python code.
/**
 * A function that destroys action capsules
 **/
void destroy_action_capsule(PyObject* capsule)
{
    free(PyCapsule_GetPointer(capsule, "action"));
}

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
PyObject* convert_C_action_to_py(void* action)
{
    // Convert to trigger_t
    trigger_t* trigger = _lf_action_to_trigger(action);

    // Create the action struct in Python
    PyObject* cap = PyObject_GC_New(generic_action_capsule_struct, &action_capsule_t);
    if(cap == NULL)
    {
        fprintf(stderr, "Failed to convert action.\n");
        exit(1);
    }

    // Create the capsule to hold the void* action
    PyObject* capsule = PyCapsule_New(action, "action", NULL);
    if(capsule == NULL)
    {
        fprintf(stderr, "Failed to convert action.\n");
        exit(1);
    }

    // Fill in the Python action struct
    ((generic_action_capsule_struct*)cap)->action = capsule;
    ((generic_action_capsule_struct*)cap)->is_present = trigger->is_present;

    // If token is not initialized, that is all we need to set
    if(trigger->token == NULL)
    {
        return cap;
    }

    // Default value is None
    if(trigger->token->value == NULL)
    {
        Py_INCREF(Py_None);
        trigger->token->value = Py_None;
    }

    // Increase ref count for this token because value is a PyObject managed by the Python's garbage collector
    trigger->token->ref_count++;
    // Actions in Python always use token type
    ((generic_action_capsule_struct*)cap)->value = trigger->token->value;

    return cap;
}

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
PyObject* make_input_port_tuple(generic_port_instance_struct*** port_array, Py_ssize_t size)
{
#ifdef VERBOSE
    printf("Port width is %d.\n", size);
#endif
    PyObject *tuple = PyTuple_New(size);
    if(tuple == NULL)
    {
        fprintf(stderr, "Couldn't create a tuple!");
        return NULL;
    }
    for (Py_ssize_t i = 0; i != size; i++) {
        if(PyTuple_SET_ITEM(tuple, i, (PyObject*)*port_array[i]) == NULL)
        {
            fprintf(stderr, "Error setting tuple value.\n");
        }
    }

    return tuple;
}


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
PyObject* make_output_port_tuple(generic_port_instance_struct** port_array, Py_ssize_t size)
{
#ifdef VERBOSE
    printf("Port width is %d.\n", size);
#endif
    PyObject *tuple = PyTuple_New(size);
    if(tuple == NULL)
    {
        fprintf(stderr, "Couldn't create a tuple!");
        return NULL;
    }
    for (Py_ssize_t i = 0; i != size; i++) {
        if(PyTuple_SET_ITEM(tuple, i, (PyObject*)port_array[i]) == NULL)
        {
            fprintf(stderr, "Error setting tuple value.\n");
        }
    }

    return tuple;
}


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
