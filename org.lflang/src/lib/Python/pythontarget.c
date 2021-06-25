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
#include "core/util.h"

//////////// set Function(s) /////////////
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
 * appropriately handling types on the recieveing end of this port.
 * @param self The output port (by name) or input of a contained
 *                 reactor in form instance_name.port_name.
 * @param args contains:
 *      @param val The value to insert into the port struct.
 */
static PyObject* py_SET(PyObject *self, PyObject *args) {
    generic_port_capsule_struct* p = (generic_port_capsule_struct*)self;
    PyObject* val, *tmp;

    if (!PyArg_ParseTuple(args, "O", &val)) {
        PyErr_SetString(PyExc_TypeError, "Could not set objects.");
        return NULL;
    }

    generic_port_instance_struct* port = 
        PyCapsule_GetPointer(p->port, "port");
    if (port == NULL) {
        error_print("Null pointer received.");
        exit(1);
    }
    
    if (val) {
        DEBUG_PRINT("Setting value %p.", port->value);
        tmp = port->value;
        Py_INCREF(val);
        // Call the core lib API to set the port
        _LF_SET(port, val);

        Py_INCREF(val);
        // Also set the values for the port capsule.      
        p->value = val;
        p->is_present = true;
        //Py_XDECREF(tmp); // Since value is allocated in Python, the Python garbage collector will manage and free this memory
    }

    Py_INCREF(Py_None);
    return Py_None;
}

//////////// schedule Function(s) /////////////
/**
 * Prototype for the internal API. @see reactor_common.c
 **/
lf_token_t* __initialize_token_with_value(lf_token_t* token, void* value, int length);

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
static PyObject* py_schedule(PyObject *self, PyObject *args) {
    generic_action_capsule_struct* act = (generic_action_capsule_struct*)self;
    long long offset;
    PyObject* value = NULL;

    if (!PyArg_ParseTuple(args, "L|O", &offset, &value))
        return NULL;
    
    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL) {
        error_print("Null pointer received.");
        exit(1);
    }

    trigger_t* trigger = _lf_action_to_trigger(action);
    lf_token_t* t = NULL;

    // Check to see if value exists and token is not NULL
    if (value && (trigger->token != NULL)) {
        // DEBUG: adjust the element_size (might not be necessary)
        trigger->token->element_size = sizeof(PyObject*);
        trigger->element_size = sizeof(PyObject*);
        t = __initialize_token_with_value(trigger->token, value, 1);

        // Also give the new value back to the Python action itself
        Py_INCREF(value);
        act->value = value;
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
static PyObject* py_schedule_copy(PyObject *self, PyObject *args) {
    generic_action_capsule_struct* act;
    long long offset;
    PyObject* value;
    int length;

    if (!PyArg_ParseTuple(args, "OLOi" ,&act, &offset, &value, &length))
        return NULL;

    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL) {
        error_print("Null pointer received.");
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
static PyObject* py_get_elapsed_logical_time(PyObject *self, PyObject *args) {
    return PyLong_FromLong(get_elapsed_logical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_logical_time(PyObject *self, PyObject *args) {
    return PyLong_FromLong(get_logical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
static PyObject* py_get_physical_time(PyObject *self, PyObject *args) {
    return PyLong_FromLong(get_physical_time());
}

/** 
 * Return the elapsed physical time in nanoseconds.
 */
instant_t get_elapsed_physical_time();
static PyObject* py_get_elapsed_physical_time(PyObject *self, PyObject *args) {
    return PyLong_FromLong(get_elapsed_physical_time());
}

/**
 * Return the start time in nanoseconds.
 */
instant_t get_start_time();
static PyObject* py_get_start_time(PyObject *self, PyObject *args) {
    return PyLong_FromLong(get_start_time());
}

/**
 * Prototype for the main function.
 */
int lf_reactor_c_main(int argc, char *argv[]);

/**
 * Prototype for request_stop().
 * @see reactor.c and reactor_threaded.c
 */
void request_stop();

///////////////// Other useful functions /////////////////////
/**
 * Stop execution at the conclusion of the current logical time.
 */
static PyObject* py_request_stop(PyObject *self) {
    request_stop();
    
    Py_INCREF(Py_None);
    return Py_None;
}


//////////////////////////////////////////////////////////////
///////////// Main function callable from Python code
static PyObject* py_main(PyObject *self, PyObject *args) {
    DEBUG_PRINT("Initializing main.");
    const char *argv[] = {TOSTRING(MODULE_NAME), NULL };
    lf_reactor_c_main(1, argv);

    Py_INCREF(Py_None);
    return Py_None;
}


///////////////// Functions used in port creation, initialization, deletion, and copying /////////////
/**
 * Called when a port_capsule has to be deallocated (generally by the Python
 * garbage collector).
 * @param self An instance of generic_port_instance_struct*
 */
static void
port_capsule_dealloc(generic_port_capsule_struct *self) {
    Py_XDECREF(self->port);
    Py_XDECREF(self->value);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

/**
 * Create a new port_capsule. Note that a LinguaFranca.port_capsule PyObject
 * follows the same structure as the @see generic_port_capsule_struct.
 * 
 * To initialize the port_capsule, this function first initializes a 
 * generic_port_capsule_struct* self using the tp_alloc property of 
 * port_capsule (@see port_capsule_t) and then assigns the members
 * of self with default values of port= NULL, value = NULL, is_present = false,
 * current_index = 0, width = -2.
 * @param type The Python type object. In this case, port_capsule_t
 * @param args The optional arguments that are:
 *      @param port A capsule that holds a void* to the underlying C port
 *      @param value value of the port
 *      @param is_present An indication of whether or not the value of the port
 *                       is present at the current logical time.
 *      @param current_index Used to reference multiports in the iterator
 *      @param width Used to indicate the width of a multiport. If the port
 *                   is not a multiport, this field will be -2.
 * @param kwds Keywords (@see Python keywords)
 */
static PyObject *
port_capsule_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    generic_port_capsule_struct *self;
    self = (generic_port_capsule_struct *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->port = NULL;
        Py_INCREF(Py_None);
        self->value = Py_None;
        self->is_present = false;
        self->current_index = 0;
        self->width = -2;
    }
    return (PyObject *) self;
}

/**
 * The iterator function that can be useful to implement iterator features for multiports.
 * For example to make
 *     for p in foo_multiport:
 *         p.set(42)
 * possible in Python.
 * FIXME: Incomplete iterator
 */
static PyObject *
port_iter(PyObject *self) {
    generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;
    generic_port_capsule_struct* pyport = (generic_port_capsule_struct*)self->ob_type->tp_new(self->ob_type, NULL, NULL);
    long long index = port->current_index;

    if (port->current_index == port->width) {
        port->current_index = 0;
        return NULL;
    }

    if (port->width == -2) {
        PyErr_Format(PyExc_TypeError,
                "Non-multiport type is not iteratable.");
        return NULL;
    }

    generic_port_instance_struct **cport = (generic_port_instance_struct **)PyCapsule_GetPointer(port->port,"port");
    if (cport == NULL) {
        error_print("Null pointer received.");
        exit(1);
    }

    //Py_INCREF(cport[index]->value);
    pyport->port = PyCapsule_New(cport[index], "port", NULL);
    pyport->value = cport[index]->value;
    pyport->is_present = cport[index]->is_present;

    port->current_index++;

    Py_INCREF(pyport);
    return pyport;
}

/**
 * The function that is responsible for getting the next iterator for a multiport.
 * This would make the following code possible:
 *     for p in foo_multiport:
 *         p.set(42)
 * FIXME: incomplete iterator next
 */
static PyObject *
port_iter_next(PyObject *self) {

}
/**
 * Get an item from a Linugua Franca port capsule type.
 * If a port is a not a multiport, it will have a width of
 * -2 (@see CGenerator.xtend). In this case, this function will
 * return the port capsule itself.
 * If a port is a multiport, this function will convert the index
 * item and convert it into a C long long, and use it to access
 * the underlying array stored in the PyCapsule as "port". A new
 * non-multiport capsule is created and returned, which in turn can be
 * used as an ordinary LinguaFranca.port_capsule.
 * @param self The port which can be a multiport or a singular port
 * @param item The index which is used to retrieve an item from the underlying
 *             C array if the port is a multiport.
 */
static PyObject *
port_capsule_get_item(PyObject *self, PyObject *item) {
    generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;
    generic_port_capsule_struct* pyport = 
        (generic_port_capsule_struct*)self->ob_type->tp_new(self->ob_type, NULL, NULL);
    long long index = -3;

    // Port is not a multiport
    if (port->width == -2) {
        return self;
    }

    index = PyLong_AsLong(item);
    if (index == -3) {
        PyErr_Format(PyExc_TypeError,
                     "multiport indices must be integers, not %.200s",
                     Py_TYPE(item)->tp_name);
        return NULL;
    }
    
    if (index != -3 && port->width > 0) {
        generic_port_instance_struct **cport = 
            (generic_port_instance_struct **)PyCapsule_GetPointer(port->port,"port");
        if (cport == NULL) {
            error_print_and_exit("Null pointer received.");
        }

        //Py_INCREF(cport[index]->value);
        pyport->port = PyCapsule_New(cport[index], "port", NULL);
        pyport->value = cport[index]->value;
        pyport->is_present = cport[index]->is_present;
        pyport->width = -2;


        LOG_PRINT("Getting item index %d. Is present is %d.", index, pyport->is_present);
    }

    
    if (pyport->value == NULL) {
        Py_INCREF(Py_None);
        pyport->value = Py_None;
    }

    //Py_INCREF(((generic_port_capsule_struct*)port)->value);
    Py_INCREF(pyport);
    //Py_INCREF(self);
    return pyport;
}

/**
 * This function is overloaded to prevent directly assigning to multiports.
 * The set function currently is the only way to assign value to ports.
 * @param self The port of type LinguaFranca.port_capsule
 * @param item The index (which is ignored)
 * @param value The value to be assigned (which is ignored)
 */
static int
port_capsule_assign_get_item(PyObject *self, PyObject *item, PyObject* value) {
    PyErr_Format(PyExc_TypeError,
                     "You cannot assign to ports directly. Please use the .set method.",
                     Py_TYPE(item)->tp_name);
    return NULL;
}

/**
 * A function that allows the invocation of len() on a port.
 * @param self A port of type LinguaFranca.port_capsule
 */ 
static Py_ssize_t
port_length(PyObject *self) {
    generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;
    DEBUG_PRINT("Getting the length, which is %d.", port->width);
    return (Py_ssize_t)port->width;
}

/**
 * Methods that convert a LinguaFranca.port_capsule into a mapping,
 * which allows it to be subscriptble.
 */ 
static PyMappingMethods port_as_mapping = {    
    (lenfunc)port_length,
    (binaryfunc)port_capsule_get_item,
    (objobjargproc)port_capsule_assign_get_item
};

/**
 * Initialize the port capsule self with the given optional values for
 * port, value, is_present, and num_destinations. If any of these arguments
 * are missing, the default values are assigned
 * @see port_intance_new 
 * @param self The port_instance PyObject that follows
 *              the generic_port_instance_struct* internal structure
 * @param args The optional arguments that are:
 *      @param port A capsule that holds a void* to the underlying C port
 *      @param value value of the port
 *      @param is_present An indication of whether or not the value of the port
 *                       is present at the current logical time.
 *      @param current_index Used to reference multiports in the iterator
 *      @param width Used to indicate the width of a multiport. If the port
 *                   is not a multiport, this field will be -2.
 */
static int
port_capsule_init(generic_port_capsule_struct *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = { "port", "value", "is_present", "width", "current_index", NULL};
    PyObject *value = NULL, *tmp, *port = NULL;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOp", kwlist,
                                     &port, &value, &self->is_present, &self->width, &self->current_index))
    {
        return -1;
    }

    if (value){
        tmp = self->value;
        Py_INCREF(value);
        self->value = value;
        Py_XDECREF(tmp);
    }
    
    if (port){
        tmp = self->port;
        Py_INCREF(port);
        self->port = port;
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
action_capsule_dealloc(generic_action_capsule_struct *self) {
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
action_capsule_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    generic_action_capsule_struct *self;
    self = (generic_action_capsule_struct *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->action = NULL;
        self->value = NULL;
        self->is_present = false;
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
action_capsule_init(generic_action_capsule_struct *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"action", "value", "is_present", NULL};
    PyObject *action = NULL, *value = NULL, *tmp;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOi", kwlist,
                                     &action, &value, &self->is_present)) {
        return -1;
    }
    
    if (action) {
        tmp = self->action;
        Py_INCREF(action);
        self->action = action;
        Py_XDECREF(tmp);
    }


    if (value) {
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
 * The members of a port_capsule, used to define
 * a native Python type.
 * port contains the port capsule, which holds a void* pointer to the underlying C port.
 * value contains the copied value of the C port. The type is always PyObject*.
 * is_present contains the copied value of the is_present field of the C port.
 * width indicates the width of a multiport or -2 if not a multiport.
 */
static PyMemberDef port_capsule_members[] = {
    {"port", T_OBJECT, offsetof(generic_port_capsule_struct, port), 0, ""},
    {"value", T_OBJECT, offsetof(generic_port_capsule_struct, value), 0, "Value of the port"},
    {"is_present", T_BOOL, offsetof(generic_port_capsule_struct, is_present), 0, "Check if value is present at current logical time"},
    {"width", T_INT, offsetof(generic_port_capsule_struct, width), 0, ""},    
    {NULL, NULL, 0, NULL}  /* Sentinel */
};

/*
 * The function members of port_capsule
 * __getitem__ is used to reference a multiport with an index (e.g., foo[2])
 * set is used to set a port value and its is_present field.
 */
static PyMethodDef port_capsule_methods[] = {
    {"__getitem__", (PyCFunction)port_capsule_get_item, METH_O|METH_COEXIST, "x.__getitem__(y) <==> x[y]"},
    {"set", (PyCFunction)py_SET, METH_VARARGS, "Set value of the port as well as the is_present field"},
    {NULL}  /* Sentinel */
};


/*
 * The definition of port_instance type object.
 * Used to describe how port_instance behaves.
 */
static PyTypeObject port_capsule_t = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.port_capsule",
    .tp_doc = "port_capsule objects",
    .tp_basicsize = sizeof(generic_port_capsule_struct),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_as_mapping = &port_as_mapping,
    .tp_iter = port_iter,
    .tp_iternext = port_iter_next,
    .tp_new = port_capsule_new,
    .tp_init = (initproc) port_capsule_init,
    .tp_dealloc = (destructor) port_capsule_dealloc,
    .tp_members = port_capsule_members,
    .tp_methods = port_capsule_methods,
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
    {"is_present", T_BOOL, offsetof(generic_action_capsule_struct, is_present), 0, "Check that shows if action is present"},
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
/**
 * Bind Python function names to the C functions.
 * The name of this struct is dynamically generated because
 * MODULE_NAME is given by the generated code. This struct
 * will be named as MODULE_NAME_methods.
 * For example, for MODULE_NAME=Foo, this struct will
 * be called Foo_methods.
 * start() initiates the main loop in the C core library
 * @see schedule_copy
 * @see get_elapsed_logical_time
 * @see get_logical_time
 * @see get_physical_time
 * @see get_elapsed_physical_time
 * @see request_stop
 */
static PyMethodDef GEN_NAME(MODULE_NAME,_methods)[] = {
  {"start", py_main, METH_VARARGS, NULL},
  {"schedule_copy", py_schedule_copy, METH_VARARGS, NULL},
  {"get_elapsed_logical_time", py_get_elapsed_logical_time, METH_NOARGS, NULL},
  {"get_logical_time", py_get_logical_time, METH_NOARGS, NULL},
  {"get_physical_time", py_get_physical_time, METH_NOARGS, NULL},
  {"get_elapsed_physical_time", py_get_elapsed_physical_time, METH_NOARGS, NULL},
  {"get_start_time", py_get_start_time, METH_NOARGS, NULL},
  {"request_stop", py_request_stop, METH_NOARGS, NULL},
  {NULL, NULL, 0, NULL}
};


/**
 * Define the Lingua Franca module.
 * The MODULE_NAME is given by the generated code.
 */
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
 * The Python runtime will call this function to initialize the module.
 * The name of this function is dynamically generated to follow
 * the requirement of PyInit_MODULE_NAME. Since the MODULE_NAME is not
 * known prior to compile time, the GEN_NAME macro is used.
 * The generated function will have the name PyInit_MODULE_NAME.
 * For example for a module named LinguaFrancaFoo, this function
 * will be called PyInit_LinguaFrancaFoo
 */
PyMODINIT_FUNC
GEN_NAME(PyInit_,MODULE_NAME)(void) {
    PyObject *m;

    // Initialize the port_instance type
    if (PyType_Ready(&port_capsule_t) < 0) {
        return NULL;
    }

    // Initialize the port_instance_token type
    if (PyType_Ready(&port_instance_token_t) < 0) {
        return NULL;
    }

    // Initialize the action_capsule type
    if (PyType_Ready(&action_capsule_t) < 0) {
        return NULL;
    }

    m = PyModule_Create(&MODULE_NAME);

    if (m == NULL) {
        return NULL;
    }

    // Add the port_instance type to the module's dictionary
    Py_INCREF(&port_capsule_t);
    if (PyModule_AddObject(m, "port_capsule", (PyObject *) &port_capsule_t) < 0) {
        Py_DECREF(&port_capsule_t);
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
void destroy_action_capsule(PyObject* capsule) {
    free(PyCapsule_GetPointer(capsule, "action"));
}

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
PyObject* convert_C_port_to_py(void* port, int width) {
    generic_port_instance_struct* cport;
    if (width == -2) {
        // Not a multiport
        cport = (generic_port_instance_struct *)port;
    }
    // Create the port struct in Python
    PyObject* cap = 
        PyObject_GC_New(generic_port_capsule_struct, &port_capsule_t);
    if (cap == NULL) {
        error_print_and_exit("Failed to convert port.");
    }

    // Create the capsule to hold the void* port
    PyObject* capsule = PyCapsule_New(port, "port", NULL);
    if (capsule == NULL) {
        error_print_and_exit("Failed to convert port.");
    }

    // Fill in the Python port struct
    ((generic_port_capsule_struct*)cap)->port = capsule;
    ((generic_port_capsule_struct*)cap)->width = width;

    if (width == -2) {
        ((generic_port_capsule_struct*)cap)->is_present = 
            cport->is_present;


        if (cport->value == NULL) {
            // Value is absent
            Py_INCREF(Py_None);
            ((generic_port_capsule_struct*)cap)->value = Py_None;
            return cap;
        }

        //Py_INCREF(cport->value);
        ((generic_port_capsule_struct*)cap)->value = cport->value;
    } else {
        // Value is absent
        Py_INCREF(Py_None);
        ((generic_port_capsule_struct*)cap)->value = Py_None;
        ((generic_port_capsule_struct*)cap)->is_present = false;
    }

    return cap;
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
 * (which is in schedule functions), PyCapsule_GetPointer(received_action,"action") can be called to retrieve 
 * the void* pointer into received_action.
 **/
PyObject* convert_C_action_to_py(void* action) {
    // Convert to trigger_t
    trigger_t* trigger = _lf_action_to_trigger(action);

    // Create the action struct in Python
    PyObject* cap = PyObject_GC_New(generic_action_capsule_struct, &action_capsule_t);
    if (cap == NULL) {
        error_print_and_exit("Failed to convert action.");
    }

    // Create the capsule to hold the void* action
    PyObject* capsule = PyCapsule_New(action, "action", NULL);
    if (capsule == NULL) {
        error_print_and_exit("Failed to convert action.");
    }

    // Fill in the Python action struct
    ((generic_action_capsule_struct*)cap)->action = capsule;
    ((generic_action_capsule_struct*)cap)->is_present = trigger->status;

    // If token is not initialized, that is all we need to set
    if (trigger->token == NULL) {
        Py_INCREF(Py_None);
        ((generic_action_capsule_struct*)cap)->value = Py_None;
        return cap;
    }

    // Default value is None
    if (trigger->token->value == NULL) {
        Py_INCREF(Py_None);
        trigger->token->value = Py_None;
    }

    // Actions in Python always use token type
    ((generic_action_capsule_struct*)cap)->value = trigger->token->value;

    return cap;
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
get_python_function(string module, string class, int instance_id, string func) {

    // Set if the interpreter is already initialized
    int is_initialized = 0;

    if (Py_IsInitialized()) {
        is_initialized = 1;
    }

    DEBUG_PRINT("Starting the function start().");

    // Necessary PyObject variables to load the react() function from test.py
    PyObject *pFileName, *pModule, *pDict, *pClasses, *pClass, *pFunc;

    PyObject *rValue;

    // Initialize the Python interpreter
    Py_Initialize();

    DEBUG_PRINT("Initialized the Python interpreter.");

    // If the Python module is already loaded, skip this.
    if (globalPythonModule == NULL) {    
        // Decode the MODULE name into a filesystem compatible string
        pFileName = PyUnicode_DecodeFSDefault(module);
        
        // Set the Python search path to be the current working directory
        char cwd[PATH_MAX];
        if ( getcwd(cwd, sizeof(cwd)) == NULL)
        {
            error_print_and_exit("Failed to get the current working directory.");
        }

        wchar_t wcwd[PATH_MAX];

        mbstowcs(wcwd, cwd, PATH_MAX);

        Py_SetPath(wcwd);

    DEBUG_PRINT("Loading module %s in %s.", module, cwd);

        pModule = PyImport_Import(pFileName);

        DEBUG_PRINT("Loaded module %p.", pModule);

        // Free the memory occupied by pFileName
        Py_DECREF(pFileName);

        // Check if the module was correctly loaded
        if (pModule != NULL) {
            // Get contents of module. pDict is a borrowed reference.
            pDict = PyModule_GetDict(pModule);
            if (pDict == NULL) {
                PyErr_Print();
                error_print("Failed to load contents of module %s.", module);
                return 1;
            }

            Py_INCREF(pModule);
            globalPythonModule = pModule;
            Py_INCREF(pDict);
            globalPythonModuleDict = pDict;

        }
    }

    if (globalPythonModule != NULL && globalPythonModuleDict != NULL) {
        Py_INCREF(globalPythonModule);
        // Convert the class name to a PyObject
        PyObject* list_name = PyUnicode_DecodeFSDefault(class);

        // Get the class list
        Py_INCREF(globalPythonModuleDict);
        pClasses = PyDict_GetItem(globalPythonModuleDict, list_name);
        if (pClasses == NULL){
            PyErr_Print();
            error_print("Failed to load class list \"%s\" in module %s.", class, module);
            return 1;
        }

        Py_DECREF(globalPythonModuleDict);

        pClass = PyList_GetItem(pClasses, instance_id);
        if (pClass == NULL) {
            PyErr_Print();
            error_print("Failed to load class \"%s[%d]\" in module %s.", class, instance_id, module);
            return 1;
        }

        DEBUG_PRINT("Loading function %s.", func);

        // Get the function react from test.py
        pFunc = PyObject_GetAttrString(pClass, func);

        DEBUG_PRINT("Loaded function %p.", pFunc);

        // Check if the funciton is loaded properly
        // and if it is callable
        if (pFunc && PyCallable_Check(pFunc)) {
            DEBUG_PRINT("Calling function %s from class %s[%d].", func , class, instance_id);
            Py_INCREF(pFunc);
            return pFunc;
        }
        else {
            // Function is not found or it is not callable
            if (PyErr_Occurred())
            {
                PyErr_Print();
            }
            error_print("Function %s was not found or is not callable.", func);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(globalPythonModule); 
    }
    else {
        PyErr_Print();
        error_print("Failed to load \"%s\".", module);
    }
    
    DEBUG_PRINT("Done with start().");

    if (is_initialized == 0) {
        /* We are the first to initilize the Pyton interpreter. Destroy it when done. */
        Py_FinalizeEx();
    }

    Py_INCREF(Py_None);
    return Py_None;
}
