/* 
* Example adapted from https://docs.python.org/3/extending/embedding.html
*/
#include <stdio.h>

#define PY_SSIZE_T_CLEAN

#include <Python.h>
#include <structmember.h>
#include <stdbool.h>
#include <unistd.h>

#define FUNC_NAME "react"
#define MODULE "linguafrancatest"

/* 
 * The contents of a port_instance.
 */
typedef struct {
    PyObject_HEAD
    int value;
    bool is_present;
    int num_destinations;
}  port_instance_object;

/*
 * The members of a port_instance, used to define
 * a native Python type.
 */
static PyMemberDef port_instance_members[] = {
    {"value", T_INT, offsetof(port_instance_object, value), 0, "Value of the port"},
    {"is_present", T_BOOL, offsetof(port_instance_object, is_present), 0, "Check if value is present at current logical time"},
    {"num_destinations", T_INT, offsetof(port_instance_object, num_destinations), 0, "Number of destinations"},
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
    .tp_basicsize = sizeof(port_instance_object),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_members = port_instance_members,
};

/* 
 * This function acts as the main loop of the C library.
 * First, it loads the Python MODULE (see above), and then looks up function
 * FUNC_NAME (see above) in MODULE. Finally, it creates a port_instance_object
 * and passes its pointer, as well as a_number to FUNC_NAME.
 */
static PyObject* py_start(PyObject *self, PyObject *args)
{

    // Set if the interpreter is already initialized
    int is_initialized = 0;

    if(Py_IsInitialized())
    {
        is_initialized = 1;
    }

    printf("Starting the function start()\n");

    // Necessary PyObject variables to load the react() function from test.py
    PyObject *pFileName, *pModule, *pFunc;
    // Argument to react()
    PyObject *pArgs;
    // Temporary variable to hold individual arguments
    PyObject *pValue;
    
    PyObject *rValue;
    
    port_instance_object * pyValue;

    // Initialize the Python interpreter
    Py_Initialize();
    printf("Initialized Python interpreter.\n");
    //}
    
    // Decode the MODULE name into a filesystem compatible string
    pFileName = PyUnicode_DecodeFSDefault(MODULE);
    
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
    
    // Import MODULE from linguafrancatest.py
    pModule = PyImport_Import(pFileName);

    // Free the memory occupied by pFileName
    // (not sure why the example does this here)
    // Overall, we need to be careful with PyObject
    // pointers. There are two types of operations represented by the followin macros:
    // Py_INCREF(obj) that increments a PyObject *'s reference count
    // Py_INCREF(obj) that decrements a PyObject *'s reference count
    // See: https://docs.python.org/3/extending/extending.html#reference-counts
    Py_DECREF(pFileName);

    // Check if the module was correctly loaded
    if (pModule != NULL) {
        // Get the function react from test.py
        pFunc = PyObject_GetAttrString(pModule, FUNC_NAME);

        // Check if the funciton is loaded properly
        // and if it is callable
        if (pFunc && PyCallable_Check(pFunc))
        {

            int a_number = 420;
            
            printf("Attempting to call function react.\n");


            // Create a tuple with size equal to
            // the number of arguments to the react function (=1)
            pArgs = PyTuple_New(2);

            printf("Creating a PyObject from carrier pointer\n");

            // Get a Python handle as a PyObject *
            // FIXME: PyCapsules are apparently safer than void *
            pyValue = PyObject_New(port_instance_object, &port_instance_t);

            pyValue->value = 42;
            pyValue->is_present = false;
            pyValue->num_destinations = 1;
            
            printf("Set port instance values\n");
            
            /* Pass the pValue and pyValue by reference to the argument list */
            // pArgs takes ownership of pyValue here.
            PyTuple_SetItem(pArgs, 0, (PyObject *)pyValue);
            
                        
            pValue = PyLong_FromLong(a_number);
            
            // pArgs takes ownership of pValue here.
            PyTuple_SetItem(pArgs, 1, pValue);

            
            printf("Added all variables\n");


            // Call the react() function with arguments pArgs
            // The output will be returned to rValue
            rValue = PyObject_CallObject(pFunc, pArgs);

            // Check if the function is executed correctly
            if (rValue != NULL)
            {
                printf("I called %s and got %ld.\n", FUNC_NAME , PyLong_AsLong(rValue));
                Py_DECREF(rValue);
            }
            else {
                // If the function call fails, print an error
                // message and get rid of the PyObjects
                Py_DECREF(pFunc);
                Py_DECREF(pValue);
                Py_DECREF(pyValue);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr, "Calling react failed.\n");
                exit(0);
            }
        
        
            // Call the react() function AGAIN with arguments pArgs
            // The output will be returned to pValue
            rValue = PyObject_CallObject(pFunc, pArgs);

            // Check if the function is executed correctly
            if (rValue != NULL)
            {
                printf("I called %s and got %ld.\n", FUNC_NAME , PyLong_AsLong(rValue));
                Py_DECREF(rValue);
            }
            else {
                // If the function call fails, print an error
                // message and get rid of the PyObjects
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                Py_DECREF(pValue);
                Py_DECREF(pyValue);
                PyErr_Print();
                fprintf(stderr, "Calling react failed.\n");
                exit(0);
            }


            // Free pArgs (or rather decrement its reference count)
            // Py_DECREF(pValue);
            // Py_DECREF(pyValue);
            Py_DECREF(pArgs);
        }
        else
        {
            // Function is not found or it is not callable
            if (PyErr_Occurred())
            {
                PyErr_Print();
            }
            fprintf(stderr, "Function %s was not found or is not callable.\n", FUNC_NAME);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule); 
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", MODULE);
    }
    
    printf("Done with start()\n");

    if(is_initialized == 0)
    {
        /* We are the first to initilize the Pyton interpreter. Destroy it when done. */
        Py_FinalizeEx();
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* py_SET(PyObject *self, PyObject *args)
{
    port_instance_object *port;
    int val;

    if (!PyArg_ParseTuple(args, "Oi" ,&port, &val))
        return NULL;
    
    port->value = val;
    port->is_present = true;

    Py_INCREF(Py_None);
    return Py_None;
}


/*
 * Bind Python function names to our C functions
 */
static PyMethodDef LinguaFranca_methods[] = {
  {"start", py_start, METH_VARARGS, NULL},
  {"SET", py_SET, METH_VARARGS, NULL},
  {NULL, NULL, 0, NULL}
};

static PyModuleDef LinguaFranca = {
    PyModuleDef_HEAD_INIT,
    "LinguaFranca",
    "LinguaFranca Python Module",
    -1,
    LinguaFranca_methods
};

/*
 * Python calls this to let us initialize our module
 */
PyMODINIT_FUNC
PyInit_LinguaFranca(void)
{
    PyObject *m;

    // Initialize the port_instance type
    if (PyType_Ready(&port_instance_t) < 0)
        return NULL;
    m = PyModule_Create(&LinguaFranca);

    if (m == NULL)
        return NULL;

    // Add the port_instance type to the module's dictionary
    Py_INCREF(&port_instance_t);
    if (PyModule_AddObject(m, "port_instance", (PyObject *) &port_instance_t) < 0) {
        Py_DECREF(&port_instance_t);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}

