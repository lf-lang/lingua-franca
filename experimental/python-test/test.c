/* 
* Example adapted from https://docs.python.org/3/extending/embedding.html
*/
#include <stdio.h>
#include <Python.h>
#include <stdbool.h>
#include <unistd.h>

#define FUNC_NAME "react"
#define MODULE "linguafrancatest"

/* Define the port instance that is passed around.*/
typedef struct {
    int value;
    bool is_present;
    int num_destinations;
}  port_instance_t;

/* 
 * Define a carrier that holds a value and a pointer
 *  to a port or action instance.
 */
typedef struct {
    void * port_action_instance;
    int value;
} carrier_t;

/* 
 * This function acts as the main loop of the C library.
 * First, it loads the MODULE, and then looks up function
 * FUNC_NAME in MODULE. Finally, it creates a my_port_instance
 * and passes its pointer, as well as a_number to FUNC_NAME.
 */
void start()
{
    // Ensure that no one else is using the interpreter
    PyGILState_STATE s = PyGILState_Ensure();

    // Set if the interpreter is already initialized
    int is_initialized = 0;

    printf("Starting the function start()\n");

    // Necessary PyObject variables to load the react() function from test.py
    PyObject *pFileName, *pModule, *pFunc;
    // Argument to react()
    PyObject *pArgs;
    // Temporary variable to hold individual arguments
    PyObject *pValue;

    // Used for the interpreter
    PyThreadState *threadState;

    if(Py_IsInitialized())
    {
        printf("Python interpreter is already initialized.\n");
        //threadState = Py_NewInterpreter();
        is_initialized = 1;
    }
    else
    {
        // Initialize the Python interpreter
        Py_InitializeEx(0);
        printf("Initialized Python interpreter.\n");
    }
    
    // Set the file name to be loaded as a module
    pFileName = PyUnicode_DecodeFSDefault(MODULE);
    
    //PyRun_SimpleString("import sys");
    //PyRun_SimpleString("sys.path.append(\".\")");
    //PyRun_SimpleString("sys.path.append(\"/home/soroosh\")");


    //printf("%ls\n",Py_GetPath());
    char cwd[PATH_MAX];
    if( getcwd(cwd, sizeof(cwd)) == NULL)
    {
        fprintf(stderr, "Failed to get the current working directory.\n");
        exit(0);
    }

    wchar_t wcwd[PATH_MAX];

    mbstowcs(&wcwd, cwd, PATH_MAX);

    Py_SetPath(wcwd);
    //printf("%ls\n",Py_GetPath());
    
    // Load the module from test.py
    pModule = PyImport_Import(pFileName);

    // Free the memory occupied by pFileName
    // (not sure why the example does this here)
    Py_DECREF(pFileName);

    // Check if the module was correctly loaded
    if (pModule != NULL) {
        // Get the function react from test.py
        pFunc = PyObject_GetAttrString(pModule, FUNC_NAME);

        // Check if the funciton is loaded properly
        // and if it is callable
        if (pFunc && PyCallable_Check(pFunc))
        {
            // Initialize the port instance
            port_instance_t my_port_instance = {42, false, 1};
            
            int a_number = 420;
            
            printf("Attempting to call function react.\n");

            // Set up the carrier
            carrier_t my_carrier = { (void *) &my_port_instance, a_number };

            // Create a tuple with size equal to
            // the number of arguments to the react function (=1)
            pArgs = PyTuple_New(1);

            printf("Creating a PyObject from carrier pointer\n");

            // Get a Python handle as a PyObject *
            // FIXME: PyCapsules are apparently safer than void *
            pValue = PyLong_FromVoidPtr((void *) &my_carrier);

            // Pass the pValue by reference to the argument list
            PyTuple_SetItem(pArgs, 0, pValue);

            // Call the react() function with arguments pArgs
            // The output will be returned to pValue
            pValue = PyObject_CallObject(pFunc, pArgs);

            // Check if the function is executed correctly
            if (pValue != NULL)
            {
                printf("I called %s and got %ld.\n", FUNC_NAME , PyLong_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                // If the function call fails, print an error
                // message and get rid of the PyObjects
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr, "Calling react failed.\n");
                exit(0);
            }
        
        
            // Call the react() function AGAIN with arguments pArgs
            // The output will be returned to pValue
            pValue = PyObject_CallObject(pFunc, pArgs);

            // Check if the function is executed correctly
            if (pValue != NULL)
            {
                printf("I called %s and got %ld.\n", FUNC_NAME , PyLong_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                // If the function call fails, print an error
                // message and get rid of the PyObjects
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr, "Calling react failed.\n");
                exit(0);
            }


            // Free pArgs (or rather decrement its reference count)
            Py_DECREF(pArgs);
        }
        else
        {
            // Function is not found or it is not callable
            if (PyErr_Occurred)
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

    // Undo Py_Initialize()

    if(is_initialized)
    {
        //Py_EndInterpreter(threadState);
    }
    else
    {
        // Necessary in standalone mode for this library
        // Not necesasry when start() is called from Python code.
        Py_FinalizeEx();        
    }
    

    printf("Done with start()\n");

    PyGILState_Release(s);
}

void SET(void *out, int val)
{
    port_instance_t *port = (port_instance_t *) out;
    port->is_present = true;
    port->value = val;
}