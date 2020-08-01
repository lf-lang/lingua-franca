# Goal: Write two files, a linguafrancatest.py file and a test.c file, where the linguafrancatest.py file is the Main entry point.  I.e., you run this as

# python linguafrancatest.py

# The first thing this should do is call a C function 

#    void start() {call react() in Python, perhaps repeatedly}

# The linguafrancatest.py file includes a function called react(). This should have one argument that is a data structure with a field that is
# a reference to a C struct that Python makes no attempt to read and, perhaps, a number field. The above C function should pass that data structure
# to react().

# The react() function in Python should call a set(a,b) function in C, where a is the uninterpreted C struct and b is a number, say,
# the number field passed to it plus 1. The set function could, say, insert the number b into the struct a.

# When react() returns, the C code could just call it again.
# 
# 
# 
# Progress:
# There were two main goals that were discussed during the design review:
# 1- Re-using the CGenerator and the C core library to implement the Python generator
# 2- Presenting the generated code as a Python program to the end-user
#
# To test the waters regarding both of these requirements, we (you) planned and implemented an example, 
# which is located in the python branch in experimental/python-test . This example has the following requirements::
# 
# 1- A test.c file that can call a react function written in Python and located in linguafrancatest.py. 
# This is the purpose of the start() function in test.c .
# To do this, we only need to initialize a Python interpreter via Py_InitializeEx(0); and interpret the linguafrancatest.py ,
# load the react function, and finally call it. To my knowledge, this is the universal and supported way of calling Python code from C/C++.
# However, the second requirement makes this problem harder as I shall expand on next.
# 
# 2- For the second requirement, the start() function should be called from a Python program.
# In our case, the start() function is being called twice in linguafrancatest.py.
# To this end, we compile test.c as a shared library and load it via ctypes.CDLL.
# This is where we have options. We can also use the CPython APIs to load C functions without the need to compile them.
# However, this is not universal like ctypes and is only supported in the C implementation of Python.
# Nonetheless, here is where we run into thread-safety issues of Python.
# The interpreter used on linguafrancatest.py actually carries over to test.c when ctypes spawns a new thread to handle this library load.
# In order to use this interpreter in test.c in a thread-safe manner, we should acquire the Global Interpreter Lock (GIL) first.
# Currently, we do this by calling PyGILState_Ensure(); , which as far as I can tell, is an "accepted" method (might be the only supported one) in
# Python to acquire GIL. Alternatively, we can spawn a new interpreter altogether. However, this can be expensive in terms of memory and CPU overhead.
# 
# Finally, a funny thing happens when start() calls the react function again in linguafrancatest.py.
# Somewhat to my surprise, the interpreter is called again on the entire Python file, causing the start() to be invoked again.
# To prevent this loop, I have added a main function which is called only when the __name__ of the module is __main__ .
# We could technically circumvent these problems by using three files: linguafrancatest.py which calls start() in test.c.
# The start() function then initializes a new Python interpreter to avoid the thread-related issues and then call reactions stored
# in a third file called reactions.py  to avoid the loop.
#
# 
# 
# This file was named linguafrancatest instead of test
# to avoid conflict with the universal test module in
# Python.
import LinguaFranca

# Function aliases
start = LinguaFranca.start
SET = LinguaFranca.SET

#
#  A function representing a reaction written by the user
#
# @param port A port instance of Python type port_instance
#        defined in the LinguaFranca module
# @param number For demonstration purposes only. In reality,
#        the port.value is most likely going to be set by the user.
def react(port, number):
    # Test the values after
    print("Value before SET: " + str(port.value))

    # FIXME: port.value can be set directly here
    port.value += 2

    # Call the SET function in LinguaFranca.c
    SET(port, number + port.value)

    # Test the values after    
    print("Value after SET: " + str(port.value))

    return 0

# The main function
def main():
    # Call start() from test.c
    start()
    start()

# As is customary in Python programs, the main() function
# should only be executed if the main module is active.
if __name__=="__main__":
    main()



