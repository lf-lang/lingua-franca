# This file was named linguafrancatest instead of test
# to avoid conflict with the universal test module in
# Python.
import ctypes

# A class to represent carrier_t from test.c
class my_carrier(ctypes.Structure):
    _fields_ = [('my_port_instance', ctypes.c_void_p), ('a_number', ctypes.c_int)]
    def __repr__(self):
        return '({0}, {1})'.format(self.my_port_instance, self.a_number)

# A class to represent port_instance_t from test.c
class my_port_instance(ctypes.Structure):
    _fields_ = [('value', ctypes.c_int), ('is_present', ctypes.c_bool), ('num_destinations', ctypes.c_int)]
    def __repr__(self):
        return '({0}, {1}, {2})'.format(self.value, self.is_present, self.num_destinations)

# Used to load functions from liblf
# @param liblf The ctypes CDLL library
# @param funcname The function name to be loaded
# @param restype Return type for the function. Can be None
# @param argtype The argument list ([]) for the function
def wrap_function(liblf , funcname, restype, argtypes):
    func = liblf.__getattr__(funcname)
    func.restype = restype
    func.argtypes = argtypes
    return func

# Load the shared library
_lf = ctypes.CDLL("./test.so")

# Function aliases
start = wrap_function(_lf, "start", None, None)
SET = wrap_function(_lf, "SET", None, [ctypes.c_void_p, ctypes.c_int])


def react(carrier):
    # print(type(carrier))

    # Get variables
    my_local_carrier = my_carrier.from_address(carrier)
    my_local_port_instance = my_port_instance.from_address(my_local_carrier.my_port_instance)

    # Test the values after
    print("Value before SET: " + str(my_local_port_instance.value))

    # Local change
    # my_local_carrier.a_number = my_local_carrier.a_number * 2

    # Call the SET function in test.c
    SET(my_local_carrier.my_port_instance, my_local_carrier.a_number)

    # Test the values after
    my_local_port_instance = my_port_instance.from_address(my_local_carrier.my_port_instance)
    print("Value after SET: " + str(my_local_port_instance.value))
    return 0

# The main function
def main():

    # Call start() from test.c
    start()
    start()

if __name__=="__main__":
    main()



