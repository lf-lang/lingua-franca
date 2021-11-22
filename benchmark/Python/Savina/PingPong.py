from LinguaFrancaBase.constants import * #Useful constants
from LinguaFrancaBase.functions import * #Useful helper functions
from LinguaFrancaBase.classes import * #Useful classes
import sys
import copy

sys.setrecursionlimit(1000000)
EXPECTED = 10000

class _Ping:
    count = EXPECTED
    pingsLeft = count
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
    def reaction_function_1(self):
        self.pingsLeft -= 1
        if self.pingsLeft > 0:
            pingpong_pong_lf.reaction_function_0(self.pingsLeft)
        else:
            return
        return 0

class _Pong:
    expected = EXPECTED
    count = 0
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
    def reaction_function_0(self , receive):
        self.count += 1
        if(self.count == self.expected):
            return
        pingpong_ping_lf.reaction_function_1()
        return 0
    def reaction_function_1(self ):
        if self.count != self.expected:
            sys.stderr.write("Pong expected to receive {:d} inputs, but it received {:d}.\n".format(self.expected, self.count))
            exit(1)
        print("Success.")
        return 0
        
# Instantiate classes
pingpong_ping_lf = _Ping(bank_index = 0, count=EXPECTED)
pingpong_pong_lf = _Pong(bank_index = 0, expected=EXPECTED)

# The main function
def main():
    pingpong_ping_lf.reaction_function_1()

import timeit
# As is customary in Python programs, the main() function
# should only be executed if the main module is active.
if __name__=="__main__":
    t = timeit.timeit("main()", setup="from __main__ import main")
    print(t)
