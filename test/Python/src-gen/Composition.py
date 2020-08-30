import LinguaFrancaComposition
import sys

# Function aliases
start = LinguaFrancaComposition.start
SET = LinguaFrancaComposition.SET

class _test:
    count:int = 0

    def reaction_function_0(self, x, x_width):
        self.count += 1
        print("Recieved " + str(x.value))
        if (x.value != self.count):
            sys.stderr.write("FAILURE: Expected " + str(self.count) + "\n")
            exit(1)
        return 0
  
    def reaction_function_1(self, shutdown):
        if(self.count == 0):
            sys.stderr.write("FAILURE: No data received.\n")
        return 0

class _source:
    count:int = 0

    def reaction_function_0(self, y):
        self.count += 1
        print("Source sending " + str(self.count))
        #print("y value is " + str(y.value))
        SET(y, self.count)
        return 0


test = _test()
source = _source()

# The main function
def main():
    start()

# As is customary in Python programs, the main() function
# should only be executed if the main module is active.
if __name__=="__main__":
    main()