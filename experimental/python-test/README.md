## The Main Build Mechanism
To generate a Makefile, use

```
python3 pymkfile.py LinguaFranca > Makefile
```

Then you can make and run the program:

```
make
python3 linguafrancatest.py
```

This should be the correct output:
```
Starting the function start()
Python interpreter is already initialized.
Attempting to call function react.
Creating a PyObject from carrier pointer
Set port instance values
Added all variables
Value before SET: 42
Value after SET: 464
I called react and got 0.
Value before SET: 464
Value after SET: 886
I called react and got 0.
Done with start()
Starting the function start()
Python interpreter is already initialized.
Attempting to call function react.
Creating a PyObject from carrier pointer
Set port instance values
Added all variables
Value before SET: 42
Value after SET: 464
I called react and got 0.
Value before SET: 464
Value after SET: 886
I called react and got 0.
Done with start()
```

## Alternative Build Mechanism
If the above procedure doesn't work, first clean the produced outputs by running:
```
make clean
```

Then use `setup.py` to create the LinguaFranca module:

    python3 setup.py build

This will create a `LinguaFranca****.so` in  `build/lib.***`. Copy the generated library to the `python-test` folder. For example, on my machine with Python 3.6:
```bash
~/lingua-franca/experimental/python-test$ mv build/lib.linux-x86_64-3.6/LinguaFranca.cpython-36m-x86_64-linux-gnu.so LinguaFranca.so
```

Proceed to running the main file:

    python3 linguafrancatest.py


## Build using setuptools
As a third alternative, the `setup.py.setuptools` is provided which uses setup tools instead of distutils to build the `.so` file.
Please replace setup.py and the rest would be similar to the previous procedures.
