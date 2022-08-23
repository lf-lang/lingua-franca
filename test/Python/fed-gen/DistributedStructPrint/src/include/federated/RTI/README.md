This folder contains the source code for the Run-Time Infrastructure (RTI) that
is necessary for federated Lingua Franca programs. To compile and install, do:

```bash
mkdir build && cd build
cmake ../
make
sudo make install
```

**Note:** To enable DEBUG messages, use the following build commands instead:

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=DEBUG ../
make
sudo make install
```

If you would like to go back to the non-DEBUG mode, you would have to remove all contents of the `build` folder.


To build a docker image for the RTI, do 
```bash
docker build -t rti:rti -f rti.Dockerfile ../../../core/
```