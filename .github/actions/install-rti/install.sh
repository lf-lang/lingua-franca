#!/bin/bash
cd org.lflang/src/lib/c/reactor-c/core/federated/RTI
mkdir build
cd build
cmake -DAUTH=ON ../
make
sudo make install
