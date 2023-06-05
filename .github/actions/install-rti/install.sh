#!/bin/bash
cd core/src/main/resources/lib/c/reactor-c/core/federated/RTI
mkdir build
cd build
if [[ "$OSTYPE" == "darwin"* ]]; then
    export OPENSSL_ROOT_DIR="/usr/local/opt/openssl"
fi
cmake -DAUTH=ON ../
make
sudo make install
