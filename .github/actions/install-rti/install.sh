#!/bin/bash
cd org.lflang/src/lib/c/reactor-c/core/federated/RTI
mkdir build
cd build
echo "TEST1"
if [[ "$OSTYPE" == "darwin"* ]]; then
    cmake  -DAUTH=ON ../
    echo "TEST2"
else
    cmake -DAUTH=ON ../
    echo "TEST3"
fi
make
sudo make install


# -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl