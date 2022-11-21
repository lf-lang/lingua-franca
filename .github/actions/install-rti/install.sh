#!/bin/bash
cd org.lflang/src/lib/c/reactor-c/core/federated/RTI
mkdir build
echo bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
cd build
# if [[ "$OSTYPE" == "darwin"* ]]; then
#     cmake -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl -DAUTH=ON ../
# else
#     cmake -DAUTH=ON ../
# fi
echo AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
echo $OPENSSL_ROOT_DIR
cmake -DAUTH=ON ../
make
sudo make install
