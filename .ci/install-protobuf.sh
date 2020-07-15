#!/bin/sh
set -ex
echo -n "Installing protobufs......"
proto_version=protobuf-3.12.3
proto_c_version=protobuf-c-1.3.3

#if [ ! -d "$proto_version" ]; then
#    wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protobuf-all-3.12.3.tar.gz
#    tar -xzvf protobuf-all-3.12.3.tar.gz
#fi
#cd $proto_version && ./configure --prefix=/usr && make && sudo make install

if [ ! -d "$proto_c_version" ]; then
    wget https://github.com/protobuf-c/protobuf-c/releases/download/v1.3.3/protobuf-c-1.3.3.tar.gz
    tar -xzvf protobuf-c-1.3.3.tar.gz
fi
cd $proto_c_version && ./configure && make && sudo make install
