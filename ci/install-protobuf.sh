#!/bin/sh
set -ex
echo -n "Installing protobufs......"
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protobuf-all-3.12.3.tar.gz
tar -xzvf protobuf-all-3.12.3.tar.gz
cd protobuf-3.12.3 && ./configure --prefix=/usr && make && sudo make install

wget https://github.com/protobuf-c/protobuf-c/releases/download/v1.3.3/protobuf-c-1.3.3.tar.gz
tar -xzvf protobuf-c-1.3.3.tar.gz
cd protobuf-c-1.3.3 && ./configure && make && sudo make install
