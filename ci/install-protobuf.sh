#!/bin/sh
set -ex
echo -n "Installing protobufs......"
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protobuf-all-3.12.3.tar.gz
tar -xzvf protobuf-all-3.12.3.tar.gz
protobuf-3.12.3 && ./configure --prefix=/usr && make && sudo make install


