# Demonstration of a gRPC Service with Lingua Franca Reactors

This directory contains support files for a demonstration that creates a gRPC
(Google RPC) service that is used by the Greeter.lf Lingua Franca app.

To run this, you need to use npm to install some modules. However, you
should not do this in this directory!!  Unfortunately, if you do it in this
directory, npm will remove a large number of files that are needed in the
directories containing this one.  Hence, you should create a working copy
somewhere else.  For example, assuming you start in this directory (the one
with this README file), you can do this:

    mkdir ~/grpcTest
    cp * ~/grpcTest
    cd ~/grpcTest
    npm install grpc
    npm install @grpc/proto-loader

These can be installed in the directory containing this file or
any directory above it in the file system hierarchy.
Then do (in the directory containing this file):

    node ./startServer.js

This will start the gRPC server. You can then generate code and run Greeter.lf,
which provides the client.