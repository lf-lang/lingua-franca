/** Launcher for a gRPC server for the Greeter example.
 *  This file creates and starts a server providing the service defined
 *  in the Greeter.proto file. The server listens on port 50051 of the local host.
 *  To start the server, just do:
 *  
 *    node startServer.js
 *
 *  @authors: Ravi Akella and Edward A. Lee
 */
var grpc = require('grpc');
var protoLoader = require('@grpc/proto-loader');
var util = require('util');

var packageDefinition = protoLoader.loadSync(
    './Greeter.proto',
    {keepCase: true,
       longs: String,
       enums: String,
       defaults: true,
    });
var packageObject = grpc.loadPackageDefinition(packageDefinition);

// Start the server.
var server = new grpc.Server();
server.addService(packageObject.Greeter.service, {sayHello: sayHelloImplementation});
server.bind('0.0.0.0:50051', grpc.ServerCredentials.createInsecure());
server.start();
console.log('gRPC server started.');

/** Implement the sayHello RPC method of the service. */
function sayHelloImplementation(call, callback) {
  console.log('Request from ' + call.getPeer() + ' for sayHello method ' + ' with args ' + util.inspect(call.request));
  callback(null, {message: 'Hello ' + call.request.name});
}

// Note that this does not shutdown the server.