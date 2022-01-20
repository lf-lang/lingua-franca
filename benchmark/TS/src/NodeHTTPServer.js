// This pure JavaScript server implements the same
// HTTP server functionality as Server.lf
// (i.e. it responds to requests the same way)
// but without any of the Lingua Franca/Reactors overhead.
// 
// The default port is 8000, the first command line argument
// can be used to set a different port.
//
// You can obtain statistics on its performance by
// installing Clinic (https://www.npmjs.com/package/clinic) and
// AutoCannon (https://www.npmjs.com/package/autocannon)
// 
// $ npm install -g clinic
// $ npm i autocannon -g
//
// Then execute the following from the directory containing
// this .js file:
// 
// $ clinic doctor --autocannon [ 'localhost:8000' ] -- node NodeHTTPServer.js

let args = process.argv.slice(2);
let port = 8000;
if (args[0]) {
    port = parseInt(args[0]);
}

const http = require("http");
const querystring = require ("querystring");

let options = {};
var server = http.createServer(options, (req, res) => {
    res.writeHead(200);
    res.end(req.toString());
}).listen(port);