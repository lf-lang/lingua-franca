var lf = require('./lf');
var fs = require('fs');


function print(object){
    console.log(JSON.stringify(object,null,' '));
}

function asWirewrap(tree){
    console.log("// @flow");
    console.log("'use strict';");
    console.log("import {Component, ReActor, InPort, OutPort, Clock, Reaction, App} from './reactor';");
    console.log("type int = number;");
    tree.value.forEach(function(node) {
        //console.log(JSON.stringify(tree,null,' '));
        (_printChildren({}))(node);
    });
    console.log("}");
}

function _printChildren(options = {}){
	 return function (elem){
		  //console.log(elem);
		  /*
			 if(elem['type'] == undefined){
			 console.error("Not implemented parse tree for:");
			 console.error(JSON.stringify(elem,null,' '));
			 throw new Error();
			 }*/
		  if (elem == undefined){
				return "";
		  }
		  switch(elem['type']){
		  case 'reactor':
				console.log("class " + elem['header']['id'] + " extends Component implements ReActor{");
				var variables = [];
				var args = elem['header']['args'];
				if(args != undefined){
					 for( var i = 0; i < args.length; i++){
						  variables.push(args[i]);
						  console.log(args[i]['id'] + " : " + args[i]['vartype'] + ";");
					 }
				}
				var printChildren = _printChildren({args: args});
				var b = elem['body'];
				b.forEach(printChildren);

				console.log("_reactions = [["); 
				var object_strs = []
				var class_strs = []
				var reacts = 0;
				for(var i = 0; i < b.length ; i ++){
					 switch(b[i]['type']){
					 case 'clock':
						  object_strs.push("[this.clock]");
						  break;
					 case 'reaction':
						  var obj_str = "new " + elem['header']['id'] + "Reaction" + reacts.toString() + "([";
						  var class_str = "class " + elem['header']['id'] + "Reaction" + reacts.toString();
						  class_str += " extends Reaction<[";

						  //outputs from reaction
						  var routs = b[i]['target'];
						  if(routs != null){
						  for(var j = 0; j < routs.length; j++){
								obj_str += "this." + routs[j]['id'];
								if(routs[j]['vartype'] != null)
									 class_str += routs[j]['vartype'];
								if(j+1 < routs.length)
									 obj_str += ", ";
									 class_str += ", ";
						  }
						  }
						  else{
								class_str += "*";
						  }
						  class_str += "],{"
						  obj_str += "],{"

						  //inputs to reaction
						  var rargs = b[i]['args'];
						  for(var j = 0; j < rargs.length; j++){
								obj_str += "this." + rargs[j]['id'];
								if(rargs[j]['vartype'] != null)
									 class_str += rargs[j]['vartype'] + " : " + rargs[j]['id'];
								if(j+1 < rargs.length)
									 obj_str += ", ";
						  }

						  obj_str += ")";
						  class_str += "}>{"
						  class_str += b[i]['embedded']['value'];
						  class_str += "}"
						  object_strs.push(obj_str);
						  class_strs.push(class_str);
						  reacts++;
						  break;
					 }
				}
				console.log(object_strs.toString() + "]];")
				console.log("}");

				for(var i = 0; i < class_strs.length ; i ++){
					 console.log(class_strs[i]);
				}
				
				break;
		  case 'reaction':
				//This is handled in the reactor code. A reaction only makes sense in the context of a reactor.
				break;
		  case 'constructor':
				var strg = "constructor(";
				var args = options['args'];
				if(args != undefined){
					 for(var i = 0; i < args.length; i++){
						  strg += args[i]['id'];
						  if(args[i]['vartype'] != undefined)
								strg += ":" + args[i]['vartype'];
						  if(args[i]['init'] != null)
								strg += " = " + args[i]['init'].toString();
						  if(i+1 < args.length)
								strg += ", ";
					 }
				}
				strg += "){";
				console.log(strg);
				console.log(elem['value']['value']);
				console.log("}");
				break;
								
		  case 'composite':
				var id = elem['header']['id'];
				console.log("class My" + id + " extends " + id + " {");
				process.stdout.write("constructor(");
				var printChildren = _printChildren();
				printChildren(elem['header']['args']);
				console.log("){");

				printChildren(elem['body']);
				console.log("}");
				console.log("}");
				break;
		  case 'call':
				process.stdout.write(elem['value']['id'] + "(")
				var printChildren = _printChildren();
				elem['value']['args'].forEach(printChildren);
				console.log(");");
				break;
		  case 'assignment':
				process.stdout.write("var " + elem['id'] + " = new "); //this new here should probably not be always there?
				var printChildren = _printChildren();
				printChildren(elem['expression']);
				console.log(";");
				break;
		  case 'port':
				console.log(elem['dst']['value'] + ".output.connect(" + elem['src']['value'] + ".input);"); 
				break;
		  case 'input':
				console.log("input: InPort<" + elem['params']['id'] + ">;");
				break;
		  case 'output':
				console.log("output: OutPort<" + elem['params']['id'] + ">;");
				break;
		  case 'clock':
				console.log("clock: Clock  = new Clock(this." + elem['header']['id'] + ");");
				break;
		  case 'embedded':
				console.log(elem['value']);
            break;
				
            
		  default:
				if (elem['elements'] == undefined){
					 return
				}
				else{
					 elem['elements'].forEach(_printChildren);
				}
		  }
	 }
}


var src_file = process.argv[2];
if(src_file == null){
	 return console.log("usage: " + process.argv[1] + " <file>");
}
fs.readFile(src_file, 'utf8', function (err,data) {
	 if (err) {
		  return console.log(err);
	 }
	 var tree = lf.parse(data);
	 asWirewrap(tree);

});

