// A grammar file to be used with pegjs
// See: https://pegjs.org/documentation
//
// To build:
// `npm install -g pegjs`
// `pegjs lf.pegjs 
//
// Copyright 2019 (see license)
// Authors: Marten Lohstroh, Andres Goens 
//

//grammar LinguaFranca

Grammar
	 = Spacing stmts:Stament+ {return { type : "root", value : stmts};}

//Core of the language
Stament
	 = Spacing stmt:Assignment Spacing {return stmt;}
	 / Spacing stmt:Declaration Spacing {return stmt;}
	 / Spacing stmt:EmbeddedStatement Spacing {return stmt;}
EmbeddedStatement
	 = "{=" stmt:( !"=" . / "=" !"}" .)* "=}" { return { type : "embedded", value : stmt.map( function(arr){if(arr.length == 2){return arr[1]}else{return (arr[0] + arr[2])}}).join("")};} 
Declaration
	 = decl:EntityDeclaration { return decl;}
	 / decl:PropertyDeclaration _ ";" {return decl;}
EntityDeclaration
	 =  Reactor / Reaction / Constructor / Composite / Preamble / Initialize
PropertyDeclaration
	 = Target / Import / Clock / Port / ReactorPrimitive / Instance 
Assignment
	 = expr:AssignmentExpression _ ";" {return expr;}
Expression
	 =  EmbeddedStatement / New / Call / id / Literal  //Not sure if "new" fits best here

//Reactor specific constructs
Target = "target" _ id:id {return {type : "target", id: id};}
//Language = "language" _ id //Added for old srcs from other repo
Import = "import" _ id:id {return {type : "import", id: id};}
New = _ "new" _ call:Call {return {type : "new", value: call};}
Call = id:id args:Arguments {return {type : "call", id: id, args: args};} 
Port = src:id _ "->" _ dst:id {return {type : "port", src : src, dst: dst};}
Reactor  = "reactor" _ h:Header _ b:Body {return {type : "reactor", header: h, body: b};}
Composite  = "composite" _ h:Header _ b:Body {return {type : "composite", header: h, body: b};}//What should composites be allowed precisely? //Just ids or also args?
Constructor  = "constructor" _ stmt:EmbeddedStatement {return {type : "constructor", value: stmt};}
Preamble  = "preamble" _ stmt:EmbeddedStatement {return {type : "preamble", value: stmt};}//Added for old srcs from other repo
Initialize  = "initialize" _ stmt:EmbeddedStatement {return {type : "initialize", value: stmt};}//Added for old srcs from other repo
Clock  = "clock" _ h:Header _ {return {type: "clock", header: h};}
Header
	 = id:id args:(_ DeclarationArgs)? {
		  if (args == null) var args = [];
		  return {type : "header", id: id, args:args[1]};
	 }
ReactorPrimitive = Input / Output / Parameter / Trigger / Action

//Reactor primitives
Input = 'input' _ params:DeclarationParam  { return {type : 'input', params:params};}
Output = 'output' _ params:DeclarationParam  { return {type : 'output', params:params};}
Parameter = 'parameter' _ params:DeclarationParam { return {type : 'parameter', params:params};}
Trigger =  'trigger' _ h:Header  { return {type : 'trigger', header:h};}
    / 'trigger' _ params:DeclarationParam { return {type : 'trigger', params:params};}
Action = 'action' _ params:DeclarationParam { return {type : 'action', params:params};}
Reaction
	 = 'reaction' args:DeclarationArgs trgt:(_ "->" _ DeclarationParamList  )? Spacing embedded:EmbeddedStatement {
		  if (trgt == null){
				return {type : 'reaction', args : args, target : null, embedded : embedded};
		  }
		  else{
				return {type : 'reaction', args : args, target : trgt[3], embedded : embedded};
		  }
	 }
Instance = 'instance' _ expr:AssignmentExpression  { return {type : 'instance', expr:expr};}//Added for old srcs from other repo

//General/auxiliary definitions
AssignmentExpression
	 = id:id _ "=" _ expr:Expression {return {type : "assignment", id:id, expr:expr};}
DeclarationArgs = "(" _ decls:DeclarationParamList _ ")" {return decls;}
Arguments = "(" _ args:ParamList _ ")" { return args;}
DeclarationParamList
	 = (params:DeclarationParam _ "," _)+ last:DeclarationParam { return params + [last];}
    / param: DeclarationParam? { return [param]; }
ParamList
	 = (params:Param _ "," _)+ last:Param { return params + [last];}
    / param:Param? { return param;}
DeclarationParam
	 = id:id cond:((":" Type _ "(" Expression ")") / (":" Type) )? {
		  if (cond == null){
				var init = null; 
				var type = null; 
		  }
		  else{

				if(cond.length == 6){
					 var init = cond[4];
					 var type = cond[1];
				}
				else{
					 var init = null;
					 var type = cond[1];
				}
		  }
		  return { type : "declparam", id : id, vartype : type, init: init};
	 }
Param
	 =  _ expr:Expression (_ "=" _ init:Literal)? _ {
		  if (typeof init == undefined) var init = null; 
		  return { type : "param", expr:expr, init: init};
	 }
Type = id:id { return id;}
Body = "{" Spacing stmt:Stament* "}" { return stmt;}
id = start:[a-zA-Z] rest:[a-zA-Z0-9\.]* { return start + rest.join('');}
Literal = NumLiteral / StringLiteral / EmbeddedLiteral
EmbeddedLiteral = EmbeddedStatement 
NumLiteral
	 = value:("-"? ([0-9]*)? "."? [0-9]+) { return parseFloat(value.join(''));}
    / value:[0-9]+ { return parseInt(value.join(''));}
StringLiteral
	 = "'" val:[^\'\"]+ "'" { return val.join('');}
    /  "\"" val:[^\"]+ "\"" { return val.join('');}
Spacing  = ([\n \t] / Comment)*
Comment  = '//' text:(!EndOfLine .)* EndOfLine {return {type: 'comment', value : text.join("") };}
_ = [\n \t]*
__ = [\n \t]+
EndOfLine  = '\r\n' / '\n' / '\r'
EndOfFile  = !.

