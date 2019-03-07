
grammar LinguaFranca;
sys : lang imp* (actor | composite);
lang : 'language' ID ';' ;
imp : 'import' path ';' ;
actor : head '{' body '}' ;
composite : compositeHead '{' compositeBody '}' ;
head : 'actor' ID '(' param* ')' ;
compositeHead : 'composite' ID '(' param* ')' ;
param : ID ':' type def? ;
def : '(' INTVAL ')' ;
body : stat* ;
compositeBody : compositeStatement* ;

stat : inp
     | outp
     | trig
     | pre
     | init
     | react
     ;

compositeStatement : stat
    | instance
    ;

inp : 'input' ID ':' type ';' ;
outp : 'output' ID ':' type ';' ;
trig : 'trigger' ID '(' trigparam ',' trigtype ')' ';' ;

instance : 'instance' ID '=' ID '(' assignments? ')' ';' ;
assignments : assignment | assignments ',' assignment;
assignment : ID '=' value;
value : ID | NUMBER | bracketed;

// FIXME: Can we replace this with something more general? E.g., scientific notation, hex, etc.?
NUMBER : '-'? INTVAL ('.' INTVAL)? ;

// FIXME: String isn't right. Doesn't support escaping.
bracketed : ('[' .*? ']') | ('{' .*? '}') | '"' .*? '"';

trigparam : ID ;
trigtype : 'PERIODIC' | 'ONCE' ;

pre : 'preamble' code ;
init : 'initialize' code ;
react : 'reaction' '(' ID ')' sets* code ;

sets : '->' ID ; // FIXME: What if multiple outputs are written to?

code : CODE ;

block : ~'}'* ;

path : ID | path '.' ID ;

type : INT | STR ;

INT : 'int' ;
STR : 'string' ;
IN : 'input' ;
OUT : 'output' ;
INTVAL : [0-9]+ ;
ID : [a-zA-Z]+ ;          // match identifiers
WS : [ \t\r\n]+ -> skip ; // skip spaces, tabs, newlines
NEWLINE : 'r'? '\n' ;
// CMT : '/' '/' (.)*? NEWLINE -> skip ;

CODE : '{-' .*? '-}' ;

LINE_COMMENT : '//' ~[\r\n]* NEWLINE -> channel(HIDDEN) ;
COMMENT : '/*' .*? '*/' -> channel(HIDDEN) ;

