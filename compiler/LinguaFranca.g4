
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

compositeStatement : inp
    | outp
    ;

inp : 'input' ID ':' type ';' ;
outp : 'output' ID ':' type ';' ;
trig : 'trigger' ID '(' trigparam ',' trigtype ')' ';' ;

trigparam : ID ;
trigtype : 'PERIODIC' | 'ONCE' ;

pre : 'preamble' code ;
init : 'initialize' code ;
react : 'reaction' '(' ID ')' sets* code ;

sets : '->' ID ;

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

