
grammar LinguaFranca;
sys : actor ;
actor : head '{' body '}' ;
head : 'actor' ID '(' ')' ;
body : stat* ;
stat : ID '=' '123' ;

INT : 'int' ;
STR : 'string' ;
IN : 'input' ;
OUT : 'output' ;
INTVAL : [0-9]+ ;
ID : [A-Za-z]+ ;          // match identifiers
WS : [ \t\r\n]+ -> skip ; // skip spaces, tabs, newlines

