grammar LinguaFranca;
sys : target lfImport* (actor | composite);
target:
    'target' ID ';' ;

lfImport : 'import' path ';' ;
actor : head '{' body '}' ;
composite : compositeHead '{' compositeBody '}' ;
head : 'actor' ID parameters? ;
compositeHead : 'composite' ID parameters? ;

parameters:
    '(' parameter (',' parameter)* ')';
parameter:
    ID ':' lfType ('(' lfValue ')')?;

lfValue:
    ID | NUMBER | STRING | code;

body : statement* ;
compositeBody : compositeStatement* ;

statement : input
     | output
     | trigger
     | preamble
     | initialize
     | reaction
     ;

compositeStatement : statement
    | instance
    | connection
    ;

connection:
    lport '->' rport ';';
lport: port;
rport: port;
port:
    ID | (ID '.' (ID | IN | OUT)); // 'input' and 'output' are common port names.

input : IN ID ':' lfType ';' ;
output : OUT ID ':' lfType ';' ;
trigger : 'trigger' ID '(' triggerParameter ',' triggerType ')' ';' ;

instance : 'instance' ID '=' lfActorClass '(' assignments? ')' ';' ;
lfActorClass: ID;
assignments : assignment | assignments ',' assignment;
assignment : ID '=' lfValue;

triggerParameter : ID ;
triggerType : 'PERIODIC' | 'ONCE' ;

preamble : 'preamble' code ;
initialize : 'initialize' code ;
reaction : 'reaction' '(' ID ')' sets* code ;

code: CODE;

sets : '->' ID ; // FIXME: What if multiple outputs are written to?

block : ~'}'* ;

path : ID | path '.' ID ;

// A lfType is in the target language, hence either an ID or target code.
lfType:
    ID | CODE ;

IN : 'input' ;
OUT : 'output' ;

// FIXME: Why does the following match an empty string?
ID:
    [a-zA-Z]+[a-zA-Z0-9]* ;          // match identifiers

WS:
    [ \t\r\n]+ -> skip ; // skip spaces, tabs, newlines

NEWLINE:
    'r'? '\n' ;
    
// CMT : '/' '/' (.)*? NEWLINE -> skip ;

fragment ESCAPED_CODE:
    '\\=}';
// FIXME: the following matches now all code into one token, breaks Source.lf
// What is the use of escape?
//CODE:
//    '{=' (ESCAPED_CODE | ~('\\')* ) '=}' ;

// Back to original CODE definition for now until above is fixed
CODE:
    '{=' .*? '=}' ;

fragment INTVAL:
    '-'? [0-9]+ ;
// FIXME: Can we replace this with something more general? E.g., scientific notation, hex, etc.?
NUMBER:
    '-'? INTVAL ('.' INTVAL)? ;

fragment ESCAPED_CHAR:
    '\\' ('n'|'t'|'r'|'\\');
STRING:
    '"' ( ESCAPED_CHAR | ~('\\'|'"') )* '"';

LINE_COMMENT:
    '//' ~[\r\n]* NEWLINE -> channel(HIDDEN) ;
    
COMMENT:
    '/*' .*? '*/' -> channel(HIDDEN) ;

