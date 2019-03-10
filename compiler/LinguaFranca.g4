grammar LinguaFranca;
sys : target imp* (actor | composite);
target:
    'target' ID ';' ;

imp : 'import' path ';' ;
actor : head '{' body '}' ;
composite : compositeHead '{' compositeBody '}' ;
head : 'actor' ID params? ;
compositeHead : 'composite' ID '(' param* ')' ;

params:
    '(' param (',' param)* ')';
param:
    ID ':' type ('(' value ')')?;

value:
    ID | NUMBER | STRING | code;

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
    | connection
    ;

connection:
    lport '->' rport ';';
lport: port;
rport: port;
port:
    ID | (ID '.' (ID | IN | OUT)); // 'input' and 'output' are common port names.

inp : IN ID ':' type ';' ;
outp : OUT ID ':' type ';' ;
trig : 'trigger' ID '(' trigparam ',' trigtype ')' ';' ;

instance : 'instance' ID '=' actorClass '(' assignments? ')' ';' ;
actorClass: ID;
assignments : assignment | assignments ',' assignment;
assignment : ID '=' value;

trigparam : ID ;
trigtype : 'PERIODIC' | 'ONCE' ;

pre : 'preamble' code ;
init : 'initialize' code ;
react : 'reaction' '(' ID ')' sets* code ;

code: CODE;

sets : '->' ID ; // FIXME: What if multiple outputs are written to?

block : ~'}'* ;

path : ID | path '.' ID ;

// A type is in the target language, hence either an ID or target code.
type:
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
CODE:
    '{=' (ESCAPED_CODE | ~('\\')* ) '=}' ;

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

