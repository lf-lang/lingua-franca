lexer grammar MTLLexer;

COMMA
    : ','
    ;

LPAREN
    : '('
    ;

RPAREN
    : ')'
    ;

LBRACKET
    : '['
    ;

RBRACKET
    : ']'
    ;

LAND
    : '&&'
    ;

LOR
    : '||'
    ;

EQUI
    : '<==>'
    ;

IMPL
    : '==>'
    ;

UNTIL
    : 'U'
    ;

NEGATION
    : '!'
    ;

NEXT
    : 'X'
    ;

GLOBALLY
    : 'G'
    ;

FINALLY
    : 'F'
    ;

WS
    : [ \t\r\n]+ -> skip
    ;

TRUE
    : 'true'
    ;

FALSE
    : 'false'
    ;

PLUS
    : '+'
    ;

MINUS
    : '-'
    ;

TIMES
    : '*'
    ;

DIV
    : '/'
    ;

EQ
    : '=='
    ;

NEQ
    : '!='
    ;

LT
    : '<'
    ;

LE
    : '<='
    ;

GT
    : '>'
    ;

GE
    : '>='
    ;

INTEGER
    : [0-9]+
    ;

ID
    : ([a-zA-Z0-9]|'_')+
    ;
