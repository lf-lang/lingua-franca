parser grammar MTLParser;

options { tokenVocab=MTLLexer; }

mtl
    : equivalence
    ;

equivalence
    : left=implication ( EQUI right=implication )?
    ;

implication
    : left=disjunction ( IMPL right=disjunction )?
    ;

disjunction
    : terms+=conjunction ( LOR terms+=conjunction )*
    ;

conjunction
    : terms+=binaryOp ( LAND terms+=binaryOp )*
    ;

binaryOp
    : left=unaryOp ( UNTIL timeInterval=interval right=unaryOp )? # Until
    ;

unaryOp
    : formula=primary # NoUnaryOp
    | NEGATION formula=primary # Negation
    | NEXT timeInterval=interval formula=primary # Next
    | GLOBALLY timeInterval=interval formula=primary # Globally
    | FINALLY timeInterval=interval formula=primary # Finally
    ;

primary
    : atom=atomicProp
    | id=ID
    | LPAREN formula=mtl RPAREN
    ;

atomicProp
    : primitive=TRUE
    | primitive=FALSE
    | left=expr op=relOp right=expr
    ;

interval
    : (LPAREN|LBRACKET) lowerbound=time COMMA upperbound=time (RPAREN|RBRACKET) # Range
    | LBRACKET instant=time RBRACKET # Singleton
    ;

time
    : value=INTEGER (unit=ID)?
    ;

sum 
    : terms+=difference (PLUS terms+=difference)*
    ;

difference
    : terms+=product (MINUS terms+=product)*
    ;

product
    : terms+=quotient (TIMES terms+=quotient)*
    ;

quotient
    : terms+=expr (DIV terms+=expr)*
    ;

relOp
    : EQ | NEQ | LT | LE | GT | GE
    ;

expr
    : ID
    | LPAREN sum RPAREN
    | INTEGER
    ;
