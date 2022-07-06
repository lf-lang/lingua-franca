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
    : left=unaryOp (UNTIL timeInterval=interval right=unaryOp)?
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
    ;

interval
    : (LPAREN|LBRACKET) lowerbound=time COMMA upperbound=time (RPAREN|RBRACKET) # Range
    | LBRACKET instant=time RBRACKET # Singleton
    ;

time
    : (ZERO | value=INTEGER unit=ID)
    ;
