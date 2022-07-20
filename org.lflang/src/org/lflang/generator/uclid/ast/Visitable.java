package org.lflang.generator.uclid.ast;

import java.util.List;

public interface Visitable {
    
    /** The {@link AstVisitor} needs a double dispatch method. */
    <T> T accept(AstVisitor<? extends T> visitor);

    /** The {@link AstVisitor} needs a double dispatch method. */
    <T> T accept(AstVisitor<? extends T> visitor, List<CAst.AstNode> nodeList);
}
