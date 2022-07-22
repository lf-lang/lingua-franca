package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.List;

import org.lflang.generator.uclid.ast.CAst.*;

/**
 * This visitor marks certain variable node as "previous."
 */
public class VariablePrecedenceVisitor extends CBaseAstVisitor<Void> {

    // This is a temporary solution and cannot handle,
    // e.g., self->s = (self->s + 1) - (2 * 2).
    @Override
    public Void visitAssignmentNode(AssignmentNode node) {
        System.out.println("******* In assignment!!!");
        if (node.left instanceof StateVarNode) {
            if (node.right instanceof AstNodeBinary) {
                AstNodeBinary n = (AstNodeBinary)node.right;
                if (n.left instanceof StateVarNode) {
                    ((StateVarNode)n.left).prev = true;
                }
                if (n.right instanceof StateVarNode) {
                    ((StateVarNode)n.right).prev = true;
                }
            }
        } else {
            System.out.println("Unreachable!"); // FIXME: Throw exception.
        }
        return null;
    }

    @Override
    public Void visitIfBlockNode(IfBlockNode node) {
        visit(((IfBodyNode)node.right).left);
        visit(((IfBodyNode)node.right).right);
        return null;
    }

    @Override
    public Void visitStatementSequenceNode(StatementSequenceNode node) {
        for (int i = 0; i < node.children.size(); i++) {
            visit(node.children.get(i));
        }
        return null;
    }
}
