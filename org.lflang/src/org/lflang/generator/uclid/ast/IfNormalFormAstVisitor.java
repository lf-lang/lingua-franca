package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.List;

/**
 * An AST visitor that converts an original AST into the If Normal Form.
 * See "Bounded Model Checking of Software using SMT Solvers instead of
 * SAT Solvers" for details about the If Normal Form 
 * (https://link.springer.com/chapter/10.1007/11691617_9).
 * 
 * There are several requirements for an AST to be in INF:
 * 1. There should be a single StatementSequence (SS) root node;
 * 2. Each child of the SS node should be an IfBlockNode;
 * 3. Variables in a subsequent child should be marked as "primed"
 *    versions of those in the previous child.
 * 
 * Limitations:
 * 1. The implementation does not take the scope into account.
 *    E.g. "int i = 0; { int j = 0; }" is treated the same as
 *    "int i = 0; int j = 0;".
 * 2. Due to the above limitation, the implementation assumes
 *    that each variable has a unique name.
 *    E.g. "{ int i = 0; }{ int i = 0; }" is an ill-formed program.
 * 
 * In this program, visit() is the normalise() in the paper.
 */
public class IfNormalFormAstVisitor extends CBaseAstVisitor<Void> {

    public CAst.StatementSequenceNode INF = new CAst.StatementSequenceNode();
    
    @Override
    public Void visitStatementSequenceNode(CAst.StatementSequenceNode node, List<CAst.AstNode> conditions) {
        // Create a new StatementSequenceNode.        
        for (CAst.AstNode child : node.children) {
            visit(child, conditions);
        }
        return null;
    }

    @Override
    public Void visitAssignmentNode(CAst.AssignmentNode node, List<CAst.AstNode> conditions) {
        this.INF.children.add(generateIfBlock(node, conditions));
        return null;
    }

    @Override
    public Void visitSetPortNode(CAst.SetPortNode node, List<CAst.AstNode> conditions) {
        this.INF.children.add(generateIfBlock(node, conditions));
        return null;
    }

	@Override
	public Void visitScheduleActionNode(CAst.ScheduleActionNode node, List<CAst.AstNode> conditions) {
		this.INF.children.add(generateIfBlock(node, conditions));
        return null;
	}

    @Override
    public Void visitIfBlockNode(CAst.IfBlockNode node, List<CAst.AstNode> conditions) {
        List<CAst.AstNode> leftConditions = new ArrayList<>(conditions);
        leftConditions.add(node.left);
        visit(((CAst.IfBodyNode)node.right).left, leftConditions);
        if (((CAst.IfBodyNode)node.right).right != null) {
            List<CAst.AstNode> rightConditions = new ArrayList<>(conditions);
            // Add a not node.
            CAst.LogicalNotNode notNode = new CAst.LogicalNotNode();
            notNode.child = node.left; // Negate the condition.
            rightConditions.add(notNode);
            visit(((CAst.IfBodyNode)node.right).right, rightConditions);
        }
        return null;
    }

    private CAst.IfBlockNode generateIfBlock(CAst.AstNode node, List<CAst.AstNode> conditions) {
        // Create an If Block node.
        CAst.IfBlockNode ifNode = new CAst.IfBlockNode();
        // Set the condition of the if block node.
        CAst.AstNode conjunction = CAstUtils.takeConjunction(conditions);
        ifNode.left = conjunction;
        // Create a new body node.
        CAst.IfBodyNode body = new CAst.IfBodyNode();
        ifNode.right = body;
        // Set the then branch to the assignment.
        body.left = node;

        return ifNode;
    }
}
