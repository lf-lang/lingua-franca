package org.lflang.generator.uclid.ast;

import java.util.List;

/** Modeled after CBaseVisitor.java */
public class CBaseAstVisitor<T> extends AbstractAstVisitor<T> implements CAstVisitor<T> {
    
	/** 
	 * These default implementations are not meant to be used.
	 * They should be overriden by the child class.
	 * In theory, this base visitor can be deleted?
	 * Let's keep it here for now for consistency.
	 */
	@Override
	public T visitAstNode(CAst.AstNode node) {
		System.out.print("[visitAstNode] ");
		System.out.println("Hi, I am " + node);
		return null;
	}

	@Override
	public T visitAstNodeUnary(CAst.AstNodeUnary node) {
		System.out.print("[visitAstNodeUnary] ");
		System.out.println("Hi, I am " + node);
		if (node.child != null) {
			T result = visit(node.child);
		} else {
			System.out.println("*** Child is empty in " + node + "!");
		}
		return null;
	}

	@Override
	public T visitAstNodeBinary(CAst.AstNodeBinary node) {
		System.out.print("[visitAstNodeBinary] ");
		System.out.println("Hi, I am " + node);
		if (node.left != null) {
			T leftResult = visit(node.left);
		} else System.out.println("*** Left child is empty in " + node + "!");
		if (node.right != null) {
			T rightResult = visit(node.right);
		} else System.out.println("*** Right child is empty in " + node + "!");
		// Aggregate results...
		return null;
	}

	@Override
	public T visitAstNodeDynamic(CAst.AstNodeDynamic node) {
		System.out.print("[visitAstNodeDynamic] ");
		System.out.println("Hi, I am " + node);
		for (CAst.AstNode n : node.children) {
			T result = visit(n);
		}
		return null;
	}

    @Override
	public T visitAssignmentNode(CAst.AssignmentNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitAssignmentNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitIfBlockNode(CAst.IfBlockNode node) {
		System.out.print("[visitIfBlockNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitIfBodyNode(CAst.IfBodyNode node) {
		System.out.print("[visitIfBodyNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitLiteralNode(CAst.LiteralNode node) {
		System.out.print("[visitLiteralNode] ");
		System.out.println("Hi, I am " + node + " with literal " + node.literal);
		return null;
	}

	@Override
	public T visitLogicalNotNode(CAst.LogicalNotNode node) {
		System.out.print("[visitLogicalNotNode] ");
		return visitAstNodeUnary(node);
	}

	@Override
	public T visitLogicalAndNode(CAst.LogicalAndNode node) {
		System.out.print("[visitLogicalAndNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitLogicalOrNode(CAst.LogicalOrNode node) {
		System.out.print("[visitLogicalOrNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitOpaqueNode(CAst.OpaqueNode node) {
		System.out.print("[visitOpaqueNode] ");
		return visitAstNode(node);
	}

	@Override
	public T visitStatementSequenceNode(CAst.StatementSequenceNode node) {
		System.out.print("[visitStatementSequenceNode] ");
		return visitAstNodeDynamic(node);
	}

	@Override
	public T visitVariableNode(CAst.VariableNode node) {
		System.out.print("[visitVariableNode] ");
		System.out.println("Hi, I am " + node + ": (" + node.type + ", " + node.name + ")");
		return null;
	}

	/** Arithmetic operators */

	@Override
	public T visitAdditionNode(CAst.AdditionNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitAdditionNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitSubtractionNode(CAst.SubtractionNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitSubtractionNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitMultiplicationNode(CAst.MultiplicationNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitMultiplicationNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitDivisionNode(CAst.DivisionNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitDivisionNode] ");
		return visitAstNodeBinary(node);
	}

	/** Comparison operators */

	@Override
	public T visitEqualNode(CAst.EqualNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitEqualNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitNotEqualNode(CAst.NotEqualNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitNotEqualNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitLessThanNode(CAst.LessThanNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitLessThanNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitLessEqualNode(CAst.LessEqualNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitLessEqualNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitGreaterThanNode(CAst.GreaterThanNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitGreaterThanNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitGreaterEqualNode(CAst.GreaterEqualNode node) {
		// The default implementation reuses visitAstNodeBinary(node).
		System.out.print("[visitGreaterEqualNode] ");
		return visitAstNodeBinary(node);
	}

	/** LF built-in operations */

	@Override
	public T visitSetPortNode(CAst.SetPortNode node) {
		System.out.print("[visitSetPortNode] ");
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitScheduleActionNode(CAst.ScheduleActionNode node) {
		System.out.print("[visitScheduleActionNode] ");
		return visitAstNodeDynamic(node);
	}

	@Override
	public T visitStateVarNode(CAst.StateVarNode node) {
		System.out.print("[visitStateVarNode] ");
		System.out.println("Hi, I am " + node + ": (" + node.name + ")");
		return null;
	}

	@Override
	public T visitTriggerValueNode(CAst.TriggerValueNode node) {
		System.out.print("[visitTriggerValueNode] ");
		System.out.println("Hi, I am " + node + ": (" + node.name + ")");
		return null;
	}

	@Override
	public T visitTriggerIsPresentNode(CAst.TriggerIsPresentNode node) {
		System.out.print("[visitTriggerIsPresentNode] ");
		System.out.println("Hi, I am " + node + ": (" + node.name + ")");
		return null;
	}

	//// With one more parameter.
	@Override
	public T visitAstNode(CAst.AstNode node, List<CAst.AstNode> nodeList) {
		return visitAstNode(node);
	}

	@Override
	public T visitAstNodeUnary(CAst.AstNodeUnary node, List<CAst.AstNode> nodeList) {
		return visitAstNodeUnary(node);
	}

	@Override
	public T visitAstNodeBinary(CAst.AstNodeBinary node, List<CAst.AstNode> nodeList) {
		return visitAstNodeBinary(node);
	}

	@Override
	public T visitAstNodeDynamic(CAst.AstNodeDynamic node, List<CAst.AstNode> nodeList) {
		return visitAstNodeDynamic(node);
	}

	@Override
	public T visitAssignmentNode(CAst.AssignmentNode node, List<CAst.AstNode> nodeList) {
		return visitAssignmentNode(node);
	}

	@Override
	public T visitIfBlockNode(CAst.IfBlockNode node, List<CAst.AstNode> nodeList) {
		return visitIfBlockNode(node);
	}

	@Override
	public T visitIfBodyNode(CAst.IfBodyNode node, List<CAst.AstNode> nodeList) {
		return visitIfBodyNode(node);
	}

	@Override
	public T visitLiteralNode(CAst.LiteralNode node, List<CAst.AstNode> nodeList) {
		return visitLiteralNode(node);
	}

	@Override
	public T visitLogicalNotNode(CAst.LogicalNotNode node, List<CAst.AstNode> nodeList) {
		return visitLogicalNotNode(node);
	}

	@Override
	public T visitLogicalAndNode(CAst.LogicalAndNode node, List<CAst.AstNode> nodeList) {
		return visitLogicalAndNode(node);
	}

	@Override
	public T visitLogicalOrNode(CAst.LogicalOrNode node, List<CAst.AstNode> nodeList) {
		return visitLogicalOrNode(node);
	}

	@Override
	public T visitOpaqueNode(CAst.OpaqueNode node, List<CAst.AstNode> nodeList) {
		return visitOpaqueNode(node);
	}

	@Override
	public T visitStatementSequenceNode(CAst.StatementSequenceNode node, List<CAst.AstNode> nodeList) {
		return visitStatementSequenceNode(node);
	}

	@Override
	public T visitVariableNode(CAst.VariableNode node, List<CAst.AstNode> nodeList) {
		return visitVariableNode(node);
	}

	/** Arithmetic operators */
	
	@Override
	public T visitAdditionNode(CAst.AdditionNode node, List<CAst.AstNode> nodeList) {
		return visitAdditionNode(node);
	}

	@Override
	public T visitSubtractionNode(CAst.SubtractionNode node, List<CAst.AstNode> nodeList) {
		return visitSubtractionNode(node);
	}

	@Override
	public T visitMultiplicationNode(CAst.MultiplicationNode node, List<CAst.AstNode> nodeList) {
		return visitMultiplicationNode(node);
	}

	@Override
	public T visitDivisionNode(CAst.DivisionNode node, List<CAst.AstNode> nodeList) {
		return visitDivisionNode(node);
	}

	/** Comparison operators */
	
	@Override
	public T visitEqualNode(CAst.EqualNode node, List<CAst.AstNode> nodeList) {
		return visitEqualNode(node);
	}

	@Override
	public T visitNotEqualNode(CAst.NotEqualNode node, List<CAst.AstNode> nodeList) {
		return visitNotEqualNode(node);
	}

	@Override
	public T visitLessThanNode(CAst.LessThanNode node, List<CAst.AstNode> nodeList) {
		return visitLessThanNode(node);
	}

	@Override
	public T visitLessEqualNode(CAst.LessEqualNode node, List<CAst.AstNode> nodeList) {
		return visitLessEqualNode(node);
	}

	@Override
	public T visitGreaterThanNode(CAst.GreaterThanNode node, List<CAst.AstNode> nodeList) {
		return visitGreaterThanNode(node);
	}

	@Override
	public T visitGreaterEqualNode(CAst.GreaterEqualNode node, List<CAst.AstNode> nodeList) {
		return visitGreaterEqualNode(node);
	}

	/** LF built-in operations */

	@Override
	public T visitSetPortNode(CAst.SetPortNode node, List<CAst.AstNode> nodeList) {
		return visitSetPortNode(node);
	}

	@Override
	public T visitScheduleActionNode(CAst.ScheduleActionNode node, List<CAst.AstNode> nodeList) {
		return visitScheduleActionNode(node);
	}

	@Override
	public T visitStateVarNode(CAst.StateVarNode node, List<CAst.AstNode> nodeList) {
		return visitStateVarNode(node);
	}

	@Override
	public T visitTriggerValueNode(CAst.TriggerValueNode node, List<CAst.AstNode> nodeList) {
		return visitTriggerValueNode(node);
	}

	@Override
	public T visitTriggerIsPresentNode(CAst.TriggerIsPresentNode node, List<CAst.AstNode> nodeList) {
		return visitTriggerIsPresentNode(node);
	}

}
