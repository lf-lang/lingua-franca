package org.lflang.analyses.c;

import java.util.List;

/**
 * A base class that provides default implementations of the visit functions. Other C AST visitors
 * extend this class.
 */
public class CBaseAstVisitor<T> extends AbstractAstVisitor<T> implements CAstVisitor<T> {

  /**
   * These default implementations are not meant to be used. They should be overriden by the child
   * class. In theory, this base visitor can be deleted? Let's keep it here for now for consistency.
   */
  @Override
  public T visitAstNode(CAst.AstNode node) {
    return null;
  }

  @Override
  public T visitAstNodeUnary(CAst.AstNodeUnary node) {
    if (node.child != null) {
      T result = visit(node.child);
    } else {
      System.out.println("*** Child is empty in " + node + "!");
    }
    return null;
  }

  @Override
  public T visitAstNodeBinary(CAst.AstNodeBinary node) {
    if (node.left != null) {
      T leftResult = visit(node.left);
    } else {
      System.out.println("*** Left child is empty in " + node + "!");
    }
    if (node.right != null) {
      T rightResult = visit(node.right);
    } else {
      System.out.println("*** Right child is empty in " + node + "!");
    }
    // Aggregate results...
    return null;
  }

  @Override
  public T visitAstNodeDynamic(CAst.AstNodeDynamic node) {
    for (CAst.AstNode n : node.children) {
      T result = visit(n);
    }
    return null;
  }

  @Override
  public T visitAssignmentNode(CAst.AssignmentNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitIfBlockNode(CAst.IfBlockNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitIfBodyNode(CAst.IfBodyNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitLiteralNode(CAst.LiteralNode node) {
    return null;
  }

  @Override
  public T visitLogicalNotNode(CAst.LogicalNotNode node) {
    return visitAstNodeUnary(node);
  }

  @Override
  public T visitLogicalAndNode(CAst.LogicalAndNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitLogicalOrNode(CAst.LogicalOrNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitOpaqueNode(CAst.OpaqueNode node) {
    return visitAstNode(node);
  }

  @Override
  public T visitStatementSequenceNode(CAst.StatementSequenceNode node) {
    return visitAstNodeDynamic(node);
  }

  @Override
  public T visitVariableNode(CAst.VariableNode node) {
    return null;
  }

  /** Arithmetic operators */
  @Override
  public T visitAdditionNode(CAst.AdditionNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitSubtractionNode(CAst.SubtractionNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitMultiplicationNode(CAst.MultiplicationNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitDivisionNode(CAst.DivisionNode node) {
    return visitAstNodeBinary(node);
  }

  /** Comparison operators */
  @Override
  public T visitEqualNode(CAst.EqualNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitNotEqualNode(CAst.NotEqualNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitNegativeNode(CAst.NegativeNode node) {
    return visitNegativeNode(node);
  }

  @Override
  public T visitLessThanNode(CAst.LessThanNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitLessEqualNode(CAst.LessEqualNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitGreaterThanNode(CAst.GreaterThanNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitGreaterEqualNode(CAst.GreaterEqualNode node) {
    return visitAstNodeBinary(node);
  }

  /** LF built-in operations */
  @Override
  public T visitSetPortNode(CAst.SetPortNode node) {
    return visitAstNodeBinary(node);
  }

  @Override
  public T visitScheduleActionNode(CAst.ScheduleActionNode node) {
    return visitAstNodeDynamic(node);
  }

  @Override
  public T visitScheduleActionIntNode(CAst.ScheduleActionIntNode node) {
    return visitAstNodeDynamic(node);
  }

  @Override
  public T visitStateVarNode(CAst.StateVarNode node) {
    return null;
  }

  @Override
  public T visitTriggerValueNode(CAst.TriggerValueNode node) {
    return null;
  }

  @Override
  public T visitTriggerIsPresentNode(CAst.TriggerIsPresentNode node) {
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
  public T visitStatementSequenceNode(
      CAst.StatementSequenceNode node, List<CAst.AstNode> nodeList) {
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
  public T visitNegativeNode(CAst.NegativeNode node, List<CAst.AstNode> nodeList) {
    return visitNegativeNode(node);
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
  public T visitScheduleActionIntNode(
      CAst.ScheduleActionIntNode node, List<CAst.AstNode> nodeList) {
    return visitScheduleActionIntNode(node);
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
