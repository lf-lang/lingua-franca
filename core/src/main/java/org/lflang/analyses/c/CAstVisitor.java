package org.lflang.analyses.c;

import java.util.List;

/** Modeled after CVisitor.java */
public interface CAstVisitor<T> extends AstVisitor<T> {

  T visitAstNode(CAst.AstNode node);

  T visitAstNodeUnary(CAst.AstNodeUnary node);

  T visitAstNodeBinary(CAst.AstNodeBinary node);

  T visitAstNodeDynamic(CAst.AstNodeDynamic node);

  T visitAssignmentNode(CAst.AssignmentNode node);

  T visitIfBlockNode(CAst.IfBlockNode node);

  T visitIfBodyNode(CAst.IfBodyNode node);

  T visitLiteralNode(CAst.LiteralNode node);

  T visitLogicalNotNode(CAst.LogicalNotNode node);

  T visitLogicalAndNode(CAst.LogicalAndNode node);

  T visitLogicalOrNode(CAst.LogicalOrNode node);

  T visitOpaqueNode(CAst.OpaqueNode node);

  T visitStatementSequenceNode(CAst.StatementSequenceNode node);

  T visitVariableNode(CAst.VariableNode node);

  T visitAdditionNode(CAst.AdditionNode node);

  T visitSubtractionNode(CAst.SubtractionNode node);

  T visitMultiplicationNode(CAst.MultiplicationNode node);

  T visitDivisionNode(CAst.DivisionNode node);

  T visitEqualNode(CAst.EqualNode node);

  T visitNotEqualNode(CAst.NotEqualNode node);

  T visitNegativeNode(CAst.NegativeNode node);

  T visitLessThanNode(CAst.LessThanNode node);

  T visitLessEqualNode(CAst.LessEqualNode node);

  T visitGreaterThanNode(CAst.GreaterThanNode node);

  T visitGreaterEqualNode(CAst.GreaterEqualNode node);

  T visitSetPortNode(CAst.SetPortNode node);

  T visitScheduleActionNode(CAst.ScheduleActionNode node);

  T visitScheduleActionIntNode(CAst.ScheduleActionIntNode node);

  T visitStateVarNode(CAst.StateVarNode node);

  T visitTriggerValueNode(CAst.TriggerValueNode node);

  T visitTriggerIsPresentNode(CAst.TriggerIsPresentNode node);

  /** Used for converting an AST into If Normal Form. */
  T visitAstNode(CAst.AstNode node, List<CAst.AstNode> nodeList);

  T visitAstNodeUnary(CAst.AstNodeUnary node, List<CAst.AstNode> nodeList);

  T visitAstNodeBinary(CAst.AstNodeBinary node, List<CAst.AstNode> nodeList);

  T visitAstNodeDynamic(CAst.AstNodeDynamic node, List<CAst.AstNode> nodeList);

  T visitAssignmentNode(CAst.AssignmentNode node, List<CAst.AstNode> nodeList);

  T visitIfBlockNode(CAst.IfBlockNode node, List<CAst.AstNode> nodeList);

  T visitIfBodyNode(CAst.IfBodyNode node, List<CAst.AstNode> nodeList);

  T visitLiteralNode(CAst.LiteralNode node, List<CAst.AstNode> nodeList);

  T visitLogicalNotNode(CAst.LogicalNotNode node, List<CAst.AstNode> nodeList);

  T visitLogicalAndNode(CAst.LogicalAndNode node, List<CAst.AstNode> nodeList);

  T visitLogicalOrNode(CAst.LogicalOrNode node, List<CAst.AstNode> nodeList);

  T visitOpaqueNode(CAst.OpaqueNode node, List<CAst.AstNode> nodeList);

  T visitStatementSequenceNode(CAst.StatementSequenceNode node, List<CAst.AstNode> nodeList);

  T visitVariableNode(CAst.VariableNode node, List<CAst.AstNode> nodeList);

  T visitAdditionNode(CAst.AdditionNode node, List<CAst.AstNode> nodeList);

  T visitSubtractionNode(CAst.SubtractionNode node, List<CAst.AstNode> nodeList);

  T visitMultiplicationNode(CAst.MultiplicationNode node, List<CAst.AstNode> nodeList);

  T visitDivisionNode(CAst.DivisionNode node, List<CAst.AstNode> nodeList);

  T visitEqualNode(CAst.EqualNode node, List<CAst.AstNode> nodeList);

  T visitNotEqualNode(CAst.NotEqualNode node, List<CAst.AstNode> nodeList);

  T visitNegativeNode(CAst.NegativeNode node, List<CAst.AstNode> nodeList);

  T visitLessThanNode(CAst.LessThanNode node, List<CAst.AstNode> nodeList);

  T visitLessEqualNode(CAst.LessEqualNode node, List<CAst.AstNode> nodeList);

  T visitGreaterThanNode(CAst.GreaterThanNode node, List<CAst.AstNode> nodeList);

  T visitGreaterEqualNode(CAst.GreaterEqualNode node, List<CAst.AstNode> nodeList);

  T visitSetPortNode(CAst.SetPortNode node, List<CAst.AstNode> nodeList);

  T visitScheduleActionNode(CAst.ScheduleActionNode node, List<CAst.AstNode> nodeList);

  T visitScheduleActionIntNode(CAst.ScheduleActionIntNode node, List<CAst.AstNode> nodeList);

  T visitStateVarNode(CAst.StateVarNode node, List<CAst.AstNode> nodeList);

  T visitTriggerValueNode(CAst.TriggerValueNode node, List<CAst.AstNode> nodeList);

  T visitTriggerIsPresentNode(CAst.TriggerIsPresentNode node, List<CAst.AstNode> nodeList);
}
