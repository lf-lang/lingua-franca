package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.List;

public class CAst {
    
    public static class AstNode implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNode(this, nodeList);
        }
    }

    public static class AstNodeUnary extends AstNode implements Visitable {
        public AstNode child;
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNodeUnary(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNodeUnary(this, nodeList);
        }
    }
    
    public static class AstNodeBinary extends AstNode implements Visitable {
        public AstNode left;
        public AstNode right;
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNodeBinary(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNodeBinary(this, nodeList);
        }
    }

    public static class AstNodeDynamic extends AstNode implements Visitable {
        public ArrayList<AstNode> children = new ArrayList<>();
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNodeDynamic(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitAstNodeDynamic(this, nodeList);
        }
    }

    public static class AssignmentNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitAssignmentNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitAssignmentNode(this, nodeList);
        }
    }

    /**
     * AST node for an if block.
     * The left node is the condition.
     * The right node is the if body.
     */
    public static class IfBlockNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitIfBlockNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitIfBlockNode(this, nodeList);
        }
    }

    /**
     * AST node for an if block.
     * The left node is the then branch.
     * The right node is the else branch.
     */
    public static class IfBodyNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitIfBodyNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitIfBodyNode(this, nodeList);
        }
    }

    public static class LiteralNode extends AstNode implements Visitable {
        public String literal;      
        public LiteralNode(String literal) {
            super();
            this.literal = literal;
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitLiteralNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitLiteralNode(this, nodeList);
        }
    }

    public static class LogicalNotNode extends AstNodeUnary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitLogicalNotNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitLogicalNotNode(this, nodeList);
        }
    }

    public static class LogicalAndNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitLogicalAndNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitLogicalAndNode(this, nodeList);
        }
    }

    public static class LogicalOrNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitLogicalOrNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitLogicalOrNode(this, nodeList);
        }
    }

    /**
     * An Ast node that indicates the code
     * represented by this node is unanalyzable.
     */
    public static class OpaqueNode extends AstNode implements Visitable {  
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitOpaqueNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitOpaqueNode(this, nodeList);
        }
    }

    public static class StatementSequenceNode extends AstNodeDynamic implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitStatementSequenceNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitStatementSequenceNode(this, nodeList);
        }
    }

    public static class VariableNode extends AstNode implements Visitable {
        public enum Type {
            UNKNOWN, INT, BOOLEAN
        }
        public Type type;
        public String name;
        public VariableNode(String name) {
            super();
            this.type = Type.UNKNOWN;
            this.name = name;
        }
        public VariableNode(Type type, String name) {
            super();
            this.type = type;
            this.name = name;
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitVariableNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitVariableNode(this, nodeList);
        }
    }

    /** Arithmetic operations */

    public static class AdditionNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitAdditionNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitAdditionNode(this, nodeList);
        }
    }

    public static class SubtractionNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitSubtractionNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitSubtractionNode(this, nodeList);
        }
    }

    public static class MultiplicationNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitMultiplicationNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitMultiplicationNode(this, nodeList);
        }
    }

    public static class DivisionNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitDivisionNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitDivisionNode(this, nodeList);
        }
    }

    /** Comparison operators */

    public static class EqualNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitEqualNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitEqualNode(this, nodeList);
        }
    }

    public static class NotEqualNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitNotEqualNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitNotEqualNode(this, nodeList);
        }
    }

    public static class LessThanNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitLessThanNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitLessThanNode(this, nodeList);
        }
    }

    public static class LessEqualNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitLessEqualNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitLessEqualNode(this, nodeList);
        }
    }

    public static class GreaterThanNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitGreaterThanNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitGreaterThanNode(this, nodeList);
        }
    }

    public static class GreaterEqualNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitGreaterEqualNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitGreaterEqualNode(this, nodeList);
        }
    }

    /** LF built-in operations */
    /**
     * AST node for an lf_set call. The left child is the port being set.
     * The right node is the value of the port.
     */
    public static class SetPortNode extends AstNodeBinary implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitSetPortNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitSetPortNode(this, nodeList);
        }
    }

    /**
     * action; // self struct variable access is a postfixExpression.
     * value; // Could be literal, variable, or pointer.
     * additionalDelay;
     */
    public static class ScheduleActionNode extends AstNodeDynamic implements Visitable {
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitScheduleActionNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitScheduleActionNode(this, nodeList);
        }
    }

    /**
     * Handle state variables appearing as self-><name>
     */
    public static class StateVarNode extends AstNode implements Visitable {
        public String name;
        public StateVarNode(String name) {
            this.name = name;
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitStateVarNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitStateVarNode(this, nodeList);
        }
    }

    /**
     * Handle trigger values appearing as <name>->value
     */
    public static class TriggerValueNode extends AstNode implements Visitable {
        public String name;
        public TriggerValueNode(String name) {
            this.name = name;
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitTriggerValueNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitTriggerValueNode(this, nodeList);
        }
    }

    /**
     * Handle trigger presence appearing as <name>->is_present
     */
    public static class TriggerIsPresentNode extends AstNode implements Visitable {
        public String name;
        public TriggerIsPresentNode(String name) {
            this.name = name;
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor) {
            return ((CAstVisitor<? extends T>)visitor).visitTriggerIsPresentNode(this);
        }
        @Override public <T> T accept(AstVisitor<? extends T> visitor, List<AstNode> nodeList) {
            return ((CAstVisitor<? extends T>)visitor).visitTriggerIsPresentNode(this, nodeList);
        }
    }
}
