package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.List;

import javax.swing.Action;

import org.lflang.generator.ActionInstance;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.StateVariableInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.uclid.UclidGenerator;
import org.lflang.generator.uclid.ast.CAst.*;

public class CToUclidVisitor extends CBaseAstVisitor<String> {

    // The Uclid generator instance
    protected UclidGenerator generator;

    // The reaction instance for the generated axiom
    protected ReactionInstance.Runtime reaction;
    
    // The reactor that contains the reaction
    protected ReactorInstance reactor;

    // A list of all the named instances
    protected List<NamedInstance> instances = new ArrayList<NamedInstance>();

    // Quantified variable
    protected String qv = "i";
    protected String qv2 = "j";

    // Unchanged variables and triggers
    protected List<StateVariableInstance> unchangedStates;
    protected List<TriggerInstance> unchangedTriggers;

    // FIXME: Make this more flexible and infer value from program.
    // Default reset value
    String defaultValue = "0";
    String defaultPresence = "false";

    public CToUclidVisitor(UclidGenerator generator, ReactionInstance.Runtime reaction) {
        this.generator = generator;
        this.reaction = reaction;
        this.reactor = reaction.getReaction().getParent();
        instances.addAll(this.reactor.inputs);
        instances.addAll(this.reactor.outputs);
        instances.addAll(this.reactor.actions);
        instances.addAll(this.reactor.states);
    }

    @Override
    public String visitAssignmentNode(AssignmentNode node) {
        String lhs = visit(node.left);
        // String lhs = "";
        // if (node.left instanceof StateVarNode) {
        //     NamedInstance instance = getInstanceByName(((StateVarNode)node.left).name);
        //     lhs = instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
        //     // this.unchangedStates.remove(instance); // Remove instance from the unchanged list.
        // } else {
        //     System.out.println("Unreachable!"); // FIXME: Throw exception.
        // }
        String rhs = visit(node.right);
        return "(" + lhs + " == " + rhs + ")";
    }

    @Override
    public String visitEqualNode(EqualNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " == " + rhs + ")";
    }

    @Override
    public String visitGreaterEqualNode(GreaterEqualNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " >= " + rhs + ")";
    }

    @Override
    public String visitGreaterThanNode(GreaterThanNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " > " + rhs + ")";
    }

    @Override
    public String visitIfBlockNode(IfBlockNode node) {
        String antecedent = visit(node.left); // Process if condition
        String consequent = visit(((IfBodyNode)node.right).left);
        return "(" + antecedent + " ==> " + "(" + consequent + "\n))";
    }

    @Override
    public String visitLessEqualNode(LessEqualNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " <= " + rhs + ")";
    }

    @Override
    public String visitLessThanNode(LessThanNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " < " + rhs + ")";
    }

    @Override
    public String visitLiteralNode(LiteralNode node) {
        return node.literal;
    }

    @Override
    public String visitLogicalAndNode(LogicalAndNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " && " + rhs + ")";
    }

    @Override
    public String visitLogicalNotNode(LogicalNotNode node) {
        return "!" + visit(node.child);
    }

    @Override
    public String visitLogicalOrNode(LogicalOrNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " || " + rhs + ")";
    }

    @Override
    public String visitMultiplicationNode(MultiplicationNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " * " + rhs + ")";
    }

    @Override
    public String visitNotEqualNode(NotEqualNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " != " + rhs + ")";
    }

    @Override
    public String visitScheduleActionNode(ScheduleActionNode node) {
        String name = ((VariableNode)node.children.get(0)).name;
        NamedInstance instance = getInstanceByName(name);
        ActionInstance action = (ActionInstance)instance;
        String delay = visit(node.children.get(1));
        String str = "\n(" 
            + "finite_exists (" + this.qv2 + " : integer) in indices :: (i > START && i <= END) && ("
            + "\n    " + action.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv2 + ")" + ")"
            + "\n    " + "&& " + "tag_same" + "(" + "g(" + this.qv2 + ")" + ", " + "tag_schedule" + "(" + "g" + "(" + this.qv + ")" + ", " + "nsec" + "(" + action.getMinDelay().toNanoSeconds() + ")" + ")" + ")";
        if (node.children.size() == 3) {
            String value = visit(node.children.get(2));
            str += "\n    " + "&& " + action.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv2 + ")" + ")" + " == " + value;
        } else {
            str += "\n    " + "&& " + action.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv2 + ")" + ")" + " == " + "0";
        }
        str += "\n))";
        return str;
    }

    @Override
    public String visitSetPortNode(SetPortNode node) {
        NamedInstance port = getInstanceByName(((VariableNode)node.left).name);
        String value = visit(node.right);
        // Remove this port from the unchanged list.
        // this.unchangedTriggers.remove(port);
        return "("
            + "("
            + port.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")"
            + " == " + value
            + ")"
            + " && "
            + "(" + port.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv + ")" + ")" + ")"
            + ")";
    }

    @Override
    public String visitStateVarNode(StateVarNode node) {
        NamedInstance instance = getInstanceByName(node.name);
        if (instance != null) {
            return instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
        }
        // FIXME: Throw exception
        return "";
    }

    @Override
    public String visitStatementSequenceNode(StatementSequenceNode node) {
        String axiom = "";
        for (int i = 0; i < node.children.size(); i++) {
            axiom += visit(node.children.get(i));
            if (i != node.children.size() - 1) axiom += "\n" + "        " + "&& ";
        }
        return axiom;
    }

    @Override
    public String visitTriggerIsPresentNode(TriggerIsPresentNode node) {
        // Find the trigger instance by name.
        NamedInstance instance = getInstanceByName(node.name);
        if (instance != null) {
            return instance.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv + ")" + ")";
        }
        // FIXME: Throw exception
        return "";
    }

    @Override
    public String visitTriggerValueNode(TriggerValueNode node) {
        // Find the trigger instance by name.
        NamedInstance instance = getInstanceByName(node.name);
        if (instance != null) {
            return instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
        }
        // FIXME: Throw exception
        return "";
    }

    @Override
    public String visitVariableNode(VariableNode node) {
        NamedInstance instance = getInstanceByName(node.name);
        if (instance != null) {
            return instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
        }
        // FIXME: Throw exception
        return "";
    }

    /////////////////////////////
    //// Private functions

    private NamedInstance getInstanceByName(String name) {
        
        // For some reason, the following one liner doesn't work:
        //
        // return this.instances.stream().filter(i -> i.getDefinition()
        //     .getName().equals(name)).findFirst().get();

        for (NamedInstance i : this.instances) {
            if (i instanceof ActionInstance) {
                if (((ActionInstance)i).getDefinition().getName().equals(name)) {
                    return i;
                }
            } else if (i instanceof PortInstance) {
                if (((PortInstance)i).getDefinition().getName().equals(name)) {
                    return i;
                }
            } else if (i instanceof StateVariableInstance) {
                if (((StateVariableInstance)i).getDefinition().getName().equals(name)) {
                    return i;
                }
            }
        }
        System.out.println("Named instance" + "not found.");
        return null;
    }
}
