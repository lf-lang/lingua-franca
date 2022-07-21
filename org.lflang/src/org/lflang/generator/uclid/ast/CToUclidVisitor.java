package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.List;

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
        // String lhs = visit(node.left);
        String lhs = "";
        if (node.left instanceof StateVarNode) {
            NamedInstance instance = getInstanceByName(((StateVarNode)node.left).name);
            lhs = instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
            this.unchangedStates.remove(instance); // Remove instance from the unchanged list.
        } else {
            System.out.println("Unreachable!"); // FIXME: Throw exception.
        }
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

        String formula = "";

        // In INF, there are no nested if blocks, so we can use a field
        // to keep track of unchanged variables.
        this.unchangedStates = new ArrayList<>(this.generator.stateVariables);
        this.unchangedTriggers = new ArrayList<>();
        this.unchangedTriggers.addAll(this.generator.outputInstances);
        this.unchangedTriggers.addAll(this.generator.actionInstances);

        String antecedent = visit(node.left); // Process if condition
        String consequent = visit(((IfBodyNode)node.right).left);

        formula += "(" + antecedent + " ==> " + "(" + consequent;

        formula += "\n//// Unchanged variables";
        // State variables retain their previous states.
        formula += "\n// State variables retain their previous states.";
        for (StateVariableInstance s : this.unchangedStates) {
            formula += "\n&& " + s.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")"
                        + " == "
                        + s.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + "-1" + ")" + ")";
        }
        // Triggers resets to their default states if time advances.
        formula += "\n// Triggers resets to their default states if time advances.";
        for (TriggerInstance t : this.unchangedTriggers) {
            // formula += "\n&& " 
            //             + "(" + "(" 
            //             + "tag_same(" + "g(" + this.qv + ")" + "," + "g(" + this.qv + "-1" + ")" + ")" + ")"
            //             + " ==> " + "(" + " true"
            //             // Retain value
            //             + "\n&& " + t.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")"
            //             + " == "
            //             + t.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + "-1" + ")" + ")"
            //             // Retain presence
            //             + "\n&& " + t.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv + ")" + ")"
            //             + " == "
            //             + t.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv + "-1" + ")" + ")"
            //             + ")" + ")"
            //             // If time advances.
            //             + "\n&& " 
            //             + "(" + "("
            //             + "tag_later(" + "g(" + this.qv + ")" + "," + "g(" + this.qv + "-1" + ")" + ")" + ")"
            //             + " ==> " + "(" + " true"
            //             // Reset value
            //             + "\n&& " + t.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")"
            //             + " == "
            //             + this.defaultValue
            //             // Reset presence
            //             + "\n&& " + t.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv + ")" + ")"
            //             + " == "
            //             + this.defaultPresence
            //             + ")" + ")";

            formula += "\n&& " 
                        + "("
                        + " true"
                        // Reset value
                        + "\n&& " + t.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")"
                        + " == "
                        + this.defaultValue
                        // Reset presence
                        + "\n&& " + t.getFullNameWithJoiner("_") + "_is_present" + "(" + "t" + "(" + this.qv + ")" + ")"
                        + " == "
                        + this.defaultPresence
                        + ")";
        }

        formula += "\n))";

        return formula;
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
    public String visitLogicalNotNode(LogicalNotNode node) {
        return "!" + visit(node.child);
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
    public String visitSetPortNode(SetPortNode node) {
        NamedInstance port = getInstanceByName(((VariableNode)node.left).name);
        String value = visit(node.right);
        // Remove this port from the unchanged list.
        this.unchangedTriggers.remove(port);
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
