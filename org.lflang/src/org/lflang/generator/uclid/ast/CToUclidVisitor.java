package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.List;

import org.lflang.generator.ActionInstance;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.StateVariableInstance;
import org.lflang.generator.uclid.ast.CAst.*;

public class CToUclidVisitor extends CBaseAstVisitor<String> {

    // The reaction instance for the generated axiom
    protected ReactionInstance.Runtime reaction;
    
    // The reactor that contains the reaction
    protected ReactorInstance reactor;

    // A list of all the named instances
    protected List<NamedInstance> instances = new ArrayList<NamedInstance>();

    // Quantified variable
    protected String qv = "i";

    public CToUclidVisitor(ReactionInstance.Runtime reaction) {
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
        String rhs = visit(node.right);
        return "(" + lhs + " == " + rhs + ")";
    }

    @Override
    public String visitIfBlockNode(IfBlockNode node) {
        String antecedent = visit(node.left); // Process if condition
        String consequent = visit(((IfBodyNode)node.right).left);
        return "(" + antecedent + " ==> " + consequent + ")";
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
    public String visitNotEqualNode(NotEqualNode node) {
        String lhs = visit(node.left);
        String rhs = visit(node.right);
        return "(" + lhs + " != " + rhs + ")";
    }

    @Override
    public String visitSetPortNode(SetPortNode node) {
        String port = visit(node.left);
        String value = visit(node.right);
        return "(" + port + " == " + value + ")";
    }

    @Override
    public String visitStateVarNode(StateVarNode node) {
        NamedInstance instance = getInstanceByName(node.name);
        if (instance != null) {
            System.out.println(instance + " returned.");
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
            if (i != node.children.size() - 1) axiom += " && ";
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
        
        // For some reason, the following one liner doesn't work.
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
