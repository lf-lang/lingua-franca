package org.lflang.generator.uclid.ast;

import java.util.List;

import org.lflang.generator.NamedInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.uclid.UclidGenerator;
import org.lflang.generator.uclid.ast.CAst.*;

public class CToUclidVisitor extends CBaseAstVisitor<String> {

    // The reaction instance for the generated axiom
    protected ReactionInstance.Runtime reaction;
    // The reactor that contains the reaction
    protected ReactorInstance reactor;

    public CToUclidVisitor(ReactionInstance.Runtime reaction) {
        this.reaction = reaction;
        this.reactor = reaction.getReaction().getParent();
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
    public String visitIfBlockNode(IfBlockNode node) {
        String antecedent = visit(node.left); // Process if condition
        String consequent = visit(((IfBodyNode)node.right).left);
        return "(" + antecedent + ")" + " ==> " + "(" + consequent + ")";
    }
    
    @Override
    public String visitLiteralNode(LiteralNode node) {
        return node.literal;
    }

    @Override
    public String visitTriggerValueNode(TriggerValueNode node) {
        // Find the trigger instance by name.
        NamedInstance instance;
        System.out.println("*** Printing all the port names.");
        for (PortInstance p : this.reactor.inputs) {
            System.out.println(p.getDefinition().getName());
        }
        instance = this.reactor.inputs.stream().filter(ins -> ins.getDefinition()
            .getName().equals(node.name)).findFirst().get();
        if (instance != null) {
            System.out.println(instance + " returned.");
            return instance.getFullNameWithJoiner("_");
        }
        return "";
    }
}
