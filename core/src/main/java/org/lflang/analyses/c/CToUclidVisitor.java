package org.lflang.analyses.c;

import java.util.ArrayList;
import java.util.List;
import org.lflang.analyses.c.CAst.*;
import org.lflang.analyses.uclid.UclidGenerator;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.StateVariableInstance;
import org.lflang.generator.TriggerInstance;

public class CToUclidVisitor extends CBaseAstVisitor<String> {

  /** The Uclid generator instance */
  private UclidGenerator generator;

  /** The reaction instance for the generated axiom */
  private ReactionInstance.Runtime reaction;

  /** The reactor that contains the reaction */
  private ReactorInstance reactor;

  /** A list of all the named instances */
  private List<NamedInstance> instances = new ArrayList<NamedInstance>();

  /** Quantified variable */
  private final String qv = "i";

  private final String qv2 = "j";

  /** Unchanged variables and triggers */
  private List<StateVariableInstance> unchangedStates;

  private List<TriggerInstance> unchangedTriggers;

  // FIXME: Make this more flexible and infer value from program.
  /** Default reset value */
  private final String defaultValue = "0";

  private final String defaultPresence = "false";

  /** Constructor */
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
  public String visitAdditionNode(AdditionNode node) {
    String lhs = visit(node.left);
    String rhs = visit(node.right);
    return "(" + lhs + " + " + rhs + ")";
  }

  @Override
  public String visitAssignmentNode(AssignmentNode node) {
    String lhs = visit(node.left);
    String rhs = visit(node.right);
    return "(" + lhs + " == " + rhs + ")";
  }

  @Override
  public String visitDivisionNode(DivisionNode node) {
    String lhs = visit(node.left);
    String rhs = visit(node.right);
    return "(" + lhs + " / " + rhs + ")";
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
    String consequent = visit(((IfBodyNode) node.right).left);
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
    return "!" + "(" + visit(node.child) + ")";
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
  public String visitNegativeNode(NegativeNode node) {
    return "(" + "-1*(" + visit(node.child) + "))";
  }

  @Override
  public String visitScheduleActionNode(ScheduleActionNode node) {
    String name = ((VariableNode) node.children.get(0)).name;
    NamedInstance instance = getInstanceByName(name);
    ActionInstance action = (ActionInstance) instance;
    String additionalDelay = visit(node.children.get(1));
    String str =
        "\n("
            + "(finite_exists ("
            + this.qv2
            + " : integer) in indices :: ("
            + this.qv2
            + " > "
            + this.qv
            + " && "
            + this.qv2
            + " <= END_TRACE) && ("
            + "\n    "
            + action.getFullNameWithJoiner("_")
            + "_is_present"
            + "("
            + "t"
            + "("
            + this.qv2
            + ")"
            + ")"
            + "\n    "
            + "&& "
            + "tag_same"
            + "("
            + "g("
            + this.qv2
            + ")"
            + ", "
            + "tag_schedule"
            + "("
            + "g"
            + "("
            + this.qv
            + ")"
            + ", "
            + "("
            + action.getMinDelay().toNanoSeconds()
            + "+"
            + additionalDelay
            + ")"
            + ")"
            + ")"
            + "\n    "
            + "&& "
            + action.getFullNameWithJoiner("_")
            + "("
            + "s"
            + "("
            + this.qv2
            + ")"
            + ")"
            + " == "
            + "0"
            + "\n)) // Closes finite_exists"
            + "\n&& "
            + action.getFullNameWithJoiner("_")
            + "_scheduled"
            + "("
            + "d"
            + "("
            + this.qv
            + ")"
            + ")"
            + "\n)";
    return str;
  }

  @Override
  public String visitScheduleActionIntNode(ScheduleActionIntNode node) {
    String name = ((VariableNode) node.children.get(0)).name;
    NamedInstance instance = getInstanceByName(name);
    ActionInstance action = (ActionInstance) instance;
    String additionalDelay = visit(node.children.get(1));
    String intValue = visit(node.children.get(2));
    String str =
        "\n("
            + "(finite_exists ("
            + this.qv2
            + " : integer) in indices :: ("
            + this.qv2
            + " > "
            + this.qv
            + " && "
            + this.qv2
            + " <= END_TRACE) && ("
            + "\n    "
            + action.getFullNameWithJoiner("_")
            + "_is_present"
            + "("
            + "t"
            + "("
            + this.qv2
            + ")"
            + ")"
            + "\n    "
            + "&& "
            + "tag_same"
            + "("
            + "g("
            + this.qv2
            + ")"
            + ", "
            + "tag_schedule"
            + "("
            + "g"
            + "("
            + this.qv
            + ")"
            + ", "
            + "("
            + action.getMinDelay().toNanoSeconds()
            + "+"
            + additionalDelay
            + ")"
            + ")"
            + ")"
            + "\n)) // Closes finite_exists"
            + "\n&& "
            + action.getFullNameWithJoiner("_")
            + "_scheduled"
            + "("
            + "d"
            + "("
            + this.qv
            + ")"
            + ")"
            + "\n&& "
            + action.getFullNameWithJoiner("_")
            + "_scheduled_payload"
            + "("
            + "pl"
            + "("
            + this.qv
            + ")"
            + ")"
            + " == "
            + intValue
            + "\n)";
    return str;
  }

  @Override
  public String visitSetPortNode(SetPortNode node) {
    NamedInstance port = getInstanceByName(((VariableNode) node.left).name);
    String value = visit(node.right);
    // Remove this port from the unchanged list.
    // this.unchangedTriggers.remove(port);
    return "("
        + "("
        + port.getFullNameWithJoiner("_")
        + "("
        + "s"
        + "("
        + this.qv
        + ")"
        + ")"
        + " == "
        + value
        + ")"
        + " && "
        + "("
        + port.getFullNameWithJoiner("_")
        + "_is_present"
        + "("
        + "t"
        + "("
        + this.qv
        + ")"
        + ")"
        + ")"
        + ")";
  }

  @Override
  public String visitStateVarNode(StateVarNode node) {
    NamedInstance instance = getInstanceByName(node.name);
    return instance.getFullNameWithJoiner("_")
        + "("
        + "s"
        + "("
        + this.qv
        + (node.prev ? "-1" : "")
        + ")"
        + ")";
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
  public String visitSubtractionNode(SubtractionNode node) {
    String lhs = visit(node.left);
    String rhs = visit(node.right);
    return "(" + lhs + " - " + rhs + ")";
  }

  @Override
  public String visitTriggerIsPresentNode(TriggerIsPresentNode node) {
    // Find the trigger instance by name.
    NamedInstance instance = getInstanceByName(node.name);
    return instance.getFullNameWithJoiner("_")
        + "_is_present"
        + "("
        + "t"
        + "("
        + this.qv
        + ")"
        + ")";
  }

  @Override
  public String visitTriggerValueNode(TriggerValueNode node) {
    // Find the trigger instance by name.
    NamedInstance instance = getInstanceByName(node.name);
    return instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
  }

  @Override
  public String visitVariableNode(VariableNode node) {
    NamedInstance instance = getInstanceByName(node.name);
    return instance.getFullNameWithJoiner("_") + "(" + "s" + "(" + this.qv + ")" + ")";
  }

  /////////////////////////////
  //// Private functions

  /** Look up an instance by name. This function throws an error if an instance is not found. */
  private NamedInstance getInstanceByName(String name) {
    for (NamedInstance i : this.instances) {
      if (i instanceof ActionInstance) {
        if (((ActionInstance) i).getDefinition().getName().equals(name)) {
          return i;
        }
      } else if (i instanceof PortInstance) {
        if (((PortInstance) i).getDefinition().getName().equals(name)) {
          return i;
        }
      } else if (i instanceof StateVariableInstance) {
        if (((StateVariableInstance) i).getDefinition().getName().equals(name)) {
          return i;
        }
      }
    }
    throw new RuntimeException("NamedInstance not found!");
  }
}
