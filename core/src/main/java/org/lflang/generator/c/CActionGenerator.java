package org.lflang.generator.c;

import static org.lflang.generator.c.CGenerator.variableStructType;

import java.util.ArrayList;
import java.util.List;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Action;
import org.lflang.target.Target;

/**
 * Generates code for actions (logical or physical) for the C and CCpp target.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Mehrdad Niknami
 * @author Christian Menard
 * @author Matt Weber
 * @author {Soroush Bateni
 * @author Alexander Schulz-Rosengarten
 * @author Hou Seng Wong
 * @ingroup Generator
 */
public class CActionGenerator {
  /**
   * For each action of the specified reactor instance, generate initialization code for the offset
   * and period fields.
   *
   * @param instance The reactor.
   */
  public static String generateInitializers(ReactorInstance instance) {
    List<String> code = new ArrayList<>();
    for (ActionInstance action : instance.actions) {
      if (!action.isShutdown()) {
        var triggerStructName = CUtil.reactorRef(action.getParent()) + "->_lf__" + action.getName();
        var minDelay = action.getMinDelay();
        var minSpacing = action.getMinSpacing();
        var offsetInitializer =
            triggerStructName
                + ".offset = "
                + CTypes.getInstance().getTargetTimeExpr(minDelay)
                + ";";
        var periodInitializer =
            triggerStructName
                + ".period = "
                + (minSpacing != null
                    ? CTypes.getInstance().getTargetTimeExpr(minSpacing)
                    : CGenerator.UNDEFINED_MIN_SPACING)
                + ";";
        var lastTimeInitializer = triggerStructName + ".last_tag = NEVER_TAG;";
        code.addAll(
            List.of(
                "// Initializing action " + action.getFullName(),
                offsetInitializer,
                periodInitializer,
                lastTimeInitializer));

        var mode = action.getMode(false);
        if (mode != null) {
          var modeParent = mode.getParent();
          var modeRef =
              "&"
                  + CUtil.reactorRef(modeParent)
                  + "->_lf__modes["
                  + modeParent.modes.indexOf(mode)
                  + "];";
          code.add(triggerStructName + ".mode = " + modeRef + ";");
        } else {
          code.add(triggerStructName + ".mode = NULL;");
        }
      }
    }
    return String.join("\n", code);
  }

  /**
   * Create a template token initialized to the payload size. This token is marked to not be freed
   * so that the trigger_t struct always has a template token. At the start of each time step, we
   * need to initialize the is_present field of each action's trigger object to false and free a
   * previously allocated token if appropriate. This code sets up the table that does that.
   *
   * @param selfStruct The variable name of the self struct
   * @param actionName The action name
   * @param payloadSize The code that returns the size of the action's payload in C.
   */
  public static String generateTokenInitializer(
      String selfStruct, String actionName, String payloadSize) {
    return String.join(
        "\n",
        "_lf_initialize_template((token_template_t*)",
        "        &(" + selfStruct + "->_lf__" + actionName + "),",
        payloadSize + ");",
        "_lf_initialize_template((token_template_t*)",
        "        &(" + selfStruct + "->_lf_" + actionName + "),",
        payloadSize + ");",
        selfStruct + "->_lf__" + actionName + ".status = absent;");
  }

  /**
   * Generate the declarations of actions in the self struct
   *
   * @param tpr The type-parameterized reactor.
   * @param body The content of the self struct
   * @param constructorCode The constructor code of the reactor
   */
  public static void generateDeclarations(
      TypeParameterizedReactor tpr, CodeBuilder body, CodeBuilder constructorCode) {
    for (Action action : ASTUtils.allActions(tpr.reactor())) {
      var actionName = action.getName();
      body.pr(CGenerator.variableStructType(action, tpr, false) + " _lf_" + actionName + ";");
      // Initialize the trigger pointer and the parent pointer in the action.
      constructorCode.pr(
          "self->_lf_" + actionName + "._base.trigger = &self->_lf__" + actionName + ";");
      constructorCode.pr("self->_lf_" + actionName + ".parent = (self_base_t*)self;");
      constructorCode.pr("self->_lf_" + actionName + ".source_id = -1;"); // Default value.
    }
  }

  /**
   * Generate the struct type definitions for the action of the reactor
   *
   * @param tpr The type-parameterized reactor.
   * @param action The action to generate the struct for
   * @param target The target of the code generation (C, CCpp or Python)
   * @param types The helper object for types related stuff
   * @param federatedExtension The code needed to support federated execution
   * @param userFacing Whether this is user-facing code.
   * @return The auxiliary struct for the port as a string
   */
  public static String generateAuxiliaryStruct(
      TypeParameterizedReactor tpr,
      Action action,
      Target target,
      CTypes types,
      CodeBuilder federatedExtension,
      boolean userFacing) {
    var code = new CodeBuilder();
    code.pr("typedef struct {");
    code.indent();
    // NOTE: The following fields are required to be the first ones so that
    // pointer to this struct can be cast to a (lf_action_base_t*) or to
    // (token_template_t*) to access these fields for any port.
    // IMPORTANT: These must match exactly the fields defined in port.h!!
    code.pr(
        String.join(
            "\n",
            "token_type_t type;", // From token_template_t
            "lf_token_t* token;", // From token_template_t
            "size_t length;", // From token_template_t
            "bool is_present;", // From lf_port_or_action_t
            "lf_action_internal_t _base;", // internal substruct
            "self_base_t* parent;", // From lf_port_or_action_t
            "bool has_value;", // From lf_action_base_t
            "int source_id;" // From lf_action_base_t
            ));
    code.pr(valueDeclaration(tpr, action, target, types));
    code.pr(federatedExtension.toString());
    code.unindent();
    code.pr("} " + variableStructType(action, tpr, userFacing) + ";");
    return code.toString();
  }

  /**
   * For the specified action, return a declaration for action struct to contain the value of the
   * action. An action of type int[10], for example, will result in this:
   *
   * <pre><code>
   *     int* value;
   * </code></pre>
   *
   * This will return an empty string for an action with no type.
   *
   * @param tpr {@link TypeParameterizedReactor}
   * @param action The action.
   * @return A string providing the value field of the action struct.
   */
  private static String valueDeclaration(
      TypeParameterizedReactor tpr, Action action, Target target, CTypes types) {
    if (target == Target.Python) {
      return "PyObject* value;";
    }
    // Do not convert to lf_token_t* using lfTypeToTokenType because there
    // will be a separate field pointing to the token.
    return action.getType() == null && target.requiresTypes
        ? ""
        : types.getTargetType(tpr.resolveType(action.getType())) + " value;";
  }
}
