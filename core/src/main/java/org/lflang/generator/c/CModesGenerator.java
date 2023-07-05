package org.lflang.generator.c;

import java.util.List;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Mode;
import org.lflang.lf.Reactor;

/**
 * Generates C code to support modal models.
 *
 * @author Edward A. Lee
 * @author Alexander Schulz-Rosengarten
 * @author Hou Seng Wong
 */
public class CModesGenerator {
  /**
   * Generate fields in the self struct for mode instances
   *
   * @param reactor
   * @param body
   * @param constructorCode
   */
  public static void generateDeclarations(
      Reactor reactor, CodeBuilder body, CodeBuilder constructorCode) {
    List<Mode> allModes = ASTUtils.allModes(reactor);
    if (!allModes.isEmpty()) {
      // Reactor's mode instances and its state.
      body.pr(String.join("\n", "reactor_mode_t _lf__modes[" + reactor.getModes().size() + "];"));

      // Initialize the mode instances
      constructorCode.pr("// Initialize modes");
      constructorCode.pr("self_base_t* _lf_self_base = (self_base_t*)self;");
      int initialMode = -1;

      for (int i = 0; i < allModes.size(); i++) {
        var mode = allModes.get(i);
        constructorCode.pr(
            mode,
            String.join(
                "\n",
                "self->_lf__modes[" + i + "].state = &_lf_self_base->_lf__mode_state;",
                "self->_lf__modes[" + i + "].name = \"" + mode.getName() + "\";",
                "self->_lf__modes[" + i + "].deactivation_time = 0;",
                "self->_lf__modes[" + i + "].flags = 0;"));
        if (initialMode < 0 && mode.isInitial()) {
          initialMode = i;
        }
      }

      assert initialMode >= 0 : "initial mode must be non-negative!!";

      // Initialize mode state with initial mode active upon start
      constructorCode.pr(
          String.join(
              "\n",
              "// Initialize mode state",
              "_lf_self_base->_lf__mode_state.parent_mode = NULL;",
              "_lf_self_base->_lf__mode_state.initial_mode = &self->_lf__modes["
                  + initialMode
                  + "];",
              "_lf_self_base->_lf__mode_state.current_mode ="
                  + " _lf_self_base->_lf__mode_state.initial_mode;",
              "_lf_self_base->_lf__mode_state.next_mode = NULL;",
              "_lf_self_base->_lf__mode_state.mode_change = no_transition;"));
    }
  }

  /**
   * Generate code for modal reactor registration and hierarchy.
   *
   * @param instance The reactor instance.
   * @param code The code builder.
   */
  public static void generateModeStructure(ReactorInstance instance, CodeBuilder code) {
    var parentMode = instance.getMode(false);
    var nameOfSelfStruct = CUtil.reactorRef(instance);
    // If this instance is enclosed in another mode
    if (parentMode != null) {
      var parentModeRef =
          "&"
              + CUtil.reactorRef(parentMode.getParent())
              + "->_lf__modes["
              + parentMode.getParent().modes.indexOf(parentMode)
              + "]";
      code.pr("// Setup relation to enclosing mode");

      // If this reactor does not have its own modes, all reactions must be linked to enclosing mode
      if (instance.modes.isEmpty()) {
        int i = 0;
        for (ReactionInstance reaction : instance.reactions) {
          code.pr(
              CUtil.reactorRef(reaction.getParent())
                  + "->_lf__reaction_"
                  + i
                  + ".mode = "
                  + parentModeRef
                  + ";");
          i++;
        }
      } else { // Otherwise, only reactions outside modes must be linked and the mode state itself
        // gets a parent relation
        code.pr(
            "((self_base_t*)"
                + nameOfSelfStruct
                + ")->_lf__mode_state.parent_mode = "
                + parentModeRef
                + ";");
        for (var reaction :
            (Iterable<ReactionInstance>)
                instance.reactions.stream().filter(it -> it.getMode(true) == null)::iterator) {
          code.pr(
              CUtil.reactorRef(reaction.getParent())
                  + "->_lf__reaction_"
                  + instance.reactions.indexOf(reaction)
                  + ".mode = "
                  + parentModeRef
                  + ";");
        }
      }
    }
    // If this reactor has modes, register for mode change handling
    if (!instance.modes.isEmpty()) {
      code.pr("// Register for transition handling");
      code.pr(
          CUtil.getEnvironmentStruct(instance)
              + ".modes->modal_reactor_states[modal_reactor_count["
              + CUtil.getEnvironmentId(instance)
              + "]++] = &((self_base_t*)"
              + nameOfSelfStruct
              + ")->_lf__mode_state;");
    }
  }

  /**
   * Generate code registering a state variable for automatic reset.
   *
   * @param modeRef The code to refer to the mode
   * @param selfRef The code to refer to the self struct
   * @param varName The variable name in the self struct
   * @param source The variable that stores the initial value
   * @param type The size of the initial value
   */
  public static String generateStateResetStructure(
      ReactorInstance instance,
      String modeRef,
      String selfRef,
      String varName,
      String source,
      String type) {
    var env = CUtil.getEnvironmentStruct(instance);
    var envId = CUtil.getEnvironmentId(instance);
    return String.join(
        "\n",
        "// Register for automatic reset",
        env
            + ".modes->state_resets[modal_state_reset_count["
            + envId
            + "]].mode = "
            + modeRef
            + ";",
        env
            + ".modes->state_resets[modal_state_reset_count["
            + envId
            + "]].target = &("
            + selfRef
            + "->"
            + varName
            + ");",
        env
            + ".modes->state_resets[modal_state_reset_count["
            + envId
            + "]].source = &"
            + source
            + ";",
        env
            + ".modes->state_resets[modal_state_reset_count["
            + envId
            + "]].size = sizeof("
            + type
            + ");",
        "modal_state_reset_count[" + envId + "]++;");
  }
}
