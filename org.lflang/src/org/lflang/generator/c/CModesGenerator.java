package org.lflang.generator.c;

import java.util.List;

import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Mode;
import org.lflang.lf.Reactor;

/**
 * Generates C code to support modal models.
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
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
        Reactor reactor,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        List<Mode> allModes = ASTUtils.allModes(reactor);
        if (!allModes.isEmpty()) {
            // Reactor's mode instances and its state.
            body.pr(String.join("\n", 
                "reactor_mode_t _lf__modes["+reactor.getModes().size()+"];"
            ));

            // Initialize the mode instances
            constructorCode.pr("// Initialize modes");
            constructorCode.pr("self_base_t* _lf_self_base = (self_base_t*)self;");
            int initialMode = -1;

            for (int i = 0; i < allModes.size(); i++){
                var mode = allModes.get(i);
                constructorCode.pr(mode, String.join("\n", 
                    "self->_lf__modes["+i+"].state = &_lf_self_base->_lf__mode_state;",
                    "self->_lf__modes["+i+"].name = \""+mode.getName()+"\";",
                    "self->_lf__modes["+i+"].deactivation_time = 0;"
                ));
                if (initialMode < 0 && mode.isInitial()) {
                    initialMode = i;
                }
            }

            assert initialMode >= 0 : "initial mode must be non-negative!!";
            
            // Initialize mode state with initial mode active upon start
            constructorCode.pr(String.join("\n", 
                "// Initialize mode state",
                "_lf_self_base->_lf__mode_state.parent_mode = NULL;",
                "_lf_self_base->_lf__mode_state.initial_mode = &self->_lf__modes["+initialMode+"];",
                "_lf_self_base->_lf__mode_state.active_mode = _lf_self_base->_lf__mode_state.initial_mode;",
                "_lf_self_base->_lf__mode_state.next_mode = NULL;",
                "_lf_self_base->_lf__mode_state.mode_change = no_transition;"
            ));
        }
    }

    /**
     * Generate the declaration of modal models state table.
     * 
     * @param hasModalReactors True if there is modal model reactors, false otherwise
     * @param modalReactorCount The number of modal model reactors
     * @param modalStateResetCount The number of modal model state resets
     */
    public static String generateModeStatesTable(
        boolean hasModalReactors, 
        int modalReactorCount,
        int modalStateResetCount
    ) {
        return String.join("\n", 
            (hasModalReactors ? 
            String.join("\n",
            "// Array of pointers to mode states to be handled in _lf_handle_mode_changes().",
            "reactor_mode_state_t* _lf_modal_reactor_states["+modalReactorCount+"];",
            "int _lf_modal_reactor_states_size = "+modalReactorCount+";"
            ) :
            ""),
            (hasModalReactors && modalStateResetCount > 0 ?
            String.join("\n",
            "// Array of reset data for state variables nested in modes. Used in _lf_handle_mode_changes().",
            "mode_state_variable_reset_data_t _lf_modal_state_reset["+modalStateResetCount+"];",
            "int _lf_modal_state_reset_size = "+modalStateResetCount+";"
            ) :
            "")
        );
    }

    /**
     * Generate code to call `_lf_process_mode_changes`.
     * 
     * @param hasModalReactors True if there is modal model reactors, false otherwise
     * @param modalStateResetCount The number of modal model state resets
     */
    public static String generateLfHandleModeChanges(
        boolean hasModalReactors, 
        int modalStateResetCount
    ) {
        if (!hasModalReactors) {
            return "";
        }
        return String.join("\n", 
            "void _lf_handle_mode_changes() {",
            "   _lf_process_mode_changes(",
            "       _lf_modal_reactor_states, ",
            "       _lf_modal_reactor_states_size, ",
            "       " + (modalStateResetCount > 0 ? "_lf_modal_state_reset" : "NULL") + ", ",
            "       " + (modalStateResetCount > 0 ? "_lf_modal_state_reset_size" : "0") + ", ",
            "       _lf_timer_triggers, ",
            "       _lf_timer_triggers_size",
            "   );",
            "}"
        );
    }
}
