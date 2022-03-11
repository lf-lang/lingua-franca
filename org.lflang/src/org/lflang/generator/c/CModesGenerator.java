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
}
