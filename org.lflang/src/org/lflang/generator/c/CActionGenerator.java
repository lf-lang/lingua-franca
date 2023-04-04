package org.lflang.generator.c;

import java.util.List;
import java.util.ArrayList;
import org.lflang.ASTUtils;
import org.lflang.Target;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Action;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import static org.lflang.generator.c.CGenerator.variableStructType;
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
 */
public class CActionGenerator {
    /**
     * For each action of the specified reactor instance, generate initialization code
     * for the offset and period fields.
     * @param instance The reactor.
     */
    public static String generateInitializers(
        ReactorInstance instance
    ) {
        List<String> code = new ArrayList<>();
        for (ActionInstance action : instance.actions) {
            if (!action.isShutdown()) {
                var triggerStructName = CUtil.reactorRef(action.getParent()) + "->_lf__" + action.getName();
                var minDelay = action.getMinDelay();
                var minSpacing = action.getMinSpacing();
                var offsetInitializer = triggerStructName+".offset = " + GeneratorBase.timeInTargetLanguage(minDelay) + ";";
                var periodInitializer = triggerStructName+".period = " + (minSpacing != null ?
                                                                         GeneratorBase.timeInTargetLanguage(minSpacing) :
                                                                         CGenerator.UNDEFINED_MIN_SPACING) + ";";
                code.addAll(List.of(
                    "// Initializing action "+action.getFullName(),
                    offsetInitializer,
                    periodInitializer
                ));

                var mode = action.getMode(false);
                if (mode != null) {
                    var modeParent = mode.getParent();
                    var modeRef = "&"+CUtil.reactorRef(modeParent)+"->_lf__modes["+modeParent.modes.indexOf(mode)+"];";
                    code.add(triggerStructName+".mode = "+modeRef+";");
                } else {
                    code.add(triggerStructName+".mode = NULL;");
                }
            }
        }
        return String.join("\n", code);
    }

    /**
     * Create a template token initialized to the payload size.
     * This token is marked to not be freed so that the trigger_t struct
     * always has a template token.
     * At the start of each time step, we need to initialize the is_present field
     * of each action's trigger object to false and free a previously
     * allocated token if appropriate. This code sets up the table that does that.
     *
     * @param selfStruct The variable name of the self struct
     * @param actionName The action name
     * @param payloadSize The code that returns the size of the action's payload in C.
     */
    public static String generateTokenInitializer(
        String selfStruct,
        String actionName,
        String payloadSize
    ) {
        return String.join("\n",
                "_lf_initialize_template((token_template_t*)",
                "        &("+selfStruct+"->_lf__"+actionName+"),",
                         payloadSize+");",
            selfStruct+"->_lf__"+actionName+".status = absent;"
        );
    }

    /**
     * Generate the declarations of actions in the self struct
     *
     * @param reactor The reactor to generate declarations for
     * @param decl The reactor's declaration
     * @param body The content of the self struct
     * @param constructorCode The constructor code of the reactor
     */
    public static void generateDeclarations(
        Reactor reactor,
        ReactorDecl decl,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        for (Action action : ASTUtils.allActions(reactor)) {
            var actionName = action.getName();
            body.pr(action, CGenerator.variableStructType(action, decl)+" _lf_"+actionName+";");
            // Initialize the trigger pointer in the action.
            constructorCode.pr(action, "self->_lf_"+actionName+".trigger = &self->_lf__"+actionName+";");
        }
    }

    /**
     * Generate the struct type definitions for the action of the
     * reactor
     *
     * @param decl The reactor declaration
     * @param action The action to generate the struct for
     * @param target The target of the code generation (C, CCpp or Python)
     * @param types The helper object for types related stuff
     * @param federatedExtension The code needed to support federated execution
     * @return The auxiliary struct for the port as a string
     */
    public static String generateAuxiliaryStruct(
        ReactorDecl decl,
        Action action,
        Target target,
        CTypes types,
        CodeBuilder federatedExtension
    ) {
        var code = new CodeBuilder();
        code.pr("typedef struct {");
        code.indent();
        // NOTE: The following fields are required to be the first ones so that
        // pointer to this struct can be cast to a (lf_action_base_t*) or to
        // (token_template_t*) to access these fields for any port.
        // IMPORTANT: These must match exactly the fields defined in port.h!!
        code.pr(String.join("\n",
                "token_type_t type;",  // From token_template_t
                "lf_token_t* token;",  // From token_template_t
                "size_t length;",      // From token_template_t
                "bool is_present;",    // From lf_action_base_t
                "bool has_value;",     // From lf_action_base_t
                "trigger_t* trigger;"  // From lf_action_base_t
        ));
        code.pr(valueDeclaration(action, target, types));
        code.pr(federatedExtension.toString());
        code.unindent();
        code.pr("} " + variableStructType(action, decl) + ";");
        return code.toString();
    }

    /**
     * For the specified action, return a declaration for action struct to
     * contain the value of the action. An action of
     * type int[10], for example, will result in this:
     * ```
     *     int* value;
     * ```
     * This will return an empty string for an action with no type.
     * @param action The action.
     * @return A string providing the value field of the action struct.
     */
    private static String valueDeclaration(
        Action action,
        Target target,
        CTypes types
    ) {
        if (target == Target.Python) {
            return "PyObject* value;";
        }
        // Do not convert to lf_token_t* using lfTypeToTokenType because there
        // will be a separate field pointing to the token.
        return action.getType() == null && target.requiresTypes ?
               "" :
               types.getTargetType(action) + " value;";
    }
}
