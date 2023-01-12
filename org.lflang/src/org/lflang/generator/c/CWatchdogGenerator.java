package org.lflang.generator.c;

import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.CReactionGenerator;

/**
 * Generates necessary C code for watchdogs.
 *
 * @author{Benjamin Asch <benjamintasch@berkeley.edu>}
 */
//FIXME: modif4watchdogs
public class CWatchdogGenerator {

    /**
     * Generate necessary initialization code inside the body of the watchdog that belongs to reactor decl.
     * @param decl The reactor that has the watchdog
     */
    public static String generateInitializationForWatchdog(Watchdog watchdog,
                                                           ReactorDecl decl) {
        Reactor reactor = ASTUtils.toDefinition(decl);

        // Construct the reactionInitialization code to go into
        // the body of the function before the verbatim code.
        CodeBuilder watchdogInitialization = new CodeBuilder();

        CodeBuilder code = new CodeBuilder();

        // Define the "self" struct.
        String structType = CUtil.selfType(decl);
        // A null structType means there are no inputs, state,
        // or anything else. No need to declare it.
        if (structType != null) {
             code.pr(String.join("\n",
                 structType+"* self = ("+structType+"*)instance_args; SUPPRESS_UNUSED_WARNING(self);"
             ));
        }

        // Declare mode if in effects field of watchdog
        if (watchdog.getEffects() != null) {
            for (VarRef effect : watchdog.getEffects()) {
                Variable variable = effect.getVariable();
                if (variable instanceof Mode) {
                    // Mode change effect
                    int idx = ASTUtils.allModes(reactor).indexOf((Mode)effect.getVariable());
                    String name = effect.getVariable().getName();
                    if (idx >= 0) {
                        watchdogInitialization.pr(
                            "reactor_mode_t* " + name + " = &self->_lf__modes[" + idx + "];\n"
                            + "lf_mode_change_type_t _lf_" + name + "_change_type = "
                            + (effect.getTransition() == ModeTransition.HISTORY ?
                                    "history_transition" : "reset_transition")
                            + ";"
                        );
                    } 
                    // FIXME: include error reporter
                    // else {
                    //     errorReporter.reportError(
                    //         watchdog,
                    //         "In generateWatchdog(): " + name + " not a valid mode of this reactor."
                    //     );
                    // }
                }
            }
        }

        // Next generate all the collected setup code.
        code.pr(watchdogInitialization.toString());
        return code.toString();
    }

    /**
     * Returns the name of the watchdog function for reaction.
     * @param decl The reactor with the watchdog
     * @param watchdog The watchdog
     * @return Name of the watchdog function for reaction
     */
    public static String generateWatchdogFunctionName(Watchdog watchdog, ReactorDecl decl) {
        return decl.getName().toLowerCase() + "_" + watchdog.getName().toLowerCase() + "_watchdog_function";
    }

    /** Return the top level C function header for the watchdog function in "decl"
     * @param decl The reactor declaration
     * @param watchdog The watchdog.
     * @return The function name for the watchdog function.
     */
    public static String generateWatchdogFunctionHeader(Watchdog watchdog,
                                                        ReactorDecl decl) {
        String functionName = generateWatchdogFunctionName(watchdog, decl);
        return CReactionGenerator.generateFunctionHeader(functionName);
    }

    /**
     * Generate the watchdog function.
     */
    public static String generateWatchdogFunction(Watchdog watchdog,
                                                  ReactorDecl decl) {
        return CReactionGenerator.generateFunction(generateWatchdogFunctionHeader(watchdog, decl),
                                                   generateInitializationForWatchdog(watchdog, decl),
                                                   watchdog.getCode());
    }

    /**
     * Generate a constructor for the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param federate A federate name, or null to unconditionally generate.
     * @param constructorCode Lines of code previously generated that need to
     *  go into the constructor.
     */
    //FIXME: this is just the old constructor meant for reactors
    public static String generateWatchdogConstructor(
        ReactorDecl reactor,
        FederateInstance federate,
        String constructorCode
    ) {
        var structType = CUtil.selfType(reactor);
        var code = new CodeBuilder();
        code.pr(structType+"* new_"+reactor.getName()+"() {");
        code.indent();
        code.pr(structType+"* self = ("+structType+"*)_lf_new_reactor(sizeof("+structType+"));");
        code.pr(constructorCode);
        code.pr("return self;");
        code.unindent();
        code.pr("}");
        return code.toString();
    }

     /**
     * Generate _lf_initialize_watchdog_mutexes function.
     */
    //FIXME: finish implementing
    public static String generateLfInitializeWatchdogMutexes(List<Reactor> reactors) {
        // need to find way to assign get watchdog from AST
        // need to assign watchdog to correct reaction
        var s = new StringBuilder();
        s.append("void _lf_initialize_watchdog_mutexes() {\n");
        
    }
}
