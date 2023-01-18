package org.lflang.generator.c;

import org.lflang.ASTUtils;
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
     * Generate watchdog definition in parent struct.
     */
    public static void generateWatchdogStruct(
        FederateInstance currentFederate,
        CodeBuilder body,
        ReactorDecl decl,
        CodeBuilder constructorCode
    ) {
        var reactor = ASTUtils.toDefinition(decl);

        // WATCHDOG QUESTION 1: I followed similar format to 
        // `CReactionGenerator.generateReactionAndTriggerStructs`
        // but am not sure why we need to check if watchdog exists in the 
        // current federate.
        for (Watchdog watchdog : ASTUtils.allWatchdogs(reactor)) {
            if (currentFederate.contains(watchdog)) {

                String watchdogName = watchdog.getName();
                // Create pointer to the watchdog_t struct
                // WATCHDOG QUESTION 2: Why need to put object at beginning of 
                // `.pr` func call?
                
                // WATCHDOG QUESTION 3: Is the space for this struct automatically allocated
                // through `_lf_new_reactor`? `_lf__startup_reaction` is also a pointer in self struct
                // but does not seem to have a separate allocation call.
                body.pr(watchdog, "watchdog_t* _lf_watchdog_"+watchdogName+";");

                // WATCHDOG QUESTION 4: Not sure if this is correct, may need to use 
                // 'getTargetTime' instead. watchdog timeout is listed as "Expression"
                // in the grammar, so I'm not sure if it is reading the timeout as 
                // a Time class or TimeValue class.
                var min_expiration = GeneratorBase.timeInTargetLanguage(watchdog.getTimeout());

                // watchdog function name
                var watchdogFunctionName = generateWatchdogFunctionName(watchdog, decl);
                // Set values of watchdog_t struct in the reactor's constructor
                // WATCHDOG QUESTION 5: should I be defining these in the constructor of the reactor?
                constructorCode.pr(watchdog, String.join("\n",
                    "self->_lf_watchdog_"+watchdogName+".self = self;",
                    "self->_lf_watchdog_"+watchdogName+".expiration = NEVER;",
                    "self->_lf_watchdog_"+watchdogName+".min_expiration = "+min_expiration+";",
                    "self->_lf_watchdog_"+watchdogName+".watchdog_function = "+watchdogFunctionName+";"
                ));

                // WATCHDOG QUESTION 6: should I be initializing mutex in this constructor?
            }
        }
    }

    /** Generate a watchdog function definition for a reactor.
     *  This function will have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param watchdog The watchdog.
     *  @param decl The reactor.
     */
    public static String generateWatchdog(
        Watchdog watchdog,
        ReactorDecl decl
    ) {
        var code = new CodeBuilder();

        // WATCHDOG QUESTION: Do I need this header? What it for?
        code.pr(
            "#include " + StringUtil.addDoubleQuotes(
                CCoreFilesUtils.getCTargetSetHeader()));

        code.pr(generateWatchdogFunction(watchdog, decl));

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
