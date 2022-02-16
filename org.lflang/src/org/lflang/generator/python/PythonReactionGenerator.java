package org.lflang.generator.python;

import java.util.Set;
import java.util.LinkedHashSet;

import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Action;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Port;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.generator.c.CReactionGenerator;
import org.lflang.generator.c.CTypes;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.CodeBuilder;
import org.lflang.ErrorReporter;
import org.lflang.JavaAstUtils;
import org.lflang.Target;
import org.lflang.ASTUtils;

public class PythonReactionGenerator {
    public static String generateCallPythonReactionCode(ReactorDecl decl, 
                                                        int reactionIndex, 
                                                        StringBuilder pyObjectDescriptor,
                                                        StringBuilder pyObjects) {
        String pythonFunctionName = PyUtil.generatePythonReactionFunctionName(reactionIndex);
        return String.join("\n", 
            "DEBUG_PRINT(\"Calling reaction function "+decl.getName()+"."+pythonFunctionName+"\");",
            "PyObject *rValue = PyObject_CallObject(",
            "    self->_lf_py_reaction_function_"+reactionIndex+", ",
            "    Py_BuildValue(\"("+pyObjectDescriptor+")\" "+pyObjects+")",
            ");",
            "if (rValue == NULL) {",
            "    error_print(\"FATAL: Calling reaction "+decl.getName()+"."+pythonFunctionName+" failed.\");",
            "    if (PyErr_Occurred()) {",
            "        PyErr_PrintEx(0);",
            "        PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "    }",
            "    "+PyUtil.generateGILReleaseCode(),
            "    Py_FinalizeEx();",
            "    exit(1);",
            "}",
            "",
            "/* Release the thread. No Python API allowed beyond this point. */",
            PyUtil.generateGILReleaseCode()
        );
    }

    public static String generateDeadlineViolationCode(ReactorDecl decl,
                                                       int reactionIndex, 
                                                       StringBuilder pyObjectDescriptor,
                                                       StringBuilder pyObjects) {
        String deadlineFunctionName = CUtil.generateDeadlineFunctionName(decl, reactionIndex);
        return String.join("\n", 
            ""+PyUtil.generateGILAcquireCode()+"",
            "",
            "DEBUG_PRINT(\"Calling deadline function "+decl.getName()+"."+deadlineFunctionName+"\");",
            "PyObject *rValue = PyObject_CallObject(",
            "    self->_lf_py_deadline_function_"+reactionIndex+", ",
            "    Py_BuildValue(\"("+pyObjectDescriptor+")\" "+pyObjects+")",
            ");",
            "if (rValue == NULL) {",
            "    error_print(\"FATAL: Calling reaction "+decl.getName()+"."+deadlineFunctionName+" failed.\\n\");",
            "    if (rValue == NULL) {",
            "        if (PyErr_Occurred()) {",
            "            PyErr_PrintEx(0);",
            "            PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "        }",
            "    }",
            "    /* Release the thread. No Python API allowed beyond this point. */",
            "    "+PyUtil.generateGILReleaseCode(),
            "    Py_FinalizeEx();",
            "    exit(1);",
            "}",
            "",
            "/* Release the thread. No Python API allowed beyond this point. */",
            PyUtil.generateGILReleaseCode()
        );
    }

    public static String generateDeadlineFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String deadlineFunctionName = CUtil.generateDeadlineFunctionName(decl, reactionIndex);
        return "void " + deadlineFunctionName + "(void* instance_args)";
    }

    public static String generateReactionFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String deadlineFunctionName = CUtil.generateReactionFunctionName(decl, reactionIndex);
        return "void " + deadlineFunctionName + "(void* instance_args)";
    }

    public static String generateCReaction(Reaction reaction, 
                                         ReactorDecl decl, 
                                         int reactionIndex, 
                                         Instantiation mainDef, 
                                         ErrorReporter errorReporter,
                                         CTypes types,
                                         boolean isFederatedAndDecentralized) {
        // Contains "O" characters. The number of these characters depend on the number of inputs to the reaction
        StringBuilder pyObjectDescriptor = new StringBuilder();

        // Contains the actual comma separated list of inputs to the reaction of type generic_port_instance_struct or generic_port_instance_with_token_struct.
        // Each input must be cast to (PyObject *)
        StringBuilder pyObjects = new StringBuilder();
        CodeBuilder code = new CodeBuilder();
        code.pr(generateReactionFunctionHeader(decl, reactionIndex) + " {");
        code.indent();

        // First, generate C initializations
        code.pr(CReactionGenerator.generateInitializationForReaction("", reaction, decl, reactionIndex, 
                                                                     types, errorReporter, mainDef, 
                                                                     isFederatedAndDecentralized, 
                                                                     Target.Python.requiresTypes));
        code.prSourceLineNumber(reaction.getCode());

        // Ensure that GIL is locked
        code.pr(PyUtil.generateGILAcquireCode());
    
        // Generate Python-related initializations
        generatePythonInitializers(reaction, decl, pyObjectDescriptor, pyObjects, code, errorReporter);
        
        // Call the Python reaction
        code.pr(generateCallPythonReactionCode(decl, reactionIndex, pyObjectDescriptor, pyObjects));
        code.unindent();
        code.pr("}");
        
        // Now generate code for the deadline violation function, if there is one.
        if (reaction.getDeadline() != null) {
            // The following name has to match the choice in generateReactionInstances
            code.pr(generateDeadlineFunctionHeader(decl, reactionIndex) + " {");
            code.indent();
            code.pr(CReactionGenerator.generateInitializationForReaction("", reaction, decl, reactionIndex, 
                                                                         types, errorReporter, mainDef, 
                                                                         isFederatedAndDecentralized,
                                                                         Target.Python.requiresTypes));        
            code.pr(generateDeadlineViolationCode(decl, reactionIndex, pyObjectDescriptor, pyObjects));
            code.unindent();
            code.pr("}");
        }
        return code.toString();
    }

    /**
     * Generate necessary Python-specific initialization code for <code>reaction<code> that belongs to reactor 
     * <code>decl<code>.
     * 
     * @param reaction The reaction to generate Python-specific initialization for.
     * @param decl The reactor to which <code>reaction<code> belongs to.
     * @param pyObjectDescriptor For each port object created, a Python-specific descriptor will be added to this that
     *  then can be used as an argument to <code>Py_BuildValue<code> 
     *  (@see <a href="https://docs.python.org/3/c-api/arg.html#c.Py_BuildValue">docs.python.org/3/c-api</a>).
     * @param pyObjects A "," delimited list of expressions that would be (or result in a creation of) a PyObject.
     */
    private static void generatePythonInitializers(Reaction reaction,
                                                      ReactorDecl decl,
                                                      StringBuilder pyObjectDescriptor,
                                                      StringBuilder pyObjects,
                                                      CodeBuilder code,
                                                      ErrorReporter errorReporter) {
        Set<Action> actionsAsTriggers = new LinkedHashSet<>();
        Reactor reactor = ASTUtils.toDefinition(decl);
        // Next, add the triggers (input and actions; timers are not needed).
        // TODO: handle triggers
        for (TriggerRef trigger : ASTUtils.convertToEmptyListIfNull(reaction.getTriggers())) {
            if (trigger instanceof VarRef) {
                VarRef triggerAsVarRef = (VarRef) trigger;
                generateVariableToSendPythonReaction(triggerAsVarRef, actionsAsTriggers, decl, pyObjectDescriptor, pyObjects, code);
            }
        }
        if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
            // No triggers are given, which means react to any input.
            // Declare an argument for every input.
            // NOTE: this does not include contained outputs. 
            for (Input input : reactor.getInputs()) {
                PythonPortGenerator.generateInputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, input, decl);
            }
        }

        // Next add non-triggering inputs.
        for (VarRef src : ASTUtils.convertToEmptyListIfNull(reaction.getSources())) {
            generateVariableToSendPythonReaction(src, actionsAsTriggers, decl, pyObjectDescriptor, pyObjects, code);
        }

        // Next, handle effects
        if (reaction.getEffects() != null) {
            for (VarRef effect : reaction.getEffects()) {
                if (effect.getVariable() instanceof Action) {
                    // It is an action, not an output.
                    // If it has already appeared as trigger, do not redefine it.
                    if (!actionsAsTriggers.contains(effect.getVariable())) {
                        PythonPortGenerator.generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects,
                            (Action) effect.getVariable(), decl);
                    }
                } else {
                    if (effect.getVariable() instanceof Output) {
                        PythonPortGenerator.generateOutputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects,
                            (Output) effect.getVariable(), decl);
                    } else if (effect.getVariable() instanceof Input) {
                        // It is the input of a contained reactor.
                        PythonPortGenerator.generateVariablesForSendingToContainedReactors(code, pyObjectDescriptor, pyObjects, 
                            effect.getContainer(), (Input) effect.getVariable(), decl);
                    } else {
                        errorReporter.reportError(
                            reaction,
                            "In generateReaction(): " + effect.getVariable().getName() + " is neither an input nor an output."
                        );
                    }

                }
            }
        }
    }

    private static void generateVariableToSendPythonReaction(VarRef varRef, 
                                                             Set<Action> actionsAsTriggers, 
                                                             ReactorDecl decl,
                                                             StringBuilder pyObjectDescriptor,
                                                             StringBuilder pyObjects,
                                                             CodeBuilder code) {
        if (varRef.getVariable() instanceof Port) {
            PythonPortGenerator.generatePortVariablesToSendToPythonReaction(code, pyObjectDescriptor, pyObjects, varRef, decl);
        } else if (varRef.getVariable() instanceof Action) {
            actionsAsTriggers.add((Action) varRef.getVariable());
            PythonPortGenerator.generateActionVariableToSendToPythonReaction(pyObjectDescriptor, pyObjects,
                (Action) varRef.getVariable(), decl);
        }
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action The action to schedule
     * @param port The port to read from
     */
    public static String generateCDelayBody(Action action, VarRef port, boolean isTokenType) {
        String ref = JavaAstUtils.generateVarRef(port);
        // Note that the action.type set by the base class is actually
        // the port type.
        if (isTokenType) {
            return String.join("\n", 
                "if ("+ref+"->is_present) {",
                "    // Put the whole token on the event queue, not just the payload.",
                "    // This way, the length and element_size are transported.",
                "    schedule_token("+action.getName()+", 0, "+ref+"->token);",
                "}"
            );
        } else {
            return String.join("\n", 
                "// Create a token.",
                "#if NUMBER_OF_WORKERS > 0",
                "// Need to lock the mutex first.",
                "lf_mutex_lock(&mutex);",
                "#endif",
                "lf_token_t* t = create_token(sizeof(PyObject*));",
                "#if NUMBER_OF_WORKERS > 0",
                "lf_mutex_unlock(&mutex);",
                "#endif",
                "t->value = self->_lf_"+ref+"->value;",
                "t->length = 1; // Length is 1",
                "",
                "// Pass the token along",
                "schedule_token("+action.getName()+", 0, t);"
            );
        }
    }
}
