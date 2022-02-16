package org.lflang.generator.python;

import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

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
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.ErrorReporter;
import org.lflang.JavaAstUtils;
import org.lflang.Target;
import org.lflang.ASTUtils;

public class PythonReactionGenerator {
    /**
     * Generate code to call reaction numbered "reactionIndex" in reactor "decl".
     * @param decl The reactor containing the reaction
     * @param reactionIndex The index of the reaction
     * @param pyObjectDescriptor CPython related descriptors for each object in "pyObjects".
     * @param pyObjects CPython related objects
     */
    public static String generateCPythonReactionCaller(ReactorDecl decl, 
                                                        int reactionIndex, 
                                                        List<String> pyObjects) {
        String pythonFunctionName = generatePythonReactionFunctionName(reactionIndex);
        String cpythonFunctionName = generateCPythonReactionFunctionName(reactionIndex);
        return generateCPythonFunctionCaller(decl.getName(), pythonFunctionName, cpythonFunctionName, pyObjects);
    }

    /**
     * Generate code to call deadline function numbered "reactionIndex" in reactor "decl".
     * @param decl The reactor containing the reaction
     * @param reactionIndex The index of the reaction
     * @param pyObjectDescriptor CPython related descriptors for each object in "pyObjects".
     * @param pyObjects CPython related objects
     */
    public static String generateCPythonDeadlineCaller(ReactorDecl decl,
                                                       int reactionIndex, 
                                                       List<String> pyObjects) {
        String pythonFunctionName = generatePythonDeadlineFunctionName(reactionIndex);
        String cpythonFunctionName = generateCPythonDeadlineFunctionName(reactionIndex);
        return generateCPythonFunctionCaller(decl.getName(), pythonFunctionName, cpythonFunctionName, pyObjects);
    }

    /**
     * Generate code to call a CPython function.
     * @param reactorDeclName The name of the reactor for debugging purposes
     * @param pythonFunctionName The name of the function in the .py file.
     * @param cpythonFunctionName The name of the function in self struct of the .c file. 
     * @param pyObjectDescriptor CPython related descriptors for each object in "pyObjects".
     * @param pyObjects CPython related objects
     */
    private static String generateCPythonFunctionCaller(String reactorDeclName,
                                                        String pythonFunctionName,
                                                        String cpythonFunctionName,
                                                        List<String> pyObjects) {
        String pyObjectsJoined = pyObjects.size() > 0 ? ", " + String.join(", ", pyObjects) : "";
        return String.join("\n", 
            "DEBUG_PRINT(\"Calling reaction function "+reactorDeclName+"."+pythonFunctionName+"\");",
            "PyObject *rValue = PyObject_CallObject(",
            "    self->"+cpythonFunctionName+", ",
            "    Py_BuildValue(\"("+"O".repeat(pyObjects.size())+")\""+pyObjectsJoined+")",
            ");",
            "if (rValue == NULL) {",
            "    error_print(\"FATAL: Calling reaction "+reactorDeclName+"."+pythonFunctionName+" failed.\");",
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

    public static String generateInitializers(Reaction reaction, 
                                              ReactorDecl decl, 
                                              int reactionIndex, 
                                              Instantiation mainDef, 
                                              ErrorReporter errorReporter,
                                              CTypes types,
                                              boolean isFederatedAndDecentralized) {
        // Contains the actual comma separated list of inputs to the reaction of type generic_port_instance_struct or generic_port_instance_with_token_struct.
        // Each input must be cast to (PyObject *) (aka their descriptors for Py_BuildValue are "O")
        List<String> pyObjects = new ArrayList<>();
        CodeBuilder code = new CodeBuilder();
        code.pr(generateCReactionFunctionHeader(decl, reactionIndex) + " {");
        code.indent();

        // First, generate C initializations
        code.pr(CReactionGenerator.generateInitializationForReaction("", reaction, decl, reactionIndex, 
                                                                     types, errorReporter, mainDef, 
                                                                     isFederatedAndDecentralized, 
                                                                     Target.Python.requiresTypes));
        code.prSourceLineNumber(reaction.getCode());

        // Ensure that GIL is locked
        code.pr(PyUtil.generateGILAcquireCode());
    
        // Generate CPython-related initializations
        code.pr(generateCPythonInitializers(reaction, decl, pyObjects, errorReporter));
        
        // Call the Python reaction
        code.pr(generateCPythonReactionCaller(decl, reactionIndex, pyObjects));
        code.unindent();
        code.pr("}");
        
        // Now generate code for the deadline violation function, if there is one.
        if (reaction.getDeadline() != null) {
            // The following name has to match the choice in generateReactionInstances
            code.pr(generateCDeadlineFunctionHeader(decl, reactionIndex) + " {");
            code.indent();
            code.pr(CReactionGenerator.generateInitializationForReaction("", reaction, decl, reactionIndex, 
                                                                         types, errorReporter, mainDef, 
                                                                         isFederatedAndDecentralized,
                                                                         Target.Python.requiresTypes));        
            code.pr(generateCPythonDeadlineCaller(decl, reactionIndex, pyObjects));
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
    private static String generateCPythonInitializers(Reaction reaction,
                                                      ReactorDecl decl,
                                                      List<String> pyObjects,
                                                      ErrorReporter errorReporter) {
        Set<Action> actionsAsTriggers = new LinkedHashSet<>();
        Reactor reactor = ASTUtils.toDefinition(decl);
        CodeBuilder code = new CodeBuilder();
        // Next, add the triggers (input and actions; timers are not needed).
        // TODO: handle triggers
        for (TriggerRef trigger : ASTUtils.convertToEmptyListIfNull(reaction.getTriggers())) {
            if (trigger instanceof VarRef) {
                VarRef triggerAsVarRef = (VarRef) trigger;
                code.pr(generateVariableToSendPythonReaction(triggerAsVarRef, actionsAsTriggers, decl, pyObjects));
            }
        }
        if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
            // No triggers are given, which means react to any input.
            // Declare an argument for every input.
            // NOTE: this does not include contained outputs. 
            for (Input input : reactor.getInputs()) {
                PythonPortGenerator.generateInputVariablesToSendToPythonReaction(pyObjects, input, decl);
            }
        }

        // Next add non-triggering inputs.
        for (VarRef src : ASTUtils.convertToEmptyListIfNull(reaction.getSources())) {
            code.pr(generateVariableToSendPythonReaction(src, actionsAsTriggers, decl, pyObjects));
        }

        // Next, handle effects
        if (reaction.getEffects() != null) {
            for (VarRef effect : reaction.getEffects()) {
                if (effect.getVariable() instanceof Action) {
                    // It is an action, not an output.
                    // If it has already appeared as trigger, do not redefine it.
                    if (!actionsAsTriggers.contains(effect.getVariable())) {
                        PythonPortGenerator.generateActionVariableToSendToPythonReaction(pyObjects,
                            (Action) effect.getVariable(), decl);
                    }
                } else {
                    if (effect.getVariable() instanceof Output) {
                        PythonPortGenerator.generateOutputVariablesToSendToPythonReaction(pyObjects, (Output) effect.getVariable());
                    } else if (effect.getVariable() instanceof Input) {
                        // It is the input of a contained reactor.
                        code.pr(PythonPortGenerator.generateVariablesForSendingToContainedReactors(pyObjects, effect.getContainer(), (Input) effect.getVariable()));
                    } else {
                        errorReporter.reportError(
                            reaction,
                            "In generateReaction(): " + effect.getVariable().getName() + " is neither an input nor an output."
                        );
                    }
                }
            }
        }
        return code.toString();
    }

    private static String generateVariableToSendPythonReaction(VarRef varRef, 
                                                             Set<Action> actionsAsTriggers, 
                                                             ReactorDecl decl,
                                                             List<String> pyObjects) {
        if (varRef.getVariable() instanceof Port) {
            return PythonPortGenerator.generatePortVariablesToSendToPythonReaction(pyObjects, varRef, decl);
        } else if (varRef.getVariable() instanceof Action) {
            actionsAsTriggers.add((Action) varRef.getVariable());
            PythonPortGenerator.generateActionVariableToSendToPythonReaction(pyObjects, (Action) varRef.getVariable(), decl);
        }
        return "";
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

    /**
     * Generate code that is executed while the reactor instance is being initialized.
     * This wraps the reaction functions in a Python function.
     * @param instance The reactor instance.
     * @param reactions The reactions of this instance.
     */
    public static String generateCPythonLinkers(ReactorInstance instance,
                                                Iterable<ReactionInstance> reactions,
                                                Instantiation mainDef,
                                                String topLevelName) {
        String nameOfSelfStruct = CUtil.reactorRef(instance);
        Reactor reactor = ASTUtils.toDefinition(instance.getDefinition().getReactorClass());
        CodeBuilder initializeTriggerObjects = new CodeBuilder();

        // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.getName().contains(GeneratorBase.GEN_DELAY_CLASS_NAME) ||
            instance.getDefinition().getReactorClass() == (mainDef != null ? mainDef.getReactorClass() : null) && reactor.isFederated()) {
            return "";
        }

        // Initialize the name field to the unique name of the instance
        initializeTriggerObjects.pr(nameOfSelfStruct+"->_lf_name = \""+instance.uniqueID()+"_lf\";");

        for (ReactionInstance reaction : reactions) {
            // Create a PyObject for each reaction
            initializeTriggerObjects.pr(
                generateCPythonFunctionLinker(nameOfSelfStruct, generateCPythonReactionFunctionName(reaction.index), 
                                              instance, generatePythonReactionFunctionName(reaction.index))
            );

            if (reaction.getDefinition().getDeadline() != null) {
                initializeTriggerObjects.pr(
                    generateCPythonFunctionLinker(nameOfSelfStruct, generateCPythonDeadlineFunctionName(reaction.index), 
                                                  instance, generatePythonDeadlineFunctionName(reaction.index))
                );
            }
        }
        return initializeTriggerObjects.toString();
    }

    /**
     * Generate code to link "pythonFunctionName" to "cpythonFunctionName" in "nameOfSelfStruct" of "instance".
     * @param nameOfSelfStruct the self struct name of instance
     * @param cpythonFunctionName the name of the cpython function
     * @param instance the reactor instance
     * @param pythonFunctionName the name of the python function
     */
    private static String generateCPythonFunctionLinker(String nameOfSelfStruct, String cpythonFunctionName, ReactorInstance instance, String pythonFunctionName) {
        return String.join("\n", 
            nameOfSelfStruct+"->"+cpythonFunctionName+" = ",
            "get_python_function(\"__main__\", ",
            "    "+nameOfSelfStruct+"->_lf_name,",
            "    "+CUtil.runtimeIndex(instance)+",",
            "    \""+pythonFunctionName+"\");"
        );
    }

    /** Return the top level C function header for the deadline function numbered "reactionIndex" in "decl"
     *  @param decl The reactor declaration
     *  @param reactionIndex The reaction index.
     *  @return The function name for the deadline function.
     */
    public static String generateCDeadlineFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String deadlineFunctionName = CUtil.generateDeadlineFunctionName(decl, reactionIndex);
        return "void " + deadlineFunctionName + "(void* instance_args)";
    }

    /** Return the top level C function header for the reaction numbered "reactionIndex" in "decl"
     *  @param decl The reactor declaration
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generateCReactionFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String deadlineFunctionName = CUtil.generateReactionFunctionName(decl, reactionIndex);
        return "void " + deadlineFunctionName + "(void* instance_args)";
    }

    /** Return the function name of the reaction inside the self struct in the .c file.
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generateCPythonReactionFunctionName(int reactionIndex) {
        return "_lf_py_reaction_function_"+reactionIndex;
    }

    /** Return the function name of the deadline function inside the self struct in the .c file.
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generateCPythonDeadlineFunctionName(int reactionIndex) {
        return "_lf_py_reaction_function_"+reactionIndex;
    }

    /** Return the function name of the reaction in the .py file.
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generatePythonReactionFunctionName(int reactionIndex) {
        return "reaction_function_" + reactionIndex;
    }

    /** Return the function name of the deadline function in the .py file.
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generatePythonDeadlineFunctionName(int reactionIndex) {
        return "deadline_function_" + reactionIndex;
    }
}
