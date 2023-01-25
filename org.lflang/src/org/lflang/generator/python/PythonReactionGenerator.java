package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.AttributeUtils;
import org.lflang.ErrorReporter;
import org.lflang.Target;

import org.lflang.generator.c.CReactionGenerator;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Action;
import org.lflang.lf.Code;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.util.StringUtil;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Port;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.generator.c.CCoreFilesUtils;
import org.lflang.generator.c.CTypes;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Mode;

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
                                                        List<String> pyObjects,
                                                        String inits) {
        String pythonFunctionName = generatePythonReactionFunctionName(reactionIndex);
        String cpythonFunctionName = generateCPythonReactionFunctionName(reactionIndex);
        return generateCPythonFunctionCaller(decl.getName(), pythonFunctionName, cpythonFunctionName, pyObjects, inits);
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
        return generateCPythonFunctionCaller(decl.getName(), pythonFunctionName, cpythonFunctionName, pyObjects, "");
    }

    /**
     * Generate code to call deadline function numbered "reactionIndex" in reactor "decl".
     * @param decl The reactor containing the reaction
     * @param reactionIndex The index of the reaction
     * @param pyObjectDescriptor CPython related descriptors for each object in "pyObjects".
     * @param pyObjects CPython related objects
     */
    public static String generateCPythonSTPCaller(ReactorDecl decl,
                                                       int reactionIndex,
                                                       List<String> pyObjects) {
        String pythonFunctionName = generatePythonSTPFunctionName(reactionIndex);
        String cpythonFunctionName = generateCPythonSTPFunctionName(reactionIndex);
        return generateCPythonFunctionCaller(decl.getName(), pythonFunctionName, cpythonFunctionName, pyObjects, "");
    }

    /**
     * Generate code to call a CPython function.
     * @param reactorDeclName The name of the reactor for debugging purposes
     * @param pythonFunctionName The name of the function in the .py file.
     * @param cpythonFunctionName The name of the function in self struct of the .c file.
     * @param pyObjects CPython related objects
     */
    private static String generateCPythonFunctionCaller(String reactorDeclName,
                                                        String pythonFunctionName,
                                                        String cpythonFunctionName,
                                                        List<String> pyObjects,
                                                        String inits) {
        String pyObjectsJoined = pyObjects.size() > 0 ? ", " + String.join(", ", pyObjects) : "";
        CodeBuilder code = new CodeBuilder();
        code.pr(PyUtil.generateGILAcquireCode());
        code.pr(inits);
        code.pr(String.join("\n",
                "LF_PRINT_DEBUG(\"Calling reaction function "+reactorDeclName+"."+pythonFunctionName+"\");",
                "PyObject *rValue = PyObject_CallObject(",
                "    self->"+cpythonFunctionName+", ",
                "    Py_BuildValue(\"("+"O".repeat(pyObjects.size())+")\""+pyObjectsJoined+")",
                ");",
                "if (rValue == NULL) {",
                "    lf_print_error(\"FATAL: Calling reaction "+reactorDeclName+"."+pythonFunctionName+" failed.\");",
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
        ));
        return code.toString();
    }

    /**
     * Generate the reaction in the .c file, which calls the Python reaction through the CPython interface.
     *
     * @param reaction The reaction to generate Python-specific initialization for.
     * @param decl The reactor to which <code>reaction<code> belongs to.
     * @param reactionIndex The index number of the reaction in decl.
     * @param mainDef The main reactor.
     * @param errorReporter An error reporter.
     * @param types A helper class for type-related stuff.
     */
    public static String generateCReaction(
        Reaction reaction,
        ReactorDecl decl,
        int reactionIndex,
        Instantiation mainDef,
        ErrorReporter errorReporter,
        CTypes types
    ) {
        // Contains the actual comma separated list of inputs to the reaction of type generic_port_instance_struct.
        // Each input must be cast to (PyObject *) (aka their descriptors for Py_BuildValue are "O")
        List<String> pyObjects = new ArrayList<>();
        CodeBuilder code = new CodeBuilder();
        String cPyInit = generateCPythonInitializers(reaction, decl, pyObjects, errorReporter);
        String cInit = CReactionGenerator.generateInitializationForReaction(
                                                "", reaction, decl, reactionIndex,
                                                types, errorReporter, mainDef,
                                                Target.Python.requiresTypes);
        code.pr(
            "#include " + StringUtil.addDoubleQuotes(
                CCoreFilesUtils.getCTargetSetHeader()));
        code.pr(generateFunction(
                    CReactionGenerator.generateReactionFunctionHeader(decl, reactionIndex),
                    cInit, reaction.getCode(),
                    generateCPythonReactionCaller(decl, reactionIndex, pyObjects, cPyInit)
        ));

        // Generate code for the STP violation handler, if there is one.
        if (reaction.getStp() != null) {
            code.pr(generateFunction(
                    CReactionGenerator.generateStpFunctionHeader(decl, reactionIndex),
                    cInit, reaction.getStp().getCode(),
                    generateCPythonSTPCaller(decl, reactionIndex, pyObjects)
                ));
        }
        // Generate code for the deadline violation function, if there is one.
        if (reaction.getDeadline() != null) {
            code.pr(generateFunction(
                CReactionGenerator.generateDeadlineFunctionHeader(decl, reactionIndex),
                cInit, reaction.getDeadline().getCode(),
                generateCPythonDeadlineCaller(decl, reactionIndex, pyObjects)
            ));
        }
        code.pr(
            "#include " + StringUtil.addDoubleQuotes(
                CCoreFilesUtils.getCTargetSetUndefHeader()));
        return code.toString();
    }

    public static String generateFunction(
        String header, String init, Code code, String pyCaller
    ) {
        var function = new CodeBuilder();
        function.pr(header + "{");
        function.indent();
        function.pr(init);
        function.prSourceLineNumber(code);
        function.pr(pyCaller);
        function.unindent();
        function.pr("}");
        return function.toString();
    }

    /**
     * Generate necessary Python-specific initialization code for <code>reaction<code> that belongs to reactor
     * <code>decl<code>.
     *
     * @param reaction The reaction to generate Python-specific initialization for.
     * @param decl The reactor to which <code>reaction<code> belongs to.
     * @param pyObjects A list of expressions that can be used as additional arguments to <code>Py_BuildValue<code>
     *  (@see <a href="https://docs.python.org/3/c-api/arg.html#c.Py_BuildValue">docs.python.org/3/c-api</a>).
     *  We will use as a format string, "(O...O)" where the number of O's is equal to the length of the list.
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
                } else if (effect.getVariable() instanceof Mode) {
                    String name = effect.getVariable().getName();
                    pyObjects.add("convert_C_mode_to_py("+name+",(self_base_t*)self, _lf_"+name+"_change_type)");
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

    /**
     * Generate parameters and their respective initialization code for a reaction function
     * The initialization code is put at the beginning of the reaction before user code
     * @param parameters The parameters used for function definition
     * @param inits The initialization code for those paramters
     * @param decl Reactor declaration
     * @param reaction The reaction to be used to generate parameters for
     */
    public static void generatePythonReactionParametersAndInitializations(List<String> parameters, CodeBuilder inits,
                                                                          ReactorDecl decl, Reaction reaction) {
        Reactor reactor = ASTUtils.toDefinition(decl);
        LinkedHashSet<String> generatedParams = new LinkedHashSet<>();

        // Handle triggers
        for (TriggerRef trigger : ASTUtils.convertToEmptyListIfNull(reaction.getTriggers())) {
            if (!(trigger instanceof VarRef)) {
                continue;
            }
            VarRef triggerAsVarRef = (VarRef) trigger;
            if (triggerAsVarRef.getVariable() instanceof Port) {
                if (triggerAsVarRef.getVariable() instanceof Input) {
                    if (((Input) triggerAsVarRef.getVariable()).isMutable()) {
                        generatedParams.add("mutable_"+triggerAsVarRef.getVariable().getName()+"");

                        // Create a deep copy
                        if (ASTUtils.isMultiport((Input) triggerAsVarRef.getVariable())) {
                            inits.
                                pr(triggerAsVarRef.getVariable().getName()+" = [Make() for i in range(len(mutable_"+triggerAsVarRef.getVariable().getName()+"))]");
                            inits.pr("for i in range(len(mutable_"+triggerAsVarRef.getVariable().getName()+")):");
                            inits.pr("    "+triggerAsVarRef.getVariable().getName()+"[i].value = copy.deepcopy(mutable_"+triggerAsVarRef.getVariable().getName()+"[i].value)");
                        } else {
                            inits.pr(triggerAsVarRef.getVariable().getName()+" = Make()");
                            inits.
                                pr(triggerAsVarRef.getVariable().getName()+".value = copy.deepcopy(mutable_"+triggerAsVarRef.getVariable().getName()+".value)");
                        }
                    } else {
                        generatedParams.add(triggerAsVarRef.getVariable().getName());
                    }
                } else {
                    // Handle contained reactors' ports
                    generatedParams.add(triggerAsVarRef.getContainer().getName()+"_"+triggerAsVarRef.getVariable().getName());
                    inits.pr(PythonPortGenerator.generatePythonPortVariableInReaction(triggerAsVarRef));
                }
            } else if (triggerAsVarRef.getVariable() instanceof Action) {
                generatedParams.add(triggerAsVarRef.getVariable().getName());
            }
        }

        // Handle non-triggering inputs
        if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
            for (Input input : ASTUtils.convertToEmptyListIfNull(reactor.getInputs())) {
                generatedParams.add(input.getName());
                if (input.isMutable()) {
                    // Create a deep copy
                    inits.pr(input.getName()+" = copy.deepcopy("+input.getName()+")");
                }
            }
        }
        for (VarRef src : ASTUtils.convertToEmptyListIfNull(reaction.getSources())) {
            if (src.getVariable() instanceof Output) {
                // Output of a contained reactor
                generatedParams.add(src.getContainer().getName()+"_"+src.getVariable().getName());
                inits.pr(PythonPortGenerator.generatePythonPortVariableInReaction(src));
            } else {
                generatedParams.add(src.getVariable().getName());
                if (src.getVariable() instanceof Input) {
                    if (((Input) src.getVariable()).isMutable()) {
                        // Create a deep copy
                        inits.pr(src.getVariable().getName()+" = copy.deepcopy("+src.getVariable().getName()+")");
                    }
                }
            }
        }

        // Handle effects
        for (VarRef effect : ASTUtils.convertToEmptyListIfNull(reaction.getEffects())) {
            if (effect.getVariable() instanceof Input) {
                generatedParams.add(effect.getContainer().getName()+"_"+effect.getVariable().getName());
                inits.pr(PythonPortGenerator.generatePythonPortVariableInReaction(effect));
            } else {
                generatedParams.add(effect.getVariable().getName());
                if (effect.getVariable() instanceof Port) {
                    if (ASTUtils.isMultiport((Port) effect.getVariable())) {
                        // Handle multiports
                    }
                }
            }
        }

        for (String s : generatedParams) {
            parameters.add(s);
        }
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
     * Generate Python code to link cpython functions to python functions for each reaction.
     * @param instance The reactor instance.
     * @param mainDef The definition of the main reactor
     */
    public static String generateCPythonReactionLinkers(
            ReactorInstance instance,
            Instantiation mainDef
    ) {
        String nameOfSelfStruct = CUtil.reactorRef(instance);
        Reactor reactor = ASTUtils.toDefinition(instance.getDefinition().getReactorClass());
        CodeBuilder code = new CodeBuilder();

        // Initialize the name field to the unique name of the instance
        code.pr(nameOfSelfStruct+"->_lf_name = \""+instance.uniqueID()+"_lf\";");

        for (ReactionInstance reaction : instance.reactions) {
            // Reactions marked with a `@_c_body` attribute are generated in C
            if (AttributeUtils.hasCBody(reaction.getDefinition())) continue;
            // Create a PyObject for each reaction
            code.pr(generateCPythonReactionLinker(instance, reaction, nameOfSelfStruct));
        }
        return code.toString();
    }

    /**
     * Generate Python code to link cpython functions to python functions for a reaction.
     * @param instance The reactor instance.
     * @param reaction The reaction of this instance to link.
     * @param nameOfSelfStruct The name of the self struct in cpython.
     */
    public static String generateCPythonReactionLinker(
            ReactorInstance instance,
            ReactionInstance reaction,
            String nameOfSelfStruct
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(generateCPythonFunctionLinker(
            nameOfSelfStruct, generateCPythonReactionFunctionName(reaction.index),
            instance, generatePythonReactionFunctionName(reaction.index))
        );
        if (reaction.getDefinition().getStp() != null) {
            code.pr(generateCPythonFunctionLinker(
                nameOfSelfStruct, generateCPythonSTPFunctionName(reaction.index),
                instance, generatePythonSTPFunctionName(reaction.index))
            );
        }
        if (reaction.getDefinition().getDeadline() != null) {
            code.pr(generateCPythonFunctionLinker(
                nameOfSelfStruct, generateCPythonDeadlineFunctionName(reaction.index),
                instance, generatePythonDeadlineFunctionName(reaction.index))
            );
        }
        return code.toString();
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
            "    \""+pythonFunctionName+"\");",
            "if("+nameOfSelfStruct+"->"+cpythonFunctionName+" == NULL) {",
            "    lf_print_error_and_exit(\"Could not load function "+pythonFunctionName+"\");",
            "}"
        );
    }

    /**
     * Generate the function that is executed whenever the deadline of the reaction
     * with the given reaction index is missed
     * @param reaction The reaction to generate deadline miss code for
     * @param reactionIndex The agreed-upon index of the reaction in the reactor (should match the C generated code)
     * @param reactionParameters The parameters to the deadline violation function, which are the same as the reaction function
     */
    public static String generatePythonFunction(String pythonFunctionName, String inits, String reactionBody, List<String> reactionParameters) {
        String params = reactionParameters.size() > 0 ? ", " + String.join(", ", reactionParameters) : "";
        CodeBuilder code = new CodeBuilder();
        code.pr("def "+pythonFunctionName+"(self"+params+"):");
        code.indent();
        code.pr(inits);
        code.pr(reactionBody);
        code.pr("return 0");
        return code.toString();
    }


    /**
     * Generate the Python code for reactions in reactor
     * @param reactor The reactor
     * @param reactions The reactions of reactor
     */
    public static String generatePythonReactions(Reactor reactor, List<Reaction> reactions) {
        CodeBuilder code = new CodeBuilder();
        int reactionIndex = 0;
        for (Reaction reaction : reactions) {
            code.pr(generatePythonReaction(reactor, reaction, reactionIndex));
            reactionIndex++;
        }
        return code.toString();
    }

    /**
     * Generate the Python code for reaction in reactor
     * @param reactor The reactor
     * @param reaction The reaction of reactor
     */
    public static String generatePythonReaction(Reactor reactor, Reaction reaction, int reactionIndex) {
        // Reactions marked with a `@_c_body` attribute are generated in C
        if (AttributeUtils.hasCBody(reaction))  return "";

        CodeBuilder code = new CodeBuilder();
        List<String> reactionParameters = new ArrayList<>(); // Will contain parameters for the function (e.g., Foo(x,y,z,...)
        CodeBuilder inits = new CodeBuilder(); // Will contain initialization code for some parameters
        PythonReactionGenerator.generatePythonReactionParametersAndInitializations(reactionParameters, inits, reactor, reaction);
        code.pr(generatePythonFunction(
            generatePythonReactionFunctionName(reactionIndex),
            inits.toString(),
            ASTUtils.toText(reaction.getCode()),
            reactionParameters
        ));
        // Generate code for the STP violation handler function, if there is one.
        if (reaction.getStp() != null) {
            code.pr(generatePythonFunction(
                generatePythonSTPFunctionName(reactionIndex),
                "",
                ASTUtils.toText(reaction.getStp().getCode()),
                reactionParameters
            ));
        }
        // Generate code for the deadline violation function, if there is one.
        if (reaction.getDeadline() != null) {
            code.pr(generatePythonFunction(
                generatePythonDeadlineFunctionName(reactionIndex),
                "",
                ASTUtils.toText(reaction.getDeadline().getCode()),
                reactionParameters
            ));
        }
        return code.toString();
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
        return "_lf_py_deadline_function_"+reactionIndex;
    }

    /** Return the function name of the STP violation handler function inside the self struct in the .c file.
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generateCPythonSTPFunctionName(int reactionIndex) {
        return "_lf_py_STP_function_"+reactionIndex;
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

    /** Return the function name of the STP violation handler function in the .py file.
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generatePythonSTPFunctionName(int reactionIndex) {
        return "STP_function_" + reactionIndex;
    }
}
