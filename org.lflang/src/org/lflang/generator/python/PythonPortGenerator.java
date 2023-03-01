package org.lflang.generator.python;

import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Action;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.c.CGenerator;
import static org.lflang.generator.c.CUtil.generateWidthVariable;

public class PythonPortGenerator {
    public static final String NONMULTIPORT_WIDTHSPEC = "-2";

    /**
     * Generate code to convert C actions to Python action capsules. See
     * pythontarget.h for details.
     * @param pyObjects A string containing a list of comma-separated expressions that will create the
     *  action capsules.
     * @param action The action itself.
     * @param decl The reactor decl that contains the action.
     */
    public static void generateActionVariableToSendToPythonReaction(List<String> pyObjects,
        Action action, ReactorDecl decl) {
        // Values passed to an action are always stored in the token->value.
        // However, sometimes token might not be initialized. Therefore, this function has an internal check for NULL in case token is not initialized.
        pyObjects.add(String.format("convert_C_action_to_py(%s)", action.getName()));
    }

    /**
     * Generate code to convert C ports to Python ports capsules (@see pythontarget.h).
     *
     * The port may be an input of the reactor or an output of a contained reactor.
     * @param pyObjects A string containing a list of comma-separated expressions that will create the
     *  port capsules.
     * @param port The port itself.
     * @param decl The reactor decl that contains the port.
     */
    public static String generatePortVariablesToSendToPythonReaction(
        List<String> pyObjects,
        VarRef port,
        ReactorDecl decl
    ) {
        if (port.getVariable() instanceof Input) {
            generateInputVariablesToSendToPythonReaction(pyObjects, (Input) port.getVariable(), decl);
            return "";
        } else {
            // port is an output of a contained reactor.
            return generateVariablesForSendingToContainedReactors(pyObjects, port.getContainer(), (Port) port.getVariable());
        }
    }

    /** Generate into the specified string builder the code to
     *  send local variables for output ports to a Python reaction function
     *  from the "self" struct.
     *  @param output The output port.
     */
    public static void generateOutputVariablesToSendToPythonReaction(
        List<String> pyObjects,
        Output output
    ) {
        // Unfortunately, for the lf_set macros to work out-of-the-box for
        // multiports, we need an array of *pointers* to the output structs,
        // but what we have on the self struct is an array of output structs.
        // So we have to handle multiports specially here a construct that
        // array of pointers.
        // FIXME: The C Generator also has this awkwardness. It makes the code generators
        // unnecessarily difficult to maintain, and it may have performance consequences as well.
        // Maybe we should change the lf_set macros.
        if (!ASTUtils.isMultiport(output)) {
            pyObjects.add(generateConvertCPortToPy(output.getName(), NONMULTIPORT_WIDTHSPEC));
        } else {
            pyObjects.add(generateConvertCPortToPy(output.getName()));
        }
    }

    /** Generate into the specified string builder the code to
     *  send local variables for input ports to a Python reaction function
     *  from the "self" struct.
     *  @param input The input port.
     */
    public static void generateInputVariablesToSendToPythonReaction(
        List<String> pyObjects,
        Input input,
        ReactorDecl decl
    ) {
        // Create the local variable whose name matches the input.getName().
        // If the input has not been declared mutable, then this is a pointer
        // to the upstream output. Otherwise, it is a copy of the upstream output,
        // which nevertheless points to the same token and value. There are 8 cases,
        // depending on whether the input is mutable, whether it is a multiport,
        // and whether it is a token type.
        // Easy case first.
        if (!input.isMutable() && !ASTUtils.isMultiport(input)) {
            // Non-mutable, non-multiport, primitive type.
            pyObjects.add(generateConvertCPortToPy(input.getName()));
        } else if (input.isMutable() && !ASTUtils.isMultiport(input)) {
            // Mutable, non-multiport, primitive type.
            // TODO: handle mutable
            pyObjects.add(generateConvertCPortToPy(input.getName()));
        } else if (!input.isMutable() && ASTUtils.isMultiport(input)) {
            // Non-mutable, multiport, primitive.
            // TODO: support multiports
            pyObjects.add(generateConvertCPortToPy(input.getName()));
        } else {
            // Mutable, multiport, primitive type
            // TODO: support mutable multiports
            pyObjects.add(generateConvertCPortToPy(input.getName()));
        }
    }

    /** Generate into the specified string builder the code to
     *  pass local variables for sending data to an input
     *  of a contained reaction (e.g. for a deadline violation).
     *  @param definition AST node defining the reactor within which this occurs
     *  @param port Input of the contained reactor.
     */
    public static String generateVariablesForSendingToContainedReactors(
        List<String> pyObjects,
        Instantiation definition,
        Port port
    ) {
        CodeBuilder code = new CodeBuilder();
        if (definition.getWidthSpec() != null) {
            String widthSpec = NONMULTIPORT_WIDTHSPEC;
            if (ASTUtils.isMultiport(port)) {
                widthSpec = "self->_lf_"+definition.getName()+"[i]."+generateWidthVariable(port.getName());
            }
            // Contained reactor is a bank.
            // Create a Python list
            code.pr(generatePythonListForContainedBank(definition.getName(), port, widthSpec));
            pyObjects.add(definition.getName()+"_py_list");
        }
        else {
            if (ASTUtils.isMultiport(port)) {
                pyObjects.add(generateConvertCPortToPy(definition.getName()+"."+port.getName()));
            } else {
                pyObjects.add(generateConvertCPortToPy(definition.getName()+"."+port.getName(), NONMULTIPORT_WIDTHSPEC));
            }
        }
        return code.toString();
    }


    /**
     * Generate code that creates a Python list (i.e., []) for contained banks to be passed to Python reactions.
     * The Python reaction will then subsequently be able to address each individual bank member of the contained
     * bank using an index or an iterator. Each list member will contain the given <code>port<code>
     * (which could be a multiport with a width determined by <code>widthSpec<code>).
     *
     * This is to accommodate reactions like <code>reaction() -> s.out<code> where s is a bank. In this example,
     * the generated Python function will have the signature <code>reaction_function_0(self, s_out)<code>, where
     * s_out is a list of out ports. This will later be turned into the proper <code>s.out<code> format using the
     * Python code generated in {@link #generatePythonPortVariableInReaction}.
     *
     * @param reactorName The name of the bank of reactors (which is the name of the reactor class).
     * @param port The port that should be put in the Python list.
     * @param widthSpec A string that should be -2 for non-multiports and the width expression for multiports.
     */
    public static String generatePythonListForContainedBank(String reactorName, Port port, String widthSpec) {
        return String.join("\n",
            "PyObject* "+reactorName+"_py_list = PyList_New("+generateWidthVariable(reactorName)+");",
            "if("+reactorName+"_py_list == NULL) {",
            "    lf_print_error(\"Could not create the list needed for "+reactorName+".\");",
            "    if (PyErr_Occurred()) {",
            "        PyErr_PrintEx(0);",
            "        PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "    }",
            "    /* Release the thread. No Python API allowed beyond this point. */",
            "    PyGILState_Release(gstate);",
            "    Py_FinalizeEx();",
            "    exit(1);",
            "}",
            "for (int i = 0; i < "+generateWidthVariable(reactorName)+"; i++) {",
            "    if (PyList_SetItem("+reactorName+"_py_list,",
            "            i,",
            "            "+generateConvertCPortToPy(reactorName + "[i]." + port.getName(), widthSpec),
            "        ) != 0) {",
            "        lf_print_error(\"Could not add elements to the list for "+reactorName+".\");",
            "        if (PyErr_Occurred()) {",
            "            PyErr_PrintEx(0);",
            "            PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "        }",
            "        /* Release the thread. No Python API allowed beyond this point. */",
            "        PyGILState_Release(gstate);",
            "        Py_FinalizeEx();",
            "        exit(1);",
            "    }",
            "}"
        );
    }

    public static String generateAliasTypeDef(ReactorDecl decl, Port port, boolean isTokenType, String genericPortType) {
        return "typedef "+genericPortType+" "+CGenerator.variableStructType(port, decl)+";";
    }

    private static String generateConvertCPortToPy(String port) {
        return String.format("convert_C_port_to_py(%s, %s)", port, generateWidthVariable(port));
    }

    private static String generateConvertCPortToPy(String port, String widthSpec) {
        return String.format("convert_C_port_to_py(%s, %s)", port, widthSpec);
    }

    /**
     * Generate into the specified string builder (<code>inits<code>) the code to
     * initialize local variable for <code>port<code> so that it can be used in the body of
     * the Python reaction.
     * @param port The port to generate code for.
     */
    public static String generatePythonPortVariableInReaction(VarRef port) {
        String containerName = port.getContainer().getName();
        String variableName = port.getVariable().getName();
        String tryStatement = "try: "+containerName+"  # pylint: disable=used-before-assignment";
        if (port.getContainer().getWidthSpec() != null) {
            // It's a bank
            return String.join("\n",
                tryStatement,
                "except NameError: "+containerName+" = [None] * len("+containerName+"_"+variableName+")",
                "for i in range(len("+containerName+"_"+variableName+")):",
                "    if "+containerName+"[i] is None: "+containerName+"[i] = Make()",
                "    "+containerName+"[i]."+variableName+" = "+containerName+"_"+variableName+"[i]"
            );
        } else {
            return String.join("\n",
                tryStatement,
                "except NameError: "+containerName+" = Make()",
                containerName+"."+variableName+" = "+containerName+"_"+variableName
            );
        }
    }
}
