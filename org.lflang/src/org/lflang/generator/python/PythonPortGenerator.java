package org.lflang.generator.python;

import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Action;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.JavaAstUtils;
import org.lflang.generator.CodeBuilder;

public class PythonPortGenerator {
    /**
     * Generate code to convert C actions to Python action capsules
     * @see pythontarget.h.
     * @param pyObjectDescriptor A string representing a list of Python format types (e.g., "O") that 
     *  can be passed to Py_BuildValue. The object type for the converted action will
     *  be appended to this string (e.g., "OO").
     * @param pyObjects A string containing a list of comma-separated expressions that will create the
     *  action capsules.
     * @param action The action itself.
     * @param decl The reactor decl that contains the action.
     */
    public static void generateActionVariableToSendToPythonReaction(StringBuilder pyObjectDescriptor, StringBuilder pyObjects,
        Action action, ReactorDecl decl) {
        pyObjectDescriptor.append("O");
        // Values passed to an action are always stored in the token->value.
        // However, sometimes token might not be initialized. Therefore, this function has an internal check for NULL in case token is not initialized.
        pyObjects.append(String.format(", convert_C_action_to_py(%s)", action.getName()));
    }

    /** 
     * Generate code to convert C ports to Python ports capsules (@see pythontarget.h).
     * 
     * The port may be an input of the reactor or an output of a contained reactor.
     * 
     * @param pyObjectDescriptor A string representing a list of Python format types (e.g., "O") that 
     *  can be passed to Py_BuildValue. The object type for the converted port will
     *  be appended to this string (e.g., "OO").
     * @param pyObjects A string containing a list of comma-separated expressions that will create the
     *  port capsules.
     * @param port The port itself.
     * @param decl The reactor decl that contains the port.
     */
    public static void generatePortVariablesToSendToPythonReaction(
        CodeBuilder code,
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        VarRef port,
        ReactorDecl decl
    ) {
        if (port.getVariable() instanceof Input) {
            generateInputVariablesToSendToPythonReaction(pyObjectDescriptor, pyObjects, (Input) port.getVariable(), decl);
        } else {
            pyObjectDescriptor.append("O");
            Output output = (Output) port.getVariable();
            String reactorName = port.getContainer().getName();
            // port is an output of a contained reactor.
            if (port.getContainer().getWidthSpec() != null) {
                String widthSpec = "-2";
                if (JavaAstUtils.isMultiport((Port) port.getVariable())) {
                    widthSpec = String.format("self->_lf_%s[i].%s_width", reactorName, output.getName());
                }
                // Output is in a bank.
                // Create a Python list
                code.pr(generatePythonListForContainedBank(reactorName, output, widthSpec));
                pyObjects.append(String.format(", %s_py_list", reactorName));
            } else {
                String widthSpec = "-2";
                if (JavaAstUtils.isMultiport((Port) port.getVariable())) {
                    widthSpec = String.format("%s.%s_width", port.getContainer().getName(), port.getVariable().getName());
                }
                pyObjects.append(String.format(", convert_C_port_to_py(%s.%s, %s)", reactorName, port.getVariable().getName(), widthSpec));
            }
        }
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
            "PyObject* "+reactorName+"_py_list = PyList_New("+reactorName+"_width);",
            "if("+reactorName+"_py_list == NULL) {",
            "    error_print(\"Could not create the list needed for "+reactorName+".\");",
            "    if (PyErr_Occurred()) {",
            "        PyErr_PrintEx(0);",
            "        PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "    }",
            "    /* Release the thread. No Python API allowed beyond this point. */",
            "    PyGILState_Release(gstate);",
            "    Py_FinalizeEx();",
            "    exit(1);",
            "}",
            "for (int i = 0; i < "+reactorName+"_width; i++) {",
            "    if (PyList_SetItem(",
            "            "+reactorName+"_py_list,",
            "            i,",
            "            convert_C_port_to_py(",
            "                self->_lf_"+reactorName+"[i]."+port.getName()+", ",
            "                "+widthSpec+"",
            "            )",
            "        ) != 0) {",
            "        error_print(\"Could not add elements to the list for "+reactorName+".\");",
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

    /** Generate into the specified string builder the code to
     *  send local variables for output ports to a Python reaction function
     *  from the "self" struct.
     *  @param builder The string builder into which to write the code.
     *  @param structs A map from reactor instantiations to a place to write
     *   struct fields.
     *  @param output The output port.
     *  @param decl The reactor declaration.
     */
    public static void generateOutputVariablesToSendToPythonReaction(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Output output,
        ReactorDecl decl
    ) {
        // Unfortunately, for the SET macros to work out-of-the-box for
        // multiports, we need an array of *pointers* to the output structs,
        // but what we have on the self struct is an array of output structs.
        // So we have to handle multiports specially here a construct that
        // array of pointers.
        // FIXME: The C Generator also has this awkwardness. It makes the code generators
        // unnecessarily difficult to maintain, and it may have performance consequences as well.
        // Maybe we should change the SET macros.
        if (!JavaAstUtils.isMultiport(output)) {
            pyObjectDescriptor.append("O");
            pyObjects.append(", convert_C_port_to_py("+output.getName()+", -2)");
        } else {
            // Set the _width variable.                
            pyObjectDescriptor.append("O");
            pyObjects.append(", convert_C_port_to_py("+output.getName()+","+output.getName()+"_width) ");
        }
    }

    /** Generate into the specified string builder the code to
     *  pass local variables for sending data to an input
     *  of a contained reaction (e.g. for a deadline violation).
     *  @param builder The string builder.
     *  @param definition AST node defining the reactor within which this occurs
     *  @param input Input of the contained reactor.
     */
    public static void generateVariablesForSendingToContainedReactors(
        CodeBuilder code,
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Instantiation definition,
        Input input,
        ReactorDecl decl
    ) {
        pyObjectDescriptor.append("O");
        
        if (definition.getWidthSpec() != null) {
            String widthSpec = "-2";
            if (JavaAstUtils.isMultiport(input)) {
                widthSpec = "self->_lf_"+definition.getName()+"[i]."+input.getName()+"_width";
            }
            // Contained reactor is a bank.
            // Create a Python list
            code.pr(generatePythonListForContainedBank(definition.getName(), input, widthSpec));
            pyObjects.append(", "+definition.getName()+"_py_list");
        }
        else {
            String widthSpec = "-2";
            if (JavaAstUtils.isMultiport(input)) {
                widthSpec = ""+definition.getName()+"."+input.getName()+"_width";
            }
            pyObjects.
                append(", convert_C_port_to_py("+definition.getName()+"."+input.getName()+", "+widthSpec+")");
        }
    }

    /** Generate into the specified string builder the code to
     *  send local variables for input ports to a Python reaction function
     *  from the "self" struct.
     *  @param builder The string builder into which to write the code.
     *  @param structs A map from reactor instantiations to a place to write
     *   struct fields.
     *  @param input The input port.
     *  @param reactor The reactor.
     */
    public static void generateInputVariablesToSendToPythonReaction(
        StringBuilder pyObjectDescriptor,
        StringBuilder pyObjects,
        Input input,
        ReactorDecl decl
    ) {
        // Create the local variable whose name matches the input.getName().
        // If the input has not been declared mutable, then this is a pointer
        // to the upstream output. Otherwise, it is a copy of the upstream output,
        // which nevertheless points to the same token and value (hence, as done
        // below, we have to use writable_copy()). There are 8 cases,
        // depending on whether the input is mutable, whether it is a multiport,
        // and whether it is a token type.
        // Easy case first.
        if (!input.isMutable() && !JavaAstUtils.isMultiport(input)) {
            // Non-mutable, non-multiport, primitive type.
            pyObjectDescriptor.append("O");
            pyObjects.append(String.format(", convert_C_port_to_py(%s, %s_width)", input.getName(), input.getName()));
        } else if (input.isMutable() && !JavaAstUtils.isMultiport(input)) {
            // Mutable, non-multiport, primitive type.
            // TODO: handle mutable
            pyObjectDescriptor.append("O");
            pyObjects.append(String.format(", convert_C_port_to_py(%s, %s_width)", input.getName(), input.getName()));
        } else if (!input.isMutable() && JavaAstUtils.isMultiport(input)) {
            // Non-mutable, multiport, primitive.
            // TODO: support multiports
            pyObjectDescriptor.append("O");
            pyObjects.append(String.format(", convert_C_port_to_py(%s, %s_width)", input.getName(), input.getName()));
        } else {
            // Mutable, multiport, primitive type
            // TODO: support mutable multiports
            pyObjectDescriptor.append("O");
            pyObjects.append(String.format(", convert_C_port_to_py(%s, %s_width)", input.getName(), input.getName()));
        }
    }

    /**
     * Generate into the specified string builder (<code>inits<code>) the code to
     * initialize local variable for <code>port<code> so that it can be used in the body of
     * the Python reaction.
     * @param port The port to generate code for.
     * @param inits The generated code will be put in <code>inits<code>.
     */
    public CodeBuilder generatePythonPortVariableInReaction(VarRef port, CodeBuilder inits) {
        String containerName = port.getContainer().getName();
        String variableName = port.getVariable().getName();
        if (port.getContainer().getWidthSpec() != null) {
            // It's a bank
            inits.pr(String.join("\n", 
                ""+containerName+" = [None] * len("+containerName+"_"+variableName+")",
                "for i in range(len("+containerName+"_"+variableName+")):",
                "    "+containerName+"[i] = Make()",
                "    "+containerName+"[i]."+variableName+" = "+containerName+"_"+variableName+"[i]"
            ));
        } else {
            inits.pr(""+containerName+" = Make");
            inits.pr(""+containerName+"."+variableName+" = "+containerName+"_"+variableName+"");
        }
        return inits;
    }
}
