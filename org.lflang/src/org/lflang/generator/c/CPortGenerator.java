package org.lflang.generator.c;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.Target;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.PortInstance;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import static org.lflang.generator.c.CGenerator.variableStructType;


/**
 * Generates C code to declare and initialize ports.
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CPortGenerator {
    /**
     * Generate fields in the self struct for input and output ports
     * 
     * @param reactor
     * @param decl
     * @param body
     * @param constructorCode
     */
    public static void generateDeclarations(
        Reactor reactor,
        ReactorDecl decl,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        generateInputDeclarations(reactor, decl, body, constructorCode);
        generateOutputDeclarations(reactor, decl, body, constructorCode);
    }

    /**
     * Generate the struct type definitions for the port of the 
     * reactor
     * 
     * @param decl The reactor declaration
     * @param port The port to generate the struct
     * @param target The target of the code generation (C, CCpp or Python)
     * @param errorReporter The error reporter
     * @param types The helper object for types related stuff
     * @param federatedExtension The code needed to support federated execution
     * @return The auxiliary struct for the port as a string
     */
    public static String generateAuxiliaryStruct(
        ReactorDecl decl,
        Port port,
        Target target,
        ErrorReporter errorReporter,
        CTypes types,
        CodeBuilder federatedExtension
    ) {
        var code = new CodeBuilder();
        code.pr("typedef struct {");
        code.indent();
        code.pr(valueDeclaration(port, target, errorReporter, types));
        code.pr(String.join("\n", 
                    "bool is_present;",
                    "int num_destinations;",
                    (CUtil.isTokenType(ASTUtils.getInferredType(port), types) ? 
                    String.join("\n",
                    "lf_token_t* token;",
                    "int length;"
                    ) : 
                    ""),
                    federatedExtension.toString()
        ));
        code.unindent();
        code.pr("} "+variableStructType(port, decl)+";");
        return code.toString();
    }

    /**
     * Allocate memory for the input port.
     * @param input The input port
     * @param reactorSelfStruct The name of the self struct
     */
    public static String initializeInputMultiport(
        PortInstance input, 
        String reactorSelfStruct
    ) {
        var portRefName = CUtil.portRefName(input);
        // If the port is a multiport, create an array.
        return input.isMultiport() ? 
                String.join("\n", 
                    portRefName+"_width = "+input.getWidth()+";",
                    "// Allocate memory for multiport inputs.",
                    portRefName+" = ("+variableStructType(input)+"**)_lf_allocate(",
                    "        "+input.getWidth()+", sizeof("+variableStructType(input)+"*),",
                    "        &"+reactorSelfStruct+"->base.allocations); ",
                    "// Set inputs by default to an always absent default input.",
                    "for (int i = 0; i < "+input.getWidth()+"; i++) {",
                    "    "+portRefName+"[i] = &"+reactorSelfStruct+"->_lf_default__"+input.getName()+";",
                    "}"
                ) :
                String.join("\n",
                    "// width of -2 indicates that it is not a multiport.",
                    portRefName+"_width = -2;"
                );
    }

    /**
     * Allocate memory for the output port.
     * @param output The output port
     * @param reactorSelfStruct The name of the self struct
     */
    public static String initializeOutputMultiport(
        PortInstance output,
        String reactorSelfStruct
    ) {
        var portRefName = CUtil.portRefName(output);
        var portStructType = variableStructType(output);
        return output.isMultiport() ?
                String.join("\n", 
                    portRefName+"_width = "+output.getWidth()+";",
                    "// Allocate memory for multiport output.",
                    portRefName+" = ("+portStructType+"*)_lf_allocate(",
                    "        "+output.getWidth()+", sizeof("+portStructType+"),",
                    "        &"+reactorSelfStruct+"->base.allocations); ",
                    portRefName+"_pointers = ("+portStructType+"**)_lf_allocate(",
                    "        "+output.getWidth()+", sizeof("+portStructType+"*),",
                    "        &"+reactorSelfStruct+"->base.allocations); ",
                    "// Assign each output port pointer to be used in",
                    "// reactions to facilitate user access to output ports",
                    "for(int i=0; i < "+output.getWidth()+"; i++) {",
                    "        "+portRefName+"_pointers[i] = &("+portRefName+"[i]);",
                    "}"
                ) :
                String.join("\n",
                    "// width of -2 indicates that it is not a multiport.",
                    portRefName+"_width = -2;"
                );
    }

     /** 
     * Generate code to set up the tables used in _lf_start_time_step for input ports.
     */
    public static String initializeStartTimeStepTableForInput(
        PortInstance input
    ) {
        var portRef = CUtil.portRefName(input);
        return input.isMultiport() ? 
                String.join("\n", 
                    "for (int i = 0; i < "+input.getWidth()+"; i++) {",
                    "    _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].token",
                    "            = &"+portRef+"[i]->token;",
                    "    _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].status",
                    "            = (port_status_t*)&"+portRef+"[i]->is_present;",
                    "    _lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count++].reset_is_present = false;",
                    "};"
                ) :
                initializeStartTimeStepTableForPort(portRef);
    }

    public static String initializeStartTimeStepTableForPort(
        String portRef
    ) {
        return String.join("\n", 
            "_lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].token = &"+portRef+"->token;",
            "_lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count].status = (port_status_t*)&"+portRef+"->is_present;",
            "_lf_tokens_with_ref_count[_lf_tokens_with_ref_count_count++].reset_is_present = false;"
        );
    }
 
    /**
     * For the specified port, return a declaration for port struct to
     * contain the value of the port. A multiport output with width 4 and
     * type int[10], for example, will result in this:
     * ```
     *     int value[10];
     * ```
     * There will be an array of size 4 of structs, each containing this value 
     * array.
     * @param port The port.
     * @return A string providing the value field of the port struct.
     */
    private static String valueDeclaration(
        Port port,
        Target target,
        ErrorReporter errorReporter,
        CTypes types
    ) {
        if (port.getType() == null && target.requiresTypes == true) {
            // This should have been caught by the validator.
            errorReporter.reportError(port, "Port is required to have a type: " + port.getName());
            return "";
        }
        // Do not convert to lf_token_t* using lfTypeToTokenType because there
        // will be a separate field pointing to the token.
        return types.getVariableDeclaration(ASTUtils.getInferredType(port), "value", false) + ";";
    }

    /**
     * Generate fields in the self struct for input ports
     * 
     * If the port is a multiport, the input field is an array of
     * pointers that will be allocated separately for each instance
     * because the sizes may be different. Otherwise, it is a simple
     * pointer.
     * 
     */
    private static void generateInputDeclarations(
        Reactor reactor,
        ReactorDecl decl,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        for (Input input : ASTUtils.allInputs(reactor)) {
            var inputName = input.getName();
            if (ASTUtils.isMultiport(input)) {                
                body.pr(input, String.join("\n", 
                    "// Multiport input array will be malloc'd later.",
                    variableStructType(input, decl)+"** _lf_"+inputName+";",
                    "int _lf_"+inputName+"_width;",
                    "// Default input (in case it does not get connected)",
                    variableStructType(input, decl)+" _lf_default__"+inputName+";"
                ));
            } else {
                // input is not a multiport.
                body.pr(input, String.join("\n", 
                    variableStructType(input, decl)+"* _lf_"+inputName+";",
                    "// width of -2 indicates that it is not a multiport.",
                    "int _lf_"+inputName+"_width;",
                    "// Default input (in case it does not get connected)",
                    variableStructType(input, decl)+" _lf_default__"+inputName+";"
                ));

                constructorCode.pr(input, String.join("\n", 
                    "// Set input by default to an always absent default input.",
                    "self->_lf_"+inputName+" = &self->_lf_default__"+inputName+";"
                ));
            }
        }
    }

    /**
     * Generate fields in the self struct for output ports
     * 
     * @param reactor
     * @param decl
     * @param body
     * @param constructorCode
     */
    private static void generateOutputDeclarations(
        Reactor reactor,
        ReactorDecl decl,
        CodeBuilder body,
        CodeBuilder constructorCode
    ) {
        for (Output output : ASTUtils.allOutputs(reactor)) {
            // If the port is a multiport, create an array to be allocated
            // at instantiation.
            var outputName = output.getName();
            if (ASTUtils.isMultiport(output)) {
                body.pr(output, String.join("\n", 
                    "// Array of output ports.",
                    variableStructType(output, decl)+"* _lf_"+outputName+";",
                    "int _lf_"+outputName+"_width;",
                    "// An array of pointers to the individual ports. Useful",
                    "// for the SET macros to work out-of-the-box for",
                    "// multiports in the body of reactions because their ",
                    "// value can be accessed via a -> operator (e.g.,foo[i]->value).",
                    "// So we have to handle multiports specially here a construct that",
                    "// array of pointers.",
                    variableStructType(output, decl)+"** _lf_"+outputName+"_pointers;"
                ));
            } else {
                body.pr(output, String.join("\n", 
                    variableStructType(output, decl)+" _lf_"+outputName+";",
                    "int _lf_"+outputName+"_width;"
                ));
            }
        }
    }
}
