package org.lflang.generator.c;

import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
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
