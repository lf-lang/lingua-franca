package org.lflang.generator.c;

import static org.lflang.ASTUtils.allMethods;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Method;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;

/**
 * Generates C code to declare and initialize methods.
 *
 * @author {Edward A. Lee <eal@berkeley.edu>}
 */
public class CMethodGenerator {
    /**
     * Generate fields in the self struct for method functions and
     * initialize them in the constructor.
     *
     * @param reactor The reactor.
     * @param body The place to put the struct declarations.
     * @param constructorCode The place to put the constructor code.
     * @param types The C-specific type conversion functions.
     */
    public static void generateDeclarations(
        Reactor reactor,
        CodeBuilder body,
        CodeBuilder constructorCode,
        CTypes types
    ) {
        for (Method method : allMethods(reactor)) {
            
            var functionName = methodFunctionName(reactor, method);
            
            // Construct function type signature.
            StringBuilder typeSignature = new StringBuilder();
            if (method.getReturn() != null) {
                typeSignature.append(types.getTargetType(InferredType.fromAST(method.getReturn())));
            } else {
                typeSignature.append("void");
            }
            typeSignature.append("(*");
            typeSignature.append(method.getName());
            typeSignature.append(")(void*");
            if (method.getArguments() != null) {
                for (var arg : method.getArguments()) {
                    typeSignature.append(", ");
                    typeSignature.append(types.getTargetType(InferredType.fromAST(arg.getType())));
                }
            }
            typeSignature.append(");");
            body.pr(typeSignature);
            
            constructorCode.pr("self->"+method.getName()+" = "+functionName+";");
        }

    }
    
    /** 
     * Generate a method function definition for a reactor.
     * This function will have a first argument that is a void* pointing to
     * the self struct, followed by any arguments given in its definition.
     * @param method The method.
     * @param decl The reactor declaration.
     * @param types The C-specific type conversion functions.
     */
    public static String generateMethod(
        Method method,
        ReactorDecl decl,
        CTypes types
    ) {
        var code = new CodeBuilder();
        var body = ASTUtils.toText(method.getCode());
        var functionName = methodFunctionName(decl, method);
        
        StringBuilder header = new StringBuilder();
        if (method.getReturn() != null) {
            header.append(types.getTargetType(InferredType.fromAST(method.getReturn())));
            header.append(" ");
        } else {
            header.append("void ");
        }
        header.append(functionName);
        header.append("(void* instance_args");
        if (method.getArguments() != null) {
            for (var arg : method.getArguments()) {
                header.append(", ");
                header.append(types.getTargetType(InferredType.fromAST(arg.getType())));
                header.append(" ");
                header.append(arg.getName());
            }
        }
        header.append(") {");
        
        code.prSourceLineNumber(method);
        code.pr(header);
        code.indent();
        
        // Define the "self" struct.
        String structType = CUtil.selfType(decl);
        // A null structType means there are no inputs, state,
        // or anything else. No need to declare it.
        if (structType != null) {
             code.pr(String.join("\n",
                 "#pragma GCC diagnostic push",
                 "#pragma GCC diagnostic ignored \"-Wunused-variable\"",
                 structType+"* self = ("+structType+"*)instance_args;"
             ));
        }
        
        code.prSourceLineNumber(method.getCode());
        code.pr(body);
        code.unindent();
        code.pr("}");
        return code.toString();
    }

    /**
     * Return the function name for specified method of the specified reactor.
     * @param reactor The reactor
     * @param method The method.
     * @return The function name for the method.
     */
    private static String methodFunctionName(ReactorDecl reactor, Method method) {
        return reactor.getName().toLowerCase() + "_method_" + method.getName();
    }
}