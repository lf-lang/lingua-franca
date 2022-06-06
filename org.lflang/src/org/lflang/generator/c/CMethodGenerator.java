package org.lflang.generator.c;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Method;
import org.lflang.lf.ReactorDecl;

/**
 * Generates C code to declare and initialize methods.
 *
 * @author {Edward A. Lee <eal@berkeley.edu>}
 */
public class CMethodGenerator {
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
        
        StringBuilder header = new StringBuilder();
        if (method.getReturn() != null) {
            header.append(types.getTargetType(InferredType.fromAST(method.getReturn())));
            header.append(" ");
        } else {
            header.append("void ");
        }
        header.append(method.getName());
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
}