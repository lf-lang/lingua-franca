package org.lflang.generator.c;

import static org.lflang.ASTUtils.allMethods;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Method;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;

/**
 * Collection of functions to generate C code to declare methods.
 *
 * @author Edward A. Lee
 */
public class CMethodGenerator {

    /**
     * Generate macro definitions for methods.
     * @param reactor The reactor.
     * @param body The place to put the macro definitions.
     */
    public static void generateMacrosForMethods(
        Reactor reactor,
        CodeBuilder body
    ) {
        for (Method method : allMethods(reactor)) {
            var functionName = methodFunctionName(reactor, method);
            body.pr("#define "+method.getName()+"(...) "+functionName+"(self, ##__VA_ARGS__)");
        }
    }

    /**
     * Generate macro undefinitions for methods.
     * @param reactor The reactor.
     * @param body The place to put the macro definitions.
     */
    public static void generateMacroUndefsForMethods(
        Reactor reactor,
        CodeBuilder body
    ) {
        for (Method method : allMethods(reactor)) {
            body.pr("#undef "+method.getName());
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

        code.prSourceLineNumber(method);
        code.prComment("Implementation of method "+method.getName()+"()");
        code.pr(generateMethodSignature(method, decl, types) + " {");
        code.indent();

        // Define the "self" struct.
        String structType = CUtil.selfType(decl);
        // A null structType means there are no inputs, state,
        // or anything else. No need to declare it.
        if (structType != null) {
             code.pr(String.join("\n",
                 structType+"* self = ("+structType+"*)instance_args;"
                         + " SUPPRESS_UNUSED_WARNING(self);"
             ));
        }

        code.prSourceLineNumber(method.getCode());
        code.pr(body);
        code.unindent();
        code.pr("}");
        return code.toString();
    }

    /**
     * Generate method functions definition for a reactor.
     * These functions have a first argument that is a void* pointing to
     * the self struct.
     * @param decl The reactor.
     * @param code The place to put the code.
     * @param types The C-specific type conversion functions.
     */
    public static void generateMethods(
            ReactorDecl decl,
            CodeBuilder code,
            CTypes types
    ) {
        var reactor = ASTUtils.toDefinition(decl);
        code.prComment("***** Start of method declarations.");
        signatures(decl, code, types);
        generateMacrosForMethods(reactor, code);
        for (Method method : allMethods(reactor)) {
            code.pr(CMethodGenerator.generateMethod(method, decl, types));
        }
        generateMacroUndefsForMethods(reactor, code);
        code.prComment("***** End of method declarations.");
    }

    /**
     * Generate function signatures for methods.
     * This can be used to declare all the methods with signatures only
     * before giving the full definition so that methods may call each other
     * (and themselves) regardless of the order of definition.
     * @param decl The reactor declaration.
     * @param types The C-specific type conversion functions.
     */
    public static void signatures(
        ReactorDecl decl,
        CodeBuilder body,
        CTypes types
    ) {
        Reactor reactor = ASTUtils.toDefinition(decl);
        for (Method method : allMethods(reactor)) {
            body.pr(generateMethodSignature(method, decl, types) + ";");
        }
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

    /**
     * Generate a method function signature for a reactor.
     * This function will have a first argument that is a void* pointing to
     * the self struct, followed by any arguments given in its definition.
     * @param method The method.
     * @param decl The reactor declaration.
     * @param types The C-specific type conversion functions.
     */
    public static String generateMethodSignature(
        Method method,
        ReactorDecl decl,
        CTypes types
    ) {
        var functionName = methodFunctionName(decl, method);

        StringBuilder result = new StringBuilder();
        if (method.getReturn() != null) {
            result.append(types.getTargetType(InferredType.fromAST(method.getReturn())));
            result.append(" ");
        } else {
            result.append("void ");
        }
        result.append(functionName);
        result.append("(void* instance_args");
        if (method.getArguments() != null) {
            for (var arg : method.getArguments()) {
                result.append(", ");
                result.append(types.getTargetType(InferredType.fromAST(arg.getType())));
                result.append(" ");
                result.append(arg.getName());
            }
        }
        result.append(")");
        return result.toString();
    }
}
