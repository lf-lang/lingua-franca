package org.lflang.generator.python;

import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.lf.Method;
import org.lflang.lf.MethodArgument;
import org.lflang.lf.Reactor;

/**
 * Collection of functions to generate Python code to declare methods.
 *
 * @author Soroush Bateni
 */
public class PythonMethodGenerator {

    /**
     * Generate a Python method definition for {@code method}.
     */
    public static String generateMethod(Method method) {
        return String.join("\n",
                           "# Implementation of method "+method.getName()+"().",
                           "def "+method.getName()+"(self, "+generateMethodArgumentList(method)+"):",
                           ASTUtils.toText(method.getCode()).indent(4)
        );
    }

    /**
     * Generate methods for a reactor class.
     *
     * @param reactor The reactor.
     */
    public static String generateMethods(
        Reactor reactor
    ) {
        return ASTUtils.allMethods(reactor)
                       .stream()
                       .map(m -> generateMethod(m))
                       .collect(Collectors.joining());
    }

    /**
     * Generate a list of arguments for {@code method} delimited with ', '.
     */
    private static String generateMethodArgumentList(Method method) {
        return String.join(", ",
                           method.getArguments()
                                 .stream()
                                 .map(MethodArgument::getName)
                                 .collect(Collectors.toList()));
    }
}
