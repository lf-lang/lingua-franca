package org.lflang.generator.c;

import java.util.List;
import java.util.stream.Collectors;

import org.lflang.InferredType;
import org.lflang.generator.ParameterInstance;
import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Assignment;
import org.lflang.lf.Expression;
import org.lflang.lf.Initializer;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reactor;

/**
 * Generates C code to declare and initialize parameters.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng Wong
 */
public class CParameterGenerator {
    /**
     * Return a C expression that can be used to initialize the specified
     * parameter instance. If the parameter initializer refers to other
     * parameters, then those parameter references are replaced with
     * accesses to the self struct of the parents of those parameters.
     */
    public static String getInitializer(ParameterInstance p) {
        // Handle the bank_index parameter.
        if (p.getName().equals("bank_index")) {
            return CUtil.bankIndex(p.getParent());
        }

        CTypes ctypes = CTypes.generateParametersIn(p.getParent().getParent());
        Initializer values = p.getActualValue();
        return ctypes.getTargetInitializer(values, p.getDefinition().getType());
    }

    /**
     * Generate code for parameters variables of a reactor in the form "parameter.type parameter.name;"
     * @param reactor {@link TypeParameterizedReactor}
     * @param types A helper class for types
     */
    public static String generateDeclarations(TypeParameterizedReactor reactor, CTypes types) {
        CodeBuilder code = new CodeBuilder();
        for (Parameter parameter : ASTUtils.allParameters(reactor.r())) {
            code.prSourceLineNumber(parameter);
            code.pr(types.getTargetType(reactor.resolveType(ASTUtils.getInferredType(parameter))) + " " + parameter.getName() + ";");
        }
        return code.toString();
    }
}
