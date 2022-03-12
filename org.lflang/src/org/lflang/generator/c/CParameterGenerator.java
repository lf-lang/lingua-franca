package org.lflang.generator.c;

import java.util.LinkedList;
import java.util.List;
import org.lflang.generator.ParameterInstance;
import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.lf.Assignment;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reactor;
import org.lflang.lf.Value;

/**
 * Generates C code to declare and initialize parameters.
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
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
        
        // Handle overrides in the intantiation.
        // In case there is more than one assignment to this parameter, we need to
        // find the last one.
        Assignment lastAssignment = null;
        for (Assignment assignment: p.getParent().getDefinition().getParameters()) {
            if (assignment.getLhs() == p.getDefinition()) {
                lastAssignment = assignment;
            }
        }
        List<String> list = new LinkedList<>();
        if (lastAssignment != null) {
            // The parameter has an assignment.
            // Right hand side can be a list. Collect the entries.
            for (Value value: lastAssignment.getRhs()) {
                if (value.getParameter() != null) {
                    // The parameter is being assigned a parameter value.
                    // Assume that parameter belongs to the parent's parent.
                    // This should have been checked by the validator.
                    list.add(CUtil.reactorRef(p.getParent().getParent()) + "->" + value.getParameter().getName());
                } else {
                    list.add(GeneratorBase.getTargetTime(value));
                }
            }
        } else {
            // there was no assignment in the instantiation. So just use the
            // parameter's initial value.
            for (Value i : p.getParent().initialParameterValue(p.getDefinition())) {
                if (ASTUtils.isOfTimeType(p.getDefinition())) {
                    list.add(GeneratorBase.getTargetTime(i));
                } else {
                    list.add(GeneratorBase.getTargetTime(i));
                }
            }
        } 
        if (list.size() == 1) {
            return list.get(0);
        } else {
            return "{" + String.join(", ", list) + "}";
        }
    }

    /**
     * Generate code for parameters variables of a reactor in the form "parameter.type parameter.name;"
     * @param reactor The reactor
     * @param types A helper class for types
     * @return 
     */
    public static String generateDeclarations(Reactor reactor, CTypes types) {
        CodeBuilder code = new CodeBuilder();
        for (Parameter parameter : ASTUtils.allParameters(reactor)) {
            code.prSourceLineNumber(parameter);
            code.pr(types.getTargetType(parameter) + " " + parameter.getName() + ";");
        }
        return code.toString();
    }
}
