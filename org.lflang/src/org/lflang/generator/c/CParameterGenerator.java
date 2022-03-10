package org.lflang.generator.c;

import java.util.LinkedList;
import java.util.List;
import org.lflang.generator.ParameterInstance;
import org.lflang.ASTUtils;
import org.lflang.generator.GeneratorBase;
import org.lflang.lf.Assignment;
import org.lflang.lf.Expression;
import org.lflang.lf.ParameterReference;

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
            for (Expression expr: lastAssignment.getRhs()) {
                if (expr instanceof ParameterReference) {
                    // The parameter is being assigned a parameter value.
                    // Assume that parameter belongs to the parent's parent.
                    // This should have been checked by the validator.
                    final var param = ((ParameterReference) expr).getParameter();
                    list.add(CUtil.reactorRef(p.getParent().getParent()) + "->" + param.getName());
                } else {
                    list.add(GeneratorBase.getTargetTime(expr));
                }
            }
        } else {
            // there was no assignment in the instantiation. So just use the
            // parameter's initial value.
            for (Expression expr : p.getParent().initialParameterValue(p.getDefinition())) {
                if (ASTUtils.isOfTimeType(p.getDefinition())) {
                    list.add(GeneratorBase.getTargetTime(expr));
                } else {
                    list.add(GeneratorBase.getTargetTime(expr));
                }
            }
        } 
        if (list.size() == 1) {
            return list.get(0);
        } else {
            return "{" + String.join(", ", list) + "}";
        }
    }
}
