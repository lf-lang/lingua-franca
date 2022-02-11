package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;

public class PythonStateGenerator {
    public static String generatePythonInstantiations(ReactorDecl decl) {
        List<String> lines = new ArrayList<>();
        lines.add("# Define state variables");
        // Next, handle state variables
        for (StateVar stateVar : ASTUtils.allStateVars(ASTUtils.toDefinition(decl))) {
            if (ASTUtils.isInitialized(stateVar)) {
                // If initialized, pass along the initialization directly if it is present
                lines.add("self."+stateVar.getName()+" = "+generatePythonInitializers(stateVar));
            } else {
                // If neither the type nor the initialization is given, use None
                lines.add("self."+stateVar.getName()+" = None");
            }
        }
        
        lines.add("");
        return String.join("\n", lines);
    }

    /**
     * Create a Python list for parameter initialization in target code.
     * 
     * @param p The parameter to create initializers for
     * @return Initialization code
     */
    private static String generatePythonInitializers(StateVar state) {
        if (state.getInit().size() > 1) {
            // state variables are initialized as mutable lists
            List<String> targetValues = state.getInit().stream().map(it -> PyUtil.getPythonTargetValue(it)).collect(Collectors.toList());
            return "[" + String.join("\n", targetValues) + "]";
        } else if (ASTUtils.isInitialized(state)) {
            return PyUtil.getPythonTargetValue(state.getInit().get(0));
        } else {
            return "None";
        }
    }
}
