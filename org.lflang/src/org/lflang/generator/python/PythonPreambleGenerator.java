package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.lf.Preamble;

public class PythonPreambleGenerator {
    /**
     * Generates preambles defined by user for a given reactor.
     * The preamble code is put inside the reactor class.
     */
    public static String generatePythonPreambles(List<Preamble> preambles) {
        List<String> preamblesCode = new ArrayList<>();
        preambles.forEach(p -> preamblesCode.add(
            String.join("\n", 
                "# From the preamble, verbatim:",
                ASTUtils.toText(p.getCode()),
                "# End of preamble."
            )
        ));
        return String.join("\n", preamblesCode);
    }
}
