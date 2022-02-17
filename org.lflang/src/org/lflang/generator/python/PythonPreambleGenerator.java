package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.lf.Reactor;

public class PythonPreambleGenerator {
    /**
     * Generates preambles defined by user for a given reactor.
     * The preamble code is put inside the reactor class.
     */
    public static String generatePythonPreambles(Reactor reactor) {
        List<String> preambles = new ArrayList<>();
        reactor.getPreambles().forEach(p -> preambles.add(
            String.join("\n", 
                "# From the preamble, verbatim:",
                ASTUtils.toText(p.getCode()),
                "# End of preamble."
            )
        ));
        return String.join("\n", preambles);
    }
}
