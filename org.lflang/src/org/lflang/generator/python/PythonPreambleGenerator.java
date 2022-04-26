package org.lflang.generator.python;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.c.CPreambleGenerator;
import org.lflang.lf.Preamble;


/**
 * Generates user-defined preambles and #define and #include directives
 * for the Python target.
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */

public class PythonPreambleGenerator {
    /**
     * Generates preambles defined by user for a given reactor.
     * The preamble code is put inside the reactor class.
     */
    public static String generatePythonPreambles(List<Preamble> preambles) {
        List<String> preamblesCode = new ArrayList<>();
        preambles.forEach(p -> preamblesCode.add(ASTUtils.toText(p.getCode())));
        return preamblesCode.size() > 0 ? String.join("\n", 
            "# From the preamble, verbatim:",
            String.join("\n", preamblesCode),
            "# End of preamble."
        ) : "";
    }

    public static String generateCDefineDirectives(
        TargetConfig targetConfig,
        int numFederates,
        boolean isFederated,
        Path srcGenPath,
        boolean clockSyncIsOn,
        boolean hasModalReactors
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateDefineDirectives(
            targetConfig, numFederates, isFederated, 
            srcGenPath, clockSyncIsOn, hasModalReactors)
        );
        code.pr("#define _LF_GARBAGE_COLLECTED");
        return code.toString();
    }

    public static String generateCIncludeStatements(
        TargetConfig targetConfig, 
        boolean isFederated,
        boolean hasModalReactors
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateIncludeStatements(targetConfig, isFederated));
        code.pr("#include \"pythontarget.c\"");
        if (hasModalReactors) {
            code.pr("#include \"modal_models/definitions.h\"");
        }
        return code.toString();
    }
}
