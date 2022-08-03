package org.lflang.generator.python;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Preamble;


/**
 * Generates user-defined preambles and #define and #include directives
 * for the Python target.
 *
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
public class Preambles {
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
        // TODO: Delete all of this. It is not used.
        CodeBuilder code = new CodeBuilder();
        code.pr(org.lflang.generator.c.Preambles.generateDefineDirectives(
            targetConfig, numFederates, isFederated,
            srcGenPath, clockSyncIsOn, hasModalReactors)
        );
        return code.toString();
    }

    public static String generateCIncludeStatements(
        TargetConfig targetConfig,
        boolean CCppMode,
        boolean isFederated,
        boolean hasModalReactors
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(org.lflang.generator.c.Preambles.generateIncludeStatements(targetConfig, CCppMode, isFederated));
        code.pr("#include \"pythontarget.h\"");
        if (hasModalReactors) {
            code.pr("#include \"modal_models/definitions.h\"");
        }
        return code.toString();
    }
}
