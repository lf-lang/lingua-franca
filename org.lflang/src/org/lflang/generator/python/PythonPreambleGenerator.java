package org.lflang.generator.python;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.TimeValue;
import org.lflang.TargetConfig.TracingOptions;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.c.CPreambleGenerator;
import org.lflang.lf.Preamble;

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
        boolean isFederated
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateIncludeStatements(targetConfig, isFederated));
        code.pr("#include \"pythontarget.c\"");
        return code.toString();
    }
}
