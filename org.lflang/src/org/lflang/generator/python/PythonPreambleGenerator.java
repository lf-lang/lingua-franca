package org.lflang.generator.python;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
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

    public static String generateDefineDirectives(
        int logLevel,
        int numFederates,
        boolean isFederated,
        CoordinationType coordinationType,
        TimeValue advanceMessageInterval,
        Path srcGenPath,
        TracingOptions tracing,
        boolean hasModalReactors
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateDefineDirectives(logLevel, numFederates, isFederated, coordinationType, advanceMessageInterval, srcGenPath, tracing, hasModalReactors));
        code.pr("#define _LF_GARBAGE_COLLECTED");
        return code.toString();
    }

    public static String generateIncludeStatements(
        int nThreads, 
        boolean isFederated,
        TracingOptions tracing
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateIncludeStatements(nThreads, isFederated, tracing));
        code.pr("#include \"pythontarget.c\"");
        return code.toString();
    }
}
