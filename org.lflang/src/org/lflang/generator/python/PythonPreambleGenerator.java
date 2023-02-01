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
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng Wong
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
        Path srcGenPath,
        boolean hasModalReactors
    ) {
        // TODO: Delete all of this. It is not used.
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateDefineDirectives(
            targetConfig, srcGenPath, hasModalReactors)
        );
        return code.toString();
    }

    public static String generateCIncludeStatements(
        TargetConfig targetConfig,
        boolean CCppMode,
        boolean hasModalReactors
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr(CPreambleGenerator.generateIncludeStatements(targetConfig, CCppMode));
        code.pr("#include \"pythontarget.h\"");
        if (hasModalReactors) {
            code.pr("#include \"modal_models/definitions.h\"");
        }
        return code.toString();
    }
}
