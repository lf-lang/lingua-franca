package org.lflang.generator.c;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.lflang.TimeValue;
import org.lflang.TargetConfig.TracingOptions;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.lf.Code;

import static org.lflang.util.StringUtil.addDoubleQuotes;

public class CPreambleGenerator {
    /** Add necessary source files specific to the target language.  */
    public static String generateIncludeStatements(
        int nThreads, 
        boolean isFederated,
        TracingOptions tracing
    ) {
        CodeBuilder code = new CodeBuilder();
        code.pr("#include \"ctarget.h\"");
        if (nThreads > 0) {
            code.pr("#include \"core/threaded/reactor_threaded.c\"");
            code.pr("#include \"core/threaded/scheduler.h\"");
        } else {
            code.pr("#include \"core/reactor.c\"");
        }
        if (isFederated) {
            code.pr("#include \"core/federated/federate.c\"");
        }
        if (tracing != null) {
            code.pr("#include \"core/trace.c\"");
        }
        code.pr("#include \"core/mixed_radix.h\"");
        return code.toString();
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
        code.pr("#define LOG_LEVEL " + logLevel);
        code.pr("#define TARGET_FILES_DIRECTORY " + addDoubleQuotes(srcGenPath.toString()));
        if (isFederated) {
            code.pr("#define NUMBER_OF_FEDERATES " + numFederates);
            code.pr(generateFederatedDirective(coordinationType));
            if (advanceMessageInterval != null) {
                code.pr("#define ADVANCE_MESSAGE_INTERVAL " + 
                    GeneratorBase.timeInTargetLanguage(advanceMessageInterval));
            }
        }
        if (tracing != null) {
            code.pr(generateTracingHeader(tracing.traceFileName));
        }
        if (hasModalReactors) {
            code.pr("#define MODAL_REACTORS");
        }
        return code.toString();
    }

    /**
     * Returns the #define directive for the given coordination type.
     * 
     * NOTE: Instead of checking #ifdef FEDERATED, we could
     *       use #if (NUMBER_OF_FEDERATES > 1).
     *       To Soroush Bateni, the former is more accurate.
     */
    private static String generateFederatedDirective(CoordinationType coordinationType) {        
        List<String> directives = new ArrayList<>();
        directives.add("#define FEDERATED");
        if (coordinationType == CoordinationType.CENTRALIZED) {
            directives.add("#define FEDERATED_CENTRALIZED");
        } else if (coordinationType == CoordinationType.DECENTRALIZED) {
            directives.add("#define FEDERATED_DECENTRALIZED");
        }
        return String.join("\n", directives);
    }

    private static String generateTracingHeader(String traceFileName) {
        if (traceFileName == null) {
            return "#define LINGUA_FRANCA_TRACE";
        }
        return "#define LINGUA_FRANCA_TRACE " + traceFileName;
    }
}
