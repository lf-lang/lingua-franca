package org.lflang.generator.c;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.lflang.TargetConfig;
import org.lflang.TargetConfig.ClockSyncOptions;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;

import static org.lflang.util.StringUtil.addDoubleQuotes;

/**
 * Generates code for preambles for the C and CCpp target.
 * This includes #include and #define directives at the top
 * of each generated ".c" file. 
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Mehrdad Niknami <mniknami@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author{Soroush Bateni <soroush@utdallas.edu>
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CPreambleGenerator {
    /** Add necessary source files specific to the target language.  */
    public static String generateIncludeStatements(
        TargetConfig targetConfig, 
        boolean isFederated
    ) {
        var tracing = targetConfig.tracing;
        CodeBuilder code = new CodeBuilder();
        code.pr("#include \"ctarget.h\"");
        if (targetConfig.threading) {
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
        TargetConfig targetConfig,
        int numFederates,
        boolean isFederated,
        Path srcGenPath,
        boolean clockSyncIsOn,
        boolean hasModalReactors
    ) {
        int logLevel = targetConfig.logLevel.ordinal();
        var coordinationType = targetConfig.coordination;
        var advanceMessageInterval = targetConfig.coordinationOptions.advance_message_interval;
        var tracing = targetConfig.tracing;
        CodeBuilder code = new CodeBuilder();
        code.pr("#define LOG_LEVEL " + logLevel);
        code.pr("#define TARGET_FILES_DIRECTORY " + addDoubleQuotes(srcGenPath.toString()));
        if (isFederated) {
            code.pr("#define NUMBER_OF_FEDERATES " + numFederates);
            code.pr(generateFederatedDefineDirective(coordinationType));
            if (advanceMessageInterval != null) {
                code.pr("#define ADVANCE_MESSAGE_INTERVAL " + 
                    GeneratorBase.timeInTargetLanguage(advanceMessageInterval));
            }
        }
        if (tracing != null) {
            code.pr(generateTracingDefineDirective(targetConfig, tracing.traceFileName));
        }
        if (hasModalReactors) {
            code.pr("#define MODAL_REACTORS");
        }
        if (clockSyncIsOn) {
            code.pr(generateClockSyncDefineDirective(
                targetConfig.clockSync,
                targetConfig.clockSyncOptions
            ));
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
    private static String generateFederatedDefineDirective(CoordinationType coordinationType) {        
        List<String> directives = new ArrayList<>();
        directives.add("#define FEDERATED");
        if (coordinationType == CoordinationType.CENTRALIZED) {
            directives.add("#define FEDERATED_CENTRALIZED");
        } else if (coordinationType == CoordinationType.DECENTRALIZED) {
            directives.add("#define FEDERATED_DECENTRALIZED");
        }
        return String.join("\n", directives);
    }

    private static String generateTracingDefineDirective(
        TargetConfig targetConfig,
        String traceFileName
    ) {
        if (traceFileName == null) {
            targetConfig.compileDefinitions.put("LINGUA_FRANCA_TRACE", "");
            return "#define LINGUA_FRANCA_TRACE";
        }
        targetConfig.compileDefinitions.put("LINGUA_FRANCA_TRACE", traceFileName);
        return "#define LINGUA_FRANCA_TRACE " + traceFileName;
    }

    /**
     * Initialize clock synchronization (if enabled) and its related options for a given federate.
     * 
     * Clock synchronization can be enabled using the clock-sync target property.
     * @see https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization
     */
    private static String generateClockSyncDefineDirective(
        ClockSyncMode mode, 
        ClockSyncOptions options
    ) {
        List<String> code = new ArrayList<>();
        code.addAll(List.of(
            "#define _LF_CLOCK_SYNC_INITIAL",
            "#define _LF_CLOCK_SYNC_PERIOD_NS "+GeneratorBase.timeInTargetLanguage(options.period),
            "#define _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL "+options.trials,
            "#define _LF_CLOCK_SYNC_ATTENUATION "+options.attenuation
        ));
        if (mode == ClockSyncMode.ON) {
            code.add("#define _LF_CLOCK_SYNC_ON");
            if (options.collectStats) {
                code.add("#define _LF_CLOCK_SYNC_COLLECT_STATS");
            }
        }
        return String.join("\n", code);
    }
}
