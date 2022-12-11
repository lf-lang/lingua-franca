package org.lflang.generator.c;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.lflang.TargetConfig;
import org.lflang.TargetConfig.ClockSyncOptions;
import org.lflang.TargetProperty.Platform;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.util.StringUtil;

import static org.lflang.util.StringUtil.addDoubleQuotes;

/**
 * Generates code for preambles for the C and CCpp target.
 * This includes #include and #define directives at the top
 * of each generated ".c" file.
 *
 * @author Edward A. Lee <eal@berkeley.edu>
 * @author Marten Lohstroh <marten@berkeley.edu>
 * @author Mehrdad Niknami <mniknami@berkeley.edu>
 * @author Christian Menard <christian.menard@tu-dresden.de>
 * @author Matt Weber <matt.weber@berkeley.edu>
 * @author Soroush Bateni <soroush@utdallas.edu>
 * @author Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>
 * @author Hou Seng Wong <housengw@berkeley.edu>
 * @author Peter Donovan <peterdonovan@berkeley.edu>
 */
public class CPreambleGenerator {
    /** Add necessary source files specific to the target language.  */
    public static String generateIncludeStatements(
        TargetConfig targetConfig,
        boolean cppMode
    ) {
        var tracing = targetConfig.tracing;
        CodeBuilder code = new CodeBuilder();
        if (cppMode) {
            code.pr("extern \"C\" {");
        }
        if (targetConfig.platformOptions.platform == Platform.ARDUINO) {
            CCoreFilesUtils.getArduinoTargetHeaders().forEach(
                it -> code.pr("#include " + StringUtil.addDoubleQuotes(it))
            );
        }
        CCoreFilesUtils.getCTargetHeader().forEach(
            it -> code.pr("#include " + StringUtil.addDoubleQuotes(it))
        );
        code.pr("#include \"core/reactor.h\"");
        code.pr("#include \"core/reactor_common.h\"");
        if (targetConfig.threading) {
            code.pr("#include \"core/threaded/scheduler.h\"");
        }
        if (tracing != null) {
            code.pr("#include \"core/trace.h\"");
        }
        code.pr("#include \"core/mixed_radix.h\"");
        code.pr("#include \"core/port.h\"");
        code.pr("int lf_reactor_c_main(int argc, const char* argv[]);");
        if (cppMode) {
            code.pr("}");
        }
        return code.toString();
    }

    public static String generateDefineDirectives(
        TargetConfig targetConfig,
        Path srcGenPath,
        boolean hasModalReactors
    ) {
        int logLevel = targetConfig.logLevel.ordinal();
        var coordinationType = targetConfig.coordination;
        var tracing = targetConfig.tracing;
        CodeBuilder code = new CodeBuilder();
        // TODO: Get rid of all of these
        code.pr("#define LOG_LEVEL " + logLevel);
        code.pr("#define TARGET_FILES_DIRECTORY " + addDoubleQuotes(srcGenPath.toString()));

        if (targetConfig.platformOptions.platform == Platform.ARDUINO) {
            code.pr("#define MICROSECOND_TIME");
            code.pr("#define BIT_32");
        }
//        if (isFederated) {
//            code.pr("#define NUMBER_OF_FEDERATES " + numFederates);
//            code.pr(generateFederatedDefineDirective(coordinationType));
//            if (advanceMessageInterval != null) {
//                code.pr("#define ADVANCE_MESSAGE_INTERVAL " +
//                    GeneratorBase.timeInTargetLanguage(advanceMessageInterval));
//            }
//        }
        if (tracing != null) {
            targetConfig.compileDefinitions.put("LINGUA_FRANCA_TRACE", tracing.traceFileName);
        }
        // if (clockSyncIsOn) {
        //     code.pr(generateClockSyncDefineDirective(
        //         targetConfig.clockSync,
        //         targetConfig.clockSyncOptions
        //     ));
        // }
        if (targetConfig.threading) {
            targetConfig.compileDefinitions.put("LF_THREADED", "1");
        } else {
            targetConfig.compileDefinitions.put("LF_UNTHREADED", "1");
        }
        code.newLine();
        return code.toString();
    }
}
