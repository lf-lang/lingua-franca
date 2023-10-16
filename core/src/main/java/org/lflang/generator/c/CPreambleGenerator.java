package org.lflang.generator.c;

import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.nio.file.Path;
import java.util.HashMap;
import org.lflang.generator.CodeBuilder;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.FedSetupProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.ThreadingProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.type.PlatformType.Platform;
import org.lflang.util.StringUtil;

/**
 * Generates code for preambles for the C and CCpp target. This includes #include and #define
 * directives at the top of each generated ".c" file.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Mehrdad Niknami
 * @author Christian Menard
 * @author Matt Weber
 * @author Soroush Bateni
 * @author Alexander Schulz-Rosengarten
 * @author Hou Seng Wong
 * @author Peter Donovan
 * @author Anirudh Rengarajan
 */
public class CPreambleGenerator {
  /** Add necessary source files specific to the target language. */
  public static String generateIncludeStatements(TargetConfig targetConfig, boolean cppMode) {

    CodeBuilder code = new CodeBuilder();
    if (cppMode || targetConfig.get(new PlatformProperty()).platform == Platform.ARDUINO) {
      code.pr("extern \"C\" {");
    }
    code.pr("#include <limits.h>");
    code.pr("#include \"include/core/platform.h\"");
    CCoreFilesUtils.getCTargetHeader()
        .forEach(it -> code.pr("#include " + StringUtil.addDoubleQuotes(it)));
    code.pr("#include \"include/core/reactor.h\"");
    code.pr("#include \"include/core/reactor_common.h\"");
    if (targetConfig.get(new ThreadingProperty())) {
      code.pr("#include \"include/core/threaded/scheduler.h\"");
    }

    if (targetConfig.get(new TracingProperty()).isEnabled()) {
      code.pr("#include \"include/core/trace.h\"");
    }
    code.pr("#include \"include/core/mixed_radix.h\"");
    code.pr("#include \"include/core/port.h\"");
    code.pr("#include \"include/core/environment.h\"");

    code.pr("int lf_reactor_c_main(int argc, const char* argv[]);");
    if (targetConfig.isSet(new FedSetupProperty())) {
      code.pr("#include \"include/core/federated/federate.h\"");
      code.pr("#include \"include/core/federated/net_common.h\"");
    }
    if (cppMode || targetConfig.get(new PlatformProperty()).platform == Platform.ARDUINO) {
      code.pr("}");
    }
    return code.toString();
  }

  public static String generateDefineDirectives(TargetConfig targetConfig, Path srcGenPath) {
    int logLevel = targetConfig.get(new LoggingProperty()).ordinal();
    var tracing = targetConfig.get(new TracingProperty());
    CodeBuilder code = new CodeBuilder();
    // TODO: Get rid of all of these
    code.pr("#define LOG_LEVEL " + logLevel);
    code.pr("#define TARGET_FILES_DIRECTORY " + addDoubleQuotes(srcGenPath.toString()));
    final var definitions = new HashMap<String, String>();
    if (tracing.isEnabled()) {
      definitions.put("LF_TRACE", tracing.traceFileName);
    }
    // if (clockSyncIsOn) {
    //     code.pr(generateClockSyncDefineDirective(
    //         targetConfig.clockSync,
    //         targetConfig.clockSyncOptions
    //     ));
    // }

    if (targetConfig.get(new ThreadingProperty())) {
      definitions.put("LF_THREADED", "1");
    } else {
      definitions.put("LF_UNTHREADED", "1");
    }
    if (targetConfig.get(new ThreadingProperty())) {
      definitions.put("LF_THREADED", "1");
    } else {
      definitions.put("LF_UNTHREADED", "1");
    }
    new CompileDefinitionsProperty().update(targetConfig, definitions);
    code.newLine();
    return code.toString();
  }
}
