package org.lflang.generator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.CharSequence;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import org.lflang.util.LFCommand;

/**
 * A helper class with functions that may be useful for code
 * generators.
 * This is created to ease our transition from Xtend and
 * possibly Eclipse. All functions in this class should
 * instead be in GeneratorUtils.kt, but Eclipse cannot
 * handle Kotlin files.
 */
public class JavaGeneratorUtils {

    private JavaGeneratorUtils() {
        // utility class
    }

    /**
     * Write the source code to file.
     * @param code The code to be written.
     * @param path The file to write the code to.
     */
    public static void writeSourceCodeToFile(CharSequence code, String path) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(path))) {
            for (int i = 0; i < code.length(); i++) {
                writer.write(code.charAt(i));
            }
        }
    }

    /**
     * Informs the context of the result of its build, if applicable.
     * @param context The context in which the build was performed.
     * @param status The status of the result.
     * @param execName The name of the executable produced by this code
     * generation process, or {@code null} if no executable was produced.
     * @param binPath The directory containing the executable (if applicable)
     */
    public static void finish(
        LFGeneratorContext context,
        GeneratorResult.Status status,
        String execName,
        Path binPath,
        Map<Path, CodeMap> codeMaps
    ) {
        finish(context, new GeneratorResult(
            status,
            execName == null ? null : binPath.resolve(execName),
            LFCommand.get("." + File.separator + execName, List.of(), binPath),
            codeMaps
        ));
    }

    /**
     * Informs the context of the result of its build, if applicable.
     * @param context The context in which the build was performed.
     * @param result The result of the build.
     */
    public static void finish(LFGeneratorContext context, GeneratorResult result) {
        reportProgress(context, "Build complete.", 100);
        context.finish(result);
    }

    /**
     * Informs the context of that its build finished unsuccessfully.
     * @param context The context in which the build was performed.
     */
    public static void unsuccessfulFinish(LFGeneratorContext context) {
        finish(context, context.getCancelIndicator().isCanceled() ? GeneratorResult.CANCELLED : GeneratorResult.FAILED);
    }

    /**
     * Informs the context of the current progress of its build,
     * if applicable.
     * @param context The context of a build.
     * @param message A message about the build's progress.
     * @param percentage The build's percent completion.
     */
    public static void reportProgress(LFGeneratorContext context, String message, int percentage) {
        context.getReportProgress().apply(message, percentage);
    }
}
