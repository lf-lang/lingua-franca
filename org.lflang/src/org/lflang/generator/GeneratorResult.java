package org.lflang.generator;

import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.lflang.util.LFCommand;

/**
 * A {@code GeneratorResult} is the outcome of a code generation task.
 */
public class GeneratorResult {
    public static GeneratorResult NOTHING = incompleteGeneratorResult(Status.NOTHING);
    public static GeneratorResult CANCELLED = incompleteGeneratorResult(Status.CANCELLED);
    public static GeneratorResult FAILED = incompleteGeneratorResult(Status.FAILED);

    /**
     * A {@code Status} is a level of completion of a code generation task.
     */
    enum Status {
        NOTHING(gr -> ""),  // Code generation was not performed.
        CANCELLED(gr -> "Code generation was cancelled."),
        FAILED(gr -> "Code generation failed."),  // This may be due to a failed validation check, in which case the
        // problem will be displayed in the editor.
        GENERATED(GetUserMessage.COMPLETED),
        COMPILED(GetUserMessage.COMPLETED);  // Some targets (e.g., Python) will never have this status.

        /**
         * A {@code GetUserMessage} is a function that translates a
         * {@code GeneratorResult} into a human-readable report for the end user.
         */
        public interface GetUserMessage {
            GetUserMessage COMPLETED = gr -> {
                if (gr.executable != null) {
                    return String.format(
                        "Code generation completed. The executable is located at %s", gr.executable
                    );
                }
                return "Code generation completed.";
            };
            String apply(GeneratorResult result);
        }

        /** The {@code GetUserMessage} associated with this {@code Status}. */
        public final GetUserMessage gum;

        /** Initializes a {@code Status} whose {@code gum} is {@code gum}. */
        Status(GetUserMessage gum) {
            this.gum = gum;
        }
    }

    private final Status status;
    private final Path executable;
    private final List<LFCommand> commands;
    private final Map<Path, CodeMap> codeMaps;

    /**
     * Initializes a GeneratorResult.
     * @param status The level of completion of a code generation task.
     * @param executable The file that stores the final output of the code
     * generation task. Examples include a fully linked binary or a Python
     * file that can be passed to the Python interpreter.
     * @param commands A sequence of commands that together are sufficient to
     * run the executable, preferably from the project root.
     * @param codeMaps A mapping from generated files to their CodeMaps.
     */
    public GeneratorResult(Status status, Path executable, List<LFCommand> commands, Map<Path, CodeMap> codeMaps) {
        this.status = status != null ? status : Status.NOTHING;
        this.executable = executable;
        this.commands = commands != null ? commands : List.of();
        this.codeMaps = codeMaps != null ? codeMaps : Collections.emptyMap();
    }

    /**
     * Returns the result of an incomplete generation task that terminated
     * with status {@code status}.
     * @return the result of an incomplete generation task that terminated
     * with status {@code status}
     */
    private static GeneratorResult incompleteGeneratorResult(Status status) {
        return new GeneratorResult(status, null, Collections.emptyList(), Collections.emptyMap());
    }

    /** Returns the status of {@code this}. */
    public Status getStatus() {
        return status;
    }

    /** Returns the commands needed to execute the executable. */
    public List<String> getExecuteCommands() {
        return commands.stream().map(LFCommand::toString).collect(Collectors.toList());
    }

    /**
     * Returns a message that can be relayed to the end user about this
     * {@code GeneratorResult}.
     */
    public String getUserMessage() {
        return status.gum.apply(this);
    }

    /**
     * Returns a map from generated sources to their code maps. The
     * completeness of this resulting map is given on a best-effort
     * basis, but those mappings that it does contain are guaranteed
     * to be correct.
     * @return An unmodifiable map from generated sources to their
     * code maps.
     */
    public Map<Path, CodeMap> getCodeMaps() {
        return Collections.unmodifiableMap(codeMaps);
    }
}
