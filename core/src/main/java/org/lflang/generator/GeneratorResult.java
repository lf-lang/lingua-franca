package org.lflang.generator;

import java.nio.file.Path;
import java.util.Collections;
import java.util.Map;
import java.util.function.BiFunction;

/**
 * A {@code GeneratorResult} is the outcome of a code generation task.
 *
 * @author Peter Donovan
 */
public class GeneratorResult {
  public static GeneratorResult NOTHING = incompleteGeneratorResult(Status.NOTHING);
  public static GeneratorResult CANCELLED = incompleteGeneratorResult(Status.CANCELLED);
  public static GeneratorResult FAILED = incompleteGeneratorResult(Status.FAILED);
  public static BiFunction<LFGeneratorContext, Map<Path, CodeMap>, GeneratorResult>
      GENERATED_NO_EXECUTABLE =
          (context, codeMaps) -> new GeneratorResult(Status.GENERATED, context, codeMaps);

  /** A {@code Status} is a level of completion of a code generation task. */
  public enum Status {
    NOTHING(result -> ""), // Code generation was not performed.
    CANCELLED(result -> "Code generation was cancelled."),
    FAILED(
        result ->
            ""), // This may be due to a failed validation check, in which case the error should
    // have been
    // sent to the error reporter and handled there. This makes a message unnecessary.
    GENERATED(GetUserMessage.COMPLETED),
    COMPILED(GetUserMessage.COMPLETED);

    /**
     * A {@code GetUserMessage} is a function that translates a {@code GeneratorResult} into a
     * human-readable report for the end user.
     */
    public interface GetUserMessage {
      GetUserMessage COMPLETED =
          result -> {
            return String.format(
                "Code generation complete. The executable is at \"%s\".",
                result.getContext().getFileConfig().getExecutable());
          };

      String apply(GeneratorResult result);
    }

    /** The {@code GetUserMessage} associated with this {@code Status}. */
    private final GetUserMessage gum;

    /** Initializes a {@code Status} whose {@code GetUserMessage} is {@code gum}. */
    Status(GetUserMessage gum) {
      this.gum = gum;
    }
  }

  private final Status status;

  private final LFGeneratorContext context;

  private final Map<Path, CodeMap> codeMaps;

  /**
   * Initialize a GeneratorResult.
   *
   * @param status The level of completion of a code generation task.
   * @param context The context within which the result was produced.
   * @param codeMaps A mapping from generated files to their CodeMaps.
   */
  public GeneratorResult(Status status, LFGeneratorContext context, Map<Path, CodeMap> codeMaps) {
    this.status = status != null ? status : Status.NOTHING;
    this.context = context;
    this.codeMaps = codeMaps != null ? codeMaps : Collections.emptyMap();
  }

  /**
   * Return the result of an incomplete generation task that terminated with status {@code status}.
   *
   * @return the result of an incomplete generation task that terminated with status {@code status}
   */
  private static GeneratorResult incompleteGeneratorResult(Status status) {
    return new GeneratorResult(status, null, Collections.emptyMap());
  }

  /** Return the status of {@code this}. */
  public Status getStatus() {
    return status;
  }

  /** Return a message that can be relayed to the end user about this {@code GeneratorResult}. */
  public String getUserMessage() {
    return status.gum.apply(this);
  }

  /**
   * Return a map from generated sources to their code maps. The completeness of this resulting map
   * is given on a best-effort basis, but those mappings that it does contain are guaranteed to be
   * correct.
   *
   * @return An unmodifiable map from generated sources to their code maps.
   */
  public Map<Path, CodeMap> getCodeMaps() {
    return Collections.unmodifiableMap(codeMaps);
  }

  public LFGeneratorContext getContext() {
    return context;
  }
}
