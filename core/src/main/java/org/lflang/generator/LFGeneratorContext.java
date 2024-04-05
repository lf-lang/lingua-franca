package org.lflang.generator;

import java.nio.file.Path;
import java.util.Map;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.target.TargetConfig;

/**
 * An {@code LFGeneratorContext} is the context of a Lingua Franca build process. It is the point of
 * communication between a build process and the environment in which it is executed.
 *
 * @author Peter Donovan
 */
public interface LFGeneratorContext extends IGeneratorContext {

  enum Mode {
    STANDALONE,
    EPOCH,
    LSP_FAST,
    LSP_MEDIUM,
    LSP_SLOW,
    UNDEFINED
  }

  /**
   * Return the mode of operation, which indicates how the compiler has been invoked (e.g., from
   * within Epoch, from the command line, or via a Language Server).
   */
  Mode getMode();

  /** Return any arguments that will override target properties. */
  GeneratorArguments getArgs();

  /** Get the error reporter for this context; construct one if it hasn't been constructed yet. */
  MessageReporter getErrorReporter();

  /** Return true if the user requested a clean build in this context. */
  default boolean isCleanRequested() {
    return getArgs().clean();
  }

  /**
   * Mark the code generation process performed in this context as finished with the result {@code
   * result}.
   *
   * @param result The result of the code generation process that was performed in this context.
   */
  void finish(GeneratorResult result);

  /**
   * Return the result of the code generation process that was performed in this context.
   *
   * @return the result of the code generation process that was performed in this context
   */
  GeneratorResult getResult();

  FileConfig getFileConfig();

  TargetConfig getTargetConfig();

  /**
   * Report the progress of a build.
   *
   * @param message A message for the LF programmer to read.
   * @param percentage The approximate percent completion of the build.
   */
  void reportProgress(String message, int percentage);

  /**
   * Conclude this build and record the result if necessary.
   *
   * @param status The status of the result.
   * @param codeMaps The generated files and their corresponding code maps.
   */
  default void finish(GeneratorResult.Status status, Map<Path, CodeMap> codeMaps) {
    finish(new GeneratorResult(status, this, codeMaps));
  }

  /** Conclude this build and record that it was unsuccessful. */
  default void unsuccessfulFinish() {
    finish(
        getCancelIndicator() != null && getCancelIndicator().isCanceled()
            ? GeneratorResult.CANCELLED
            : GeneratorResult.FAILED);
  }

  /**
   * Return the {@code LFGeneratorContext} that best describes the given {@code context} when
   * building {@code Resource}.
   *
   * @param resource
   * @param fsa
   * @param context The context of a Lingua Franca build process.
   * @return The {@code LFGeneratorContext} that best describes the given {@code context} when
   *     building {@code Resource}.
   */
  static LFGeneratorContext lfGeneratorContextOf(
      Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
    if (context instanceof LFGeneratorContext) return (LFGeneratorContext) context;

    if (resource.getURI().isPlatform())
      return new MainContext(Mode.EPOCH, resource, fsa, context.getCancelIndicator());

    return new MainContext(Mode.LSP_FAST, resource, fsa, context.getCancelIndicator());
  }
}
