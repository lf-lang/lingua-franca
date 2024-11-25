package org.lflang.generator;

import org.eclipse.xtext.util.CancelIndicator;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.target.TargetConfig;

/**
 * A {@code SubContext} is the context of a process within a build process. For example, compilation
 * of generated code may optionally be given a {@code SubContext} because compilation is part of a
 * complete build.
 *
 * @author Peter Donovan
 */
public class SubContext implements LFGeneratorContext {

  private final LFGeneratorContext containingContext;
  private final int startPercentProgress;
  private final int endPercentProgress;
  private GeneratorResult result = null;

  protected MessageReporter messageReporter;

  /**
   * Initializes the context within {@code containingContext} of the process that extends from
   * {@code startPercentProgress} to {@code endPercentProgress}.
   *
   * @param containingContext The context of the containing build process.
   * @param startPercentProgress The percent progress of the containing build process when this
   *     nested process starts.
   * @param endPercentProgress The percent progress of the containing build process when this nested
   *     process ends.
   */
  public SubContext(
      LFGeneratorContext containingContext, int startPercentProgress, int endPercentProgress) {
    this.containingContext = containingContext;
    this.startPercentProgress = startPercentProgress;
    this.endPercentProgress = endPercentProgress;
  }

  @Override
  public boolean isCleanRequested() {
    return false;
  } /// never clean in a sub context

  @Override
  public CancelIndicator getCancelIndicator() {
    return containingContext.getCancelIndicator();
  }

  @Override
  public Mode getMode() {
    return containingContext.getMode();
  }

  @Override
  public GeneratorArguments getArgs() {
    return containingContext.getArgs();
  }

  @Override
  public MessageReporter getErrorReporter() {
    return containingContext.getErrorReporter();
  }

  @Override
  public void finish(GeneratorResult result) {
    this.result = result;
  }

  @Override
  public GeneratorResult getResult() {
    return result;
  }

  @Override
  public FileConfig getFileConfig() {
    return containingContext.getFileConfig();
  }

  @Override
  public TargetConfig getTargetConfig() {
    return containingContext.getTargetConfig();
  }

  @Override
  public void reportProgress(String message, int percentage) {
    containingContext.reportProgress(
        message,
        startPercentProgress * (100 - percentage) / 100 + endPercentProgress * percentage / 100);
  }
}
