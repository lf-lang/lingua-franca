package org.lflang.generator;

import com.google.common.collect.ImmutableMap;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.stream.Collectors;
import org.eclipse.xtext.util.CancelIndicator;
import org.lflang.MessageReporter;
import org.lflang.util.LFCommand;

/**
 * Validate generated code.
 *
 * @author Peter Donovan
 */
public abstract class Validator {

  /**
   * Files older than {@code FILE_AGE_THRESHOLD_MILLIS} may be skipped in validation on the grounds
   * that they probably have not been updated since the last validator pass.
   */
  // This will cause silent validation failures if it takes too long to write all generated code to
  // the file system.
  private static final long FILE_AGE_THRESHOLD_MILLIS = 10000;

  protected static class Pair<S, T> {
    public final S first;
    public final T second;

    public Pair(S first, T second) {
      this.first = first;
      this.second = second;
    }
  }

  protected final MessageReporter messageReporter;
  protected final ImmutableMap<Path, CodeMap> codeMaps;

  /**
   * Initialize a {@code Validator} that reports errors to {@code errorReporter} and adjusts
   * document positions using {@code codeMaps}.
   */
  protected Validator(MessageReporter messageReporter, Map<Path, CodeMap> codeMaps) {
    this.messageReporter = messageReporter;
    this.codeMaps = ImmutableMap.copyOf(codeMaps);
  }

  /**
   * Validate this Validator's group of generated files.
   *
   * @param context The context of the current build.
   */
  public final void doValidate(LFGeneratorContext context)
      throws ExecutionException, InterruptedException {
    if (!validationEnabled(context)) return;
    final List<Callable<Pair<ValidationStrategy, LFCommand>>> tasks =
        getValidationStrategies().stream()
            .map(
                it ->
                    (Callable<Pair<ValidationStrategy, LFCommand>>)
                        () -> {
                          it.second.run(context.getCancelIndicator());
                          return it;
                        })
            .collect(Collectors.toList());
    for (Future<Pair<ValidationStrategy, LFCommand>> f : getFutures(tasks)) {
      f.get()
          .first
          .getErrorReportingStrategy()
          .report(f.get().second.getErrors(), messageReporter, codeMaps);
      f.get()
          .first
          .getOutputReportingStrategy()
          .report(f.get().second.getOutput(), messageReporter, codeMaps);
    }
  }

  /**
   * Return whether generated code validation is enabled for this build.
   *
   * @param context The context of the current build.
   */
  private boolean validationEnabled(LFGeneratorContext context) {
    return context.getArgs().lint() || validationEnabledByDefault(context);
  }

  /**
   * Return whether validation of generated code is enabled by default.
   *
   * @param context The context of the current build.
   * @return Whether validation of generated code is enabled by default.
   */
  protected boolean validationEnabledByDefault(LFGeneratorContext context) {
    return context.getMode() != LFGeneratorContext.Mode.STANDALONE;
  }

  /**
   * Invoke all the given tasks.
   *
   * @param tasks Any set of tasks.
   * @param <T> The return type of the tasks.
   * @return Futures corresponding to each task, or an empty list upon failure.
   * @throws InterruptedException If interrupted while waiting.
   */
  private static <T> List<Future<T>> getFutures(List<Callable<T>> tasks)
      throws InterruptedException {
    List<Future<T>> futures = List.of();
    switch (tasks.size()) {
      case 0:
        break;
      case 1:
        try {
          futures = List.of(CompletableFuture.completedFuture(tasks.get(0).call()));
        } catch (Exception e) {
          System.err.println(e.getMessage()); // This should never happen
        }
        break;
      default:
        ExecutorService service =
            Executors.newFixedThreadPool(
                Math.min(Runtime.getRuntime().availableProcessors(), tasks.size()));
        futures = service.invokeAll(tasks);
        service.shutdown();
    }
    return futures;
  }

  /**
   * Run the given command, report any messages produced using the reporting strategies given by
   * {@code getBuildReportingStrategies}, and return its return code.
   */
  public final int run(LFCommand command, CancelIndicator cancelIndicator) {
    final int returnCode = command.run(cancelIndicator);
    getBuildReportingStrategies().first.report(command.getErrors(), messageReporter, codeMaps);
    getBuildReportingStrategies().second.report(command.getOutput(), messageReporter, codeMaps);
    return returnCode;
  }

  /**
   * Return the validation strategies and validation commands corresponding to each generated file.
   *
   * @return the validation strategies and validation commands corresponding to each generated file
   */
  private List<Pair<ValidationStrategy, LFCommand>> getValidationStrategies() {
    final List<Pair<ValidationStrategy, LFCommand>> commands = new ArrayList<>();
    long mostRecentDateModified =
        codeMaps.keySet().stream().map(it -> it.toFile().lastModified()).reduce(0L, Math::max);
    for (Path generatedFile : codeMaps.keySet()) {
      if (generatedFile.toFile().lastModified()
          > mostRecentDateModified - FILE_AGE_THRESHOLD_MILLIS) {
        final Pair<ValidationStrategy, LFCommand> p = getValidationStrategy(generatedFile);
        if (p.first == null || p.second == null) continue;
        commands.add(p);
        if (p.first.isFullBatch()) break;
      }
    }
    return commands;
  }

  /**
   * Return the validation strategy and command corresponding to the given file if such a strategy
   * and command are available.
   *
   * @return the validation strategy and command corresponding to the given file if such a strategy
   *     and command are available
   */
  private Pair<ValidationStrategy, LFCommand> getValidationStrategy(Path generatedFile) {
    List<ValidationStrategy> sorted =
        getPossibleStrategies().stream()
            .sorted(Comparator.comparingInt(vs -> -vs.getPriority()))
            .toList();
    for (ValidationStrategy strategy : sorted) {
      LFCommand validateCommand = strategy.getCommand(generatedFile);
      if (validateCommand != null) {
        return new Pair<>(strategy, validateCommand);
      }
    }
    return new Pair<>(null, null);
  }

  /**
   * List all validation strategies that exist for the implementor without filtering by platform or
   * availability.
   */
  protected abstract Collection<ValidationStrategy> getPossibleStrategies();

  /** Return the appropriate output and error reporting strategies for the main build process. */
  protected abstract Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy>
      getBuildReportingStrategies();
}
