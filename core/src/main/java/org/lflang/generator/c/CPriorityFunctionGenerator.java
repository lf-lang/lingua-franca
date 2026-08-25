package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import org.lflang.MessageReporter;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.DeadlineStats;
import org.lflang.generator.ReactorInstance;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.FedSetupProperty;

/**
 * Generates the C {@code get_priority_value} function that maps reaction deadlines to OS thread
 * priorities.
 *
 * <p>For federated applications, deadline statistics are read from {@code
 * federation_properties.json} (written once for the whole federation). For non-federated
 * applications, statistics are collected from the main reactor instance tree.
 *
 * @ingroup Generator
 */
public class CPriorityFunctionGenerator {

  /** Smallest alpha (ms^-1) the log-grid search will consider. */
  private static final double MIN_ALPHA_LIMIT = 1e-12;

  /** Largest alpha (ms^-1) the log-grid search will consider. */
  private static final double MAX_ALPHA_LIMIT = 1e3;

  private final CodeBuilder code;
  private final TargetConfig targetConfig;
  private final ReactorInstance main;
  private final Path srcPath;
  private final MessageReporter messageReporter;
  private final boolean emitExternC;

  /**
   * @param code Destination for the generated C function.
   * @param targetConfig Target configuration (used to detect federated builds).
   * @param main Main reactor instance (used for non-federated deadline collection).
   * @param srcPath Source directory; federated builds read {@code
   *     include/federation_properties.json} under this path.
   * @param messageReporter Reporter for warnings when the federation JSON is missing/unreadable.
   * @param emitExternC Whether to wrap the definition in {@code extern "C"} (CCpp / C++ mode).
   */
  public CPriorityFunctionGenerator(
      CodeBuilder code,
      TargetConfig targetConfig,
      ReactorInstance main,
      Path srcPath,
      MessageReporter messageReporter,
      boolean emitExternC) {
    this.code = code;
    this.targetConfig = targetConfig;
    this.main = main;
    this.srcPath = srcPath;
    this.messageReporter = messageReporter;
    this.emitExternC = emitExternC;
  }

  /** Generate and emit {@code get_priority_value}. */
  public void generate() {
    DeadlineStats stats = resolveStats();
    emitPriorityFunction(stats);
  }

  private DeadlineStats resolveStats() {
    if (targetConfig.isSet(FedSetupProperty.INSTANCE)) {
      Path jsonPath = srcPath.resolve(DeadlineStats.FEDERATION_PROPERTIES_REL_PATH);
      try {
        if (Files.exists(jsonPath)) {
          return DeadlineStats.readJson(jsonPath);
        }
        messageReporter
            .nowhere()
            .warning(
                "Federation properties JSON file not found at "
                    + jsonPath
                    + ", using default values");
      } catch (IOException | RuntimeException e) {
        messageReporter
            .nowhere()
            .warning(
                "Failed to read federation properties JSON: "
                    + e.getMessage()
                    + ", using default values");
      }
      return DeadlineStats.empty();
    }
    return DeadlineStats.fromReactorInstance(main);
  }

  private void emitPriorityFunction(DeadlineStats stats) {
    double minDeadline = stats.getMinDeadlineMs();
    double maxDeadline = stats.getMaxDeadlineMs();
    double medianDeadline = stats.getMedianDeadlineMs();
    List<Double> distinctDeadlinesMs = stats.getDistinctDeadlinesMs();

    // No valid deadlines — all inferred deadlines are the no-deadline sentinel, so every
    // reaction gets the lowest priority (1), matching the documented no-deadline rule.
    if (stats.isEmpty()) {
      prGetPriorityFunction(
          String.join(
              "\n",
              "// Priority assignment function (no deadlines found)",
              "int get_priority_value(interval_t rel_deadline) {",
              "    (void) rel_deadline;",
              "    return 1;",
              "}"));
      return;
    }

    // Generate the priority function: precompute exp() and derived values in Java so the
    // generated C uses numeric literals (one exp() remains for the runtime rel_deadline_ms term).
    if (Double.compare(minDeadline, maxDeadline) == 0) {
      prGetPriorityFunction(
          String.join(
              "\n",
              "int get_priority_value(interval_t rel_deadline) {",
              "  if (rel_deadline == 0) return 98;",
              // Negative values (e.g. NEVER if deadline(never) leaks into index) and large
              // sentinels encode the absence of a deadline -> lowest priority (1).
              "  if (rel_deadline < 0 || rel_deadline == FOREVER || rel_deadline >= 281474976710655LL) return 1;",
              "  return 98;",
              "}"));
    } else {
      // Prefer per-program alpha optimization if we have the full set of deadlines.
      // (Federated applications get this from federation_properties.json.)
      final double alpha;
      if (distinctDeadlinesMs != null && distinctDeadlinesMs.size() >= 2) {
        alpha = findBestAlphaForDeadlines(distinctDeadlinesMs, minDeadline, maxDeadline);
      } else {
        // Fallback: old heuristic (kept only for robustness if deadlinesMs isn't available).
        final double alphaMax = 0.025;
        final double alphaMin = 0.005;
        alpha =
            alphaMax
                - (medianDeadline - minDeadline)
                    / (maxDeadline - minDeadline)
                    * (alphaMax - alphaMin);
      }
      final double expMinusAlphaDmin = Math.exp(-alpha * minDeadline);
      final double expMinusAlphaDmax = Math.exp(-alpha * maxDeadline);
      final double denom = expMinusAlphaDmin - expMinusAlphaDmax;
      final double k = 96.0 / denom;
      final double p = 98.0 - 96.0 * expMinusAlphaDmin / denom;
      final double negAlpha = -alpha;
      // Keep the mapping safe for sentinel "no deadline" values and any values outside the
      // [minDeadline,maxDeadline] fitting interval.
      final double maxDeadlineMsFinal = maxDeadline;
      prGetPriorityFunction(
          String.join(
              "\n",
              "int get_priority_value(interval_t rel_deadline) {",
              "  if (rel_deadline == 0) return 98;",
              // Negative values (e.g. NEVER) and large sentinels (e.g. TimeValue.MAX_VALUE)
              // encode the absence of a deadline. Values above maxDeadline would otherwise map
              // to priorities below the curve floor; treat all as \"no deadline\" -> priority 1.
              "  if (rel_deadline < 0 || rel_deadline == FOREVER || rel_deadline >= 281474976710655LL) return 1;",
              "  double rel_deadline_ms = rel_deadline / 1000000.0;",
              "  if (rel_deadline_ms >= " + formatDoubleForC(maxDeadlineMsFinal) + ") return 2;",
              "  const double K = " + formatDoubleForC(k) + ";",
              "  const double P = " + formatDoubleForC(p) + ";",
              "  const double neg_alpha = " + formatDoubleForC(negAlpha) + ";",
              "  double continuous_fun_value = K * exp(neg_alpha * rel_deadline_ms) + P;",
              "  int prio = (int)round(continuous_fun_value);",
              "  if (prio < 2) prio = 2;",
              "  if (prio > 98) prio = 98;",
              "  return prio;",
              "}"));
    }
  }

  /**
   * Emit {@code get_priority_value}, which is called from reactor-c (C translation unit). CCpp
   * compiles the generated main file as C++, so the definition needs C linkage.
   */
  private void prGetPriorityFunction(String function) {
    if (emitExternC) {
      code.pr("extern \"C\"");
    }
    code.pr(function);
  }

  /**
   * Find alpha (ms^-1) that minimizes collisions after rounding, for a given program's distinct
   * inferred deadlines.
   *
   * <p>Optimization goal:
   *
   * <ol>
   *   <li>Minimize collision count: {@code collisions = m - |distinct priorities|}.
   *   <li>Deterministic tie-break for equal collision count: choose smaller alpha.
   * </ol>
   *
   * <p>Note: monotonicity does not need to be enforced explicitly here: the continuous mapping
   * {@code g(d) = K*exp(-alpha*d)+P} is strictly decreasing in {@code d} for {@code alpha>0}, and
   * {@code round(·)} is monotone. Therefore integer priorities cannot increase as deadlines
   * increase.
   */
  private static double findBestAlphaForDeadlines(
      List<Double> sortedDistinctDeadlinesMs, double minDeadlineMs, double maxDeadlineMs) {
    // Search alpha on a log scale: good values can vary over orders of magnitude.
    //
    // We start from a tight, canonical bracket derived from two extreme workloads
    // (see scripts/priority_alpha_search.py), and automatically widen it if the best alpha lands
    // on a boundary. This keeps compile-time cost low while avoiding missing the optimum.
    double alphaLo = 1e-5; // ms^-1 (canonical gentle end)
    double alphaHi = 0.025; // ms^-1 (canonical steep end)
    final int coarseSteps = 2000;
    final int fineSteps = 2000;

    // Hard safety limits (only used if boundary widening keeps triggering).
    final int maxExpansions = 8;

    AlphaSearchOutcome outcome = null;
    for (int attempt = 0; attempt <= maxExpansions; attempt++) {
      outcome =
          searchAlphaWithRefinement(
              sortedDistinctDeadlinesMs,
              minDeadlineMs,
              maxDeadlineMs,
              alphaLo,
              alphaHi,
              coarseSteps,
              fineSteps);
      if (outcome == null) {
        // Every alpha in [alphaLo, alphaHi] underflowed exp(-alpha * d), so the whole
        // bracket is too steep. Shift the window one decade downward.
        if (alphaLo > MIN_ALPHA_LIMIT) {
          alphaHi = alphaLo;
          alphaLo = Math.max(MIN_ALPHA_LIMIT, alphaLo / 10.0);
          continue;
        }
        return fallbackAlpha(maxDeadlineMs);
      }
      if (outcome.hitLowerBoundary && alphaLo > MIN_ALPHA_LIMIT) {
        alphaLo = Math.max(MIN_ALPHA_LIMIT, alphaLo / 10.0);
        continue;
      }
      if (outcome.hitUpperBoundary && alphaHi < MAX_ALPHA_LIMIT) {
        alphaHi = Math.min(MAX_ALPHA_LIMIT, alphaHi * 10.0);
        continue;
      }
      break;
    }
    return outcome != null ? outcome.best.alpha : fallbackAlpha(maxDeadlineMs);
  }

  /**
   * Alpha that keeps {@code exp(-alpha * maxDeadlineMs)} representable in double, used only if the
   * log-grid search never finds a finite candidate. Do not clamp this up to the search floor {@code
   * minAlphaLimit}: that floor can still overflow {@code alpha * d} for huge deadlines.
   */
  private static double fallbackAlpha(double maxDeadlineMs) {
    if (!(maxDeadlineMs > 0.0) || !Double.isFinite(maxDeadlineMs)) {
      return MIN_ALPHA_LIMIT;
    }
    // Target alpha * d_max = 1 so neither exponential underflows, even if 1/d_max
    // is below the search bracket.
    return 1.0 / maxDeadlineMs;
  }

  private record AlphaCandidate(double alpha, int collisions) {}

  private record AlphaSearchOutcome(
      AlphaCandidate best, boolean hitLowerBoundary, boolean hitUpperBoundary) {}

  private record GridSearchResult(AlphaCandidate best, int bestIndex, int steps) {}

  private static AlphaSearchOutcome searchAlphaWithRefinement(
      List<Double> sortedDistinctDeadlinesMs,
      double minDeadlineMs,
      double maxDeadlineMs,
      double alphaLo,
      double alphaHi,
      int coarseSteps,
      int fineSteps) {
    final GridSearchResult coarse =
        bestAlphaOnLogGrid(
            sortedDistinctDeadlinesMs, minDeadlineMs, maxDeadlineMs, alphaLo, alphaHi, coarseSteps);
    if (coarse == null) return null;

    // Refine around the coarse winner (in log space).
    final double lo = Math.log10(alphaLo);
    final double hi = Math.log10(alphaHi);
    final double step = (hi - lo) / (coarseSteps - 1);
    final double center = Math.log10(coarse.best.alpha);
    final double refineLo = Math.max(lo, center - 3.0 * step);
    final double refineHi = Math.min(hi, center + 3.0 * step);
    final GridSearchResult fine =
        bestAlphaOnLogGrid(
            sortedDistinctDeadlinesMs,
            minDeadlineMs,
            maxDeadlineMs,
            Math.pow(10.0, refineLo),
            Math.pow(10.0, refineHi),
            fineSteps);
    if (fine == null) return null;

    final boolean hitLower = fine.bestIndex == 0;
    final boolean hitUpper = fine.bestIndex == fine.steps - 1;
    return new AlphaSearchOutcome(fine.best, hitLower, hitUpper);
  }

  private static GridSearchResult bestAlphaOnLogGrid(
      List<Double> sortedDistinctDeadlinesMs,
      double minDeadlineMs,
      double maxDeadlineMs,
      double alphaLo,
      double alphaHi,
      int steps) {
    AlphaCandidate best = null;
    int bestIndex = -1;
    final double lo = Math.log10(alphaLo);
    final double hi = Math.log10(alphaHi);
    for (int i = 0; i < steps; i++) {
      final double t = (double) i / (steps - 1);
      final double alpha = Math.pow(10.0, lo + t * (hi - lo));
      final AlphaCandidate cand =
          scoreAlpha(alpha, sortedDistinctDeadlinesMs, minDeadlineMs, maxDeadlineMs);
      if (cand == null) continue;
      if (best == null) {
        best = cand;
        bestIndex = i;
        continue;
      }
      if (cand.collisions < best.collisions) {
        best = cand;
        bestIndex = i;
      } else if (cand.collisions == best.collisions && cand.alpha < best.alpha) {
        best = cand;
        bestIndex = i;
      }
    }
    return best != null ? new GridSearchResult(best, bestIndex, steps) : null;
  }

  /** Score a candidate alpha on the program deadlines. */
  private static AlphaCandidate scoreAlpha(
      double alpha,
      List<Double> sortedDistinctDeadlinesMs,
      double minDeadlineMs,
      double maxDeadlineMs) {
    if (!(alpha > 0.0) || !Double.isFinite(alpha)) return null;
    if (sortedDistinctDeadlinesMs == null || sortedDistinctDeadlinesMs.size() < 2) return null;

    final double expMinusAlphaDmin = Math.exp(-alpha * minDeadlineMs);
    final double expMinusAlphaDmax = Math.exp(-alpha * maxDeadlineMs);
    final double denom = expMinusAlphaDmin - expMinusAlphaDmax;
    if (denom == 0.0 || !Double.isFinite(denom)) return null;

    final double K = 96.0 / denom;
    final double P = 98.0 - 96.0 * expMinusAlphaDmin / denom;

    final Set<Integer> used = new HashSet<>();
    for (double d : sortedDistinctDeadlinesMs) {
      final double g = K * Math.exp(-alpha * d) + P;
      final int p = (int) Math.round(g);
      used.add(p);
    }

    final int m = sortedDistinctDeadlinesMs.size();
    final int distinctPriorities = used.size();
    final int collisions = m - distinctPriorities;
    return new AlphaCandidate(alpha, collisions);
  }

  /** Format a finite double as a C floating literal (decimal, US locale). */
  private static String formatDoubleForC(double value) {
    if (!Double.isFinite(value)) {
      throw new IllegalArgumentException(
          "Non-finite double cannot be emitted as C literal: " + value);
    }
    return String.format(Locale.US, "%.17g", value);
  }
}
