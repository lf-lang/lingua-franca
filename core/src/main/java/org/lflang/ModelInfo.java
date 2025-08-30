package org.lflang;

import static org.eclipse.xtext.xbase.lib.IterableExtensions.filter;
import static org.eclipse.xtext.xbase.lib.IteratorExtensions.toIterable;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.graph.InstantiationGraph;
import org.lflang.lf.Assignment;
import org.lflang.lf.Deadline;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Model;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reactor;
import org.lflang.target.Target;
import org.lflang.util.FileUtil;

/**
 * A helper class for analyzing the AST. This class is instantiated once for each compilation.
 *
 * NOTE: the validator used on imported files uses the same instance! Hence, this class should
 * not contain any info specific to any particular resource that is involved in the compilation.
 *
 * @author Marten Lohstroh
 * @ingroup Infrastructure
 */
public class ModelInfo {

  /**
   * Data structure for tracking dependencies between reactor classes. An instantiation of class A
   * inside of class B implies that B depends on A.
   */
  public InstantiationGraph instantiationGraph;

  /** The AST that the info gathered in this class pertains to. */
  public Model model;

  /**
   * The set of assignments that assign a too-large constant to a parameter that is used to specify
   * a deadline. These assignments are to be reported during validation.
   */
  public Set<Assignment> overflowingAssignments;

  /** The set of deadlines that use a too-large constant to specify their time interval. */
  public Set<Deadline> overflowingDeadlines;

  /**
   * The set of parameters used to specify a deadline while having been assigned a default value the
   * is too large for this purpose. These parameters are to be reported during validation.
   */
  public Set<Parameter> overflowingParameters;

  /** Cycles found during topology analysis. */
  private Set<NamedInstance<?>> topologyCycles = new LinkedHashSet<NamedInstance<?>>();

  /** Whether or not the model information has been updated at least once. */
  public boolean updated;

  /**
   * Redo all analysis based on the given model.
   *
   * @param model The model to analyze.
   * @param reporter The reporter to use for reporting messages.
   */
  public void update(Model model, MessageReporter reporter) {
    this.updated = true;
    this.model = model;
    this.instantiationGraph = new InstantiationGraph(model, true);

    if (this.instantiationGraph.getCycles().size() == 0) {
      List<ReactorInstance> topLevelReactorInstances = new LinkedList<>();
      var main =
          model.getReactors().stream().filter(it -> it.isMain() || it.isFederated()).findFirst();
      if (main.isPresent()) {
        var inst = new ReactorInstance(main.get(), reporter);
        topLevelReactorInstances.add(inst);
      } else {
        model
            .getReactors()
            .forEach(it -> topLevelReactorInstances.add(new ReactorInstance(it, reporter)));
      }
      // don't store the graph into a field, only the cycles.
      for (ReactorInstance top : topLevelReactorInstances) {
        this.topologyCycles.addAll(top.getCycles());
      }
    }

    // may be null if the target is invalid
    var target = Target.forName(model.getTarget().getName()).orElse(null);

    // Perform C-specific traversals.
    if (target == Target.C) {
      this.collectOverflowingNodes();
    }

    checkCaseInsensitiveNameCollisions(model, reporter);
  }

  public void checkCaseInsensitiveNameCollisions(Model model, MessageReporter reporter) {
    var reactorNames = new HashSet<>();
    var bad = new ArrayList<>();
    for (var reactor : model.getReactors()) {
      var lowerName = getName(reactor).toLowerCase();
      if (reactorNames.contains(lowerName)) bad.add(lowerName);
      reactorNames.add(lowerName);
    }
    for (var badName : bad) {
      model.getReactors().stream()
          .filter(it -> getName(it).toLowerCase().equals(badName))
          .forEach(
              it ->
                  reporter
                      .at(it)
                      .error("Multiple reactors have the same name up to case differences."));
    }
  }

  private String getName(Reactor r) {
    return r.getName() != null
        ? r.getName()
        : FileUtil.nameWithoutExtension(FileUtil.toPath(model.eResource().getURI()));
  }

  public Set<NamedInstance<?>> topologyCycles() {
    return this.topologyCycles;
  }

  /**
   * Collect all assignments, deadlines, and parameters that can cause the time interval of a
   * deadline to overflow. In the C target, only 48 bits are allotted for deadline intervals, which
   * are specified in nanosecond precision.
   */
  private void collectOverflowingNodes() {

    this.overflowingAssignments = new HashSet<>();
    this.overflowingDeadlines = new HashSet<>();
    this.overflowingParameters = new HashSet<>();

    // Visit all deadlines in the model; detect possible overflow.
    for (var deadline : filter(toIterable(model.eAllContents()), Deadline.class)) {
      // If the time value overflows, mark this deadline as overflowing.
      if (isTooLarge(ASTUtils.getLiteralTimeValue(deadline.getDelay()))) {
        this.overflowingDeadlines.add(deadline);
      }

      // If any of the upstream parameters overflow, report this deadline.
      final var delay = deadline.getDelay();
      if (delay instanceof ParameterReference
          && detectOverflow(
              new HashSet<>(), ((ParameterReference) deadline.getDelay()).getParameter())) {
        this.overflowingDeadlines.add(deadline);
      }
    }
  }

  /**
   * In the C target, only 48 bits are allotted for deadline intervals, which are specified in
   * nanosecond precision. Check whether the given time value exceeds the maximum specified value.
   *
   * @return true if the time value is greater than the specified maximum, false otherwise.
   */
  private boolean isTooLarge(TimeValue time) {
    return time != null && time.toNanoSeconds() > TimeValue.MAX_LONG_DEADLINE;
  }

  /**
   * Given a parameter that is used in a deadline specification, recursively track down its
   * definition and check whether it is overflowing. Also detect and report overrides that are
   * overflowing.
   *
   * @return true if there exists a parameter corresponding to a value that does not fit in the
   *     available bits.
   */
  private boolean detectOverflow(Set<Instantiation> visited, Parameter current) {
    var overflow = false;

    // Determine whether the parameter's default value overflows or not.
    if (isTooLarge(ASTUtils.getDefaultAsTimeValue(current))) {
      this.overflowingParameters.add(current);
      overflow = true;
    }

    // Iterate over the instantiations of the reactor in which the
    // current parameter was found.
    Set<Instantiation> instantiations =
        this.instantiationGraph.getInstantiations((Reactor) current.eContainer());
    for (var instantiation : instantiations) {
      // Only visit each instantiation once per deadline to avoid cycles.
      if (!visited.contains(instantiation)) {
        visited.add(instantiation);
        // Find assignments that override the current parameter.
        for (var assignment : instantiation.getParameters()) {
          if (assignment.getLhs().equals(current)) {
            Expression expr = assignment.getRhs().getExpr();
            if (expr instanceof ParameterReference) {
              // Check for overflow in the referenced parameter.
              overflow =
                  detectOverflow(visited, ((ParameterReference) expr).getParameter()) || overflow;
            } else {
              // The right-hand side of the assignment is a
              // constant; check whether it is too large.
              if (isTooLarge(ASTUtils.getLiteralTimeValue(expr))) {
                this.overflowingAssignments.add(assignment);
                overflow = true;
              }
            }
          }
        }
      }
    }
    return overflow;
  }
}
