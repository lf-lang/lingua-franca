/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang;

import static org.eclipse.xtext.xbase.lib.IterableExtensions.filter;
import static org.eclipse.xtext.xbase.lib.IteratorExtensions.toIterable;

import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

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
import org.lflang.lf.STP;


/**
 * A helper class for analyzing the AST. This class is instantiated once for each compilation.
 * <p>
 * NOTE: the validator used on imported files uses the same instance! Hence, this class should not contain any info
 * specific to any particular resource that is involved in the compilation.
 *
 * @author Marten Lohstroh
 */
public class ModelInfo {

    /**
     * Data structure for tracking dependencies between reactor classes. An
     * instantiation of class A inside of class B implies that B depends on A.
     */
    public InstantiationGraph instantiationGraph;

    /**
     * The AST that the info gathered in this class pertains to.
     */
    public Model model;

    /**
     * The set of assignments that assign a too-large constant to a parameter
     * that is used to specify a deadline. These assignments are to be reported
     * during validation.
     */
    public Set<Assignment> overflowingAssignments;

    /**
     * The set of deadlines that use a too-large constant to specify their time
     * interval.
     */
    public Set<Deadline> overflowingDeadlines;
    
    /**
     * The set of STP offsets that use a too-large constant to specify their time
     * interval.
     */
    public Set<STP> overflowingSTP;

    /**
     * The set of parameters used to specify a deadline while having been
     * assigned a default value the is too large for this purpose. These
     * parameters are to be reported during validation.
     */
    public Set<Parameter> overflowingParameters;

    /** Cycles found during topology analysis. */
    private Set<NamedInstance<?>> topologyCycles = new LinkedHashSet<NamedInstance<?>>();

    /**
     * Whether or not the model information has been updated at least once.
     */
    public boolean updated;

    /**
     * Redo all analysis based on the given model.
     *
     * @param model the model to analyze.
     */
    public void update(Model model, ErrorReporter reporter) {
        this.updated = true;
        this.model = model;
        this.instantiationGraph = new InstantiationGraph(model, true);

        if (this.instantiationGraph.getCycles().size() == 0) {
            List<ReactorInstance> topLevelReactorInstances = new LinkedList<>();
            var main = model.getReactors().stream().filter(it -> it.isMain() || it.isFederated()).findFirst();
            if (main.isPresent()) {
                var inst = new ReactorInstance(main.get(), reporter);
                topLevelReactorInstances.add(inst);
            } else {
                model.getReactors().forEach(
                    it -> topLevelReactorInstances.add(new ReactorInstance(it, reporter))
                );
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
    }

    public Set<NamedInstance<?>> topologyCycles() {
        return this.topologyCycles;
    }

    /**
     * Collect all assignments, deadlines, and parameters that can cause the
     * time interval of a deadline to overflow. In the C target, only 48 bits
     * are allotted for deadline intervals, which are specified in nanosecond
     * precision.
     */
    private void collectOverflowingNodes() {

        this.overflowingAssignments = new HashSet<>();
        this.overflowingDeadlines = new HashSet<>();
        this.overflowingParameters = new HashSet<>();
        this.overflowingSTP = new HashSet<>();

        // Visit all deadlines in the model; detect possible overflow.
        for (var deadline : filter(toIterable(model.eAllContents()), Deadline.class)) {
            // If the time value overflows, mark this deadline as overflowing.
            if (isTooLarge(ASTUtils.getLiteralTimeValue(deadline.getDelay()))) {
                this.overflowingDeadlines.add(deadline);
            }

            // If any of the upstream parameters overflow, report this deadline.
            final var delay = deadline.getDelay();
            if (delay instanceof ParameterReference
                && detectOverflow(new HashSet<>(), ((ParameterReference) deadline.getDelay()).getParameter())) {
                this.overflowingDeadlines.add(deadline);
            }
        }
        // Visit all STP offsets in the model; detect possible overflow.
        for (var stp : filter(toIterable(model.eAllContents()), STP.class)) {
            // If the time value overflows, mark this deadline as overflowing.
            if (isTooLarge(ASTUtils.getLiteralTimeValue(stp.getValue()))) {
                this.overflowingSTP.add(stp);
            }
        }
    }

    /**
     * In the C target, only 48 bits are allotted for deadline intervals, which
     * are specified in nanosecond precision. Check whether the given time value
     * exceeds the maximum specified value.
     *
     * @return true if the time value is greater than the specified maximum,
     * false otherwise.
     */
    private boolean isTooLarge(TimeValue time) {
        return time != null && time.toNanoSeconds() > TimeValue.MAX_LONG_DEADLINE;
    }

    /**
     * Given a parameter that is used in a deadline specification, recursively
     * track down its definition and check whether it is overflowing. Also
     * detect and report overrides that are overflowing.
     * @return true if there exists a parameter corresponding to a value that
     * does not fit in the available bits.
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
        Set<Instantiation> instantiations = this.instantiationGraph.getInstantiations((Reactor) current.eContainer());
        for (var instantiation : instantiations) {
            // Only visit each instantiation once per deadline to avoid cycles.
            if (!visited.contains(instantiation)) {
                visited.add(instantiation);
                // Find assignments that override the current parameter.
                for (var assignment : instantiation.getParameters()) {
                    if (assignment.getLhs().equals(current)) {
                        if (assignment.getRhs().getExprs().isEmpty()) continue;  // This error should be caught elsewhere.
                        Expression expr = ASTUtils.asSingleExpr(assignment.getRhs());
                        if (expr instanceof ParameterReference) {
                            // Check for overflow in the referenced parameter.
                            overflow = detectOverflow(visited, ((ParameterReference)expr).getParameter()) || overflow;
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
