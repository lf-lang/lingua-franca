/* A helper class for analyzing the AST. */

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

package org.icyphy

import java.util.HashMap
import java.util.HashSet
import java.util.Set
import org.icyphy.graph.InstantiationGraph
import org.icyphy.graph.ReactionGraph
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Deadline
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Target

import static extension org.icyphy.ASTUtils.*

/**
 * A helper class for analyzing the AST.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ModelInfo {

    /**
     * Data structure for tracking dependencies between reactor classes. An 
     * instantiation of class A inside of class B implies that B depends on A.
     */
    public InstantiationGraph instantiationGraph
    
    /**
     * A mapping from reactors to the sites of their instantiation.
     */
    public HashMap<Reactor, Set<Instantiation>> instantiationMap = new HashMap()

    /**
     * The AST that the info gather in this class pertains to.
     */
    public Model model

    /**
     * The set of assignments that assign a too-large constant to a parameter
     * that is used to specify a deadline. These assignments are to be reported
     * during validation.
     */
    public Set<Assignment> overflowingAssignments

    /**
     * The set of deadlines that use a too-large constant to specify their time 
     * interval.
     */
    public Set<Deadline> overflowingDeadlines

    /**
     * The set of parameters used to specify a deadline while having been
     * assigned a default value the is too large for this purpose. These
     * parameters are to be reported during validation.
     */
    public Set<Parameter> overflowingParameters

    /**
     * Data structure for tracking dependencies between reactions.
     */
    public ReactionGraph reactionGraph
    
    // FIXME: If a there is a cyclic instantiation somewhere inside an imported file, report an error
    // on the import statement; establish a mapping for this.

    /**
     * Redo all analysis based on the given model.
     * @param model the model to analyze.
     */
    def update(Model model) {
        this.model = model
        
        this.instantiationGraph = new InstantiationGraph(model, true)
        
        if (this.instantiationGraph.cycles.size == 0) {
            this.reactionGraph = new ReactionGraph(this.model)    
        }
        
        // Find the target. A target must exist because the grammar requires it.
        var Targets target
        for (t : model.eAllContents.toIterable.filter(Target)) {
            target = Targets.get(t.name)
        }
        
        // Perform C-specific traversals.
        if (target == Targets.C) {
            this.collectOverflowingNodes()
        }
    }

    /**
     * Collect all assignments, deadlines, and parameters that can cause the
     * time interval of a deadline to overflow. In the C target, only 16 bits
     * are allotted for deadline intervals, which are specified in nanosecond
     * precision.
     */
    private def collectOverflowingNodes() {

        this.overflowingAssignments = new HashSet()
        this.overflowingDeadlines = new HashSet()
        this.overflowingParameters = new HashSet()

        // Visit all deadlines in the model; detect possible overflow.
        for (deadline : model.eAllContents.toIterable.filter(Deadline)) {
            // If the time value overflows, mark this deadline as overflowing.
            if (isTooLarge(deadline.delay.getTimeValue)) {
                this.overflowingDeadlines.add(deadline)
            }

            // If any of the upstream parameters overflow, report this deadline.
            if (detectOverflow(new HashSet<Instantiation>(),
                deadline.delay.parameter)) {
                this.overflowingDeadlines.add(deadline)
            }
        }
    }

    /**
     * In the C target, only 16 bits are allotted for deadline intervals, which
     * are specified in nanosecond precision. Check whether the given time value
     * exceeds the maximum specified value.
     * @return true if the time value is greater than the specified maximum,
     * false otherwise.
     */
    private def boolean isTooLarge(TimeValue time) {
        if (time !== null && time.toNanoSeconds > TimeValue.MAX_BIGINT_DEADLINE)
            true
        else
            false
    }
    
    /**
     * Given a parameter that is used in a deadline specification, recursively
     * track down its definition and check whether it is overflowing. Also 
     * detect and report overrides that are overflowing.
     */
    private def boolean detectOverflow(Set<Instantiation> visited,
        Parameter current) {
        var overflow = false

        // Determine whether the parameter's default value overflows or not.
        if (isTooLarge(current.getInitialTimeValue)) {
            this.overflowingParameters.add(current)
            overflow = true
        }

        // Iterate over the instantiations of the reactor in which the
        // current parameter was found.
        for (instantiation : this.instantiationMap.get(
            current.eContainer as Reactor) ?: emptySet) {
            // Only visit each instantiation once per deadline to avoid cycles.
            if (!visited.contains(instantiation)) {
                visited.add(instantiation)
                // Find assignments that override the current parameter.
                for (assignment : instantiation.parameters) {
                    if (assignment.lhs.equals(current)) {
                        if (assignment.rhs.get(0).parameter !== null) {
                            // Check for overflow in the referenced parameter.
                            overflow = detectOverflow(visited,
                                assignment.rhs.get(0).parameter) || overflow
                        } else {
                            // The right-hand side of the assignment is a 
                            // constant; check whether it is too large.
                            if (isTooLarge(
                                assignment.rhs.get(0).getTimeValue)) {
                                this.overflowingAssignments.add(assignment)
                                overflow = true
                            }
                        }
                    }
                }
            }
        }
        return overflow
    }
}
