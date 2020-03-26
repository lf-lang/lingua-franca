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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy

import java.util.HashMap
import java.util.HashSet
import java.util.Set
import org.icyphy.linguaFranca.Deadline
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Assignment

/**
 * A helper class for analyzing the AST.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ModelInfo {

    /**
     * 
     */
    public AnnotatedDependencyGraph<Reactor> instantiationGraph

    /**
     * 
     */
    public HashMap<Reactor, Set<Instantiation>> instantiationMap = new HashMap()

    /**
     * 
     */
    public Model model

    /**
     * 
     */
    public Set<Assignment> overflowingAssignments

    /**
     * 
     */
    public Set<Deadline> overflowingDeadlines

    /**
     * 
     */
    public Set<Parameter> overflowingParameters

    /**
     * 
     */
    def update(Model model) {
        this.model = model
        this.refreshInstantiationMap()
        this.collectOverflowingNodes()
        this.refreshInstantiationGraph()
    }
    
    /**
     * 
     */
    private def refreshInstantiationMap() {
        this.instantiationMap = new HashMap()
        for (instantiation : model.eAllContents.toIterable.filter(
            Instantiation)) {
            var set = this.instantiationMap.get(instantiation.reactorClass)
            if (set === null)
                set = new HashSet<Instantiation>()
            set.add(instantiation)
            this.instantiationMap.put(instantiation.reactorClass, set)
        }
    }

    /**
     * 
     */
    private def refreshInstantiationGraph() {
        this.instantiationGraph = new AnnotatedDependencyGraph()
        for (instantiation : this.model.eAllContents.toIterable.filter(
            Instantiation)) {
            this.instantiationGraph.addEdge(
                new AnnotatedNode(instantiation.eContainer as Reactor),
                new AnnotatedNode(instantiation.reactorClass))
        }
        this.instantiationGraph.detectCycles()
    }

    /**
     * 
     */
    private def collectOverflowingNodes() {

        this.overflowingAssignments = new HashSet()
        this.overflowingDeadlines = new HashSet()
        this.overflowingParameters = new HashSet()

        // Visit all deadlines in the model and determine whether they could overflow.
        for (deadline : model.eAllContents.toIterable.filter(Deadline)) {
            // If the time value overflows, mark this deadline as overflowing.
            if (deadline.time.unit != TimeUnit.NONE && isTooLarge(
                new TimeValue(deadline.time.time, deadline.time.unit))) {
                this.overflowingDeadlines.add(deadline)
            }

            // If any of the upstream parameters overflow, mark this deadline as overflowing.
            if (detectOverflow(new HashSet<Instantiation>(),
                deadline.time.parameter)) {
                this.overflowingDeadlines.add(deadline)
            }
        }
    }

    /**
     * 
     */
    private def boolean isTooLarge(TimeValue time) {
        if (time.toNanoSeconds > TimeValue.MAX_BIGINT_DEADLINE)
            true
        else
            false
    }
    
    /**
     * 
     */
    private def boolean detectOverflow(Set<Instantiation> visited,
        Parameter current) {
        var overflow = false

        // Determine whether the parameter's default value overflows or not.
        if (isTooLarge(new TimeValue(current.time, current.unit))) {
            this.overflowingParameters.add(current)
            overflow = true
        }

        // Iterate over the instantiations of the reactor in which the current parameter was found.
        for (instantiation : this.instantiationMap.get(
            current.eContainer as Reactor) ?: emptySet) {
            // Only visit each instantiation once per deadline (i.e., avoid cycles).
            if (!visited.contains(instantiation)) {
                visited.add(instantiation)
                // Find assignments that override the current parameter.
                for (assignment : instantiation.parameters) {
                    if (assignment.lhs.equals(current)) {
                        if (assignment.rhs.parameter !== null) {
                            // Go on to check for overflow in the referenced parameter.
                            overflow = detectOverflow(visited,
                                assignment.rhs.parameter) || overflow
                        } else {
                            // The right-hand side of the assignment is a constant that is too large.
                            if (isTooLarge(new TimeValue(assignment.rhs.time,
                                assignment.rhs.unit))) {
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
