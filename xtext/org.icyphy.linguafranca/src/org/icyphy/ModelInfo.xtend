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
    public AnnotatedDependencyGraph<Instantiation> instantiationGraph

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
