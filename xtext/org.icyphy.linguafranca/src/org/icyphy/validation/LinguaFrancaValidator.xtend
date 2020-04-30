/* Validation checks for Lingua Franca code. */

/*************
 * Copyright (c) 2019, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.icyphy.validation

import java.util.Arrays
import org.eclipse.emf.ecore.EStructuralFeature
import org.eclipse.xtext.validation.Check
import org.icyphy.AnnotatedNode
import org.icyphy.ModelInfo
import org.icyphy.Targets
import org.icyphy.Targets.BuildTypes
import org.icyphy.Targets.LoggingLevels
import org.icyphy.Targets.TargetProperties
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Deadline
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.KeyValuePair
import org.icyphy.linguaFranca.LinguaFrancaPackage.Literals
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Preamble
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.Visibility

import static extension org.icyphy.ASTUtils.*

/**
 * Custom validation checks for Lingua Franca programs.
 *  
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author(Christian Menard <christian.menard@tu-dresden.de>}
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#validation
 */
class LinguaFrancaValidator extends AbstractLinguaFrancaValidator {

    var reactorClasses = newHashSet()
    var parameters = newHashSet()
    var inputs = newHashSet()
    var outputs = newHashSet()
    var timers = newHashSet()
    var actions = newHashSet()
    var allNames = newHashSet() // Names of contained objects must be unique.
    var Targets target;

    var info = new ModelInfo()

    // //////////////////////////////////////////////////
    // // Helper functions for checks to be performed on multiple entities
    // Check the name of a feature for illegal substrings.
    private def checkName(String name, EStructuralFeature feature) {

        // Raises an error if the string starts with two underscores.
        if (name.length() >= 2 && name.substring(0, 2).equals("__")) {
            error(UNDERSCORE_MESSAGE + name, feature)
        }

        if (this.target.keywords.contains(name)) {
            error(RESERVED_MESSAGE + name, feature)
        }

        if (this.target == Targets.TS) {
            // "actions" is a reserved word within a TS reaction
            if (name.equals("actions")) {
                error(ACTIONS_MESSAGE + name, feature)
            }
        }

    }

    // //////////////////////////////////////////////////
    // // Functions to set up data structures for performing checks.
    // FAST ensures that these checks run whenever a file is modified.
    // Alternatives are NORMAL (when saving) and EXPENSIVE (only when right-click, validate).
    @Check(FAST)
    def reset(Model model) {
        reactorClasses.clear()
    }

    @Check(FAST)
    def resetSets(Reactor reactor) {
        parameters.clear()
        inputs.clear()
        outputs.clear()
        timers.clear()
        actions.clear()
        allNames.clear()
    }

    // //////////////////////////////////////////////////
    // // The following checks are in alphabetical order.
    @Check(FAST)
    def checkAction(Action action) {
        checkName(action.name, Literals.VARIABLE__NAME)
        if (action.origin == ActionOrigin.NONE) {
            error(
                "Action must have modifier `logical` or `physical`.",
                Literals.ACTION__ORIGIN
            )
        }

        if (allNames.contains(action.name)) {
            error(
                UNIQUENESS_MESSAGE + action.name,
                Literals.VARIABLE__NAME
            )
        }
        actions.add(action.name);
        allNames.add(action.name)
    }

    @Check(FAST)
    def checkAssignment(Assignment assignment) {
        // If the left-hand side is a time parameter, make sure the assignment has units
        if (assignment.lhs.isOfTimeType) {
            if (assignment.rhs.size > 1) {
                 error("Incompatible type.", Literals.ASSIGNMENT__RHS)
            } else {
                val v = assignment.rhs.get(0)
                if (!v.isValidTime) {
                    if (v.parameter === null) {
                        // This is a value. Check that units are present.
                    error(
                        "Invalid time units: " + assignment.rhs +
                            ". Should be one of " + TimeUnit.VALUES.filter [
                                it != TimeUnit.NONE
                            ], Literals.ASSIGNMENT__RHS)
                    } else {
                        // This is a reference to another parameter. Report problem.
                error(
                    "Cannot assign parameter: " +
                        v.parameter.name + " to " +
                        assignment.lhs.name +
                        ". The latter is a time parameter, but the former is not.",
                    Literals.ASSIGNMENT__RHS)
                    }
                }
            }
            // If this assignment overrides a parameter that is used in a deadline,
            // report possible overflow.
            if (this.target == Targets.C &&
                this.info.overflowingAssignments.contains(assignment)) {
                error(
                    "Time value used to specify a deadline exceeds the maximum of " +
                        TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                    Literals.ASSIGNMENT__RHS)
            }
        }
        // FIXME: lhs is list => rhs is list
        // lhs is fixed with size n => rhs is fixed with size n
        // FIXME": similar checks for decl/init
        // Specifically for C: list can only be literal or time lists
    }

    @Check(FAST)
    def checkConnection(Connection connection) {
        var reactor = connection.eContainer as Reactor
        for (reaction : reactor.reactions) {
            for (effect : reaction.effects) {
                if (connection.rightPort.container === effect.container &&
                    connection.rightPort.variable === effect.variable) {
                    error(
                        "Cannot connect: Port named '" + effect.variable.name +
                            "' is already effect of a reaction.",
                        Literals.CONNECTION__RIGHT_PORT)
                }
            }
        }

        for (c : reactor.connections) {
            if (c !== connection &&
                connection.rightPort.container === c.rightPort.container &&
                connection.rightPort.variable === c.rightPort.variable) {
                error(
                    "Cannot connect: Port named '" + c.rightPort.variable.name +
                        "' may only be connected to a single upstream port.",
                    Literals.CONNECTION__RIGHT_PORT)
            }
        }
    }

    @Check(FAST)
    def checkDeadline(Deadline deadline) {
        if (this.target == Targets.C &&
            this.info.overflowingDeadlines.contains(deadline)) {
            error(
                "Deadline exceeds the maximum of " +
                    TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                Literals.DEADLINE__DELAY)
        }
    }

    @Check(FAST)
    def checkInput(Input input) {
        checkName(input.name, Literals.VARIABLE__NAME)
        if (allNames.contains(input.name)) {
            error(
                UNIQUENESS_MESSAGE + input.name,
                Literals.VARIABLE__NAME
            )
        }
        inputs.add(input.name)
        allNames.add(input.name)
        if (target.requiresTypes) {
            if (input.type === null) {
                error("Input must have a type.", Literals.PORT__TYPE)
            }
        }
        
        // mutable has no meaning in C++
        if (input.mutable && this.target == Targets.CPP) {
            warning(
                "The mutable qualifier has no meaning for the C++ target and should be removed. " +
                "In C++, any value can be made mutable by calling get_mutable_copy().",
                Literals.INPUT__MUTABLE
            )
        }
    }

    @Check(FAST)
    def checkInstantiation(Instantiation instantiation) {
        checkName(instantiation.name, Literals.INSTANTIATION__NAME)
        if (allNames.contains(instantiation.name)) {
            error(
                UNIQUENESS_MESSAGE + instantiation.name,
                Literals.INSTANTIATION__NAME
            )
        }
        allNames.add(instantiation.name)
        if (instantiation.reactorClass.isMain || instantiation.reactorClass.isFederated) {
            error(
                "Cannot instantiate a main (or federated) reactor: " +
                    instantiation.reactorClass.name,
                Literals.INSTANTIATION__REACTOR_CLASS
            )
        }
        
        // Report error if this instantiation is part of a cycle.
        if (this.info.instantiationGraph.cycles.size > 0) {
            for (cycle : this.info.instantiationGraph.cycles) {
                val instance = new AnnotatedNode(instantiation.reactorClass)
                val reactor = new AnnotatedNode(
                    instantiation.eContainer as Reactor)
                if (cycle.contains(reactor) && cycle.contains(instance)) {
                    error(
                        "Instantiation is part of a cycle: " +
                            instantiation.reactorClass.name,
                        Literals.INSTANTIATION__REACTOR_CLASS
                    )
                }
            }
        }
    }

    /** Check target parameters, which are key-value pairs. */
    @Check(FAST)
    def checkKeyValuePair(KeyValuePair param) {
        // Check only if the container's container is a Target.
        if (param.eContainer.eContainer instanceof Target) {

            if (!TargetProperties.isValidName(param.name)) {
                warning(
                    "Unrecognized target parameter: " + param.name +
                        ". Recognized parameters are " +
                        TargetProperties.values().join(", "),
                    Literals.KEY_VALUE_PAIR__NAME)
            }
            val prop = TargetProperties.get(param.name)

            if (!prop.supportedBy.contains(this.target)) {
                warning(
                    "The target parameter: " + param.name +
                        " is not supported by the " + this.target +
                        " and will thus be ignored.",
                    Literals.KEY_VALUE_PAIR__NAME)
            }

            switch prop {
                case BUILD_TYPE:
                    if (!Arrays.asList(BuildTypes.values()).exists [
                        it.toString.equals(param.value.id)
                    ]) {
                        error(
                            "Target property build-type is required to be one of " +
                                BuildTypes.values(),
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case CMAKE_INCLUDE:
                    if (param.value.literal === null) {
                        error(
                            "Target property cmake-include is required to be a string.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case COMPILER:
                    if (param.value.literal === null) {
                        error(
                            "Target property compile is required to be a string.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case COORDINATION:
                    if (param.value.id.isNullOrEmpty || 
                        !(param.value.id.equals("centralized") || 
                        param.value.id.equals("distributed"))) {
                        error("Target property 'coordination' can either be " +
                            "'centralized' or 'distributed'.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    } 
                case FLAGS:
                    if (param.value.literal === null) {
                        error(
                            "Target property flags is required to be a string.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case FAST:
                    if (!param.value.id.equals('true') &&
                        !param.value.id.equals('false')) {
                        error(
                            "Target property fast is required to be true or false.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case KEEPALIVE:
                    if (!param.value.id.equals('true') &&
                        !param.value.id.equals('false')) {
                        error(
                            "Target property keepalive is required to be true or false.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case LOGGING:
                    if (!Arrays.asList(LoggingLevels.values()).exists [
                        it.toString.equals(param.value.id)
                    ]) {
                        error(
                            "Target property logging is required to be one of " +
                                LoggingLevels.values(),
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case NO_COMPILE:
                    if (!param.value.id.equals('true') &&
                        !param.value.id.equals('false')) {
                        error(
                            "Target property no-compile is required to be true or false.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case NO_RUNTIME_VALIDATION:
                    if (!param.value.id.equals('true') &&
                        !param.value.id.equals('false')) {
                        error(
                            "Target property no-runtime-validation is required to be true or false.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                case THREADS: {
                    if (param.value.literal === null) {
                        error(
                            "Target property threads is required to be a non-negative integer.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                    try {
                        val value = Integer.decode(param.value.literal)
                        if (value < 0) {
                            error(
                                "Target property threads is required to be a non-negative integer.",
                                Literals.KEY_VALUE_PAIR__VALUE)
                        }
                    } catch (NumberFormatException ex) {
                        error(
                            "Target property threads is required to be a non-negative integer.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
                }
                case TIMEOUT:
                    if (param.value.unit === null) {
                        error(
                            "Target property timeout requires a time unit. Should be one of " +
                                TimeUnit.VALUES.filter[it != TimeUnit.NONE],
                            Literals.KEY_VALUE_PAIR__VALUE)
                    } else if (param.value.time < 0) {
                        error(
                            "Target property timeout requires a non-negative time value with units.",
                            Literals.KEY_VALUE_PAIR__VALUE)
                    }
            }
        }
    }

    @Check(FAST)
    def checkOutput(Output output) {
        checkName(output.name, Literals.VARIABLE__NAME)
        if (allNames.contains(output.name)) {
            error(
                UNIQUENESS_MESSAGE + output.name,
                Literals.VARIABLE__NAME
            )
        }
        outputs.add(output.name);
        allNames.add(output.name)
        if (this.target.requiresTypes) {
            if (output.type === null) {
                error("Output must have a type.", Literals.PORT__TYPE)
            }
        }
    }

    @Check(NORMAL)
    def checkModel(Model model) {
        info.update(model)
    }

    @Check(FAST)
    def checkParameter(Parameter param) {
        checkName(param.name, Literals.PARAMETER__NAME)
        if (allNames.contains(param.name)) {
            error(
                UNIQUENESS_MESSAGE + param.name,
                Literals.PARAMETER__NAME
            )
        }
        parameters.add(param.name)
        allNames.add(param.name)

        if (param.init.exists[it.parameter !== null]) {
            // Initialization using parameters is forbidden.
            error("Parameter cannot be initialized using parameter.",
                Literals.PARAMETER__INIT)
        }
        
        if (param.init === null || param.init.size == 0) {
            // All parameters must be initialized.
            error("Uninitialized parameter.", Literals.PARAMETER__INIT)
        } else if (param.isOfTimeType) {
             // We do additional checks on types because we can make stronger
             // assumptions about them.
             
             // If the parameter is not a list, cannot be initialized
             // using a one.
             if (param.init.size > 1 && param.type.arraySpec === null) {
                error("Time parameter cannot be initialized using a list.",
                    Literals.PARAMETER__INIT)
            } else {
                // The parameter is a singleton time.
                val init = param.init.get(0)
                if (init.time === null) {
                    if (init !== null && !init.isZero) {
                        if (init.isInteger) {
                            error("Missing time units. Should be one of " +
                                TimeUnit.VALUES.filter [
                                    it != TimeUnit.NONE
                                ], Literals.PARAMETER__INIT)
                        } else {
                            error("Invalid time literal.",
                                Literals.PARAMETER__INIT)
                        }
                    }
                } // If time is not null, we know that a unit is also specified.    
            }
        } else if (this.target.requiresTypes) {
            // Report missing target type.
            if (param.inferredType.isUndefined()) {
                error("Type declaration missing.", Literals.PARAMETER__TYPE)
            }
        }

        if (this.target == Targets.C &&
            this.info.overflowingParameters.contains(param)) {
            error(
                "Time value used to specify a deadline exceeds the maximum of " +
                    TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                Literals.PARAMETER__INIT)
        }
    }

    @Check(FAST)
    def checkPreamble(Preamble preamble) {
        if (this.target == Targets.CPP && preamble.visibility == Visibility.NONE) {
            error(
                "Preambles for the C++ target need a visibility qualifier (private or public)!",
                Literals.PREAMBLE__VISIBILITY
            )
        } else if (this.target != Targets.CPP && preamble.visibility != Visibility.NONE) {
            warning(
                '''The «preamble.visibility» qualifier has no meaning for the «this.target.name» target. It should be removed.''',
                Literals.PREAMBLE__VISIBILITY
            )
        }
    }

	@Check(FAST)
	def checkReaction(Reaction reaction) {
		if (reaction.triggers === null || reaction.triggers.size == 0){
			warning("Reaction has no trigger.", Literals.REACTION__TRIGGERS)
		}
	}

    @Check(FAST)
    def checkReactor(Reactor reactor) {
        checkName(reactor.name, Literals.REACTOR__NAME)
        if (reactorClasses.contains(reactor.name)) {
            error(
                "Names of reactor classes must be unique: " + reactor.name,
                Literals.REACTOR__NAME
            )
        }
        reactorClasses.add(reactor.name);
        
        // C++ reactors may not be called 'preamble'
        if (this.target == Targets.CPP && reactor.name.equalsIgnoreCase("preamble")) {
            error(
                "Reactor cannot be named '" + reactor.name + "'",
                Literals.REACTOR__NAME
            )
        }
        
        if (!reactor.isFederated && !reactor.host.isNullOrEmpty) {
            error(
                "Cannot assign a host to reactor '" + reactor.name + "' because it is not federated.",
                Literals.REACTOR__HOST
            )
        }
        // FIXME: In TypeScript, there are certain classes that a reactor class should not collide with
        // (essentially all the classes that are imported by default).
    }

    @Check(FAST)
    def checkState(StateVar stateVar) {
        checkName(stateVar.name, Literals.STATE_VAR__NAME)
        if (allNames.contains(stateVar.name)) {
            error(
                UNIQUENESS_MESSAGE + stateVar.name,
                Literals.STATE_VAR__NAME
            )
        }
        inputs.add(stateVar.name);
        allNames.add(stateVar.name)

        if (stateVar.isOfTimeType) {
            // If the state is declared to be a time,
            // make sure that it is initialized correctly.
            if (stateVar.init !== null) {
                for (init : stateVar.init) {
                    if (stateVar.type !== null && stateVar.type.isTime &&
                        !init.isValidTime) {
                        if (stateVar.isParameterized) {
                            error(
                                "Referenced parameter does not denote a time.",
                                Literals.STATE_VAR__INIT)
                        } else {
                            if (init !== null && !init.isZero) {
                                if (init.isInteger) {
                                    error(
                                        "Missing time units. Should be one of " +
                                            TimeUnit.VALUES.filter [
                                                it != TimeUnit.NONE
                                            ], Literals.STATE_VAR__INIT)
                                } else {
                                    error("Invalid time literal.",
                                        Literals.STATE_VAR__INIT)
                                }
                            }
                        }
                    }
                }
            }
        } else if (this.target.requiresTypes && stateVar.inferredType.isUndefined) {
            // Report if a type is missing
            error("State must have a type.", Literals.STATE_VAR__TYPE)
        }

        if (this.target == Targets.C && stateVar.init.size > 1) {
            // In C, if initialization is done with a list, elements cannot
            // refer to parameters.
            if (stateVar.init.exists[it.parameter !== null]) {
                error("List items cannot refer to a parameter.",
                    Literals.STATE_VAR__INIT)
            }
        }

    }

    @Check(FAST)
    def checkTarget(Target target) {
        if (!Targets.isValidName(target.name)) {
            warning("Unrecognized target: " + target.name,
                Literals.TARGET__NAME)
        } else {
            this.target = Targets.get(target.name);
        }
    }

    @Check(FAST)
    def checkValueAsTime(Value value) {
        val container = value.eContainer

        if (container instanceof Timer || container instanceof Action ||
            container instanceof Connection || container instanceof Deadline) {

            // If parameter is referenced, check that it is of the correct type.
            if (value.parameter !== null) {
                if (!value.parameter.isOfTimeType) {
                    error("Parameter is not of time type",
                        Literals.VALUE__PARAMETER)
                }
            } else if (value.time === null) {
                if (value.literal !== null && !value.literal.isZero) {
                    if (value.literal.isInteger) {
                            error("Missing time units. Should be one of " +
                                TimeUnit.VALUES.filter [
                                    it != TimeUnit.NONE
                                ], Literals.VALUE__LITERAL)
                        } else {
                            error("Invalid time literal.",
                                Literals.VALUE__LITERAL)
                        }
                } else if (value.code !== null && !value.code.isZero) {
                    if (value.code.isInteger) {
                            error("Missing time units. Should be one of " +
                                TimeUnit.VALUES.filter [
                                    it != TimeUnit.NONE
                                ], Literals.VALUE__CODE)
                        } else {
                            error("Invalid time literal.",
                                Literals.VALUE__CODE)
                        }
                }
            }
        }
    }
    
    @Check(FAST)
    def checkTimer(Timer timer) {
        checkName(timer.name, Literals.VARIABLE__NAME)
        if (allNames.contains(timer.name)) {
            error(
                UNIQUENESS_MESSAGE + timer.name,
                Literals.VARIABLE__NAME
            )
        }
        timers.add(timer.name);
        allNames.add(timer.name)
    }
    
    @Check(FAST)
    def checkType(Type type) {
        // FIXME: disallow the use of generics in C
        if (this.target == Targets.CPP) {
            if (type.stars.size > 0) {
                warning(
                    "Raw pointers should be avoided in conjunction with LF. Ports " +
                    "and actions implicitly use smart pointers. In this case, " +
                    "the pointer here is likely not needed. For parameters and state " +
                    "smart pointers should be used explicitly if pointer semantics " +
                    "are really needed.",
                    Literals.TYPE__STARS
                )
            }
        }
    }

    static val UNIQUENESS_MESSAGE = "Names of contained objects (inputs, outputs, actions, timers, parameters, state, and reactors) must be unique: "
    static val UNDERSCORE_MESSAGE = "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": "
    static val ACTIONS_MESSAGE = "\"actions\" is a reserved word for the TypeScript target for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation): "
    static val RESERVED_MESSAGE = "Reserved words in the target language are not allowed for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation): "

}
