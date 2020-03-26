/* Validation checks for Lingua Franca code. */

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
package org.icyphy.validation

import java.util.Arrays
import org.eclipse.emf.ecore.EStructuralFeature
import org.eclipse.xtext.validation.Check
import org.icyphy.AnnotatedNode
import org.icyphy.ModelInfo
import org.icyphy.Targets
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
import org.icyphy.linguaFranca.QueuingPolicy
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer

/**
 * This class contains custom validation rules.
 *  
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * 
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
        if (name.length() >= 2 && name.substring(0,2).equals("__")) {
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
        } else if (action.origin == ActionOrigin.LOGICAL && action.policy != QueuingPolicy.NONE) {
            error(
                "Logical action cannot specify a queuing policy.",
                Literals.ACTION__POLICY
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
            if (assignment.rhs.parameter === null) {
                // This is a value. Check that units are present
                if (assignment.rhs.unit == TimeUnit.NONE) {
                    error(
                        "Invalid time units: " + assignment.rhs.unit +
                            ". Should be one of " + TimeUnit.VALUES.filter [
                                it != TimeUnit.NONE
                            ], Literals.ASSIGNMENT__RHS)
                }
            } else {
                // This is a reference to another parameter.
                // Check that types match.
                if (!assignment.rhs.parameter.isOfTimeType) {
                    error(
                        "Cannot assign parameter: " +
                            assignment.rhs.parameter.name + " to " +
                            assignment.lhs.name +
                            ". The latter is a time parameter, but the former is not.",
                        Literals.ASSIGNMENT__RHS)
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
    }

    @Check(FAST)
    def checkConnection(Connection connection) {
        var reactor  = connection.eContainer as Reactor
        for (reaction : reactor.reactions) {
            for (effect : reaction.effects) {
                if (connection.rightPort.variable === effect.variable) {
                    error("Cannot connect: Port named '" + effect.variable.name + "' is already effect of a reaction.", Literals.CONNECTION__RIGHT_PORT)
                }
            }
        }
    }

    @Check(FAST)
    def checkDeadline(Deadline deadline) {
        if (this.target == Targets.C && this.info.overflowingDeadlines.contains(deadline)) {
            error("Deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE + " nanoseconds.", Literals.DEADLINE__TIME)
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
        if (instantiation.reactorClass.isMain) {
            error(
                "Cannot instantiate a main reactor: " 
                + instantiation.reactorClass.name,
                Literals.INSTANTIATION__REACTOR_CLASS
            )
        }
        // Report error if this instantiation is part of a cycle.
        if (this.info.instantiationGraph.cycles.size > 0) {
            for (cycle : this.info.instantiationGraph.cycles) {
                val instance = new AnnotatedNode(instantiation.reactorClass)
                val reactor = new AnnotatedNode(instantiation.eContainer as Reactor)
                if (cycle.contains(reactor) && cycle.contains(instance)) {
                    error("Instantiation is part of a cycle: " 
                        + instantiation.reactorClass.name,
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
                    ". Recognized parameters are " + TargetProperties.values().join(", "),
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
            case CMAKE_INCLUDE:
                if (param.value.literal === null) {
                    error("Target property cmake_include is required to be a string.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case COMPILER:
                if (param.value.literal === null) {
                    error("Target property compile is required to be a string.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case FLAGS:
                if (param.value.literal === null) {
                    error("Target property flags is required to be a string.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case FAST:
                if (!param.value.id.equals('true') && !param.value.id.equals('false')) {
                    error("Target property fast is required to be true or false.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case FEDERATES:
                if (param.value.keyvalue === null) {
                    error("Target property federates is required to be a set of key-value pairs.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                } else {
                    for (federate : param.value.keyvalue.pairs) {
                        if (federate.value.keyvalue === null) {
                            error("Each federate needs to be defined by key-value pairs.",
                                Literals.KEY_VALUE_PAIR__VALUE)
                        }
                        if (federate.name == "RTI") {
                            // Check port and host parameter form.
                            for (property : federate.value.keyvalue.pairs) {
                                switch property.name {
                                case "port": // FIXME: use enum
                                    if (property.value.literal === null) {
                                        error("port property needs to be a number.",
                                            Literals.KEY_VALUE_PAIR__VALUE)
                                    } else {
                                        try {
                                            Integer.parseInt(property.value.literal)
                                        } catch (NumberFormatException ex) {
                                            error("port property needs to be a number.",
                                                    Literals.KEY_VALUE_PAIR__VALUE)
                                        }
                                    }
                                case "host":
                                    if (property.value.literal === null) {
                                        error("host property needs to be a string.",
                                            Literals.KEY_VALUE_PAIR__VALUE)
                                    }
                                }
                            }
                        } else {
                            // Check that there is a 'reactors' property that is an array of ids.
                            var foundReactors = false
                            // Check that each federate specifies a set of reactors.
                            for (property : federate.value.keyvalue.pairs) {
                                if (property.name.equals("reactors")) {
                                    foundReactors = true
                                            if (property.value.array === null) {
                                                error("Each reactor property needs to be an array of ids.",
                                                        Literals.KEY_VALUE_PAIR__VALUE)
                                            }
                                    for (reactor : property.value.array.elements) {
                                        if (reactor.id === null) {
                                            error("Each reactor property needs to be an array of ids.",
                                                    Literals.KEY_VALUE_PAIR__VALUE)
                                        }
                                    }
                                    // FIXME: Should check that the reactor is an instance
                                    // main reactor.
                                }
                            }
                            if (!foundReactors) {
                                error("Each federate needs to have a reactor property.",
                                    Literals.KEY_VALUE_PAIR__VALUE)
                            }
                        }
                    }
                }
            case KEEPALIVE:
                if (!param.value.id.equals('true') && !param.value.id.equals('false')) {
                    error("Target property keepalive is required to be true or false.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case LOGGING:
                if (!Arrays.asList(LoggingLevels.values()).exists[it.toString.equals(param.value.id)]) {
                    error("Target property logging is required to be one of " +
                        LoggingLevels.values(),
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case NO_COMPILE:
                 if (!param.value.id.equals('true') && !param.value.id.equals('false')) {
                    error("Target property no-compile is required to be true or false.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case THREADS: {
                if (param.value.literal === null) {
                    error("Target property threads is required to be a non-negative integer.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
                try {
                    val value = Integer.decode(param.value.literal)
                    if (value < 0) {
                        error("Target property threads is required to be a non-negative integer.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                    }
                } catch (NumberFormatException ex) {
                    error("Target property threads is required to be a non-negative integer.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            }
            case TIMEOUT:
                if (param.value.unit === null) {
                    error("Target property timeout requires a time unit. Should be one of " +
                        TimeUnit.VALUES.filter[it != TimeUnit.NONE],
                        Literals.KEY_VALUE_PAIR__VALUE)
                } else if (param.value.time < 0) {
                    error("Target property timeout requires a non-negative time value with units.",
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
        if (this.target.requiresTypes) {
            if (!param.ofTimeType && param.type === null) {
                error("Parameters must have a type.", Literals.PARAMETER__TYPE)
            }
        }
        
        if (this.target == Targets.C && this.info.overflowingParameters.contains(param)) {
            error("Time value used to specify a deadline exceeds the maximum of " + TimeValue.MAX_LONG_DEADLINE + " nanoseconds.", Literals.PARAMETER__TIME)
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
        if (this.target == Targets.CPP && reactor.isMain && reactor.name.equalsIgnoreCase("main")) {
            error(
                "Main reactor cannot be named '" + reactor.name + "'",
                Literals.REACTOR__NAME
            )
        }
    }

    @Check(FAST)
    def checkState(org.icyphy.linguaFranca.State state) {
        checkName(state.name, Literals.STATE__NAME)
        if (allNames.contains(state.name)) {
            error(
                UNIQUENESS_MESSAGE + state.name,
                Literals.STATE__NAME
            )
        }
        inputs.add(state.name);
        allNames.add(state.name)
        if (this.target.requiresTypes) {
            if (!state.ofTimeType && state.parameter === null && state.type === null) {
                error("State must have a type.", Literals.STATE__TYPE)
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
    def checkTime(TimeOrValue timeOrValue) {
        // Only parameter assignments are allowed to be target types.
        // Time parameters can go without units only if they are 0.
        if (!(timeOrValue.eContainer instanceof Assignment)) {
            
            // If a literal value is provided, check that it is zero.
            if (timeOrValue.value !== null && !timeOrValue.value.isEmpty) {
                try {
                    val number = Integer.parseInt(timeOrValue.value)
                    if (number != 0) {
                        if (timeOrValue.unit == TimeUnit.NONE) {
                            error("Missing time units. Should be one of " +
                                TimeUnit.VALUES.filter[it != TimeUnit.NONE],
                                Literals.TIME_OR_VALUE__UNIT)
                        }    
                    }
                } catch(NumberFormatException e) {
                    error("Invalid time literal",
                        Literals.TIME_OR_VALUE__UNIT)
                }
            }
            // If parameter is referenced, check that it is of the correct type.
            if (timeOrValue.parameter !== null) {
                if (!timeOrValue.parameter.isOfTimeType) {
                    error("Parameter is not of time type",
                        Literals.TIME_OR_VALUE__PARAMETER)
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

    static val UNIQUENESS_MESSAGE = "Names of contained objects (inputs, outputs, actions, timers, parameters, state, and reactors) must be unique: "
    static val UNDERSCORE_MESSAGE = "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": "
    static val ACTIONS_MESSAGE = "\"actions\" is a reserved word for the TypeScript target for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation): "
    static val RESERVED_MESSAGE = "Reserved words in the target language are not allowed for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation): "
    
}
