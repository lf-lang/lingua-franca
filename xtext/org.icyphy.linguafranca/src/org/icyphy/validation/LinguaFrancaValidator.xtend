package org.icyphy.validation

import org.eclipse.emf.ecore.EStructuralFeature
import org.eclipse.xtext.validation.Check
import org.icyphy.AnnotatedDependencyGraph
import org.icyphy.AnnotatedNode
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Connection
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
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#validation
 */
class LinguaFrancaValidator extends AbstractLinguaFrancaValidator {

    public static val KNOWN_TARGETS = #{
        'C', 
        'Cpp',
        'TypeScript'
    }
    
    public static val LOGGING_LEVELS = #{
        'ERROR',
        'WARN',
        'INFO',
        'LOG',
        'DEBUG'
    }
    
    public static val TARGET_REQUIRES_TYPES = #{
        'C' -> true,
        'Cpp' -> true,
        'TypeScript' -> false
    }
    // Allowed target parameters, in alphabetical order.
    public static val TARGET_PARAMETERS = #{
        'cmake_include',
        'compile', 
        'fast',
        'federates',
        'hosts',
        'keepalive',
        'logging',
        'run', 
        'threads',
        'timeout'
    }

    var reactorClasses = newHashSet()
    var parameters = newHashSet()
    var inputs = newHashSet()
    var outputs = newHashSet()
    var timers = newHashSet()
    var actions = newHashSet()
    var allNames = newHashSet() // Names of contained objects must be unique.
    var depGraph = new AnnotatedDependencyGraph()
    var target = "";
    
    // //////////////////////////////////////////////////
    // // Helper functions for checks to be performed on multiple entities

    // Check the name of a feature for illegal substrings.
    def checkName(String name, EStructuralFeature feature) {
        // Raises an error if the string starts with two underscores.
        if (name.length() >= 2 && name.substring(0,2).equals("__")) {
            error(UNDERSCORE_MESSAGE + name, feature)
        }
        
        if (this.target.equals("TypeScript") && name.equals("actions")) {
            error(ACTIONS_MESSAGE + name, feature)
        }
    }
    
    // //////////////////////////////////////////////////
    // // Functions to set up data structures for performing checks.
    // FAST ensures that these checks run whenever a file is modified.
    // Alternatives are NORMAL (when saving) and EXPENSIVE (only when right-click, validate).
    // FIXME: Only checking uniqueness of reactor class definitions per file
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
        if (TARGET_REQUIRES_TYPES.get(this.target)) {
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
        if (this.depGraph.cycles.size > 0) {
            for (cycle : this.depGraph.cycles) {
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
            if (!TARGET_PARAMETERS.contains(param.name)) {
                warning(
                    "Unrecognized target parameter: " + param.name +
                    ". Recognized parameters are " + TARGET_PARAMETERS,
                    Literals.KEY_VALUE_PAIR__NAME)
            }
            switch param.name {
            case "cmake_include":
                if (param.value.literal === null) {
                    error("Target property cmake_include is required to be a string.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case "compile":
                if (param.value.literal === null) {
                    error("Target property compile is required to be a string.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case "fast":
                if (!param.value.id.equals('true') && !param.value.id.equals('false')) {
                    error("Target property fast is required to be true or false.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case "federates":
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
                                case "port":
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
            case "keepalive":
                if (!param.value.id.equals('true') && !param.value.id.equals('false')) {
                    error("Target property keepalive is required to be true or false.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case "logging":
                if (!LOGGING_LEVELS.contains(param.value.id)) {
                    error("Target property logging is required to be one of " +
                        LOGGING_LEVELS,
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case "run":
                if (param.value.literal === null) {
                    error("Target property run is required to be a string.",
                        Literals.KEY_VALUE_PAIR__VALUE)
                }
            case "threads": {
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
            case "timeout":
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
        if (TARGET_REQUIRES_TYPES.get(this.target)) {
            if (output.type === null) {
                error("Output must have a type.", Literals.PORT__TYPE)
            }
        }
    }

    @Check(NORMAL)
    def checkModel(Model model) {
        this.depGraph = new AnnotatedDependencyGraph()
        for (instantiation : model.eAllContents.toIterable.filter(Instantiation)) {
            this.depGraph.addEdge(new AnnotatedNode(instantiation.eContainer as Reactor), new AnnotatedNode(instantiation.reactorClass))    
        }
        this.depGraph.detectCycles
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
        if (TARGET_REQUIRES_TYPES.get(this.target)) {
            if (!param.ofTimeType && param.type === null) {
                error("Parameters must have a type.", Literals.PARAMETER__TYPE)
            }
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
        if (this.target.equals('Cpp') && reactor.isMain && reactor.name.equalsIgnoreCase("main")) {
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
        if (TARGET_REQUIRES_TYPES.get(this.target)) {
            if (!state.ofTimeType && state.parameter === null && state.type === null) {
                error("State must have a type.", Literals.STATE__TYPE)
            }
        }
    }

    @Check(FAST)
    def checkTarget(Target target) {
        if (!KNOWN_TARGETS.contains(target.name)) {
            warning("Unrecognized target: " + target.name,
                Literals.TARGET__NAME)
        } else {
            this.target = target.name;
        }
    }
    
    @Check(FAST)
    def checkTime(TimeOrValue timeOrValue) {
        // Only parameter assignments are allowed to be target types.
        // Time parameters can go without units only if they are 0.
        if (!(timeOrValue.eContainer instanceof Assignment) &&
            timeOrValue.time != 0) {
            if (timeOrValue.unit == TimeUnit.NONE) {
                error("Missing time units. Should be one of " +
                    TimeUnit.VALUES.filter[it != TimeUnit.NONE],
                    Literals.TIME_OR_VALUE__UNIT)
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
    static val ACTIONS_MESSAGE = "\"actions\" is a reserved name for the TypeScript target for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation)." 
}
