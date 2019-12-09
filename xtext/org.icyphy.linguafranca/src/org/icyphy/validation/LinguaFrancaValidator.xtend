package org.icyphy.validation

import org.eclipse.xtext.validation.Check
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaPackage.Literals
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
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
        'Accessor',
        'Accessors', 
        'C', 
        'Cpp',
        'SCL',
        'TypeScript'
    }
    public static val TARGET_PARAMETERS = #{
        'compile', 
        'run', 
        'threads',
        'timeout',
        'cmake_include'
    }

    var reactorClasses = newHashSet()
    var parameters = newHashSet()
    var inputs = newHashSet()
    var outputs = newHashSet()
    var timers = newHashSet()
    var actions = newHashSet()
    var allNames = newHashSet()
    var containedNames = newHashSet() // Names of contained reactor instances.

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
        containedNames.clear()
    }

    @Check(FAST)
    def recordParameter(Parameter param) {
        parameters.add(param.name)
        allNames.add(param.name)
    }

    // //////////////////////////////////////////////////
    // // The following checks are in alphabetical order.
    @Check(FAST)
    def checkAction(Action action) {
        if (allNames.contains(action.name)) {
            error(
                "Names of parameters, inputs, timers, and actions must be unique: " +
                    action.name,
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
    def checkInput(Input input) {
        if (allNames.contains(input.name)) {
            error(
                "Names of parameters, inputs, timers, and actions must be unique: " +
                    input.name,
                Literals.VARIABLE__NAME
            )
        }
        inputs.add(input.name);
        allNames.add(input.name)
    }

    @Check(FAST)
    def checkInstance(Instantiation instance) {
        if (containedNames.contains(instance.name)) {
            error(
                "Names of instances must be unique: " + instance.name,
                Literals.INSTANTIATION__NAME
            )
        }
        containedNames.add(instance.name)
    }

    @Check(FAST)
    def checkOutput(Output output) {
        if (allNames.contains(output.name)) {
            error(
                "Names of parameters, inputs, timers, and actions must be unique: " +
                    output.name,
                Literals.VARIABLE__NAME
            )
        }
        outputs.add(output.name);
        allNames.add(output.name)
    }

    @Check(FAST)
    def checkTarget(Target target) {
        if (!KNOWN_TARGETS.contains(target.name)) {
            warning("Unrecognized target: " + target.name,
                Literals.TARGET__NAME)
        }
        if (target.properties !== null) {
            for (property : target.properties) {
                if (!TARGET_PARAMETERS.contains(property.name)) {
                    warning(
                        "Unrecognized target parameter: " + property.name,
                        Literals.TARGET__PROPERTIES
                    )
                }
                // Make sure the value of the parameter is a string,
                // a parsable integer, or a time.
                if(property.value.value !== null){
                    // This is a Literal
                    if (!property.value.value.startsWith('"') ||
                        !property.value.value.endsWith('"')) {
                        try {
                            Integer.decode(property.value.value)
                        } catch (NumberFormatException ex) {
                            error(
                                "Target property literal is required to be an integer or a string surrounded by quotation marks.",
                                Literals.TARGET__PROPERTIES
                            )
                        }
                    }
                } else {
                    // This is a Time
                    if (property.value.unit == TimeUnit.NONE) {
                        error("Missing time units. Should be one of " +
                        TimeUnit.VALUES.filter[it != TimeUnit.NONE],
                        Literals.TIME_OR_VALUE__UNIT)
                    }
                }
            }
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
        if (allNames.contains(timer.name)) {
            error(
                "Names of parameters, inputs, timers, and actions must be unique: " +
                    timer.name,
                Literals.VARIABLE__NAME
            )
        }
        timers.add(timer.name);
        allNames.add(timer.name)
    }

    @Check(FAST)
    def checkReactor(Reactor reactor) {
        if (reactorClasses.contains(reactor.name)) {
            error(
                "Names of reactor classes must be unique: " + reactor.name,
                Literals.REACTOR__MAIN
            )
        }
        reactorClasses.add(reactor.name);
    }

}
