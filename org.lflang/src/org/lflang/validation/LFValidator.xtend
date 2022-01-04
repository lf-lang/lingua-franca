/* Validation checks for Lingua Franca code. */

/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.

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
package org.lflang.validation

import com.google.inject.Inject

import java.util.ArrayList
import java.util.HashSet
import java.util.List
import java.util.Set

import org.eclipse.emf.common.util.EList
import org.eclipse.emf.ecore.EStructuralFeature
import org.eclipse.xtext.validation.Check
import org.eclipse.xtext.validation.ValidationMessageAcceptor;
import org.eclipse.xtend.lib.annotations.Accessors

import org.lflang.FileConfig
import org.lflang.ModelInfo
import org.lflang.Target
import org.lflang.TargetProperty
import org.lflang.TimeValue
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.Assignment
import org.lflang.lf.Connection
import org.lflang.lf.Deadline
import org.lflang.lf.Host
import org.lflang.lf.IPV4Host
import org.lflang.lf.IPV6Host
import org.lflang.lf.Import
import org.lflang.lf.ImportedReactor
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.KeyValuePair
import org.lflang.lf.KeyValuePairs
import org.lflang.lf.LfPackage.Literals
import org.lflang.lf.Model
import org.lflang.lf.NamedHost
import org.lflang.lf.Output
import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Preamble
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.Serializer
import org.lflang.lf.STP
import org.lflang.lf.StateVar
import org.lflang.lf.TargetDecl
import org.lflang.lf.Timer
import org.lflang.lf.Type
import org.lflang.lf.TypedVariable
import org.lflang.lf.Value
import org.lflang.lf.VarRef
import org.lflang.lf.Variable
import org.lflang.lf.Visibility
import org.lflang.lf.WidthSpec

import static extension org.lflang.ASTUtils.*
import static extension org.lflang.JavaAstUtils.*
import org.lflang.federated.serialization.SupportedSerializers

/**
 * Custom validation checks for Lingua Franca programs.
 *
 * Also see: https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#validation
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author(Christian Menard <christian.menard@tu-dresden.de>}
 *
 */
class LFValidator extends BaseLFValidator {

    var Target target

    public var info = new ModelInfo()

    @Accessors(PUBLIC_GETTER)
    val ValidatorErrorReporter errorReporter = new ValidatorErrorReporter(getMessageAcceptor(),
        new ValidatorStateAccess())

    @Inject(optional = true)
    ValidationMessageAcceptor messageAcceptor

    /**
     * Regular expression to check the validity of IPV4 addresses (due to David M. Syzdek).
     */
    static val ipv4Regex = "((25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])\\.){3,3}" +
                                "(25[0-5]|(2[0-4]|1{0,1}[0-9]){0,1}[0-9])"

    /**
     * Regular expression to check the validity of IPV6 addresses (due to David M. Syzdek),
     * with minor adjustment to allow up to six IPV6 segments (without truncation) in front
     * of an embedded IPv4-address.
     **/
    static val ipv6Regex =
                "(([0-9a-fA-F]{1,4}:){7,7}[0-9a-fA-F]{1,4}|" +
                "([0-9a-fA-F]{1,4}:){1,7}:|" +
                "([0-9a-fA-F]{1,4}:){1,6}:[0-9a-fA-F]{1,4}|" +
                "([0-9a-fA-F]{1,4}:){1,5}(:[0-9a-fA-F]{1,4}){1,2}|" +
                "([0-9a-fA-F]{1,4}:){1,4}(:[0-9a-fA-F]{1,4}){1,3}|" +
                "([0-9a-fA-F]{1,4}:){1,3}(:[0-9a-fA-F]{1,4}){1,4}|" +
                "([0-9a-fA-F]{1,4}:){1,2}(:[0-9a-fA-F]{1,4}){1,5}|" +
                 "[0-9a-fA-F]{1,4}:((:[0-9a-fA-F]{1,4}){1,6})|" +
                                 ":((:[0-9a-fA-F]{1,4}){1,7}|:)|" +
        "fe80:(:[0-9a-fA-F]{0,4}){0,4}%[0-9a-zA-Z]{1,}|" +
        "::(ffff(:0{1,4}){0,1}:){0,1}" + ipv4Regex + "|" +
        "([0-9a-fA-F]{1,4}:){1,4}:"    + ipv4Regex + "|" +
        "([0-9a-fA-F]{1,4}:){1,6}"     + ipv4Regex + ")"

    static val usernameRegex = "^[a-z_]([a-z0-9_-]{0,31}|[a-z0-9_-]{0,30}\\$)$"

    static val hostOrFQNRegex = "^([a-z0-9]+(-[a-z0-9]+)*)|(([a-z0-9]+(-[a-z0-9]+)*\\.)+[a-z]{2,})$"

    public static val GLOBALLY_DUPLICATE_NAME = 'GLOBALLY_DUPLICATE_NAME'

    static val spacingViolationPolicies = #['defer', 'drop', 'replace']

    val List<String> targetPropertyErrors = new ArrayList

    val List<String> targetPropertyWarnings = new ArrayList

    def List<String> getTargetPropertyErrors() {
        this.targetPropertyErrors
    }

    override ValidationMessageAcceptor getMessageAcceptor() {
        return messageAcceptor === null ? this : messageAcceptor
    }
    
    /**
     * Returns true if target is C or a C-based target like CCpp.
     */
    def boolean isCBasedTarget() {
        return (this.target == Target.C || this.target == Target.CCPP);
    }

    @Check
    def checkImportedReactor(ImportedReactor reactor) {
        if (reactor.unused) {
            warning("Unused reactor class.",
                Literals.IMPORTED_REACTOR__REACTOR_CLASS)
        }

        if (info.instantiationGraph.hasCycles) {
            val cycleSet = newHashSet
            info.instantiationGraph.cycles.forEach[forEach[cycleSet.add(it)]]
            if (dependsOnCycle(reactor.toDefinition, cycleSet, newHashSet)) {
                error("Imported reactor '" + reactor.toDefinition.name +
                    "' has cyclic instantiation in it.",
                    Literals.IMPORTED_REACTOR__REACTOR_CLASS)
            }
        }
    }

    @Check
    def checkImport(Import imp) {
        if (imp.reactorClasses.get(0).toDefinition.eResource.errors.size > 0) {
            error("Error loading resource.", Literals.IMPORT__IMPORT_URI) // FIXME: print specifics.
            return
        }

        // FIXME: report error if resource cannot be resolved.

        for (reactor : imp.reactorClasses) {
            if (!reactor.unused) {
                return
            }
        }
        warning("Unused import.", Literals.IMPORT__IMPORT_URI)
    }

    // //////////////////////////////////////////////////
    // // Helper functions for checks to be performed on multiple entities
    // Check the name of a feature for illegal substrings.
    private def checkName(String name, EStructuralFeature feature) {

        // Raises an error if the string starts with two underscores.
        if (name.length() >= 2 && name.substring(0, 2).equals("__")) {
            error(UNDERSCORE_MESSAGE + name, feature)
        }

        if (this.target.isReservedIdent(name)) {
            error(RESERVED_MESSAGE + name, feature)
        }

        if (this.target == Target.TS) {
            // "actions" is a reserved word within a TS reaction
            if (name.equals("actions")) {
                error(ACTIONS_MESSAGE + name, feature)
            }
        }

    }

    /**
     * Report whether a given reactor has dependencies on a cyclic
     * instantiation pattern. This means the reactor has an instantiation
     * in it -- directly or in one of its contained reactors -- that is
     * self-referential.
     * @param reactor The reactor definition to find out whether it has any
     * dependencies on cyclic instantiations.
     * @param cycleSet The set of all reactors that are part of an
     * instantiation cycle.
     * @param visited The set of nodes already visited in this graph traversal.
     */
    private def boolean dependsOnCycle(Reactor reactor, Set<Reactor> cycleSet,
        Set<Reactor> visited) {
        val origins = info.instantiationGraph.getUpstreamAdjacentNodes(reactor)
        if (visited.contains(reactor)) {
            return false
        } else {
            visited.add(reactor)
            if (origins.exists[cycleSet.contains(it)] || origins.exists [
                it.dependsOnCycle(cycleSet, visited)
            ]) {
                // Reached a cycle.
                return true
            }
        }
        return false
    }

    /**
     * Report whether a given imported reactor is used in this resource or not.
     * @param reactor The imported reactor to check whether it is used.
     */
    private def boolean isUnused(ImportedReactor reactor) {
        val instantiations = reactor.eResource.allContents.filter(Instantiation)
        val subclasses = reactor.eResource.allContents.filter(Reactor)
        if (instantiations.
            forall[it.reactorClass !== reactor && it.reactorClass !== reactor.reactorClass] &&
            subclasses.forall [
                it.superClasses.forall [
                    it !== reactor && it !== reactor.reactorClass
                ]
            ]) {
            return true
        }
        return false
    }


    // //////////////////////////////////////////////////
    // // Functions to set up data structures for performing checks.
    // FAST ensures that these checks run whenever a file is modified.
    // Alternatives are NORMAL (when saving) and EXPENSIVE (only when right-click, validate).

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
        if (action.policy !== null &&
            !spacingViolationPolicies.contains(action.policy)) {
            error(
                "Unrecognized spacing violation policy: " + action.policy +
                    ". Available policies are: " +
                    spacingViolationPolicies.join(", ") + ".",
                Literals.ACTION__POLICY)
        }
    }

    @Check(FAST)
    def checkAssignment(Assignment assignment) {
        // If the left-hand side is a time parameter, make sure the assignment has units
        if (assignment.lhs.isOfTimeType) {
            if (assignment.rhs.size > 1) {
                 error("Incompatible type.", Literals.ASSIGNMENT__RHS)
            } else if (assignment.rhs.size > 0) {
                val v = assignment.rhs.get(0)
                if (!v.isValidTime) {
                    if (v.parameter === null) {
                        // This is a value. Check that units are present.
                    error(
                        "Missing time unit.", Literals.ASSIGNMENT__RHS)
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
            if (isCBasedTarget &&
                this.info.overflowingAssignments.contains(assignment)) {
                error(
                    "Time value used to specify a deadline exceeds the maximum of " +
                        TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                    Literals.ASSIGNMENT__RHS)
            }
        }

        if(!assignment.braces.isNullOrEmpty() && this.target != Target.CPP) {
            error("Brace initializers are only supported for the C++ target", Literals.ASSIGNMENT__BRACES)
        }

        // FIXME: lhs is list => rhs is list
        // lhs is fixed with size n => rhs is fixed with size n
        // FIXME": similar checks for decl/init
        // Specifically for C: list can only be literal or time lists
    }

    @Check(FAST)
    def checkWidthSpec(WidthSpec widthSpec) {
        if (!this.target.supportsMultiports()) {
            error("Multiports and banks are currently not supported by the given target.",
                Literals.WIDTH_SPEC__TERMS)
        } else {
            for (term : widthSpec.terms) {
                if (term.parameter !== null) {
                    if (!this.target.supportsParameterizedWidths()) {
                        error("Parameterized widths are not supported by this target.", Literals.WIDTH_SPEC__TERMS)
                    }
                } else if (term.port !== null) {
                    // Widths given with `widthof()` are not supported (yet?).
                    // This feature is currently only used for after delays.
                    error("widthof is not supported.", Literals.WIDTH_SPEC__TERMS)
                } else if (term.code !== null) {
                     if (this.target != Target.CPP) {
                        error("This target does not support width given as code.", Literals.WIDTH_SPEC__TERMS)
                    }
                } else if (term.width < 0) {
                    error("Width must be a positive integer.", Literals.WIDTH_SPEC__TERMS)
                }
            }
        }
    }

    @Check(FAST)
    def checkConnection(Connection connection) {

        // Report if connection is part of a cycle.
        for (cycle : this.info.topologyCycles()) {
            for (lp : connection.leftPorts) {
                for (rp : connection.rightPorts) {
                    var leftInCycle = false
                    val reactorName = (connection.eContainer as Reactor).name

                    if ((lp.container === null && cycle.exists [
                        it.definition === lp.variable
                    ]) || cycle.exists [
                        (it.definition === lp.variable && it.parent === lp.container)
                    ]) {
                        leftInCycle = true
                    }

                    if ((rp.container === null && cycle.exists [
                        it.definition === rp.variable
                    ]) || cycle.exists [
                        (it.definition === rp.variable && it.parent === rp.container)
                    ]) {
                        if (leftInCycle) {
                            // Only report of _both_ reference ports are in the cycle.
                            error('''Connection in reactor «reactorName» creates ''' +
                                    '''a cyclic dependency between «lp.toText» and ''' +
                                    '''«rp.toText».''', Literals.CONNECTION__DELAY
                            )
                        }
                    }
                }
            }
        }

        // FIXME: look up all ReactorInstance objects that have a definition equal to the
        // container of this connection. For each of those occurrences, the widths have to match.
        // For the C target, since C has such a weak type system, check that
        // the types on both sides of every connection match. For other languages,
        // we leave type compatibility that language's compiler or interpreter.
        if (isCBasedTarget) {
            var type = null as Type
            for (port : connection.leftPorts) {
                // If the variable is not a port, then there is some other
                // error. Avoid a class cast exception.
                if (port.variable instanceof Port) {
                    if (type === null) {
                        type = (port.variable as Port).type
                    } else {
                        // Unfortunately, xtext does not generate a suitable equals()
                        // method for AST types, so we have to manually check the types.
                        if (!sameType(type, (port.variable as Port).type)) {
                            error("Types do not match.", Literals.CONNECTION__LEFT_PORTS)
                        }
                    }
                }
            }
            for (port : connection.rightPorts) {
                // If the variable is not a port, then there is some other
                // error. Avoid a class cast exception.
                if (port.variable instanceof Port) {
                    if (type === null) {
                        type = (port.variable as Port).type
                    } else {
                        if (!sameType(type, (port.variable as Port).type)) {
                            error("Types do not match.", Literals.CONNECTION__RIGHT_PORTS)
                        }
                    }
                }
            }
        }

        // Check whether the total width of the left side of the connection
        // matches the total width of the right side. This cannot be determined
        // here if the width is not given as a constant. In that case, it is up
        // to the code generator to check it.
        var leftWidth = 0
        for (port : connection.leftPorts) {
            val width = inferPortWidth(port, null, null) // null args imply incomplete check.
            if (width < 0 || leftWidth < 0) {
                // Cannot determine the width of the left ports.
                leftWidth = -1
            } else {
                leftWidth += width
            }
        }
        var rightWidth = 0
        for (port : connection.rightPorts) {
            val width = inferPortWidth(port, null, null) // null args imply incomplete check.
            if (width < 0 || rightWidth < 0) {
                // Cannot determine the width of the left ports.
                rightWidth = -1
            } else {
                rightWidth += width
            }
        }

        if (leftWidth !== -1 && rightWidth !== -1 && leftWidth != rightWidth) {
            if (connection.isIterated) {
                if (leftWidth == 0 || rightWidth % leftWidth != 0) {
                    // FIXME: The second argument should be Literals.CONNECTION, but
                    // stupidly, xtext will not accept that. There seems to be no way to
                    // report an error for the whole connection statement.
                    warning('''Left width «leftWidth» does not divide right width «rightWidth»''',
                            Literals.CONNECTION__LEFT_PORTS
                    )
                }
            } else {
                // FIXME: The second argument should be Literals.CONNECTION, but
                // stupidly, xtext will not accept that. There seems to be no way to
                // report an error for the whole connection statement.
                warning('''Left width «leftWidth» does not match right width «rightWidth»''',
                        Literals.CONNECTION__LEFT_PORTS
                )
            }
        }

        val reactor = connection.eContainer as Reactor

        // Make sure the right port is not already an effect of a reaction.
        for (reaction : reactor.reactions) {
            for (effect : reaction.effects) {
                for (rightPort : connection.rightPorts) {
                    if (rightPort.container === effect.container &&
                            rightPort.variable === effect.variable) {
                        error("Cannot connect: Port named '" + effect.variable.name +
                            "' is already effect of a reaction.",
                            Literals.CONNECTION__RIGHT_PORTS
                        )
                    }
                }
            }
        }

        // Check that the right port does not already have some other
        // upstream connection.
        for (c : reactor.connections) {
            if (c !== connection) {
                for (thisRightPort : connection.rightPorts) {
                    for (thatRightPort : c.rightPorts) {
                        if (thisRightPort.container === thatRightPort.container &&
                                thisRightPort.variable === thatRightPort.variable) {
                            error(
                                "Cannot connect: Port named '" + thisRightPort.variable.name +
                                    "' may only appear once on the right side of a connection.",
                                Literals.CONNECTION__RIGHT_PORTS)
                        }
                    }
                }
            }
        }
    }

    /**
     * Return true if the two types match. Unfortunately, xtext does not
     * seem to create a suitable equals() method for Type, so we have to
     * do this manually.
     */
    private def boolean sameType(Type type1, Type type2) {
        // Most common case first.
        if (type1.id !== null) {
            if (type1.stars !== null) {
                if (type2.stars === null) return false
                if (type1.stars.length != type2.stars.length) return false
            }
            return (type1.id.equals(type2.id))
        }
        if (type1 === null) {
            if (type2 === null) return true
            return false
        }
        // Type specification in the grammar is:
        // (time?='time' (arraySpec=ArraySpec)?) | ((id=(DottedName) (stars+='*')* ('<' typeParms+=TypeParm (',' typeParms+=TypeParm)* '>')? (arraySpec=ArraySpec)?) | code=Code);
        if (type1.time) {
            if (!type2.time) return false
            // Ignore the arraySpec because that is checked when connection
            // is checked for balance.
            return true
        }
        // Type must be given in a code body.
        return (type1.code.body.equals(type2?.code?.body))
    }

    @Check(FAST)
    def checkDeadline(Deadline deadline) {
        if (isCBasedTarget &&
            this.info.overflowingDeadlines.contains(deadline)) {
            error(
                "Deadline exceeds the maximum of " +
                    TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                Literals.DEADLINE__DELAY)
        }
    }
@Check(FAST)
    def checkSTPOffset(STP stp) {
        if (isCBasedTarget &&
            this.info.overflowingDeadlines.contains(stp)) {
            error(
                "STP offset exceeds the maximum of " +
                    TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                Literals.DEADLINE__DELAY)
        }
    }

    @Check(FAST)
    def checkInput(Input input) {
        checkName(input.name, Literals.VARIABLE__NAME)
        if (target.requiresTypes) {
            if (input.type === null) {
                error("Input must have a type.", Literals.TYPED_VARIABLE__TYPE)
            }
        }

        // mutable has no meaning in C++
        if (input.mutable && this.target == Target.CPP) {
            warning(
                "The mutable qualifier has no meaning for the C++ target and should be removed. " +
                "In C++, any value can be made mutable by calling get_mutable_copy().",
                Literals.INPUT__MUTABLE
            )
        }

        // Variable width multiports are not supported (yet?).
        if (input.widthSpec !== null && input.widthSpec.ofVariableLength) {
            error("Variable-width multiports are not supported.", Literals.PORT__WIDTH_SPEC)
        }
    }

    @Check(FAST)
    def checkInstantiation(Instantiation instantiation) {
        checkName(instantiation.name, Literals.INSTANTIATION__NAME)
        val reactor = instantiation.reactorClass.toDefinition
        if (reactor.isMain || reactor.isFederated) {
            error(
                "Cannot instantiate a main (or federated) reactor: " +
                    instantiation.reactorClass.name,
                Literals.INSTANTIATION__REACTOR_CLASS
            )
        }

        // Report error if this instantiation is part of a cycle.
        // FIXME: improve error message.
        // FIXME: Also report if there exists a cycle within.
        if (this.info.instantiationGraph.cycles.size > 0) {
            for (cycle : this.info.instantiationGraph.cycles) {
                val container = instantiation.eContainer as Reactor
                if (cycle.contains(container) && cycle.contains(reactor)) {
                    error(
                        "Instantiation is part of a cycle: " +
                            cycle.fold(newArrayList, [ list, r |
                                list.add(r.name);
                                list
                            ]).join(', ') + ".",
                        Literals.INSTANTIATION__REACTOR_CLASS
                    )
                }
            }
        }
        // Variable width multiports are not supported (yet?).
        if (instantiation.widthSpec !== null
                && instantiation.widthSpec.ofVariableLength
        ) {
            if (isCBasedTarget) {
                warning("Variable-width banks are for internal use only.",
                    Literals.INSTANTIATION__WIDTH_SPEC
                )
            } else {
                error("Variable-width banks are not supported.",
                    Literals.INSTANTIATION__WIDTH_SPEC
                )
            }
        }
    }

    /** Check target parameters, which are key-value pairs. */
    @Check(FAST)
    def checkKeyValuePair(KeyValuePair param) {
        // Check only if the container's container is a Target.
        if (param.eContainer.eContainer instanceof TargetDecl) {

            val prop = TargetProperty.forName(param.name)

            // Make sure the key is valid.
            if (prop === null) {
                warning(
                    "Unrecognized target parameter: " + param.name +
                        ". Recognized parameters are: " +
                        TargetProperty.getOptions().join(", ") + ".",
                    Literals.KEY_VALUE_PAIR__NAME)
            }

            // Check whether the property is supported by the target.
            if (!prop.supportedBy.contains(this.target)) {
                warning(
                    "The target parameter: " + param.name +
                        " is not supported by the " + this.target +
                        " target and will thus be ignored.",
                    Literals.KEY_VALUE_PAIR__NAME)
            }

            // Report problem with the assigned value.
            prop.type.check(param.value, param.name, this)
            targetPropertyErrors.forEach [
                error(it, Literals.KEY_VALUE_PAIR__VALUE)
            ]
            targetPropertyErrors.clear()
            targetPropertyWarnings.forEach [
                warning(it, Literals.KEY_VALUE_PAIR__VALUE)
            ]
            targetPropertyWarnings.clear()
        }
    }

    @Check(FAST)
    def checkOutput(Output output) {
        checkName(output.name, Literals.VARIABLE__NAME)
        if (this.target.requiresTypes) {
            if (output.type === null) {
                error("Output must have a type.", Literals.TYPED_VARIABLE__TYPE)
            }
        }

        // Variable width multiports are not supported (yet?).
        if (output.widthSpec !== null && output.widthSpec.ofVariableLength) {
            error("Variable-width multiports are not supported.", Literals.PORT__WIDTH_SPEC)
        }
    }

    @Check(FAST)
    def checkModel(Model model) {
        // Since we're doing a fast check, we only want to update
        // if the model info hasn't been initialized yet. If it has,
        // we use the old information and update it during a normal
        // check (see below).
        if (!info.updated) {
            info.update(model, errorReporter)
        }
    }

    @Check(NORMAL)
    def updateModelInfo(Model model) {
        info.update(model, errorReporter)
    }

    @Check(FAST)
    def checkParameter(Parameter param) {
        checkName(param.name, Literals.PARAMETER__NAME)

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
                            error("Missing time unit.", Literals.PARAMETER__INIT)
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

        if (isCBasedTarget &&
            this.info.overflowingParameters.contains(param)) {
            error(
                "Time value used to specify a deadline exceeds the maximum of " +
                    TimeValue.MAX_LONG_DEADLINE + " nanoseconds.",
                Literals.PARAMETER__INIT)
        }
        
        if(!param.braces.isNullOrEmpty && this.target != Target.CPP) {
            error("Brace initializers are only supported for the C++ target", Literals.PARAMETER__BRACES)
        }
        
    }

    @Check(FAST)
    def checkPreamble(Preamble preamble) {
        if (this.target == Target.CPP) {
            if (preamble.visibility == Visibility.NONE) {
                error(
                    "Preambles for the C++ target need a visibility qualifier (private or public)!",
                    Literals.PREAMBLE__VISIBILITY
                )
            } else if (preamble.visibility == Visibility.PRIVATE) {
                val container = preamble.eContainer
                if (container !== null && container instanceof Reactor) {
                    val reactor = container as Reactor
                    if (reactor.isGeneric) {
                        warning(
                            "Private preambles in generic reactors are not truly private. " +
                                "Since the generated code is placed in a *_impl.hh file, it will " +
                                "be visible on the public interface. Consider using a public " +
                                "preamble within the reactor or a private preamble on file scope.",
                            Literals.PREAMBLE__VISIBILITY)
                    }
                }
            }
        } else if (preamble.visibility != Visibility.NONE) {
            warning(
                '''The «preamble.visibility» qualifier has no meaning for the «this.target.name» target. It should be removed.''',
                Literals.PREAMBLE__VISIBILITY
            )
        }
    }

	@Check(FAST)
    def checkReaction(Reaction reaction) {

        if (reaction.triggers === null || reaction.triggers.size == 0) {
            warning("Reaction has no trigger.", Literals.REACTION__TRIGGERS)
        }
        val triggers = new HashSet<Variable>
        // Make sure input triggers have no container and output sources do.
        for (trigger : reaction.triggers) {
            if (trigger instanceof VarRef) {
                triggers.add(trigger.variable)
                if (trigger.variable instanceof Input) {
                    if (trigger.container !== null) {
                        error('''Cannot have an input of a contained reactor as a trigger: «trigger.container.name».«trigger.variable.name»''',
                            Literals.REACTION__TRIGGERS)
                    }
                } else if (trigger.variable instanceof Output) {
                    if (trigger.container === null) {
                        error('''Cannot have an output of this reactor as a trigger: «trigger.variable.name»''',
                            Literals.REACTION__TRIGGERS)
                    }
                }
            }
        }

		// Make sure input sources have no container and output sources do.
        // Also check that a source is not already listed as a trigger.
        for (source : reaction.sources) {
            if (triggers.contains(source.variable)) {
                error('''Source is already listed as a trigger: «source.variable.name»''',
                    Literals.REACTION__SOURCES)
            }
            if (source.variable instanceof Input) {
                if (source.container !== null) {
                    error('''Cannot have an input of a contained reactor as a source: «source.container.name».«source.variable.name»''',
                        Literals.REACTION__SOURCES)
                }
            } else if (source.variable instanceof Output) {
                if (source.container === null) {
                    error('''Cannot have an output of this reactor as a source: «source.variable.name»''',
                        Literals.REACTION__SOURCES)
                }
            }
        }

        // Make sure output effects have no container and input effects do.
        for (effect : reaction.effects) {
            if (effect.variable instanceof Input) {
                if (effect.container === null) {
                    error('''Cannot have an input of this reactor as an effect: «effect.variable.name»''',
                        Literals.REACTION__EFFECTS)
                }
            } else if (effect.variable instanceof Output) {
                if (effect.container !== null) {
                    error('''Cannot have an output of a contained reactor as an effect: «effect.container.name».«effect.variable.name»''',
                        Literals.REACTION__EFFECTS)
                }
            }
        }

        // Report error if this reaction is part of a cycle.
        for (cycle : this.info.topologyCycles()) {
            val reactor = (reaction.eContainer) as Reactor
            if (cycle.exists[it.definition === reaction]) {
                // Report involved triggers.
                val trigs = new ArrayList()
                reaction.triggers.forEach [ t |
                    (t instanceof VarRef && cycle.exists [ c |
                        c.definition === (t as VarRef).variable
                    ]) ? trigs.add((t as VarRef).toText) : {
                    }
                ]
                if (trigs.size > 0) {
                    error('''Reaction triggers involved in cyclic dependency in reactor «reactor.name»: «trigs.join(', ')».''',
                        Literals.REACTION__TRIGGERS)
                }

                // Report involved sources.
                val sources = new ArrayList()
                reaction.sources.forEach [ t |
                    (cycle.exists[c|c.definition === t.variable])
                        ? sources.add(t.toText)
                        : {
                    }
                ]
                if (sources.size > 0) {
                    error('''Reaction sources involved in cyclic dependency in reactor «reactor.name»: «sources.join(', ')».''',
                        Literals.REACTION__SOURCES)
                }

                // Report involved effects.
                val effects = new ArrayList()
                reaction.effects.forEach [ t |
                    (cycle.exists[c|c.definition === t.variable])
                        ? effects.add(t.toText)
                        : {
                    }
                ]
                if (effects.size > 0) {
                    error('''Reaction effects involved in cyclic dependency in reactor «reactor.name»: «effects.join(', ')».''',
                        Literals.REACTION__EFFECTS)
                }

                if (trigs.size + sources.size == 0) {
                    error(
                    '''Cyclic dependency due to preceding reaction. Consider reordering reactions within reactor «reactor.name» to avoid causality loop.''',
                        reaction.eContainer,
                    Literals.REACTOR__REACTIONS,
                    reactor.reactions.indexOf(reaction))
                } else if (effects.size == 0) {
                    error(
                    '''Cyclic dependency due to succeeding reaction. Consider reordering reactions within reactor «reactor.name» to avoid causality loop.''',
                    reaction.eContainer,
                    Literals.REACTOR__REACTIONS,
                    reactor.reactions.indexOf(reaction))
                }
                // Not reporting reactions that are part of cycle _only_ due to reaction ordering.
                // Moving them won't help solve the problem.
            }
        }
    // FIXME: improve error message.
    }

    @Check(FAST)
    def checkReactor(Reactor reactor) {
        val name = FileConfig.nameWithoutExtension(reactor.eResource)
        if (reactor.name === null) {
            if (!reactor.isFederated && !reactor.isMain) {
                error(
                    "Reactor must be named.",
                    Literals.REACTOR_DECL__NAME
                )
            }
            // Prevent NPE in tests below.
            return
        } else {
            if (reactor.isFederated || reactor.isMain) {
                if(!reactor.name.equals(name)) {
                    // Make sure that if the name is omitted, the reactor is indeed main.
                    error(
                        "Name of main reactor must match the file name (or be omitted).",
                        Literals.REACTOR_DECL__NAME
                    )
                }
                // Do not allow multiple main/federated reactors.
                if (reactor.eResource.allContents.filter(Reactor).filter[it.isMain || it.isFederated].size > 1) {
                    var attribute = Literals.REACTOR__MAIN
                    if (reactor.isFederated) {
                       attribute = Literals.REACTOR__FEDERATED
                    }
                    if (reactor.isMain || reactor.isFederated) {
                        error(
                            "Multiple definitions of main or federated reactor.",
                            attribute
                        )
                    }
                }
            } else if (reactor.eResource.allContents.filter(Reactor).exists[it.isMain || it.isFederated] && reactor.name.equals(name)) {
                // Make sure that if a main reactor is specified, there are no
                // ordinary reactors that clash with it.
                error(
                    "Name conflict with main reactor.",
                    Literals.REACTOR_DECL__NAME
                )
            }
        }

        // If there is a main reactor (with no name) then disallow other (non-main) reactors
        // matching the file name.

        checkName(reactor.name, Literals.REACTOR_DECL__NAME)

        // C++ reactors may not be called 'preamble'
        if (this.target == Target.CPP && reactor.name.equalsIgnoreCase("preamble")) {
            error(
                "Reactor cannot be named '" + reactor.name + "'",
                Literals.REACTOR_DECL__NAME
            )
        }

        if (reactor.host !== null) {
            if (!reactor.isFederated) {
                error(
                    "Cannot assign a host to reactor '" + reactor.name +
                    "' because it is not federated.",
                    Literals.REACTOR__HOST
                )
            }
        }

        var variables = new ArrayList()
        variables.addAll(reactor.inputs)
        variables.addAll(reactor.outputs)
        variables.addAll(reactor.actions)
        variables.addAll(reactor.timers)

        // Perform checks on super classes.
        for (superClass : reactor.superClasses ?: emptyList) {
            var conflicts = new HashSet()

            // Detect input conflicts
            checkConflict(superClass.toDefinition.inputs, reactor.inputs, variables, conflicts)
            // Detect output conflicts
            checkConflict(superClass.toDefinition.outputs, reactor.outputs, variables, conflicts)
            // Detect output conflicts
            checkConflict(superClass.toDefinition.actions, reactor.actions, variables, conflicts)
            // Detect conflicts
            for (timer : superClass.toDefinition.timers) {
                if (timer.hasNameConflict(variables.filter[it | !reactor.timers.contains(it)])) {
                    conflicts.add(timer)
                } else {
                    variables.add(timer)
                }
            }

            // Report conflicts.
            if (conflicts.size > 0) {
                val names = new ArrayList();
                conflicts.forEach[it | names.add(it.name)]
                error(
                '''Cannot extend «superClass.name» due to the following conflicts: «names.join(',')».''',
                Literals.REACTOR__SUPER_CLASSES
                )
            }
        }
    }
    /**
     * For each input, report a conflict if:
     *   1) the input exists and the type doesn't match; or
     *   2) the input has a name clash with variable that is not an input.
     * @param superVars List of typed variables of a particular kind (i.e.,
     * inputs, outputs, or actions), found in a super class.
     * @param sameKind Typed variables of the same kind, found in the subclass.
     * @param allOwn Accumulator of non-conflicting variables incorporated in the
     * subclass.
     * @param conflicts Set of variables that are in conflict, to be used by this
     * function to report conflicts.
     */
    def <T extends TypedVariable> checkConflict (EList<T> superVars,
        EList<T> sameKind, List<Variable> allOwn,
        HashSet<Variable> conflicts) {
        for (superVar : superVars) {
                val match = sameKind.findFirst [ it |
                it.name.equals(superVar.name)
            ]
            val rest = allOwn.filter[it|!sameKind.contains(it)]
            if ((match !== null && superVar.type !== match.type) || superVar.hasNameConflict(rest)) {
                conflicts.add(superVar)
            } else {
                allOwn.add(superVar)
            }
        }
    }

    /**
     * Report whether the name of the given element matches any variable in
     * the ones to check against.
     * @param element The element to compare against all variables in the given iterable.
     * @param toCheckAgainst Iterable variables to compare the given element against.
     */
    def boolean hasNameConflict(Variable element,
        Iterable<Variable> toCheckAgainst) {
        if (toCheckAgainst.filter[it|it.name.equals(element.name)].size > 0) {
            return true
        }
        return false
    }

    @Check(FAST)
    def checkHost(Host host) {
        val addr = host.addr
        val user = host.user
        if (user !== null && !user.matches(usernameRegex)) {
            warning(
                "Invalid user name.",
                Literals.HOST__USER
            )
        }
        if (host instanceof IPV4Host && !addr.matches(ipv4Regex)) {
            warning(
                "Invalid IP address.",
                Literals.HOST__ADDR
            )
        } else if (host instanceof IPV6Host && !addr.matches(ipv6Regex)) {
            warning(
                "Invalid IP address.",
                Literals.HOST__ADDR
            )
        } else if (host instanceof NamedHost && !addr.matches(hostOrFQNRegex)) {
            warning(
                "Invalid host name or fully qualified domain name.",
                Literals.HOST__ADDR
            )
        }
    }
    
    /**
     * Check if the requested serialization is supported.
     */
    @Check(FAST)
    def checkSerializer(Serializer serializer) {
        var boolean isValidSerializer = false;
        for (SupportedSerializers method : SupportedSerializers.values()) {
          if (method.name().equalsIgnoreCase(serializer.type)){
              isValidSerializer = true;
          }          
        }
        
        if (!isValidSerializer) {
            error(
                "Serializer can be " + SupportedSerializers.values.toList, 
                Literals.SERIALIZER__TYPE
            );
        }
    }

    @Check(FAST)
    def checkState(StateVar stateVar) {
        checkName(stateVar.name, Literals.VARIABLE__NAME)

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
                                        "Missing time unit.", Literals.STATE_VAR__INIT)
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

        if (isCBasedTarget && stateVar.init.size > 1) {
            // In C, if initialization is done with a list, elements cannot
            // refer to parameters.
            if (stateVar.init.exists[it.parameter !== null]) {
                error("List items cannot refer to a parameter.",
                    Literals.STATE_VAR__INIT)
            }
        }
        
        if(!stateVar.braces.isNullOrEmpty && this.target != Target.CPP) {
            error("Brace initializers are only supported for the C++ target", Literals.STATE_VAR__BRACES)
        }
    }

    @Check(FAST)
    def checkTargetDecl(TargetDecl target) {
        val targetOpt = Target.forName(target.name);
        if (targetOpt.isEmpty()) {
            error("Unrecognized target: " + target.name,
                Literals.TARGET_DECL__NAME)
        } else {
            this.target = targetOpt.get();
        }
    }

    /**
     * Check for consistency of the target properties, which are
     * defined as KeyValuePairs.
     *
     * @param targetProperties The target properties defined
     *  in the current Lingua Franca program.
     */
    @Check(EXPENSIVE)
    def checkTargetProperties(KeyValuePairs targetProperties) {
        
        val fastTargetProperties = targetProperties.pairs.filter(
            pair |
                // Check to see if fast is defined
                TargetProperty.forName(pair.name) == TargetProperty.FAST
        )
        
        val fastTargetProperty = fastTargetProperties.findFirst[t | true];

        if (fastTargetProperty !== null) {
            // Check for federated
            if (info.model.reactors.exists(
                reactor |
                    // Check to see if the program has a federated reactor
                    reactor.isFederated
            )) {
                error(
                    "The fast target property is incompatible with federated programs.",
                    fastTargetProperty,
                    Literals.KEY_VALUE_PAIR__NAME
                )
            }
            
            // Check for physical actions
            if (info.model.reactors.exists(
                reactor |
                    // Check to see if the program has a physical action in a reactor
                    reactor.actions.exists(action|(action.origin == ActionOrigin.PHYSICAL))
            )) {
                error(
                    "The fast target property is incompatible with physical actions.",
                    fastTargetProperty,
                    Literals.KEY_VALUE_PAIR__NAME
                )
            }

        }
        
        val clockSyncTargetProperties = targetProperties.pairs.filter(
            pair |
                // Check to see if clock-sync is defined
                TargetProperty.forName(pair.name) == TargetProperty.CLOCK_SYNC
        )
        
        val clockSyncTargetProperty = clockSyncTargetProperties.findFirst[t | true];
        if (clockSyncTargetProperty !== null) {
            if (info.model.reactors.exists(
                reactor |
                    // Check to see if the program has a federated reactor defined.
                    reactor.isFederated
            ) == false) {
                warning(
                    "The clock-sync target property is incompatible with non-federated programs.",
                    clockSyncTargetProperty,
                    Literals.KEY_VALUE_PAIR__NAME
                )
            }
        }
    }

    @Check(FAST)
    def checkValueAsTime(Value value) {
        val container = value.eContainer

        if (container instanceof Timer || container instanceof Action ||
            container instanceof Connection || container instanceof Deadline) {

            // If parameter is referenced, check that it is of the correct type.
            if (value.parameter !== null) {
                if (!value.parameter.isOfTimeType && target.requiresTypes === true) {
                    error("Parameter is not of time type",
                        Literals.VALUE__PARAMETER)
                }
            } else if (value.time === null) {
                if (value.literal !== null && !value.literal.isZero) {
                    if (value.literal.isInteger) {
                            error("Missing time unit.", Literals.VALUE__LITERAL)
                        } else {
                            error("Invalid time literal.",
                                Literals.VALUE__LITERAL)
                        }
                } else if (value.code !== null) {
                     error("Invalid time literal.", Literals.VALUE__CODE)
                }
            }
        }
    }

    @Check(FAST)
    def checkTimer(Timer timer) {
        checkName(timer.name, Literals.VARIABLE__NAME)
    }

    @Check(FAST)
    def checkType(Type type) {
        // FIXME: disallow the use of generics in C
        if (this.target == Target.CPP) {
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
        else if (this.target == Target.Python) {
            if (type !== null) {
                error(
                    "Types are not allowed in the Python target",
                    Literals.TYPE__ID
                )
            }
        }
    }
    
    @Check(FAST)
    def checkVarRef(VarRef varRef) {
        // check correct usage of interleaved
        if (varRef.isInterleaved) {
            if (this.target != Target.CPP && !isCBasedTarget && this.target != Target.Python) {
                error("This target does not support interleaved port references.", Literals.VAR_REF__INTERLEAVED)
            }
            if (!(varRef.eContainer instanceof Connection)) {
                error("interleaved can only be used in connections.", Literals.VAR_REF__INTERLEAVED)
            }

            if (varRef.variable instanceof Port) {
                // This test only works correctly if the variable is actually a port. If it is not a port, other
                // validator rules will produce error messages.
                if (varRef.container === null || varRef.container.widthSpec === null ||
                    (varRef.variable as Port).widthSpec === null
                ) {
                    error("interleaved can only be used for multiports contained within banks.", Literals.VAR_REF__INTERLEAVED)
                }
            }
        }
    }

    static val UNDERSCORE_MESSAGE = "Names of objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation) may not start with \"__\": "
    static val ACTIONS_MESSAGE = "\"actions\" is a reserved word for the TypeScript target for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation): "
    static val RESERVED_MESSAGE = "Reserved words in the target language are not allowed for objects (inputs, outputs, actions, timers, parameters, state, reactor definitions, and reactor instantiation): "

}
