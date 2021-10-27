package org.lflang.generator.ts

import org.lflang.federated.FederateInstance
import org.lflang.lf.Instantiation
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import org.lflang.toDefinition
import org.lflang.toText
import java.util.*

/**
 * Generator for child reactor instantiations in TypeScript target.
 */
class TSInstanceGenerator (
    // TODO(hokeun): Remove dependency on TSGenerator.
    private val tsGenerator: TSGenerator,
    private val tsReactorGenerator: TSReactorGenerator,
    reactor: Reactor,
    federate: FederateInstance
) {
    private val childReactors: List<Instantiation>

    init {
        // Next handle child reactors instantiations.
        // If the app isn't federated, instantiate all
        // the child reactors. If the app is federated
        if (!reactor.isFederated) {
            childReactors = reactor.instantiations
        } else {
            childReactors = LinkedList<Instantiation>()
            childReactors.add(federate.instantiation)
        }
    }

    private fun getInitializerList(param: Parameter, i: Instantiation): List<String> =
        tsGenerator.getInitializerListW(param, i)

    private fun getTargetInitializer(param: Parameter, i: Instantiation): String {
        return tsReactorGenerator.getTargetInitializerHelper(param, getInitializerList(param, i))
    }

    fun generateClassProperties(): String {
        val childReactorClassProperties = LinkedList<String>()
        for (childReactor in childReactors) {
            val childReactorParams = if (childReactor.typeParms.isEmpty()) {""} else {
                childReactor.typeParms.joinToString(", ", "<", ">") { it.toText() }}
            childReactorClassProperties.add("${childReactor.name}: ${childReactor.reactorClass.name}$childReactorParams")
        }
        return childReactorClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val childReactorInstantiations = LinkedList<String>()
        for (childReactor in childReactors) {
            val childReactorArguments = StringJoiner(", ");
            childReactorArguments.add("this")

            // Iterate through parameters in the order they appear in the
            // reactor class, find the matching parameter assignments in
            // the reactor instance, and write the corresponding parameter
            // value as an argument for the TypeScript constructor
            for (parameter in childReactor.reactorClass.toDefinition().parameters) {
                childReactorArguments.add(getTargetInitializer(parameter, childReactor))
            }

            childReactorInstantiations.add(
                "this.${childReactor.name} = new ${childReactor.reactorClass.name}($childReactorArguments)")
        }
        return childReactorInstantiations.joinToString("\n")
    }
}