package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.federated.FederateInstance
import org.lflang.generator.getTargetInitializer
import org.lflang.isBank
import org.lflang.joinWithLn
import org.lflang.lf.Instantiation
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import org.lflang.lf.TypeParm
import org.lflang.reactor
import org.lflang.toDefinition
import org.lflang.toText
import java.util.*

/**
 * Generator for child reactor instantiations in TypeScript target.
 */
class TSInstanceGenerator(
    private val errorReporter: ErrorReporter,
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

    private fun getTypeParams(typeParms: List<TypeParm>): String =
        if (typeParms.isEmpty()) ""
        else typeParms.joinToString(", ", "<", ">") { it.toText() }

    private fun getReactorParameterList(parameters: List<Parameter>): String =
        parameters.joinToString(", ", "[__Reactor, ", "]") { TSTypes.getTargetType(it) }


    fun generateClassProperties(): String =
        childReactors.joinWithLn { childReactor ->
            if (childReactor.isBank) {
                "${childReactor.name}: " +
                        "__Bank<${childReactor.reactorClass.name}${getTypeParams(childReactor.typeParms)}, " +
                        "${getReactorParameterList(childReactor.reactor.parameters)}>"
            } else {
                "${childReactor.name}: " +
                        "${childReactor.reactorClass.name}${getTypeParams(childReactor.typeParms)}"
            }
        }

    fun generateInstantiations(): String {
        val childReactorInstantiations = LinkedList<String>()
        for (childReactor in childReactors) {
            val childReactorArguments = StringJoiner(", ");
            childReactorArguments.add("this")

            for (parameter in childReactor.reactorClass.toDefinition().parameters) {
                childReactorArguments.add(TSTypes.getTargetInitializer(parameter, childReactor))
            }
            if (childReactor.isBank) {
                childReactorInstantiations.add(
                    "this.${childReactor.name} = " +
                            "new __Bank<${childReactor.reactorClass.name}${getTypeParams(childReactor.typeParms)}, " +
                            "${getReactorParameterList(childReactor.reactor.parameters)}>" +
                            "(this, ${childReactor.widthSpec.toTSCode()}, " +
                            "${childReactor.reactorClass.name}, " +
                            "$childReactorArguments)")
            } else {
                childReactorInstantiations.add(
                    "this.${childReactor.name} = " +
                            "new ${childReactor.reactorClass.name}($childReactorArguments)")
            }
        }
        return childReactorInstantiations.joinToString("\n")
    }
}
