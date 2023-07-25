package org.lflang.generator.ts

import org.lflang.generator.getTargetInitializer
import org.lflang.isBank
import org.lflang.joinWithLn
import org.lflang.lf.*
import org.lflang.reactor
import org.lflang.toDefinition
import org.lflang.toText
import java.util.*

/**
 * Generator for child reactor instantiations in TypeScript target.
 */
class TSInstanceGenerator(
    reactor: Reactor
) {
    private val childReactors: List<Instantiation>

    init {
        // Next handle child reactors instantiations.
        // instantiate all the child reactors.
        childReactors = reactor.instantiations
    }

    private fun getTypeParams(typeParms: List<Type>): String =
        if (typeParms.isEmpty()) ""
        else typeParms.joinToString(", ", "<", ">") { TSTypes.getInstance().getTargetType(it) }

    private fun getReactorParameterList(parameters: List<Parameter>): String =
        parameters.joinToString(", ", "[__Reactor, ", "]") { TSTypes.getInstance().getTargetType(it) }


    fun generateClassProperties(): String =
        childReactors.joinWithLn { childReactor ->
            if (childReactor.isBank) {
                "${childReactor.name}: " +
                        "__Bank<${childReactor.reactorClass.name}${getTypeParams(childReactor.typeArgs)}, " +
                        "${getReactorParameterList(childReactor.reactor.parameters)}>"
            } else {
                "${childReactor.name}: " +
                        "${childReactor.reactorClass.name}${getTypeParams(childReactor.typeArgs)}"
            }
        }

    fun generateInstantiations(): String {
        val childReactorInstantiations = LinkedList<String>()
        var portID = 0
        for (childReactor in childReactors) {
            val childReactorArguments = StringJoiner(", ")
            childReactorArguments.add("this")
            if (childReactor.reactorClass.name.take(15) == "NetworkReceiver") {
                // Assume that network receiver reactors are sorted by portID
                childReactorArguments.add(portID.toString())
                portID++
            }

            for (parameter in childReactor.reactorClass.toDefinition().parameters) {
                childReactorArguments.add(TSTypes.getInstance().getTargetInitializer(parameter, childReactor))
            }
            if (childReactor.isBank) {
                childReactorInstantiations.add(
                    "this.${childReactor.name} = " +
                            "new __Bank<${childReactor.reactorClass.name}${getTypeParams(childReactor.typeArgs)}, " +
                            "${getReactorParameterList(childReactor.reactor.parameters)}>" +
                            "(this, ${childReactor.widthSpec.toTSCode()}, " +
                            "${childReactor.reactorClass.name}, " +
                            "$childReactorArguments)")
            } else {
                childReactorInstantiations.add(
                    "this.${childReactor.name} = " +
                            "new ${childReactor.reactorClass.name}($childReactorArguments)")
                if (childReactor.reactorClass.name.take(15) == "NetworkReceiver") {
                    childReactorInstantiations.add(
                        "this.registerNetworkReceiver(\n"
                        + "\tthis.${childReactor.name} as __NetworkReactor<unknown>\n)")
                }
                if (childReactor.reactorClass.name.take(13) == "NetworkSender") {
                    childReactorInstantiations.add(
                        "this.registerNetworkSender(this.${childReactor.name})")
                }
            }
        }
        return childReactorInstantiations.joinToString("\n")
    }
}
