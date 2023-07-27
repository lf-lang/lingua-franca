package org.lflang.generator.ts

import org.lflang.AttributeUtils
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
            var isNetworkSender = false
            var isNetworkReceiver = false
            val networkReactorAttribute = AttributeUtils.findAttributeByName(childReactor.reactorClass, "_NetworkReactor")
            if (networkReactorAttribute != null) {
                isNetworkSender = networkReactorAttribute.getAttrParms().get(0).getName() == "Sender"
                isNetworkReceiver = networkReactorAttribute.getAttrParms().get(0).getName() == "Receiver"
            }
            val childReactorArguments = StringJoiner(", ")
            childReactorArguments.add("this")

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
                if (isNetworkReceiver) {
                    // Assume that network receiver reactors are sorted by portID
                    childReactorInstantiations.add(
                        "this.registerNetworkReceiver(\n"
                        + "\t${portID},\n"
                        + "\tthis.${childReactor.name} as __NetworkReceiver<unknown>\n)")
                }
                if (isNetworkSender) {
                    childReactorInstantiations.add(
                        "this.registerNetworkSender(this.${childReactor.name})")
                }
            }
        }
        return childReactorInstantiations.joinToString("\n")
    }
}
