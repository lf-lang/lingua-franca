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
        else typeParms.joinToString(", ", "<", ">") { TSTypes.getTargetType(it) }

    private fun getReactorParameterList(parameters: List<Parameter>): String =
        parameters.joinToString(", ", "[__Reactor, ", "]") { TSTypes.getTargetType(it) }


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
        for (childReactor in childReactors) {
            val childReactorArguments = StringJoiner(", ")
            childReactorArguments.add("this")

            for (parameter in childReactor.reactorClass.toDefinition().parameters) {
                childReactorArguments.add(TSTypes.getTargetInitializer(parameter, childReactor))
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
            }
        }
        return childReactorInstantiations.joinToString("\n")
    }
}
