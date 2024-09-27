package org.lflang.generator.uc

import org.lflang.target.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.generator.cpp.toCppCode
import org.lflang.inferredType
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import org.lflang.target.property.ExportDependencyGraphProperty
import org.lflang.target.property.ExportToYamlProperty
import org.lflang.target.property.FastProperty
import org.lflang.target.property.TimeOutProperty
import org.lflang.target.property.WorkersProperty
import org.lflang.toUnixString

/** C++ code generator responsible for generating the main file including the main() function */
class UcMainGenerator(
    private val main: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: UcFileConfig,
) {
    // Cxxopts generation
//    private fun generateParameterParser(param: Parameter): String {
//        with(CppParameterGenerator) {
//            with(param) {
//                return if(inferredType.isTime) {
//                    """
//                        $targetType $name${CppTypes.getCppInitializer(init, inferredType)};
//                        options
//                            .add_options()("$name", "The $name parameter passed to the main reactor ${main.name}.", cxxopts::value<$targetType>($name)->default_value(time_to_string($name)), "'FLOAT UNIT'");
//                    """.trimIndent()
//                } else {
//                    """
//                        $targetType $name${CppTypes.getCppInitializer(init, inferredType)};
//                        options
//                            .add_options()("$name", "The $name parameter passed to the main reactor ${main.name}.", cxxopts::value<$targetType>($name)->default_value(any_to_string($name)), "'$targetType'");
//                    """.trimIndent()
//                }
//            }
//        }
//    }
//
//    private fun generateMainReactorInstantiation(): String =
//            """auto main = std ::make_unique<${main.name}> ("${main.name}", &e, ${main.name}::Parameters{${main.parameters.joinToString(", ") { ".${it.name} = ${it.name}" }}});"""
//
    fun generateMainSource() = with(PrependOperator) {
        """
            |#include "reactor-uc/reactor-uc.h"
            |#include "${fileConfig.getReactorHeaderPath(main).toUnixString()}"
            |static Environment env;
            |static ${main.name} main_reactor;
            |int lf_main(int argc, char **argv) {
            |   Environment_ctor(&env, &main_reactor.super);
            |   ${if (targetConfig.isSet(TimeOutProperty.INSTANCE)) "env.set_stop_time(&env, ${targetConfig.get(TimeOutProperty.INSTANCE).toCppCode()};" else ""}
            |   ${main.name}_ctor(&main_reactor, &env);
            |   env.assemble(&env);
            |   env.start(&env);
            |}
        """.trimMargin()
    }

    fun generateMainHeader() = with(PrependOperator) {
        """
            |#ifndef REACTOR_UC_LF_MAIN_H
            |#define REACTOR_UC_LF_MAIN_H
            |
            |int lf_main(int argc, char **argv);
            |
            |#endif
            |
        """.trimMargin()
    }
}
