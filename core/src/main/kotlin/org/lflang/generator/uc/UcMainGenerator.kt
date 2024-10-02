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

class UcMainGenerator(
    private val main: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: UcFileConfig,
) {
    fun generateMainSource() = with(PrependOperator) {
        """
            |#include "reactor-uc/reactor-uc.h"
            |#include "${fileConfig.getReactorHeaderPath(main).toUnixString()}"
            |static Environment env;
            |static ${main.name} main_reactor;
            |int lf_main(int argc, char **argv) {
            |   Environment_ctor(&env, &main_reactor.super);
            |   ${if (targetConfig.isSet(TimeOutProperty.INSTANCE)) "env.set_stop_time(&env, ${targetConfig.get(TimeOutProperty.INSTANCE).toCppCode()};" else ""}
            |   ${main.name}_ctor(&main_reactor, &env, NULL);
            |   env.assemble(&env);
            |   env.start(&env);
            |}
            |
            |// The following is to support convenient compilation of LF programs
            |// targeting POSIX. Only for testing purposes.
            |#ifdef PLATFORM_POSIX
            |int main(int argc, char **argv) {
            |   lf_main(argc, argv);
            |}
            |#endif
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
