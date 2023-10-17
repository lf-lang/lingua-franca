package org.lflang.generator.cpp

import org.lflang.MessageReporter
import org.lflang.target.TargetConfig
import org.lflang.generator.GeneratorCommandFactory
import org.lflang.generator.LFGeneratorContext
import org.lflang.target.property.BuildTypeProperty
import org.lflang.target.property.LoggingProperty
import org.lflang.target.property.NoRuntimeValidationProperty
import org.lflang.target.property.PrintStatisticsProperty
import org.lflang.target.property.TracingProperty
import org.lflang.toDefinition
import java.nio.file.Path

/** Abstract class for generating platform specific files and invoking the target compiler. */
abstract class CppPlatformGenerator(protected val generator: CppGenerator) {
    protected val codeMaps = generator.codeMaps
    protected val cppSources = generator.cppSources
    protected val messageReporter: MessageReporter = generator.messageReporter
    protected val fileConfig: CppFileConfig = generator.fileConfig
    protected val targetConfig: TargetConfig = generator.targetConfig
    protected val commandFactory: GeneratorCommandFactory = generator.commandFactory
    protected val mainReactor = generator.mainDef.reactorClass.toDefinition()

    open val srcGenPath: Path = generator.fileConfig.srcGenPath

    abstract fun generatePlatformFiles()

    abstract fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean = false): Boolean

    protected val cmakeArgs: List<String>
        get() = listOf(
            "-DCMAKE_BUILD_TYPE=${targetConfig.get(BuildTypeProperty.INSTANCE)}",
            "-DREACTOR_CPP_VALIDATE=${if (targetConfig.get(NoRuntimeValidationProperty())) "OFF" else "ON"}",
            "-DREACTOR_CPP_PRINT_STATISTICS=${if (targetConfig.get(PrintStatisticsProperty())) "ON" else "OFF"}",
            "-DREACTOR_CPP_TRACE=${if (targetConfig.get(TracingProperty()).isEnabled) "ON" else "OFF"}",
            "-DREACTOR_CPP_LOG_LEVEL=${targetConfig.get(LoggingProperty.INSTANCE).severity}",
            "-DLF_SRC_PKG_PATH=${fileConfig.srcPkgPath}",
        )
}
