package org.lflang.generator.cpp

import org.lflang.MessageReporter
import org.lflang.TargetConfig
import org.lflang.generator.GeneratorCommandFactory
import org.lflang.generator.LFGeneratorContext
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
            "-DCMAKE_BUILD_TYPE=${targetConfig.cmakeBuildType}",
            "-DREACTOR_CPP_VALIDATE=${if (targetConfig.noRuntimeValidation) "OFF" else "ON"}",
            "-DREACTOR_CPP_PRINT_STATISTICS=${if (targetConfig.printStatistics) "ON" else "OFF"}",
            "-DREACTOR_CPP_TRACE=${if (targetConfig.tracing != null) "ON" else "OFF"}",
            "-DREACTOR_CPP_LOG_LEVEL=${targetConfig.logLevel.severity}",
            "-DLF_SRC_PKG_PATH=${fileConfig.srcPkgPath}",
        )
}
