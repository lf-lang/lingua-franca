package org.lflang.generator.cpp

import org.lflang.ErrorReporter
import org.lflang.TargetConfig
import org.lflang.generator.GeneratorCommandFactory
import org.lflang.generator.LFGeneratorContext
import org.lflang.toDefinition
import java.nio.file.Path

/** Abstract class for generating platform specific files and invoking the target compiler. */
abstract class CppPlatformGenerator(protected val generator: CppGenerator) {
    protected val codeMaps = generator.codeMaps
    protected val cppSources = generator.cppSources
    protected val errorReporter: ErrorReporter = generator.errorReporter
    protected val fileConfig: CppFileConfig = generator.fileConfig
    protected val targetConfig: TargetConfig = generator.targetConfig
    protected val commandFactory: GeneratorCommandFactory = generator.commandFactory
    protected val mainReactor = generator.mainDef.reactorClass.toDefinition()

    open val srcGenPath: Path = generator.fileConfig.srcGenPath

    abstract fun generatePlatformFiles()

    abstract fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean = false): Boolean
}