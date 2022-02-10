package org.lflang.generator.cpp

import org.lflang.ErrorReporter
import org.lflang.TargetConfig
import org.lflang.generator.GeneratorCommandFactory
import org.lflang.generator.LFGeneratorContext

abstract class CppPlatformGenerator(protected val generator: CppGenerator) {
    protected val codeMaps = generator.codeMaps
    protected val cppSources = generator.cppSources
    protected val errorReporter: ErrorReporter = generator.errorReporter
    protected val fileConfig = generator.cppFileConfig
    protected val targetConfig: TargetConfig = generator.targetConfig
    protected val relSrcGenPath = generator.relSrcGenPath
    protected val commandFactory: GeneratorCommandFactory = generator.commandFactory

    abstract fun generatePlatformFiles()

    abstract fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean = false): Boolean
}