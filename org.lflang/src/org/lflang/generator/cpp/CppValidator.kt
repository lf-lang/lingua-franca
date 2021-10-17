package org.lflang.generator.cpp

import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.generator.CodeMap
import org.lflang.generator.CommandErrorReportingStrategy
import org.lflang.generator.PerLineReportingStrategy
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.util.regex.Pattern

class CppValidator(
    private val fileConfig: CppFileConfig,
    private val errorReporter: ErrorReporter,
    private val codeMaps: Map<Path, CodeMap>
) {

    companion object {
        val cmakeCxxStandard: Pattern = Pattern.compile("CMAKE_CXX_STANDARD:STRING=(?<cppStandard>.*)")
        val cmakeIncludes: Pattern = Pattern.compile("${CppCmakeGenerator.includesVarName}:STRING=(?<includes>.*)")

        private val gxxErrorLine: Pattern = Pattern.compile(
            "(?<path>.+\\.((cc)|(hh))):(?<line>\\d+):(?<column>\\d+): (?<severity>(error)|(warning)): (?<message>.*)"
        )
        // Happily, the two tools seem to produce errors that follow the same format.
        private val clangTidyErrorLine: Pattern = gxxErrorLine
    }

    enum class CppValidationStrategy(
        val errorParsingStrategy: CommandErrorReportingStrategy,
        val outputParsingStrategy: CommandErrorReportingStrategy,
        val time: Int
    ) {
        // Note: Clang-tidy is slow (on the order of tens of seconds) for checking C++ files.
        CLANG_TIDY({ _, _, _ -> }, PerLineReportingStrategy(clangTidyErrorLine), 5) {
            override fun getCommand(validator: CppValidator, generatedFile: Path): LFCommand? {
                val args = mutableListOf(generatedFile.toString(), "--checks=*", "--quiet", "--", "-std=c++${validator.cppStandard}")
                validator.includes.forEach { args.add("-I${it}") }
                return LFCommand.get("clang-tidy", args, validator.fileConfig.outPath)
            }
        },
        GXX(PerLineReportingStrategy(gxxErrorLine), { _, _, _ -> }, 1) {
            override fun getCommand(validator: CppValidator, generatedFile: Path): LFCommand? {
                val args: MutableList<String> = mutableListOf("-fsyntax-only", "-Wall", "-std=c++${validator.cppStandard}")
                validator.includes.forEach { args.add("-I${it}") }
                args.add(generatedFile.toString())
                return LFCommand.get("g++", args, validator.fileConfig.outPath)
            }
        };

        abstract fun getCommand(validator: CppValidator, generatedFile: Path): LFCommand?
    }

    fun doValidate(cancelIndicator: CancelIndicator) {
        if (!cmakeCachePath.toFile().exists()) return
        for (generatedFile: Path in codeMaps.keys) {
            // FIXME: Respond to cancel. Only validate changed files?
            validateFile(generatedFile, cancelIndicator)
            if (cancelIndicator.isCanceled) return
        }
    }

    private fun validateFile(generatedFile: Path, cancelIndicator: CancelIndicator) {
        for (strategy in CppValidationStrategy.values().sortedBy {strategy -> strategy.time}) {
            val validateCommand = strategy.getCommand(this, generatedFile)
            if (validateCommand != null) {
                validateCommand.run(cancelIndicator)
                strategy.outputParsingStrategy.report(validateCommand.output.toString(), errorReporter, codeMaps[generatedFile])
                strategy.errorParsingStrategy.report(validateCommand.errors.toString(), errorReporter, codeMaps[generatedFile])
                break
            }
        }
    }

    private val includes: List<String>
        get() {
            for (line in cmakeCache) {
                val matcher = cmakeIncludes.matcher(line)
                if (matcher.matches()) return matcher.group("includes").split(';')
            }
            return listOf()
        }

    private val cppStandard: String
        get() {
            for (line in cmakeCache) {
                val matcher = cmakeCxxStandard.matcher(line)
                if (matcher.matches()) return matcher.group("cppStandard")
            }
            return ""
        }

    private val cmakeCache: List<String> by lazy { Files.readAllLines(cmakeCachePath) }

    private val cmakeCachePath: Path = fileConfig.buildPath.resolve("CMakeCache.txt")
}