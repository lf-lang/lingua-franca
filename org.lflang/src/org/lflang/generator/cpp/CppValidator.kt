package org.lflang.generator.cpp

import org.lflang.ErrorReporter
import org.lflang.generator.ValidationStrategy
import org.lflang.generator.CodeMap
import org.lflang.generator.DiagnosticReporting
import org.lflang.generator.HumanReadableReportingStrategy
import org.lflang.generator.Validator
import org.lflang.util.LFCommand
import java.io.File
import java.nio.file.Path
import java.nio.file.Paths
import java.util.regex.Pattern

/**
 * A validator for generated C++.
 *
 * @author Peter Donovan <peterdonovan@berkeley.edu>
 */
class CppValidator(
    private val fileConfig: CppFileConfig,
    errorReporter: ErrorReporter,
    codeMaps: Map<Path, CodeMap>
): Validator(errorReporter, codeMaps) {

    companion object {
        /** This matches a line in the CMake cache. */
        private val CMAKE_CACHED_VARIABLE: Pattern = Pattern.compile("(?<name>[\\w_]+):(?<type>[\\w_]+)=(?<value>.*)")

        /** This matches a line of error reports from g++. */
        private val GXX_ERROR_LINE: Pattern = Pattern.compile(
            "(?<path>.+\\.((cc)|(hh))):(?<line>\\d+):(?<column>\\d+): (?<severity>(error)|(warning)): (?<message>.*?) ?(?<type>(\\[.*])?)"
        )
        private val GXX_LABEL: Pattern = Pattern.compile("(~*)(\\^~*)")
        // Happily, the two tools seem to produce errors that follow the same format.
        /** This matches a line of error reports from Clang-Tidy. */
        private val CLANG_TIDY_ERROR_LINE: Pattern = GXX_ERROR_LINE
        private val CLANG_TIDY_LABEL: Pattern = GXX_LABEL
        /** This matches a line of error reports from MSVC.  */
        private val MSVC_ERROR_LINE: Pattern = Pattern.compile(
            "(?<path>.+\\.((cc)|(hh)))\\((?<line>\\d+)(,\\s*(?<column>\\d+))?\\)\\s*:.*?(?<severity>(error)|(warning)) [A-Z]+\\d+:\\s*(?<message>.*?)"
        )
        private val MSVC_LABEL: Pattern = Pattern.compile("(?<=\\s)\\^(?=\\s)")  // Unused with the current MSVC settings
    }

    private class CppValidationStrategy(
        private val errorReportingStrategy: DiagnosticReporting.Strategy,
        private val outputReportingStrategy: DiagnosticReporting.Strategy,
        private val time: Int,
        private val getCommand: (p: Path) -> LFCommand?
    ): ValidationStrategy {

        override fun getCommand(generatedFile: Path) = getCommand.invoke(generatedFile)

        override fun getErrorReportingStrategy() = errorReportingStrategy

        override fun getOutputReportingStrategy() = outputReportingStrategy

        override fun isFullBatch() = false

        override fun getPriority() = -time
    }

    /**
     * [CppValidationStrategyFactory] instances map validator instances to validation strategy instances.
     * @param compilerId The CMake compiler ID of the compiler whose output is most similar to the output
     * used by the given strategy.
     * @param create The function that creates a strategy from a validator.
     */
    private enum class CppValidationStrategyFactory(val compilerId: String, val create: ((CppValidator) -> CppValidationStrategy)) {

        // Note: Clang-tidy is slow (on the order of tens of seconds) for checking C++ files.
        CLANG_TIDY("Clang", { cppValidator -> CppValidationStrategy(
            { _, _, _ -> },
            HumanReadableReportingStrategy(CLANG_TIDY_ERROR_LINE, CLANG_TIDY_LABEL),
            5,
            { generatedFile: Path ->
                val args = mutableListOf(generatedFile.toString(), "--checks=*", "--quiet", "--", "-std=c++${cppValidator.cppStandard}")
                cppValidator.includes.forEach { args.add("-I$it") }
                LFCommand.get("clang-tidy", args, cppValidator.fileConfig.outPath)
            }
        )}),
        GXX("GNU", { cppValidator -> CppValidationStrategy(
            HumanReadableReportingStrategy(GXX_ERROR_LINE, GXX_LABEL),
            { _, _, _ -> },
            1,
            { generatedFile: Path ->
                val args: MutableList<String> = mutableListOf("-fsyntax-only", "-Wall", "-std=c++${cppValidator.cppStandard}")
                cppValidator.includes.forEach { args.add("-I$it") }
                args.add(generatedFile.toString())
                LFCommand.get("g++", args, cppValidator.fileConfig.outPath)
            }
        )}),
        MSVC("MSVC", { cppValidator -> CppValidationStrategy(
            { _, _, _ -> },
            HumanReadableReportingStrategy(MSVC_ERROR_LINE, MSVC_LABEL),
            3,
            { generatedFile: Path ->
                cppValidator.cmakeGeneratorInstance?.let { path ->
                    val setUpDeveloperEnvironment: Path = Paths.get(path)
                        .resolve("Common7${File.separator}Tools${File.separator}VsDevCmd.bat")
                    val args: MutableList<String> = mutableListOf("&", "cl", "/Zs", "/diagnostics:column", "/std:c++${cppValidator.cppStandard}")
                    cppValidator.includes.forEach { args.add("/I$it") }
                    args.add(generatedFile.toString())
                    LFCommand.get(setUpDeveloperEnvironment.toString(), args, cppValidator.fileConfig.outPath)
                }
            }
        )});
    }

    /**
     * Return the appropriate output and error reporting
     * strategies for the build process carried out by
     * CMake and Make.
     */
    override fun getBuildReportingStrategies(): Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy> {
        val compilerId: String = getFromCache(CppCmakeGenerator.compilerIdName) ?: "GNU"  // This is just a guess.
        val mostSimilarValidationStrategy = CppValidationStrategyFactory.values().find { it.compilerId == compilerId }
        if (mostSimilarValidationStrategy === null) {
            return Pair(DiagnosticReporting.Strategy { _, _, _ -> }, DiagnosticReporting.Strategy { _, _, _ -> })
        }
        return Pair(
            mostSimilarValidationStrategy.create(this).errorReportingStrategy,
            mostSimilarValidationStrategy.create(this).outputReportingStrategy,
        )
    }

    private fun getFromCache(variableName: String): String? {
        if (cmakeCache.exists()) cmakeCache.useLines {
            for (line in it) {
                val matcher = CMAKE_CACHED_VARIABLE.matcher(line)
                if (matcher.matches() && matcher.group("name") == variableName) return matcher.group("value")
            }
        }
        return null
    }

    /** The include directories required by the generated files. */
    private val includes: List<String>
        get() = getFromCache(CppCmakeGenerator.includesVarName)?.split(';') ?: listOf()

    /** The C++ standard used by the generated files. */
    private val cppStandard: String?
        get() = getFromCache("CMAKE_CXX_STANDARD")

    /** The desired instance of the C++ build system. */
    private val cmakeGeneratorInstance: String?
        get() = getFromCache("CMAKE_GENERATOR_INSTANCE")

    /** The CMake cache. */
    private val cmakeCache: File = fileConfig.buildPath.resolve("CMakeCache.txt").toFile()

    override fun getPossibleStrategies(): Collection<ValidationStrategy> = CppValidationStrategyFactory.values().map { it.create(this) }
}
