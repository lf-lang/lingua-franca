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
import java.util.regex.Pattern

class CppValidator(
    private val fileConfig: CppFileConfig,
    errorReporter: ErrorReporter,
    codeMaps: Map<Path, CodeMap>
): Validator(errorReporter, codeMaps) {

    companion object {
        /** This matches the line in the CMake cache that states the C++ standard. */
        private val cmakeCxxStandard: Pattern = Pattern.compile("CMAKE_CXX_STANDARD:STRING=(?<cppStandard>.*)")
        /** This matches the line in the CMake cache that states the compiler includes for a given target. */
        private val cmakeIncludes: Pattern = Pattern.compile("${CppCmakeGenerator.includesVarName}:STRING=(?<includes>.*)")

        /** This matches a line of error reports from g++. */
        private val gxxErrorLine: Pattern = Pattern.compile(
            "(?<path>.+\\.((cc)|(hh))):(?<line>\\d+):(?<column>\\d+): (?<severity>(error)|(warning)): (?<message>.*?) ?(?<type>(\\[.*])?)"
        )
        private val gxxLabel: Pattern = Pattern.compile("(~*)(\\^~*)")
        // Happily, the two tools seem to produce errors that follow the same format.
        /** This matches a line of error reports from Clang-Tidy. */
        private val clangTidyErrorLine: Pattern = gxxErrorLine
        private val clangTidyLabel: Pattern = gxxLabel
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
     */
    private enum class CppValidationStrategyFactory(val create: (CppValidator) -> CppValidationStrategy) {

        // Note: Clang-tidy is slow (on the order of tens of seconds) for checking C++ files.
        CLANG_TIDY({ cppValidator -> CppValidationStrategy(
            { _, _, _ -> },
            HumanReadableReportingStrategy(clangTidyErrorLine, clangTidyLabel),
            5,
            { generatedFile: Path ->
                val args = mutableListOf(generatedFile.toString(), "--checks=*", "--quiet", "--", "-std=c++${cppValidator.cppStandard}")
                cppValidator.includes.forEach { args.add("-I$it") }
                LFCommand.get("clang-tidy", args, cppValidator.fileConfig.outPath)
            }
        )}),
        GXX({ cppValidator -> CppValidationStrategy(
            HumanReadableReportingStrategy(gxxErrorLine, gxxLabel),
            { _, _, _ -> },
            1,
            { generatedFile: Path ->
                val args: MutableList<String> = mutableListOf("-fsyntax-only", "-Wall", "-std=c++${cppValidator.cppStandard}")
                cppValidator.includes.forEach { args.add("-I$it") }
                args.add(generatedFile.toString())
                LFCommand.get("g++", args, cppValidator.fileConfig.outPath)
            }
        )});
    }

    /**
     * Returns the appropriate output and error reporting
     * strategies for the build process carried out by
     * CMake and Make.
     */
    override fun getBuildReportingStrategies(): Pair<DiagnosticReporting.Strategy, DiagnosticReporting.Strategy>
        = Pair(
            CppValidationStrategyFactory.GXX.create(this).errorReportingStrategy,
            CppValidationStrategyFactory.GXX.create(this).errorReportingStrategy
        )
        // This is a rather silly function, but it is left as-is because the appropriate reporting strategy
        //  could in principle be a function of the build process carried out by CMake. It just so happens
        //  that the compilers that are supported in the current version seem to use the same reporting format,
        //  so this ends up being a constant function.

    /** The include directories required by the generated files. */
    private val includes: List<String>
        get() {
            // FIXME: assert cmakeCache.exists()?
            if (cmakeCache.exists()) cmakeCache.useLines {
                for (line in it) {
                    val matcher = cmakeIncludes.matcher(line)
                    if (matcher.matches()) return matcher.group("includes").split(';')
                }
            }
            return listOf()
        }

    /** The C++ standard used by the generated files. */
    private val cppStandard: String
        get() {
            if (cmakeCache.exists()) cmakeCache.useLines {
                for (line in it) {
                    val matcher = cmakeCxxStandard.matcher(line)
                    if (matcher.matches()) return matcher.group("cppStandard")
                }
            }
            return ""
        }

    /** The CMake cache. */
    private val cmakeCache: File = fileConfig.buildPath.resolve("CMakeCache.txt").toFile()

    override fun getPossibleStrategies(): Collection<ValidationStrategy> = CppValidationStrategyFactory.values().map { it.create(this) }
}
