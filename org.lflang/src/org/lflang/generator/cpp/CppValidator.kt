package org.lflang.generator.cpp

import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.generator.CodeMap
import org.lflang.generator.CommandErrorReportingStrategy
import org.lflang.generator.PerLineReportingStrategy
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.util.concurrent.Callable
import java.util.concurrent.Executors
import java.util.regex.Pattern

class CppValidator(
    private val fileConfig: CppFileConfig,
    private val errorReporter: ErrorReporter,
    private val codeMaps: Map<Path, CodeMap>
) {

    companion object {
        /** This matches the line in the CMake cache that states the C++ standard. */
        private val cmakeCxxStandard: Pattern = Pattern.compile("CMAKE_CXX_STANDARD:STRING=(?<cppStandard>.*)")
        /** This matches the line in the CMake cache that states the compiler includes for a given target. */
        private val cmakeIncludes: Pattern = Pattern.compile("${CppCmakeGenerator.includesVarName}:STRING=(?<includes>.*)")

        /** This matches a line of error reports from g++. */
        private val gxxErrorLine: Pattern = Pattern.compile(
            "(?<path>.+\\.((cc)|(hh))):(?<line>\\d+):(?<column>\\d+): (?<severity>(error)|(warning)): (?<message>.*?) ?(?<type>(\\[.*])?)"
        )
        // Happily, the two tools seem to produce errors that follow the same format.
        /** This matches a line of error reports from Clang-Tidy. */
        private val clangTidyErrorLine: Pattern = gxxErrorLine
    }

    /**
     * This describes a strategy for validating a C++ source document.
     * @param errorReportingStrategy a strategy for parsing the stderr of the validation command
     * @param outputReportingStrategy a strategy for parsing the stdout of the validation command
     * @param time a number that is large for strategies that take a long time
     */
    private enum class CppValidationStrategy(
        val errorReportingStrategy: CommandErrorReportingStrategy,
        val outputReportingStrategy: CommandErrorReportingStrategy,
        val time: Int
    ) {
        // Note: Clang-tidy is slow (on the order of tens of seconds) for checking C++ files.
        CLANG_TIDY({ _, _, _ -> }, PerLineReportingStrategy(clangTidyErrorLine), 5) {
            override fun getCommand(validator: CppValidator, generatedFile: Path): LFCommand? {
                val args = mutableListOf(generatedFile.toString(), "--checks=*", "--quiet", "--", "-std=c++${validator.cppStandard}")
                validator.includes.forEach { args.add("-I$it") }
                return LFCommand.get("clang-tidy", args, validator.fileConfig.outPath)
            }
        },
        GXX(PerLineReportingStrategy(gxxErrorLine), { _, _, _ -> }, 1) {
            override fun getCommand(validator: CppValidator, generatedFile: Path): LFCommand? {
                val args: MutableList<String> = mutableListOf("-fsyntax-only", "-Wall", "-std=c++${validator.cppStandard}")
                validator.includes.forEach { args.add("-I$it") }
                args.add(generatedFile.toString())
                return LFCommand.get("g++", args, validator.fileConfig.outPath)
            }
        };

        /**
         * Returns the command that produces validation
         * output in association with `generatedFile`.
         * @param validator the C++ validator instance
         * corresponding to the relevant group of generated
         * files
         */
        abstract fun getCommand(validator: CppValidator, generatedFile: Path): LFCommand?
    }

    /**
     * Validates this Validator's group of generated files.
     * @param cancelIndicator the cancel indicator for the
     * current operation
     */
    fun doValidate(cancelIndicator: CancelIndicator) {
        if (!cmakeCachePath.toFile().exists()) return
        val futures = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors())
            .invokeAll(getValidationStrategies().map { Callable {it.second.run(cancelIndicator); it} })
        for (f in futures) {
            val (strategy, command) = f.get()
            strategy.errorReportingStrategy.report(command.errors.toString(), errorReporter, codeMaps)
            strategy.outputReportingStrategy.report(command.output.toString(), errorReporter, codeMaps)
        }
    }

    /**
     * Runs the given command, reports any messages it
     * produces, and returns its return code.
     */
    fun run(compileCommand: LFCommand, cancelIndicator: CancelIndicator): Int {
        val returnCode = compileCommand.run(cancelIndicator)
        val (errorReportingStrategy, outputReportingStrategy) = getBuildReportingStrategies()
        errorReportingStrategy.report(compileCommand.errors.toString(), errorReporter, codeMaps)
        outputReportingStrategy.report(compileCommand.output.toString(), errorReporter, codeMaps)
        return returnCode
    }

    /**
     * Returns the appropriate output and error reporting
     * strategies for the build process carried out by
     * CMake and Make.
     */
    private fun getBuildReportingStrategies(): Pair<CommandErrorReportingStrategy, CommandErrorReportingStrategy> {
        // This is a rather silly function, but it is left as-is because the appropriate reporting strategy
        //  could in principle be a function of the build process carried out by CMake. It just so happens
        //  that the compilers that are supported in the current version seem to use the same reporting format,
        //  so this ends up being a constant function.
        return Pair(CppValidationStrategy.GXX.errorReportingStrategy, CppValidationStrategy.GXX.errorReportingStrategy)
    }

    /**
     * Returns the validation strategies and validation
     * commands corresponding to each generated file.
     * @return the validation strategies and validation
     * commands corresponding to each generated file
     */
    private fun getValidationStrategies(): List<Pair<CppValidationStrategy, LFCommand>> {
        val commands = mutableListOf<Pair<CppValidationStrategy, LFCommand>>()
        for (generatedFile: Path in codeMaps.keys) {
            val p = getValidationStrategy(generatedFile)
            val (strategy, command) = p
            if (strategy == null || command == null) continue
            commands.add(Pair(strategy, command))
        }
        return commands
    }

    /**
     * Returns the validation strategy and command
     * corresponding to the given file, if such a strategy
     * and command are available.
     * @return the validation strategy and command
     * corresponding to the given file, if such a strategy
     * and command are available
     */
    private fun getValidationStrategy(generatedFile: Path): Pair<CppValidationStrategy?, LFCommand?> {
        for (strategy in CppValidationStrategy.values().sortedBy {strategy -> strategy.time}) {
            val validateCommand = strategy.getCommand(this, generatedFile) //
            if (validateCommand != null) {
                return Pair(strategy, validateCommand)
            }
        }
        return Pair(null, null)
    }

    /** The include directories required by the generated files. */
    private val includes: List<String>
        get() {
            for (line in cmakeCache) {
                val matcher = cmakeIncludes.matcher(line)
                if (matcher.matches()) return matcher.group("includes").split(';')
            }
            return listOf()
        }

    /** The C++ standard used by the generated files. */
    private val cppStandard: String
        get() {
            for (line in cmakeCache) {
                val matcher = cmakeCxxStandard.matcher(line)
                if (matcher.matches()) return matcher.group("cppStandard")
            }
            return ""
        }

    /** The content of the CMake cache. */ // FIXME: Most of this data will never be used. Should it really be cached?
    private val cmakeCache: List<String> by lazy { Files.readAllLines(cmakeCachePath) }

    /** The path to the CMake cache. */
    private val cmakeCachePath: Path = fileConfig.buildPath.resolve("CMakeCache.txt")
}
