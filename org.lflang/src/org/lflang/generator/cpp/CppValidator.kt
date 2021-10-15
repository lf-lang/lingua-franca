package org.lflang.generator.cpp

import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.generator.CodeMap
import org.lflang.generator.Position
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
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
    }

    // -std=c++17 or more generally -std=c++${CMAKE_CXX_STANDARD}
    // -I TARGET_INCLUDE_DIRECTORIES
    fun doValidate(cancelIndicator: CancelIndicator) {
        if (!cmakeCachePath.toFile().exists()) return
        for (generatedFile: Path in codeMaps.keys) {
            validateFile(generatedFile, cancelIndicator)
            if (cancelIndicator.isCanceled) return
        }
    }

    private fun validateFile(generatedFile: Path, cancelIndicator: CancelIndicator) {
        val validateCommand = getValidateCommand(generatedFile) ?: return
        validateCommand.run()
        for (line in validateCommand.errors.toString().lines()) {
            reportErrorLine(line, generatedFile)
        }
    }

    private fun getValidateCommand(generatedFile: Path): LFCommand? {
        // TODO: Support compilers other than g++
        val args: MutableList<String> = mutableListOf("-fsyntax-only", "-std=c++${cppStandard}")
        includes.forEach { args.add("-I${it}") }
        args.add(fileConfig.srcGenPath.resolve(generatedFile).toString())
        return LFCommand.get("g++", args, fileConfig.outPath)
    }

    private fun reportErrorLine(line: String, generatedFile: Path) {
        val matcher = gxxErrorLine.matcher(line)
        if (matcher.matches()) {
            val path = Paths.get(matcher.group("path"))
            val generatedFilePosition = Position.fromOneBased(
                Integer.parseInt(matcher.group("line")), Integer.parseInt(matcher.group("column"))
            )
            val lfFilePosition = codeMaps[generatedFile]?.adjusted(
                fileConfig.srcFile, generatedFilePosition, fileConfig.srcGenPath.resolve(generatedFile)
            ) ?: Position.ORIGIN
            errorReporter.reportError(path, lfFilePosition.oneBasedLine, matcher.group("message"))
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