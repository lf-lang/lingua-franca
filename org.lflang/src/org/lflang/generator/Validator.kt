package org.lflang.generator

import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.util.LFCommand
import java.nio.file.Path
import java.util.concurrent.Callable
import java.util.concurrent.CompletableFuture
import java.util.concurrent.Executors
import java.util.concurrent.Future

/**
 * Validates generated code.
 */
abstract class Validator(
    protected val errorReporter: ErrorReporter,
    protected val codeMaps: Map<Path, CodeMap>
) {
    /**
     * Validates this Validator's group of generated files.
     * @param cancelIndicator the cancel indicator for the
     * current operation
     */
    fun doValidate(cancelIndicator: CancelIndicator) {
        val tasks = getValidationStrategies().map { Callable {it.second.run(cancelIndicator); it} }
        val futures: List<Future<Pair<ValidationStrategy, LFCommand>>> = when (tasks.size) {
            0 -> listOf()
            1 -> listOf(CompletableFuture.completedFuture(tasks.first().call()))
            else -> Executors.newFixedThreadPool(
                Runtime.getRuntime().availableProcessors().coerceAtMost(tasks.size)
            ).invokeAll(tasks)
        }
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
        val (errorReportingStrategy, outputReportingStrategy) = buildReportingStrategies
        errorReportingStrategy.report(compileCommand.errors.toString(), errorReporter, codeMaps)
        outputReportingStrategy.report(compileCommand.output.toString(), errorReporter, codeMaps)
        return returnCode
    }

    /**
     * Returns the validation strategies and validation
     * commands corresponding to each generated file.
     * @return the validation strategies and validation
     * commands corresponding to each generated file
     */
    private fun getValidationStrategies(): List<Pair<ValidationStrategy, LFCommand>> {
        val commands = mutableListOf<Pair<ValidationStrategy, LFCommand>>()
        for (generatedFile: Path in codeMaps.keys) {
            val p = getValidationStrategy(generatedFile)
            val (strategy, command) = p
            if (strategy == null || command == null) continue
            commands.add(Pair(strategy, command))
            if (strategy.isFullBatch) break
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
    private fun getValidationStrategy(generatedFile: Path): Pair<ValidationStrategy?, LFCommand?> {
        for (strategy in possibleStrategies.sortedBy { strategy -> strategy.priority}) {
            val validateCommand = strategy.getCommand(generatedFile)
            if (validateCommand != null) {
                return Pair(strategy, validateCommand)
            }
        }
        return Pair(null, null)
    }

    /**
     * Lists all validation strategies that exist for the implementor,
     * without filtering by platform or availability.
     */
    protected abstract val possibleStrategies: Iterable<ValidationStrategy>

    /**
     * Returns the appropriate output and error reporting
     * strategies for the main build process.
     */
    protected abstract val buildReportingStrategies: Pair<CommandErrorReportingStrategy, CommandErrorReportingStrategy>
}
