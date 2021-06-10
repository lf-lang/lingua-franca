package org.lflang.generator

import org.eclipse.core.resources.IMarker
import org.eclipse.core.resources.IResource
import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.Mode
import org.lflang.toPath
import java.nio.file.Path

/**
 * An error reporter that prints messages to the command line output and also sets markers
 * in the Eclipse IDE if running in integrated mode.
 */
class EclipseErrorReporter(private val fileConfig: FileConfig) : ErrorReporter {

    private var errorsOccurred = false

    private val EObject.node get() = NodeModelUtils.getNode(this)

    /**
     * Report a warning or error on the specified object
     *
     * The caller should not throw an exception so execution can continue.
     * This will print the error message to stderr.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     * @param message The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param obj The Ecore object, or null if it is not known.
     */
    private fun report(message: String, severity: Int, obj: EObject): String {
        val line = obj.node.startLine
        val file = obj.eResource().toPath()
        return report(message, severity, line, file)
    }

    /**
     * Report a warning or error on the specified line of the specified file.
     *
     * The caller should not throw an exception so execution can continue.
     * This will print the error message to stderr.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     * @param message The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param line The line number or null if it is not known.
     * @param file The file, or null if it is not known.
     */
    private fun report(
        message: String,
        severity: Int,
        line: Int?,
        file: Path?,
    ): String {
        val isError = severity == IMarker.SEVERITY_ERROR
        if (isError) {
            errorsOccurred = true
        }

        val header = if (isError) "ERROR" else "WARNING"

        if (line == null || file == null)
            System.err.println("$header: $message")
        else
            System.err.println("$header: $file $line\n$message")

        // If running in INTEGRATED mode, create a marker in the IDE for the error.
        // See: https://help.eclipse.org/2020-03/index.jsp?topic=%2Forg.eclipse.platform.doc.isv%2Fguide%2FresAdv_markers.htm
        if (fileConfig.compilerMode == Mode.INTEGRATED) {
            val iResource = if (file != null) fileConfig.getIResource(file) else fileConfig.iResource

            val marker = iResource.createMarker(IMarker.PROBLEM)
            marker.setAttribute(IMarker.MESSAGE, message)
            marker.setAttribute(IMarker.LINE_NUMBER, line ?: 1)
            // Human-readable line number information.
            marker.setAttribute(IMarker.LOCATION, line?.toString() ?: "1")
            // Mark as an error or warning.
            marker.setAttribute(IMarker.SEVERITY, severity)
            marker.setAttribute(IMarker.PRIORITY, IMarker.PRIORITY_HIGH)

            marker.setAttribute(IMarker.USER_EDITABLE, false)

            // NOTE: It might be useful to set a start and end.
            // marker.setAttribute(IMarker.CHAR_START, 0);
            // marker.setAttribute(IMarker.CHAR_END, 5);
        }

        // Return a string that can be inserted into the generated code.
        return "$header: $message"
    }


    /**
     * Report an error.
     * @param message The error message.
     */
    override fun reportError(message: String): String = report(message, IMarker.SEVERITY_ERROR, null, null)

    /**
     * Report a warning.
     * @param message The warning message.
     */
    override fun reportWarning(message: String): String = report(message, IMarker.SEVERITY_WARNING, null, null)

    /**
     * Report an error on the receiving parse tree object.
     *
     * @param obj     The parse tree object.
     * @param message The error message.
     */
    override fun reportError(obj: EObject, message: String): String = report(message, IMarker.SEVERITY_ERROR, obj)

    /**
     * Report a warning on the receiving parse tree object.
     *
     * @param obj     The parse tree object.
     * @param message The error message.
     */
    override fun reportWarning(obj: EObject, message: String): String = report(message, IMarker.SEVERITY_WARNING, obj)

    /**
     * Report an error at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The line number to report at.
     * @param file    The file to report at.
     * @return a string that describes the error.
     */
    override fun reportError(file: Path, line: Int, message: String): String = report(message, IMarker.SEVERITY_ERROR, line, file)

    /**
     * Report a warning at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The line number to report at.
     * @param file    The file to report at.
     * @return a string that describes the warning.
     */
    override fun reportWarning(file: Path, line: Int, message: String): String =
        report(message, IMarker.SEVERITY_WARNING, line, file)

    /**
     * Check if errors where reported.
     *
     * @return true if errors where reported
     */
    override fun getErrorsOccurred(): Boolean {
        return errorsOccurred
    }

    /**
     * Clear markers in the IDE if running in integrated mode.
     * This has the side effect of setting the iResource variable to point to
     * the IFile for the Lingua Franca program.
     * Also reset the flag indicating that generator errors occurred.
     */
    override fun reset() {
        errorsOccurred = false

        if (fileConfig.compilerMode == Mode.INTEGRATED) {
            try {
                val resource = fileConfig.getIResource(fileConfig.srcFile)
                // First argument can be null to delete all markers.
                // But will that delete xtext markers too?
                resource.deleteMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE)
            } catch (e: Exception) {
                // Ignore, but print a warning
                reportWarning("Deleting markers in the IDE failed: $e")
            }
        }
    }

}