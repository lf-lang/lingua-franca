package org.lflang.generator

import org.eclipse.core.resources.IMarker
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import java.nio.file.Path

class EclipseErrorReporter(private val fileConfig: FileConfig) : ErrorReporter {

    private var errorsOccurred = false

    // TODO move somewhere else
    private fun Resource.toPath() = org.lflang.FileConfig.toPath(this)

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
     * Report a warning or error on the specified line of the specified resource.
     *
     * The caller should not throw an exception so execution can continue.
     * This will print the error message to stderr.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     * @param message The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param line The line number or null if it is not known.
     * @param resource The resource, or null if it is not known.
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

        /* TODO!
        // If running in INTEGRATED mode, create a marker in the IDE for the error.
        // See: https://help.eclipse.org/2020-03/index.jsp?topic=%2Forg.eclipse.platform.doc.isv%2Fguide%2FresAdv_markers.htm
        if (mode === Mode.INTEGRATED) {
            var myResource = resource
            if (myResource === null && object !== null) {
                // Attempt to identify the IResource from the object.
                val eResource = object.eResource
                if (eResource !== null) {
                    val uri = FileConfig.toPath(eResource).toUri();
                    myResource = getEclipseResource(uri);
                }
            }
            // If the resource is still null, use the resource associated with
            // the top-level file.
            if (myResource === null) {
                myResource = fileConfig.iResource
            }
            if (myResource !== null) {
                val marker = myResource.createMarker(IMarker.PROBLEM)
                marker.setAttribute(IMarker.MESSAGE, message);
                if (line !== null) {
                    marker.setAttribute(IMarker.LINE_NUMBER, line);
                } else {
                    marker.setAttribute(IMarker.LINE_NUMBER, 1);
                }
                // Human-readable line number information.
                marker.setAttribute(IMarker.LOCATION, lineAsString);
                // Mark as an error or warning.
                marker.setAttribute(IMarker.SEVERITY, severity);
                marker.setAttribute(IMarker.PRIORITY, IMarker.PRIORITY_HIGH);

                marker.setAttribute(IMarker.USER_EDITABLE, false);

                // NOTE: It might be useful to set a start and end.
                // marker.setAttribute(IMarker.CHAR_START, 0);
                // marker.setAttribute(IMarker.CHAR_END, 5);
            }
        }*/

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
}