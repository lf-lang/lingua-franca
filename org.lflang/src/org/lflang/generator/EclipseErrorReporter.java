/*************
 * Copyright (c) 2019, The University of California at Berkeley. Copyright (c)
 * 2019, TU Dresden
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;

import org.eclipse.core.resources.IMarker;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.validation.EObjectDiagnosticImpl;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.util.FileUtil;

/**
 * An error reporter that prints messages to the command line output and also
 * sets markers in the Epoch IDE.
 *
 * This class should only be used in EPOCH Mode.
 */
public class EclipseErrorReporter implements ErrorReporter {

    private FileConfig fileConfig = null;

    private boolean errorsOccurred = false;

    public EclipseErrorReporter(FileConfig fc) {
        fileConfig = fc;
    }

    // private val EObject.node get() = NodeModelUtils.getNode(this)

    /**
     * Report a warning or error on the specified object
     *
     * The caller should not throw an exception so execution can continue. This
     * will print the error message to stderr. If running in EPOCH mode
     * (within the Eclipse IDE), then this also adds a marker to the editor.
     * 
     * @param message  The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param obj      The Ecore object, or null if it is not known.
     */
    private String report(String message, Severity severity, EObject obj) {
        if (obj != null) {
             final EObjectDiagnosticImpl diagnostic = new EObjectDiagnosticImpl(severity, null, message, obj, null, -1, null);
             final int line = diagnostic.getLine();
             Path file = null;
             try {
                 file = FileUtil.toPath(diagnostic.getUriToProblem());
             } catch (IOException e) {
                // just continue with null
             }
             return report(message, severity, line, file);
        }
        
        return report(message, severity, null, null);
    }

    /**
     * Report a warning or error on the specified line of the specified file.
     *
     * The caller should not throw an exception so execution can continue. This
     * will print the error message to stderr. If running in EPOCH mode
     * (within the Eclipse IDE), then this also adds a marker to the editor.
     * 
     * @param message  The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param line     The line number or null if it is not known.
     * @param file     The file, or null if it is not known.
     */
    private String report(String message, Severity severity, Integer line, Path file) {
        final boolean isError = severity == Severity.ERROR;
        if (isError) {
            errorsOccurred = true;
        }

        final String header = isError ? "ERROR" : "WARNING";

        if (line == null || file == null)
            System.err.println(header + ": " + message);
        else
            System.err.println(header + ": " + file + " line " + line
                    + "\n" + message);

        // Determine the iResource to report on
        IResource iResource = file != null ? FileUtil.getIResource(file) : null;
        // if we couldn't find an iResource (for whatever reason), then use the
        // iResource of the main file
        if (iResource == null) {
            iResource = fileConfig.iResource;
        }

        // Create a marker in the IDE for the error.
        // See: https://help.eclipse.org/2020-03/index.jsp?topic=%2Forg.eclipse.platform.doc.isv%2Fguide%2FresAdv_markers.html
        try {
            IMarker marker = iResource.createMarker(IMarker.PROBLEM);

            // Mark as LF compilation marker to be able to remove marker at next compile run
            marker.setAttribute(this.getClass().getName(), true);

            marker.setAttribute(IMarker.MESSAGE, message);
            marker.setAttribute(IMarker.LINE_NUMBER,
                    line == null ? 1 : line);
            // Human-readable line number information.
            marker.setAttribute(IMarker.LOCATION,
                    line == null ? "1" : line.toString());
            // Mark as an error or warning.
            marker.setAttribute(IMarker.SEVERITY, isError ? IMarker.SEVERITY_ERROR : IMarker.SEVERITY_WARNING);
            marker.setAttribute(IMarker.PRIORITY, IMarker.PRIORITY_HIGH);

            marker.setAttribute(IMarker.USER_EDITABLE, false);

        } catch (CoreException e) {
            // Ignore, but print a warning
            System.err.println("WARNING: Setting markers in the IDE failed:\n" + e.toString());
        }

        // NOTE: It might be useful to set a start and end.
        // marker.setAttribute(IMarker.CHAR_START, 0);
        // marker.setAttribute(IMarker.CHAR_END, 5);

        // Return a string that can be inserted into the generated code.
        return header + ": " + message;
    }

    /**
     * Report an error.
     * 
     * @param message The error message.
     */
    @Override
    public String reportError(String message) {
        return report(message, Severity.ERROR, null, null);
    }

    /**
     * Report a warning.
     * 
     * @param message The warning message.
     */
    @Override
    public String reportWarning(String message) {
        return report(message, Severity.WARNING, null, null);
    }

    /**
     * Report an error on the receiving parse tree object.
     *
     * @param obj     The parse tree object.
     * @param message The error message.
     */
    @Override
    public String reportError(EObject obj, String message) {
        return report(message, Severity.ERROR, obj);
    }

    /**
     * Report a warning on the receiving parse tree object.
     *
     * @param obj     The parse tree object.
     * @param message The error message.
     */
    @Override
    public String reportWarning(EObject obj, String message) {
        return report(message, Severity.WARNING, obj);
    }

    /**
     * Report an error at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The line number to report at, or null if not known.
     * @param file    The file to report at.
     * @return a string that describes the error.
     */
    @Override
    public String reportError(Path file, Integer line, String message) {
        return report(message, Severity.ERROR, line, file);
    }

    /**
     * Report a warning at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The line number to report at, or null if not known.
     * @param file    The file to report at.
     * @return a string that describes the warning.
     */
    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return report(message, Severity.WARNING, line, file);
    }

    /**
     * Check if errors where reported.
     *
     * @return true if errors where reported
     */
    @Override
    public boolean getErrorsOccurred() {
        return errorsOccurred;
    }

    /**
     * Clear markers in the IDE.
     * This has the side effect of setting the
     * iResource variable to point to the IFile for the Lingua Franca program.
     */
    public void clearMarkers() {
        try {
            IResource resource = FileUtil.getIResource(fileConfig.srcFile);
            IMarker[] markers = resource.findMarkers(null, true, IResource.DEPTH_INFINITE);
            
            for (IMarker marker : markers) {
                // Only remove those markers created by the LF compilation
                if (marker.getAttribute(this.getClass().getName(), false)) {
                    marker.delete();
                }
            }
        } catch (Exception e) {
            // Ignore, but print a warning
            reportWarning("Deleting markers in the IDE failed: $e");
        }
    }

}
