/* Abstract class for basic code generation utilities. */

/*************
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy.generator

import java.util.HashMap
import java.util.ArrayList
import java.util.regex.Pattern
import java.io.File
import java.io.ByteArrayOutputStream
import java.io.OutputStream
import java.io.IOException
import java.nio.file.StandardCopyOption
import java.nio.file.Paths
import java.nio.file.Files
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.core.resources.IMarker
import org.eclipse.core.runtime.Path
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.validation.AbstractLinguaFrancaValidator


/**
 * This class provides basic code generation capabilities
 * such as executing commands, writing code into a string buffer
 * and reporting errors.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Christian Menard <christian.menard@tu-dresden.de}
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 */
abstract class CodeGenerator extends AbstractLinguaFrancaValidator {
    
    ////////////////////////////////////////////
    //// Protected fields.
    
    /**
     * Indicator of whether generator errors occurred.
     */
    protected var generatorErrorsOccurred = false
    
    /**
     * If running in an Eclipse IDE, the iResource refers to the
     * IFile representing the Lingua Franca program.
     */
    protected var iResource = null as IResource
    
    /**
     * All code goes into this string buffer.
     */
    protected var code = new StringBuilder
    
    /**
     * {@link #Mode.STANDALONE Mode.STANDALONE} if the code generator is being
     * called from the command line, {@link #Mode.INTEGRATED Mode.INTEGRATED}
     * if it is being called from the Eclipse IDE, and 
     * {@link #Mode.UNDEFINED Mode.UNDEFINED} otherwise.
     */
    protected var mode = Mode.UNDEFINED
    
    ////////////////////////////////////////////
    //// Protected methods.
    
    /**
     * Execute the command given by the specified list of strings, print the
     * command, its return code, and its output to stderr and stdout, and
     * return the return code, which is 0 if the command succeeds.
     * 
     * If the command fails to execute, then a second attempt is made using a
     * Bash shell with the --login option, which sources the user's 
     * ~/.bash_profile, ~/.bash_login, or ~/.bashrc (whichever
     * is first found) before running the command. This helps to ensure that
     * the user's PATH variable is set according to their usual environment,
     * assuming that they use a bash shell.
     * 
     * More information: Unfortunately, at least on a Mac if you are running
     * within Eclipse, the PATH variable is extremely limited; supposedly, it
     * is given by the default provided in /etc/paths, but at least on my machine,
     * it does not even include directories in that file for some reason.
     * One way to add a directory like
     * /usr/local/bin to the path once-and-for-all is this:
     * 
     * sudo launchctl config user path /usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
     * 
     * But asking users to do that is not ideal. Hence, we try a more hack-y
     * approach of just trying to execute using a bash shell.
     * Also note that while ProcessBuilder can configured to use custom
     * environment variables, these variables do not affect the command that is
     * to be executed but merely the environment in which the command executes.
     * 
     * @param command The command.
     * @param directory The directory in which to execute the command.
     * @return 0 if the command succeeds, otherwise, an error code.
     */
    protected def executeCommand(ArrayList<String> command, String directory) {
        println("In directory: " + directory)
        println("Executing command: " + command.join(" "))
        var builder = new ProcessBuilder(command);
        builder.directory(new File(directory));
        try {
            val stdout = new ByteArrayOutputStream()
            val stderr = new ByteArrayOutputStream()
            val returnCode = builder.runSubprocess(stdout, stderr)
            if (stdout.size() > 0) {
                println("--- Standard output from command:")
                println(stdout.toString())
                println("--- End of standard output.")
            }
            if (stderr.size() > 0) {
                println("--- Standard error from command:")
                println(stderr.toString())
                println("--- End of standard error.")
            }
            if (returnCode !== 0) {
                // Throw an exception, which will be caught below for a second attempt.
                throw new Exception("Command returns error code " + returnCode)
            }
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (stderr.toString.length > 0 && mode === Mode.INTEGRATED) {
                reportCommandErrors(stderr.toString())
            }
            return returnCode
        } catch (Exception ex) {
            
            println("--- Exception: " + ex)
            // Try running with bash.
            // The --login option forces bash to look for and load the first of
            // ~/.bash_profile, ~/.bash_login, and ~/.bashrc that it finds.
            var bashCommand = new ArrayList<String>()
            bashCommand.addAll("bash", "--login", "-c")
            bashCommand.addAll(command.join(" "))
            // bashCommand.addAll("bash", "--login", "-c", 'ls', '-a')
            println("--- Attempting instead to run: " + bashCommand.join(" "))
            builder.command(bashCommand)
            val stdout = new ByteArrayOutputStream()
            val stderr = new ByteArrayOutputStream()
            val returnCode = builder.runSubprocess(stdout, stderr)
            if (stdout.size() > 0) {
                println("--- Standard output from command:")
                println(stdout.toString())
                println("--- End of standard output.")
            }
            if (stderr.size() > 0) {
                println("--- Standard error from command:")
                println(stderr.toString())
                println("--- End of standard error.")
            }
            
            if (returnCode !== 0) {
                if (mode === Mode.INTEGRATED) {
                    reportCommandErrors(stderr.toString())
                } else {
                    reportError("Bash command returns error code " + returnCode)
                }
            }
            return returnCode
        }
    }
    
    /** If the mode is INTEGRATED (the code generator is running in an
     *  an Eclipse IDE), then refresh the project. This will ensure that
     *  any generated files become visible in the project.
     */
    protected def refreshProject() {
        if (mode == Mode.INTEGRATED) {
            // Find name of current project
            val id = "((:?[a-z]|[A-Z]|_\\w)*)";
            var pattern = if (File.separator.equals("/")) { // Linux/Mac file separator
                Pattern.compile(
                "platform:" + File.separator + "resource" + File.separator +
                    id + File.separator);
            } else { // Windows file separator
                Pattern.compile(
                "platform:" + File.separator + File.separator + "resource" + File.separator + File.separator +
                id + File.separator + File.separator );
            }
            val matcher = pattern.matcher(code);
            var projName = ""
            if (matcher.find()) {
                projName = matcher.group(1)
            }
            try {
                val members = ResourcesPlugin.getWorkspace().root.members
                for (member : members) {
                    // Refresh current project, or simply entire workspace if project name was not found
                    if (projName == "" ||
                        projName.equals(
                            member.fullPath.toString.substring(1))) {
                        member.refreshLocal(IResource.DEPTH_INFINITE, null)
                        println("Refreshed " + member.fullPath.toString)
                    }
                }
            } catch (IllegalStateException e) {
                println("Unable to refresh workspace: " + e)
            }
        }
    }
    
    /**
     * Parsed error message from a compiler is returned here.
     */
    protected static class ErrorFileAndLine {
        public var filepath = null as String
        public var line = "1"
        public var character = "0"
        public var message = ""
        public var isError = true // false for a warning.
    }
    
    /**
     * Given a line of text from the output of a compiler, return
     * an instance of ErrorFileAndLine if the line is recognized as
     *  the first line of an error message. Otherwise, return null.
     *  This base class simply returns null.
     *  @param line A line of output from a compiler or other external
     *   tool that might generate errors.
     *  @return If the line is recognized as the start of an error message,
     *   then return a class containing the path to the file on which the
     *   error occurred (or null if there is none), the line number (or the
     *   string "1" if there is none), the character position (or the string
     *   "0" if there is none), and the message (or an empty string if there
     *   is none).
     */
    protected def parseCommandOutput(String line) {
        return null as ErrorFileAndLine
    }
    
    /** Parse the specified string for command errors that can be reported
     *  using marks in the Eclipse IDE. In this class, we attempt to parse
     *  the messages to look for file and line information, thereby generating
     *  marks on the appropriate lines.
     *  @param stderr The output on standard error of executing a command.
     */
    protected def reportCommandErrors(String stderr) {
        // First, split the message into lines.
        val lines = stderr.split("\\r?\\n")
        var message = new StringBuilder()
        var lineNumber = null as Integer
        var resource = iResource  // Default resource.
        var severity = IMarker.SEVERITY_ERROR
        for (line: lines) {
            val parsed = parseCommandOutput(line)
            if (parsed !== null) {
                // Found a new line number designator.
                // If there is a previously accumulated message, report it.
                if (message.length > 0) {
                    report(message.toString(), severity, lineNumber, resource)
                    if (iResource != resource) {
                        // Report an error also in the top-level resource.
                        // FIXME: It should be possible to descend through the import
                        // statements to find which one matches and mark all the
                        // import statements down the chain. But what a pain!
                        report("Error in imported file: " + resource.fullPath, IMarker.SEVERITY_ERROR,
                            null, iResource
                        )
                    }
                }
                if (parsed.isError) {
                    severity = IMarker.SEVERITY_ERROR
                } else {
                    severity = IMarker.SEVERITY_WARNING
                }
                
                // Start accumulating a new message.
                message = new StringBuilder()
                // Append the message on the line number designator line.
                message.append(parsed.message)
                
                // Set the new line number.
                try {
                    lineNumber = Integer.decode(parsed.line)
                } catch (Exception ex) {
                    // Set the line number unknown.
                    lineNumber = null
                }
                // FIXME: Ignoring the position within the line.
                
                // Determine the resource within which the error occurred.
                val workspaceRoot = ResourcesPlugin.getWorkspace().getRoot()
                // Sadly, Eclipse defines an interface called "URI" that conflicts with the
                // Java one, so we have to give the full class name here.
                val uri = new java.net.URI(parsed.filepath)
                val files = workspaceRoot.findFilesForLocationURI(uri)
                // No idea why there might be more than one file matching the URI,
                // but Eclipse seems to think there might be. We will just use the
                // first one. If there is no such file, then reset the line to
                // unknown and keep the resource as before.
                if (files === null || files.length === 0 || files.get(0) === null) {
                    lineNumber = null
                } else if (files.get(0) != resource) {
                    // The resource has changed, which means that the error
                    // occurred in imported code.
                    resource = files.get(0)
                }
            } else {
                if (message.length > 0) {
                    message.append("\n")
                }
                message.append(line)
            }
        }
        if (message.length > 0) {
            report(message.toString, severity, lineNumber, resource)
            if (iResource != resource) {
                // Report an error also in the top-level resource.
                // FIXME: It should be possible to descend through the import
                // statements to find which one matches and mark all the
                // import statements down the chain. But what a pain!
                report("Error in imported file: " + resource.fullPath, IMarker.SEVERITY_ERROR,
                    null, iResource
                )
            }
        }
    }
    
    /**
     *  Lookup a file in the classpath and copy its contents to a destination path 
     *  in the filesystem.
     * 
     *  This also creates new directories for any directories on the destination
     *  path that do not yet exist.
     * 
     *  @param source The source file as a path relative to the classpath.
     *  @param destination The file system path that the source file is copied to.
     */
    protected def copyFileFromClassPath(String source, String destination) {
        val sourceStream = this.class.getResourceAsStream(source)

        if (sourceStream === null) {
            throw new IOException("A required target resource could not be found: " + source + "\n"
                + "Perhaps a git submodule is missing or not up to date.\n"
                + "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.\n"
                + "Also try to refresh and clean the project explorer if working from eclipse.")
        }

        // copy the file
        try {
        // make sure the directory exists
        val destFile = new File(destination);
        destFile.getParentFile().mkdirs();

            Files.copy(sourceStream, Paths.get(destination), StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException ex) {
             throw new IOException("A required target resource could not be copied: " + source + "\n"
                + "Perhaps a git submodule is missing or not up to date.\n"
                + "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.", ex)
        } finally {
            sourceStream.close()
        }
    }
    
    /** Report a warning or error on the specified line of the specified resource.
     *  The caller should not throw an exception so execution can continue.
     *  This will print the error message to stderr.
     *  If running in INTEGRATED mode (within the Eclipse IDE), then this also
     *  adds a marker to the editor.
     *  @param message The error message.
     *  @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     *  @param line The line number or null if it is not known.
     *  @param object The Ecore object, or null if it is not known.
     *  @param resource The resource, or null if it is not known.
     */
    protected def report(String message, int severity, Integer line, EObject object, IResource resource) {        
        if (severity === IMarker.SEVERITY_ERROR) {
            generatorErrorsOccurred = true;
        }
        val header = (severity === IMarker.SEVERITY_ERROR)? "ERROR: " : "WARNING: "
        val lineAsString = (line === null)? "" : "Line " + line
        var fullPath = resource?.fullPath?.toString
        if (fullPath === null) {
            fullPath = object?.eResource?.toPath
        }
        if (fullPath === null) {
            if (line === null) {
                fullPath = ""
            } else {
                fullPath = "path unknown"
            }
        }
        val toPrint = header + fullPath + " " + lineAsString + "\n" + message
        System.err.println(toPrint)
        
        // If running in INTEGRATED mode, create a marker in the IDE for the error.
        // See: https://help.eclipse.org/2020-03/index.jsp?topic=%2Forg.eclipse.platform.doc.isv%2Fguide%2FresAdv_markers.htm
        if (mode === Mode.INTEGRATED) {
            var myResource = resource
            if (myResource === null && object !== null) {
                // Attempt to identify the IResource from the object.
                val eResource = object.eResource
                if (eResource !== null) {
                    val uri = new java.net.URI("file:/" + eResource.toPath)
                    val workspaceRoot = ResourcesPlugin.getWorkspace().getRoot()
                    val files = workspaceRoot.findFilesForLocationURI(uri)
                    if (files !== null && files.length > 0 && files.get(0) !== null) {
                        myResource = files.get(0)
                    }
                }
            }
            // If the resource is still null, use the resource associated with
            // the top-level file.
            if (myResource === null) {
                myResource = iResource
            }
            if (myResource !== null) {
                val marker = myResource.createMarker(IMarker.PROBLEM)
                marker.setAttribute(IMarker.MESSAGE, toPrint);
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
        }
        
        // Return a string that can be inserted into the generated code.
        if (severity === IMarker.SEVERITY_ERROR) {
            return "[[ERROR: " + message + "]]"
        }
        return ""
    }
    
    /** Report a warning or error on the specified parse tree object in the
     *  current resource.
     *  The caller should not throw an exception so execution can continue.
     *  If running in INTEGRATED mode (within the Eclipse IDE), then this also
     *  adds a marker to the editor.
     *  @param message The error message.
     *  @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     *  @param object The parse tree object or null if not known.
     */
    protected def report(String message, int severity, EObject object) {        
        var line = null as Integer
        if (object !== null) {
            val node = NodeModelUtils.getNode(object)
            if (node !== null) {
                line = node.getStartLine
            }
        }
        return report(message, severity, line, object, null)
    }

    /** Report a warning or error on the specified parse tree object in the
     *  current resource.
     *  The caller should not throw an exception so execution can continue.
     *  If running in INTEGRATED mode (within the Eclipse IDE), then this also
     *  adds a marker to the editor.
     *  @param message The error message.
     *  @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     *  @param resource The resource.
     */
    protected def report(String message, int severity, Integer line, IResource resource) {        
        return report(message, severity, line, null, resource)
    }

    /** Report an error.
     *  @param message The error message.
     */
    protected def reportError(String message) {
        return report(message, IMarker.SEVERITY_ERROR, null)
    }

    /** Report an error on the specified parse tree object.
     *  @param object The parse tree object.
     *  @param message The error message.
     */
    protected def reportError(EObject object, String message) {
        return report(message, IMarker.SEVERITY_ERROR, object)
    }

    /** Report a warning on the specified parse tree object.
     *  @param object The parse tree object.
     *  @param message The error message.
     */
    protected def reportWarning(EObject object, String message) {
        return report(message, IMarker.SEVERITY_WARNING, object)
    }
    
    /**
     * Execute a process while forwarding output and error to system streams.
     *
     * Executing a process directly with `processBuiler.start()` could
     * lead to a deadlock as the subprocess blocks when output or error
     * buffers are full. This method ensures that output and error messages
     * are continuously read and forwards them to the system's output and
     * error streams.
     *
     * @param processBuilder The process to be executed.
     * @author{Christian Menard <christian.menard@tu-dresden.de}
     */
    protected def runSubprocess(ProcessBuilder processBuilder) {
        return runSubprocess(processBuilder, System.out, System.err);
    }

    /**
     * Execute a process while forwarding output and error streams.
     *
     * Executing a process directly with `processBuiler.start()` could
     * lead to a deadlock as the subprocess blocks when output or error
     * buffers are full. This method ensures that output and error messages
     * are continuously read and forwards them to the given streams.
     *
     * @param processBuilder The process to be executed.
     * @param outStream The stream to forward the process' output to.
     * @param errStream The stream to forward the process' error messages to.
     * @author{Christian Menard <christian.menard@tu-dresden.de}
     */
    protected def runSubprocess(ProcessBuilder processBuilder,
                                OutputStream outStream,
                                OutputStream errStream) {
        val process = processBuilder.start()

        var outThread = new Thread([|
                var buffer = newByteArrayOfSize(64)
                var len = process.getInputStream().read(buffer)
                while(len != -1) {
                    outStream.write(buffer, 0, len)
                    len = process.getInputStream().read(buffer)
                }
            ])
        outThread.start()

        var errThread = new Thread([|
                var buffer = newByteArrayOfSize(64)
                var len = process.getErrorStream().read(buffer)
                while(len != -1) {
                    errStream.write(buffer, 0, len)
                    len = process.getErrorStream().read(buffer)
                }
            ])
        errThread.start()

        val returnCode = process.waitFor()
        outThread.join()
        errThread.join()

        return returnCode
    }
    
    
    /**
     * Create a string representing the absolute file path of a resource.
     */
    protected def toPath(Resource resource) {
        return resource.getURI.toPath
    }

    /**
     * Create a string representing the absolute file path of a URI.
     */
    protected def toPath(URI uri) {
        if (uri.isPlatform) {
            val file = ResourcesPlugin.workspace.root.getFile(
                new Path(uri.toPlatformString(true)))
            return file.rawLocation.toFile.absolutePath
        } else if (uri.isFile) {
            val file = new File(uri.toFileString)
            return file.absolutePath
        } else {
            throw new IOException("Unrecognized file protocol in URI " +
                uri.toString)
        }
    }

    /**
     * Create a string representing the absolute file path of a file relative to a file system access object.
     */
    protected def getAbsolutePath(IFileSystemAccess2 fsa, String file) {
        return fsa.getURI(file).toPath
    }
    
    /**
     * Extract the name of a file from a path represented as a string.
     * If the file ends with '.lf', the extension is removed.
     */
    protected def getFilename(String path) {
        var File f = new File(path)
        var name = f.getName()
        if (name.endsWith('.lf')) {
            name = name.substring(0, name.length - 3)
        }
        return name
    }
    
    /**
     * Extract the directory from a path represented as a string.
     */
    protected def getDirectory(String path) {
        var File f = new File(path)
        f.getParent()
    }
    
    /**
     * Append the specified text plus a final newline to the current
     * code buffer.
     * @param format A format string to be used by String.format or
     * the text to append if no further arguments are given.
     * @param args Additional arguments to pass to the formatter.
     */
    protected def pr(String format, Object... args) {
        pr(code,
            if (args !== null && args.length > 0) String.format(format,
                args) else format)
    }

    /**
     * Append the specified text plus a final newline to the specified
     * code buffer.
     * @param builder The code buffer.
     * @param text The text to append.
     */
    protected def pr(StringBuilder builder, Object text) {
        // Handle multi-line text.
        var string = text.toString
        var indent = indentation.get(builder)
        if (indent === null) {
            indent = ""
        }
        if (string.contains("\n")) {
            // Replace all tabs with four spaces.
            string = string.replaceAll("\t", "    ")
            // Use two passes, first to find the minimum leading white space
            // in each line of the source text.
            var split = string.split("\n")
            var offset = Integer.MAX_VALUE
            var firstLine = true
            for (line : split) {
                // Skip the first line, which has white space stripped.
                if (firstLine) {
                    firstLine = false
                } else {
                    var numLeadingSpaces = line.indexOf(line.trim());
                    if (numLeadingSpaces < offset) {
                        offset = numLeadingSpaces
                    }
                }
            }
            // Now make a pass for each line, replacing the offset leading
            // spaces with the current indentation.
            firstLine = true
            for (line : split) {
                builder.append(indent)
                // Do not trim the first line
                if (firstLine) {
                    builder.append(line)
                    firstLine = false
                } else {
                    builder.append(line.substring(offset))
                }
                builder.append("\n")
            }
        } else {
            builder.append(indent)
            builder.append(text)
            builder.append("\n")
        }
    }

    /**
     * Prints an indented block of text with the given begin and end markers,
     * but only if the actions print any text at all.
     * This is helpful to avoid the production of empty blocks.
     * @param begin The prologue of the block.
     * @param end The epilogue of the block.
     * @param actions Actions that print the interior of the block. 
     */
    protected def prBlock(String begin, String end, Runnable... actions) {
        val i = code.length
        indent()
        for (action : actions) {
            action.run()
        }
        unindent()
        if (i < code.length) {
            val inserted = code.substring(i, code.length)
            code.delete(i, code.length)
            pr(begin)
            code.append(inserted)
            pr(end)
        }
    }

    /**
     * Print a comment to the generated file.
     * Particular targets will need to override this if comments
     * start with something other than '//'.
     * @param comment The comment.
     */
    protected def prComment(String comment) {
        pr(code, '// ' + comment);
    }
    
    /**
     * Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
    }
    
    /**
     * Increase the indentation of the output code produced
     * on the specified builder.
     * @param The builder to indent.
     */
    protected def indent(StringBuilder builder) {
        var prefix = indentation.get(builder)
        if (prefix === null) {
            prefix = ""
        }
        val buffer = new StringBuffer(prefix)
        for (var i = 0; i < 4; i++) {
            buffer.append(' ');
        }
        indentation.put(builder, buffer.toString)
    }
    
    /**
     * Increase the indentation of the output code produced.
     */
    protected def indent() {
        indent(code)
    }
    
    /**
     * Get the code produced so far.
     * @return The code produced so far as a String.
     */
    protected def getCode() {
        code.toString()
    }
    
    /** Reduce the indentation by one level for generated code
     *  in the specified code buffer.
     */
    protected def unindent(StringBuilder builder) {
        var indent = indentation.get(builder)
        if (indent !== null) {
            val end = indent.length - 4;
            if (end < 0) {
                indent = ""
            } else {
                indent = indent.substring(0, end)
            }
            indentation.put(builder, indent)
        }
    }
    
    /** Reduce the indentation by one level for generated code
     *  in the default code buffer.
     */
    protected def unindent() {
        unindent(code)
    }
    
    ////////////////////////////////////////////
    //// Public methods.
    
    /**
     * Return true if errors occurred in the last call to doGenerate().
     * This will return true if any of the reportError methods was called.
     * @return True if errors occurred.
     */
    def errorsOccurred() {
        return generatorErrorsOccurred;
    }
    
    /**
     * Remove quotation marks surrounding the specified string.
     */
    def withoutQuotes(String s) {
        var result = s
        if (s.startsWith("\"") || s.startsWith("\'")) {
            result = s.substring(1)
        }
        if (result.endsWith("\"") || result.endsWith("\'")) {
            result = result.substring(0, result.length - 1)
        }
        result
    }
    
    ////////////////////////////////////////////
    //// Private Fields.
    
    /**
     * Map from builder to its current indentation.
     */
    var indentation = new HashMap<StringBuilder, String>()
    
    ////////////////////////////////////////////
    //// Enums
    
    enum Mode {
        STANDALONE,
        INTEGRATED,
        UNDEFINED
    }
    
}