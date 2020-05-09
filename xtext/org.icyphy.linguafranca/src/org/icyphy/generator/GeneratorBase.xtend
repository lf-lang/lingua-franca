/* Generator base class for shared code between code generators. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy.generator

import java.io.ByteArrayOutputStream
import java.io.File
import java.io.IOException
import java.io.OutputStream
import java.nio.file.Files
import java.nio.file.Paths
import java.nio.file.StandardCopyOption
import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.List
import java.util.Map
import java.util.Set
import java.util.regex.Pattern
import org.eclipse.core.resources.IMarker
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.core.runtime.Path
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.InferredType
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.VarRef
import org.icyphy.validation.AbstractLinguaFrancaValidator

import static extension org.icyphy.ASTUtils.*

/** Generator base class for shared code between code generators.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Chris Gill, <cdgill@wustl.edu>}
 *  @author{Christian Menard <christian.menard@tu-dresden.de}
 */
abstract class GeneratorBase extends AbstractLinguaFrancaValidator {

    ////////////////////////////////////////////
    //// Public fields.

    // Map from time units to an expression that can convert a number in
    // the specified time unit into nanoseconds. This expression may need
    // to have a suffix like 'LL' or 'L' appended to it, depending on the
    // target language, to ensure that the result is a 64-bit long.            
    public static var timeUnitsToNs = #{TimeUnit.NSEC -> 1L,
        TimeUnit.NSECS -> 1L, TimeUnit.USEC -> 1000L, TimeUnit.USECS -> 1000L,
        TimeUnit.MSEC -> 1000000L, TimeUnit.MSECS -> 1000000L,
        TimeUnit.SEC -> 1000000000L, TimeUnit.SECS -> 1000000000L,
        TimeUnit.SECOND -> 1000000000L, TimeUnit.SECONDS -> 1000000000L,
        TimeUnit.MIN -> 60000000000L, TimeUnit.MINS -> 60000000000L,
        TimeUnit.MINUTE -> 60000000000L, TimeUnit.MINUTES -> 60000000000L,
        TimeUnit.HOUR -> 3600000000000L, TimeUnit.HOURS -> 3600000000000L,
        TimeUnit.DAY -> 86400000000000L, TimeUnit.DAYS -> 86400000000000L,
        TimeUnit.WEEK -> 604800000000000L, TimeUnit.WEEKS -> 604800000000000L}
    
    public static var GEN_DELAY_CLASS_NAME = "__GenDelay"
    
    static protected CharSequence listItemSeparator = ', '
    
    ////////////////////////////////////////////
    //// Protected fields.
        
    /** All code goes into this string buffer. */
    protected var code = new StringBuilder

    /** Path to the directory containing the .lf file. */
    protected var String directory
    
    /** The root filename for the main file containing the source code,
     *  without the .lf extension.
     */
    protected var String filename

    /** Indicator of whether generator errors occurred. */
    protected var generatorErrorsOccurred = false
    
    /** If running in an Eclipse IDE, the iResource refers to the
     *  IFile representing the Lingua Franca program.
     */
    protected var iResource = null as IResource
    
    /** Definition of the main (top-level) reactor */
    protected Instantiation mainDef
    
    /** Mode.STANDALONE if the code generator is being called
     *  from the command line, Mode.INTEGRATED if it is being called
     *  from the Eclipse IDE, and Mode.UNDEFINED otherwise.
     */
    protected var mode = Mode.UNDEFINED
    
    /** A list of Reactor definitions in the main
     *  resource, including non-main reactors defined
     *  in imported resources.
     */
    protected var List<Reactor> reactors
    
    /** The file containing the main source code. */
    protected var Resource resource
    
    /** The full path to the file containing the .lf file including the
     *  full filename with the .lf extension. This starts out as the
     *  main .lf file, but while a file is being imported, it temporarily
     *  changes to the full path of the imported file.
     */
    protected var String sourceFile
    
    /** A map of all resources to the set of resource they import */
    protected var importedResources = new HashMap<Resource, Set<Resource>>;
    
    ////////////////////////////////////////////
    //// Target properties, if they are included.
    
    /** A list of federate instances or a list with a single empty string
     *  if there are no federates specified.
     */
    protected var List<FederateInstance> federates = new LinkedList<FederateInstance>
    
    /** A map from federate names to federate instances. */
    protected var Map<String,FederateInstance> federateByName
            = new HashMap<String,FederateInstance>()

    /** A map from federate IDs to federate instances. */
    protected var Map<Integer,FederateInstance> federateByID
            = new HashMap<Integer,FederateInstance>()

    /** A map from reactor names to the federate instance that contains the reactor. */
    protected var Map<String,FederateInstance> federateByReactor

    /** The federation RTI properties, which defaults to
     *  localhost: 15045
     */
    protected val federationRTIProperties = newLinkedHashMap(
        'host' -> 'localhost',
        'port' -> 15045
    ) 

    /** The build-type target parameter, or null if there is none. */
    protected String targetBuildType

    /** The cmake-include target parameter, or null if there is none. */
    protected String targetCmakeInclude
    
    /** The compiler target parameter, or null if there is none. */
    protected String targetCompiler

    /** The compiler flags target parameter, or null if there is none. */
    protected String targetCompilerFlags

    /** The compiler target no-compile parameter, or false if there is none. */
    protected boolean targetNoCompile = false
    
    /** The compiler target no-runtime-validation parameter, or false if there is none. */
    protected boolean targetNoRuntimeValidation = false
        
    /** The fast target parameter, or false if there is none. */
    protected boolean targetFast = false
    
    /** The value of the keepalive target parameter, or false if there is none. */
    protected boolean targetKeepalive
    
    /** The level of logging or null if not given. */
    protected String targetLoggingLevel

    /** The threads target parameter, or the default 0 if there is none. */
    protected int targetThreads = 0

    /** The timeout parameter, or the default -1 if there is none. */
    protected int targetTimeout = -1

    /** The threads timeout unit parameter, or the default null if there is none. */
    protected TimeUnit targetTimeoutUnit

    ////////////////////////////////////////////
    //// Private fields.

    /** Map from builder to its current indentation. */
    var indentation = new HashMap<StringBuilder, String>()
    
    /** Recursion stack used to detect cycles in imports */
    var importRecursionStack = new HashSet<Resource>();
    
    /** A flag indicating whether a cycle was found while processing imports */
    var cyclicImports = false;

    ////////////////////////////////////////////
    //// Code generation functions to override for a concrete code generator.
    
    /** Analyze the model, setting target variables, filenames,
     *  working directory, and federates. This also performs any
     *  transformations that are needed on the AST of the model,
     *  including handling delays on connections and communication
     *  between federates.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: What is this?
     */
    def void analyzeModel(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
        generatorErrorsOccurred = false
        
        var target = resource.findTarget
        if (target.config !== null) {
            for (param: target.config.pairs ?: emptyList) {
                switch param.name {
                    case "build-type":
                        targetBuildType = param.value.id
                    case "cmake-include":
                        targetCmakeInclude = param.value.literal.withoutQuotes
                    case "compiler":
                        targetCompiler = param.value.literal.withoutQuotes
                    case "fast":
                        if (param.value.literal == 'true') {
                            targetFast = true
                        }
                    case "flags":
                        targetCompilerFlags = param.value.literal.withoutQuotes
                    case "no-compile":
                        if (param.value.literal == 'true') {
                            targetNoCompile = true
                        }
                    case "no-runtime-validation":
                        if (param.value.literal == 'true') {
                            targetNoRuntimeValidation = true
                        }
                    case "keepalive":
                        if (param.value.literal == 'true') {
                            targetKeepalive = true
                        }
                    case "logging":
                        targetLoggingLevel = param.value.id
                    case "threads":
                        targetThreads = Integer.decode(param.value.literal)
                    case "timeout": {
                        targetTimeout = param.value.time
                        targetTimeoutUnit = param.value.unit
                    }
                }
            }
        }
        
        // Override target properties if specified as command line arguments.
        if (context instanceof StandaloneContext) {
            if (context.args.containsKey("no-compile")) {
                targetNoCompile = true
            }
            if (context.args.containsKey("target-compiler")) {
                targetCompiler = context.args.getProperty("target-compiler")
            }
            if (context.args.containsKey("target-flags")) {
                targetCompilerFlags = context.args.getProperty("target-flags")
            }
        }

        println("Generating code for: " + resource.getURI.toString)
        
        // Find the main reactor and create an AST node for its instantiation.
        for (reactor : resource.allContents.toIterable.filter(Reactor)) {
            if (reactor.isMain || reactor.isFederated) {
                // Creating an definition for the main reactor because there isn't one.
                this.mainDef = LinguaFrancaFactory.eINSTANCE.createInstantiation()
                this.mainDef.setName(reactor.name)
                this.mainDef.setReactorClass(reactor)
            }
        }
        
        this.resource = resource
        // Figure out the file name for the target code from the source file name.
        resource.analyzeResource
        
        // Clear any markers that may have been created by a pervious build.
        // Markers mark problems in the Eclipse IDE when running in integrated mode.
        clearMarkers()
        
        // If federates are specified in the target, create a mapping
        // from Instantiations in the main reactor to federate names.
        // Also create a list of federate names or a list with a single
        // empty name if there are no federates specified.
        // This must be done before desugaring delays below.
        resource.analyzeFederates

        // Replace connections annotated with the "after" keyword by ones
        // that go through a delay reactor. 
        resource.insertGeneratedDelays()
            
    }
    
    /**
     * Find connections that have a delay associated with them and reroute them via
     * a generated delay reactor.
     * @param resource The AST.
     */
    def void insertGeneratedDelays(Resource resource) {
        resource.insertGeneratedDelays(this)
    }
    
    /** Generate code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation. This base class invokes generateReactor()
     *  for each contained reactor, including any reactors defined
     *  in imported .lf files (except any main reactors in those
     *  imported files). If errors occur during generation,
     *  then a subsequent call to errorsOccurred() will return true.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context Context relating a specific invocation of the code generator. 
     */
    def void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {

        analyzeModel(resource, fsa, context)

        // First, produce any preamble code that the code generator needs
        // to produce before anything else goes into the code generated files.
        generatePreamble()
        
        // Collect a list of reactors defined in this resource and (non-main)
        // reactors defined in imported resources.
        reactors = newLinkedList
        
        // Next process all the imports and call generateReactor on any
        // reactors defined in the imports.
        processImports(resource)
        
        // Abort compilation if a dependency cycle was detected while 
        // processing imports. If compilation would continue, dependency
        // cycles between reactor instantiations across files could lead
        // to a stack overflow!
        if (cyclicImports) {
            throw new Exception("Aborting compilation due to dependency cycles in imports!") 
        }

        // Recursively generate reactor class code from their definitions
        // NOTE: We do not generate code for the main reactor here
        // because that code needs to be customized for federates in
        // a distributed execution.  Subclasses are required to
        // generate the main reactor code.
        // FIXME: It may be better to also not generate code for
        // non-main reactors that are not instantiated in a particular
        // federate. But it seems harmless to generate it since a good
        // compiler will remove it anyway as dead code.
        for (reactor : resource.allContents.toIterable.filter(Reactor)) {
            if (!reactor.isMain && !reactor.isFederated) {
                generateReactor(reactor)
            }
        }
    }
    
    /** Return true if errors occurred in the last call to doGenerate().
     *  This will return true if any of the reportError methods was called.
     *  @return True if errors occurred.
     */
    def errorsOccurred() {
        return generatorErrorsOccurred;
    }
    
    /** Collect data in a reactor or composite definition.
     *  Subclasses should override this and be sure to call
     *  super.generateReactor(reactor).
     *  @param reactor The parsed reactor AST data structure.
     */
    def void generateReactor(Reactor reactor) {
        reactors.add(reactor)

        // Reset indentation, in case it has gotten messed up.
        indentation.put(code, "")
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param the action to schedule
     * @param the port to read from
     */
    abstract def String generateDelayBody(Action action, VarRef port);

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     * @param the action that triggers the reaction
     * @param the port to write to
     */
    abstract def String generateForwardBody(Action action, VarRef port);
    
    /**
     * Generate code for the generic type to be used in the class definition
     * of a generated delay reactor.
     */
    abstract def String generateDelayGeneric();
    
    /**
     * Generate code for referencing a port, action, or timer.
     * @param reference The referenced variable.
     */
    def String generateVarRef(VarRef reference) {
        var prefix = "";
        if (reference.container !== null) {
            prefix = reference.container.name + "." 
        }
        return prefix + reference.variable.name
    }

    /** Given a representation of time that may possibly include units,
     *  return a string that the target language can recognize as a value.
     *  In this base class, if units are given, e.g. "msec", then
     *  we convert the units to upper case and return an expression
     *  of the form "MSEC(value)". Particular target generators will need
     *  to either define functions or macros for each possible time unit
     *  or override this method to return something acceptable to the
     *  target language.
     *  @param time A TimeValue that represents a time.
     *  @return A string, such as "MSEC(100)" for 100 milliseconds.
     */
    def String timeInTargetLanguage(TimeValue time) {
        if (time !== null) {
            if (time.unit != TimeUnit.NONE) {
                return time.unit.name() + '(' + time.time + ')'
            } else {
                return time.time.toString()
            }    
        }
        return "0" // FIXME: do this or throw exception?
    }

    /** Remove quotation marks surrounding the specified string.
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

    // //////////////////////////////////////////
    // // Protected methods.

    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set is a set of case-insensitive
     *  strings specifying target names. If any target is acceptable,
     *  return null.
     * 
     */
    protected abstract def Set<String> acceptableTargets()
    
    /** Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
    }
    
    /** Clear markers in the IDE if running in integrated mode.
     *  This has the side effect of setting the iResource variable
     *  to point to the IFile for the Lingua Franca program.
     */
    protected def clearMarkers() {
        if (mode == Mode.INTEGRATED) {
            val uri = resource.getURI()
            val platformResourceString = uri.toPlatformString(true);
            iResource = ResourcesPlugin.getWorkspace().getRoot().getFile(new Path(platformResourceString))
            try {
                // First argument can be null to delete all markers.
                // But will that delete xtext markers too?
                iResource.deleteMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE);
            } catch (Exception e) {
                // Ignore, but print a warning.
                println("Warning: Deleting markers in the IDE failed: " + e)
            }
        }
    } 
    
    /** Execute the command given by the specified list of strings,
     *  print the command, its return code, and its output to
     *  stderr and stdout, and return the return code, which is 0
     *  if the command succeeds.
     * 
     *  If the command fails to execute, then a second attempt is
     *  made using a bash shell with the --login option, which sources
     *  the user's ~/.bash_profile, ~/.bash_login, or ~/.bashrc (whichever
     *  is first found) before running the command. This helps to ensure
     *  that the user's PATH variable is set according to their usual
     *  environment, assuming that they use a bash shell.
     * 
     *  More information: Unfortunately, at least on a Mac,
     *  if you are running within Eclipse, the PATH variable
     *  is extremely limited; supposedly, it is given by the default
     *  provided in /etc/paths, but at least on my machine,
     *  it does not even include directories in that file for some reason.
     *  One way to add a directory like
     *  /usr/local/bin to the path once-and-for-all is this:
     * 
     *     sudo launchctl config user path /usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
     * 
     *  But asking users to do that is not ideal. Hence, we try a more
     *  hack-y approach of just trying to execute using a bash shell.
     * 
     *  @param command The command.
     *  @param directory The directory in which to execute the command.
     *  @return 0 if the command succeeds, otherwise, an error code.
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
    
    /** Return the target. */
    def findTarget(Resource resource) {
        var target = null as Target
        for (t : resource.allContents.toIterable.filter(Target)) {
            if (target !== null) {
                throw new RuntimeException("There is more than one target!")
            }
            target = t
        }
        if (target === null) {
            throw new RuntimeException("No target found!")
        }
        target
    }

    /**
     * Generate code for the body of a reaction that handles input from the network
     * that is handled by the specified action. This base class throws an exception.
     * @param action The action that has been created to handle incoming messages.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @throws UnsupportedOperationException If the target does not support this operation.
     */
    def String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        InferredType type
    ) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.")
    }
    
    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network. This base class throws an exception.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @throws UnsupportedOperationException If the target does not support this operation.
     */
    def String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        InferredType type
    ) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.")
    }
    
    /** Generate any preamble code that appears in the code generated
     *  file before anything else.
     */
    protected def generatePreamble() {
        prComment("Code generated by the Lingua Franca compiler from file:")
        prComment(sourceFile)
    }

    /** Get the code produced so far.
     *  @return The code produced so far as a String.
     */
    protected def getCode() {
        code.toString()
    }
        
    /** Increase the indentation of the output code produced.
     */
    protected def indent() {
        indent(code)
    }

    /** Increase the indentation of the output code produced
     *  on the specified builder.
     *  @param The builder to indent.
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

    /** Open a non-Lingua Franca import file at the specified URI
     *  in the specified resource set. Throw an exception if the
     *  file import is not supported. This base class always throws
     *  an exception because the only supported imports, by default,
     *  are Lingua Franca files.
     *  @param importStatement The original import statement (used for error reporting).
     *  @param resourceSet The resource set in which to find the file.
     *  @param resolvedURI The URI to import.
     */
    protected def openForeignImport(
        Import importStatement, ResourceSet resourceSet, URI resolvedURI
    ) {
        reportError(importStatement, "Unsupported imported file type: "
            + importStatement.importURI
        )
    }
    
    /** Open an import at the Lingua Franca file at the specified URI
     *  in the specified resource set and call generateReactor() on
     *  any non-main reactors given in that file.
     *  @param importStatement The import statement.
     *  @param resourceSet The resource set in which to find the file.
     *  @param resolvedURI The URI to import.
     *  @return The imported resource or null if the import fails.
     */
    protected def openLFImport(Import importStatement, ResourceSet resourceSet, URI resolvedURI) {
        val importResource = resourceSet?.getResource(resolvedURI, true);
        if (importResource === null) {
            reportError(importStatement, "Cannot find import file: " + resolvedURI)
            return null
        } else {
            // Make sure the target of the import is acceptable.
            var targetOK = (acceptableTargets === null)
            var offendingTarget = ""
            for (target : importResource.allContents.toIterable.filter(Target)) {
                for (acceptableTarget : acceptableTargets ?: emptyList()) {
                    if (acceptableTarget.equalsIgnoreCase(target.name)) {
                        targetOK = true
                    }
                }
                if (!targetOK) offendingTarget = target.name
            }
            if (!targetOK) {
                reportError(importStatement, "Import target " + offendingTarget
                    + " is not an acceptable target in import "
                    + importResource.getURI
                    + ". Acceptable targets are: "
                    + acceptableTargets.join(", ")
                )
                return null
            } else {
                // Temporarily change the sourceFile variable to point to the
                // import file. Then change it back.
                val previousSourceFile = sourceFile
                sourceFile = importResource.toPath
                try {
                    // Process any imports that the import has.
                    processImports(importResource)
                    // Call generateReactor for each reactor contained by the import
                    // that is not a main reactor.
                    for (reactor : importResource.allContents.toIterable.filter(Reactor)) {
                        if (!reactor.isMain && !reactor.isFederated) {
                            println("Including imported reactor: " + reactor.name)
                            generateReactor(reactor)
                        }
                    }
                } finally {
                    sourceFile = previousSourceFile
                }
            }
        }
        return importResource
    }

    /** Append the specified text plus a final newline to the current
     *  code buffer.
     *  @param text The text to append.
     */
    protected def pr(String format, Object... args) {
        pr(code,
            if (args !== null && args.length > 0) String.format(format,
                args) else format)
    }

    /** Append the specified text plus a final newline to the specified
     *  code buffer.
     *  @param builder The code buffer.
     *  @param text The text to append.
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

    /** Prints an indented block of text with the given begin and end markers,
     *  but only if the actions print any text at all.
     *  This is helpful to avoid the production of empty blocks.
     *  @param begin The prolog of the block.
     *  @param end The epilog of the block.
     *  @param actions Actions that print the interior of the block. 
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

    /** Print a comment to the generated file.
     *  Particular targets will need to override this if comments
     *  start with something other than '//'.
     *  @param comment The comment.
     */
    protected def prComment(String comment) {
        pr(code, '// ' + comment);
    }

    /** Process any imports included in the resource defined by the
     *  specified resource. This will open the import, check for
     *  compatibility, and call generateReactor on any reactors the
     *  import defines that are not main reactors.
     *  If the target is not acceptable to this
     *  generator, as reported by acceptableTargets, report an error,
     *  ignore the import, and continue.
     *  @param resource The resource (file) that may contain import
     *   statements.
     */
    protected def void processImports(Resource resource) {
        // if the resource is in the recursion stack, then there is a cycle in the imports
        if (importRecursionStack.contains(resource)) {
            cyclicImports = true
            throw new Exception("There is a dependency cycle in the import statements!")
        }
        
        // abort if the resource was visited already
        if (importedResources.keySet.contains(resource)) {
            return
        }
        
        // add resource to imported resources and to the recoursion stack
        importedResources.put(resource, new HashSet<Resource>())        
        importRecursionStack.add(resource);

        for (importStatement : resource.allContents.toIterable.filter(Import)) {
            // Resolve the import as a URI relative to the current resource's URI.
            val URI currentURI = resource?.getURI;
            val URI importedURI = URI?.createFileURI(importStatement.importURI);
            val URI resolvedURI = importedURI?.resolve(currentURI);
            val ResourceSet resourceSet = resource?.resourceSet;
            
            // Check for self import.
            if (resolvedURI.equals(currentURI)) {
                reportError(importStatement,
                    "Recursive imports are not permitted: " + importStatement.importURI)
                return
            }
            try {
                if (importStatement.importURI.endsWith(".lf")) {
                    // Handle Lingua Franca imports.
                    val imported = openLFImport(importStatement, resourceSet, resolvedURI)
                    if (imported !== null) {
                        importedResources.get(resource).add(imported)
                    }
                } else {
                    // Handle other supported imports (if any).
                    openForeignImport(importStatement, resourceSet, resolvedURI)
                }
            } catch (Exception ex) {
                reportError(
                    importStatement,
                    "Import error: " + importStatement.importURI +
                    "\nException message: " + ex.message
                )
            }
        }
        
        // remove this resource from the recursion stack
        importRecursionStack.remove(resource);
    }
    
    /** Parse the specified string for command errors that can be reported
     *  using marks in the Eclipse IDE. In this base class, this is simply
     *  reported as a mark on the first line of the file. Subclasses can
     *  be much smarter by parsing the argument and looking for line numbers
     *  and file names to report on the relevant lines.
     *  @param stderr The output on standard error of executing a command.
     */
    protected def reportCommandErrors(String stderr) {
        report(stderr, IMarker.SEVERITY_ERROR, null)
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

    /** Report a warning or error on the specified line of the specified resource.
     *  The caller should not throw an exception so execution can continue.
     *  This will print the error message to stderr.
     *  If running in INTEGRATED mode (within the Eclipse IDE), then this also
     *  adds a marker to the editor.
     *  @param message The error message.
     *  @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     *  @param line The line number or null if it is not known.
     *  @param resource The resource.
     */
    protected def report(String message, int severity, Integer line, IResource resource) {        
        if (severity === IMarker.SEVERITY_ERROR) {
            generatorErrorsOccurred = true;
        }
        val header = (severity === IMarker.SEVERITY_ERROR)? "ERROR" : "WARNING"
        val lineAsString = (line === null)? "unknown" : "" + line
        val toPrint = header + ": " + resource.fullPath + ": Line " + lineAsString + ":\n" + message
        System.err.println(toPrint)
        
        // If running in INTEGRATED mode, create a marker in the IDE for the error.
        // See: https://help.eclipse.org/2020-03/index.jsp?topic=%2Forg.eclipse.platform.doc.isv%2Fguide%2FresAdv_markers.htm
        if (mode === Mode.INTEGRATED) {
            val marker = resource.createMarker(IMarker.PROBLEM)
            marker.setAttribute(IMarker.MESSAGE, toPrint);
            if (line !== null) {
                marker.setAttribute(IMarker.LINE_NUMBER, line);
            } else {
                marker.setAttribute(IMarker.LINE_NUMBER, 1);
            }
            // Human-readable line number information.
            marker.setAttribute(IMarker.LOCATION, "Line " + lineAsString);
            // Mark as an error or warning.
            marker.setAttribute(IMarker.SEVERITY, severity);
            marker.setAttribute(IMarker.PRIORITY, IMarker.PRIORITY_HIGH);
        
            marker.setAttribute(IMarker.USER_EDITABLE, false);
        
            // NOTE: It might be useful to set a start and end.
            // marker.setAttribute(IMarker.CHAR_START, 0);
            // marker.setAttribute(IMarker.CHAR_END, 5);
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
        return report(message, severity, line, iResource)
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

    /** Reduce the indentation by one level for generated code
     *  in the default code buffer.
     */
    protected def unindent() {
        unindent(code)
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

    /**
     * Create a list of default parameter initializers in target code.
     * 
     * @param param The parameter to create initializers for
     * @return A list of initializers in target code
     */
    protected def getInitializerList(Parameter param) {
        var list = new LinkedList<String>();

        for (i : param?.init) {
            if (param.isOfTimeType) {
                list.add(i.targetTime)
            } else {
                list.add(i.targetValue)
            }
        }
        return list
    }
    
    /**
     * Create a list of state initializers in target code.
     * 
     * @param state The state variable to create initializers for
     * @return A list of initializers in target code
     */
    protected def List<String> getInitializerList(StateVar state) {
        if (!state.isInitialized) {
            return null
        }

        var list = new LinkedList<String>();

        for (i : state?.init) {
            if (i.parameter !== null) {
                list.add(i.parameter.targetReference)
            } else if (state.isOfTimeType) {
                list.add(i.targetTime)
            } else {
                list.add(i.targetValue)
            }
        }
        return list
    }
    
    /**
     * Create a list of parameter initializers in target code in the context
     * of an reactor instantiation.
     * 
     * This respects the parameter assignments given in the reactor
     * instantiation and falls back to the reactors default initializers
     * if no value is assigned to it. 
     * 
     * @param param The parameter to create initializers for
     * @return A list of initializers in target code
     */
    protected def getInitializerList(Parameter param, Instantiation i) {
        if (i === null || param === null) {
            return null
        }
        
        val assignments = i.parameters.filter[p | p.lhs === param]
        
        if (assignments.size == 0) {
            // the parameter was not overwritten in the instantiation
            return param.initializerList
        } else {
            // the parameter was overwritten in the instantiation
            var list = new LinkedList<String>();
            for (init : assignments.get(0)?.rhs) {
                if (param.isOfTimeType) {
                    list.add(init.targetTime)
                } else {
                    list.add(init.targetValue)
                }
            }
            return list
        }
    }

    /**
     * Generate target code for a parameter reference.
     * 
     * @param param The parameter to generate code for
     * @return Parameter reference in target code
     */
    protected def String getTargetReference(Parameter param) {
        return param.name
    }
        
    ////////////////////////////////////////////////////
    //// Private functions
    
    /** Analyze the resource (the .lf file) that is being parsed
     *  to determine whether code is being mapped to single or to
     *  multiple target machines. If it is being mapped to multiple
     *  machines, then set the 'federates' list, the 'federateIDs'
     *  map, and the 'federationRTIHost' and 'federationRTIPort'
     *  variables.
     * 
     *  In addition, analyze the connections between federates.
     *  Ensure that every cycle has a non-zero delay (microstep
     *  delays will not be sufficient). Construct the dependency
     *  graph between federates. And replace connections between
     *  federates with a pair of reactions, one triggered by
     *  the sender's output port, and the other triggered by
     *  an action.
     * 
     *  This class is target independent, so the target code
     *  generator still has quite a bit of work to do.
     *  It needs to provide the body of the sending and
     *  receiving reactions. It also needs to provide the
     *  runtime infrastructure that uses the dependency
     *  information between federates. See the C target
     *  for a reference implementation.
     */
    private def analyzeFederates(Resource resource) {
        // Next, if there actually are federates, analyze the topology
        // interconnecting them and replace the connections between them
        // with an action and two reactions.
        if (mainDef === null || !mainDef.reactorClass.isFederated) {
            // Ensure federates is never empty.
            var federateInstance = new FederateInstance(null, 0, this)
            federates.add(federateInstance)
            federateByName.put("", federateInstance)
            federateByID.put(0, federateInstance)
        } else {
            // In a federated execution, we need keepalive to be true,
            // otherwise a federate could exit simply because it hasn't received
            // any messages.
            targetKeepalive = true
            
            if (mainDef.reactorClass.host !== null) {
                // Get the host information, if specified.
                // If not specified, this defaults to 'localhost'
                if (mainDef.reactorClass.host.addr !== null) {
                    federationRTIProperties.put('host', mainDef.reactorClass.host.addr)                
                }
                // Get the port information, if specified.
                // If not specified, this defaults to 14045
                if (mainDef.reactorClass.host.port !== 0) {
                    federationRTIProperties.put('port', mainDef.reactorClass.host.port)                
                }
                
            }
            
            // Create a FederateInstance for each top-level reactor.
            for (instantiation : mainDef.reactorClass.instantiations) {
                // Assign an integer ID to the federate.
                var federateID = federates.length
                // Add the federate name to the list of names.
                var federateInstance = new FederateInstance(instantiation, federateID, this)
                federates.add(federateInstance)
                federateByName.put(instantiation.name, federateInstance)
                federateByID.put(federateID, federateInstance)

                if (federateByReactor === null) {
                    federateByReactor = new HashMap<String, FederateInstance>()
                }
                for (reactorName : federateInstance.containedReactorNames) {
                    federateByReactor.put(reactorName, federateInstance)
                }
            }
            
            // Analyze the connection topology of federates.
            // First, find all the connections between federates.
            // Those that are labeled "physical" create no dependency.
            // Otherwise, there is a dependency. This may have a delay
            // which corresponds to the "lookahead" of HLA.
            // FIXME: If there is no delay, we may have to transmit
            // the microstep, not just the timestamp.
            // FIXME: Now that each top-level reactor is a federate,
            // this is redundant with the connectivity information in
            // ReactorInstanace.
            
            // For each connection between federates, replace it in the
            // AST with an action (which inherits the delay) and two reactions.
            // The action will be physical.
            if (mainDef !== null) {
                var connectionsToRemove = new LinkedList<Connection>()
                for (connection : mainDef.reactorClass.connections) {
                    var leftFederate = federateByReactor.get(connection.leftPort.container.name)
                    var rightFederate = federateByReactor.get(connection.rightPort.container.name)
                    if (leftFederate !== rightFederate) {
                        // Connection spans federates.
                        // First, update the dependencies in the FederateInstances.
                        // Exclude physical connections because these do not create real dependencies.
                        if (leftFederate !== rightFederate && !connection.physical) {
                            var dependsOn = rightFederate.dependsOn.get(leftFederate)
                            if (dependsOn === null) {
                                dependsOn = new HashSet<Value>()
                                rightFederate.dependsOn.put(leftFederate, dependsOn)
                            }
                            if (connection.delay !== null) {
                                dependsOn.add(connection.delay)
                            }
                            var sendsTo = leftFederate.sendsTo.get(rightFederate)
                            if (sendsTo === null) {
                                sendsTo = new HashSet<Value>()
                                leftFederate.sendsTo.put(rightFederate, sendsTo)
                            }
                            if (connection.delay !== null) {
                                sendsTo.add(connection.delay)
                            }
                            // Check for causality loops between federates.
                            // FIXME: This does not detect cycles involving more than one federate.
                            var reverseDependency = leftFederate.dependsOn.get(rightFederate)
                            if (reverseDependency !== null) {
                                // Check that at least one direction has a delay.
                                if (reverseDependency.size === 0 && dependsOn.size === 0) {
                                    // Found a causality loop.
                                    val message = "Causality loop found between federates "
                                            + leftFederate.name + " and " + rightFederate.name
                                    reportError(connection, message)
                                    // This is a fatal error, so throw an exception.
                                    throw new Exception(message)
                                }
                            }
                        }
                        
                        // Next, replace the connection in the AST with an action
                        // (which inherits the delay) and two reactions.
                        // The action will be physical if the connection physical and
                        // otherwise will be logical.
                        connection.makeCommunication(leftFederate, rightFederate, this)
                        
                        // To avoid concurrent modification exception, collect a list
                        // of connections to remove.
                        connectionsToRemove.add(connection)
                    }
                }
                for (connection : connectionsToRemove) {
                    // Remove the original connection for the parent.
                    mainDef.reactorClass.connections.remove(connection)
                }
            }
        }
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
    
    /** Extract the name of a file from a path represented as a string.
     *  If the file ends with '.lf', the extension is removed.
     */
    protected def getFilename(String path) {
        var File f = new File(path)
        var name = f.getName()
        if (name.endsWith('.lf')) {
            name = name.substring(0, name.length - 3)
        }
        return name
    }
    
    /** Extract the directory from a path represented as a string.
     */
    protected def getDirectory(String path) {
        var File f = new File(path)
        f.getParent()
    }
    
    /** Analyze the resource (the .lf file) that is being parsed
     *  to generate code to set the following variables:
     *  directory, filename, mode, sourceFile.
     */
    private def analyzeResource(Resource resource) {
        sourceFile = resource.toPath;
        
        // Strip the filename of the extension.
        var File f = new File(sourceFile);
        filename = f.getName();
        directory = f.getParent();
       
        if (filename.endsWith('.lf')) {
            filename = filename.substring(0, filename.length - 3)
        }

        if (resource.URI.isPlatform) {
            mode = Mode.INTEGRATED
        } else if (resource.URI.isFile) {
            mode = Mode.STANDALONE
        } else {
            System.err.println(
                "ERROR: Source file protocol is not recognized: " +
                    resource.URI);
        }

        println('******** filename: ' + filename)
        println('******** sourceFile: ' + sourceFile)
        println('******** directory: ' + directory)
        println('******** mode: ' + mode)
    }
    
    /** Execute a process while forwarding output and error to system streams.
     *
     *  Executing a process directly with `processBuiler.start()` could
     *  lead to a deadlock as the subprocess blocks when output or error
     *  buffers are full. This method ensures that output and error messages
     *  are continuously read and forwards them to the system's output and
     *  error streams.
     *
     *  @param processBuilder The process to be executed.
     *  @author{Christian Menard <christian.menard@tu-dresden.de}
     */
    protected def runSubprocess(ProcessBuilder processBuilder) {
        return runSubprocess(processBuilder, System.out, System.err);
    }

    /** Execute a process while forwarding output and error streams.
     *
     *  Executing a process directly with `processBuiler.start()` could
     *  lead to a deadlock as the subprocess blocks when output or error
     *  buffers are full. This method ensures that output and error messages
     *  are continuously read and forwards them to the given streams.
     *
     *  @param processBuilder The process to be executed.
     *  @param outStream The stream to forward the process' output to.
     *  @param errStream The stream to forward the process' error messages to.
     *  @author{Christian Menard <christian.menard@tu-dresden.de}
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
     * Return true if the target supports generics (i.e., parametric
     * polymorphism), false otherwise.
     */
    abstract def boolean supportsGenerics()
    
    abstract def String getTargetTimeType()

    abstract def String getTargetUndefinedType()
    
    abstract def String getTargetFixedSizeListType(String baseType, Integer size)

    abstract def String getTargetVariableSizeListType(String baseType);
    
    def String getTargetType(InferredType type) {
        if (type.isUndefined) {
            return targetUndefinedType
        } else if (type.isTime) {
            if (type.isFixedSizeList) {
                return targetTimeType.getTargetFixedSizeListType(type.listSize)
            } else if (type.isVariableSizeList) {
                return targetTimeType.targetVariableSizeListType
            } else {
                return targetTimeType
            }
        } else if (type.isFixedSizeList) {
            return type.baseType.getTargetFixedSizeListType(type.listSize)
        } else if (type.isVariableSizeList) {
            return type.baseType.targetVariableSizeListType
        }
        return type.toText
    }
    
    protected def getTargetType(Parameter p) {
        return p.inferredType.targetType
    }
    
    protected def getTargetType(StateVar s) {
        return s.inferredType.targetType
    }
    
    protected def getTargetType(Action a) {
        return a.inferredType.targetType
    }
    
    protected def getTargetType(Port p) {
        return p.inferredType.targetType
    }
    
    protected def getTargetType(Type t) {
        InferredType.fromAST(t).targetType
    }

    /**
     * Get textual representation of a time in the target language.
     * 
     * @param t A time AST node
     * @return A time string in the target language
     */
    protected def getTargetTime(Time t) {
        val value = new TimeValue(t.interval, t.unit)
        return value.timeInTargetLanguage
    }

    /**
     * Get textual representation of a value in the target language.
     * 
     * If the value evaluates to 0, it is interpreted as a normal value.
     * 
     * @param v A time AST node
     * @return A time string in the target language
     */
    protected def getTargetValue(Value v) {
        if (v.time !== null) {
            return v.time.targetTime
        }
        return v.toText
    }
    
    /**
     * Get textual representation of a value in the target language.
     * 
     * If the value evaluates to 0, it is interpreted as a time.
     * 
     * @param v A time AST node
     * @return A time string in the target language
     */
    protected def getTargetTime(Value v) {   
        if (v.time !== null) {
            return v.time.targetTime
        } else if (v.isZero) {
            val value = new TimeValue(0, TimeUnit.NONE)
            return value.timeInTargetLanguage
        }
        return v.toText 
    }

    enum Mode {
        STANDALONE,
        INTEGRATED,
        UNDEFINED
    }

}
