/* Generator base class for shared code between code generators. */

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

import java.io.File
import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.List
import java.util.Map
import java.util.Set
import org.eclipse.core.resources.IMarker
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.core.runtime.Path
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.InferredType
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.VarRef

import static extension org.icyphy.ASTUtils.*

/**
 * Generator base class for shared code between code generators.
 * This class implements common capabilities for LF code generators.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Chris Gill, <cdgill@wustl.edu>}
 *  @author{Christian Menard <christian.menard@tu-dresden.de}
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 */
abstract class GeneratorBase extends CodeGenerator {

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
    
    
    ////////////////////////////////////////////
    //// Protected fields.

    static protected CharSequence listItemSeparator = ', '

    /**
     * Path to the directory containing the .lf file.
     */
    protected var String directory
    
    /**
     * The root filename for the main file containing the source code,
     * without the .lf extension.
     */
    protected var String filename

    /** 
     * The main (top-level) reactor instance.
     */
    protected ReactorInstance main
    
    /**
     * Definition of the main (top-level) reactor
     */
    protected Instantiation mainDef
    
    /**
     * A list of Reactor definitions in the main resource, including non-main 
     * reactors defined in imported resources.
     */
    protected var List<Reactor> reactors
    
    /**
     * The file containing the main source code.
     */
    protected var Resource resource
    
    /** 
     * The path from the LF file directory 
     * to the directory containing generated
     * RTI C code. This can be overridden in a target
     * generator to change the directory.
     */
    protected var rtiSrcPath = File.separator + "src-gen";
    
    /** 
     * The path from the LF file directory
     * to the directory containing the compiled
     * RTI binary. This can be overridden in a target
     * generator to change the directory.
     */
    protected var rtiBinPath = File.separator + "bin"
    
    /**
     * The full path to the file containing the .lf file including the
     * full filename with the .lf extension. This starts out as the
     * main .lf file, but while a file is being imported, it temporarily
     * changes to the full path of the imported file.
     */
    protected var String sourceFile
    
    /**
     * Variant of {@link #GeneratorBase.sourceFile GeneratorBase.sourceFile}
     * used on the Windows platform.
     */
    protected var String windowsSourceFile
    
    /** 
     * Additional sources to add to the 
     * compile RTI command if appropriate.
     */
    protected var compileAdditionalSources = null as ArrayList<String>
    
    /**
     * Additional libraries to add to the compile RTI 
     * command using the "-l" command-line option.
     */
    protected var compileLibraries = null as ArrayList<String>
    
    /**
     * The set of unordered reactions. An unordered reaction is one that does
     * not have any dependency on other reactions in the containing reactor, 
     * and where no other reaction in the containing reactor depends on it.
     * There is currently no way in the syntax of LF to make a reaction
     * unordered, deliberately, because it can introduce unexpected
     * nondeterminacy. However, certain automatically generated reactions are
     * known to be safe to be unordered because they do not interact with the
     * state of the containing reactor. To make a reaction unordered, when
     * the Reaction instance is created, add that instance to this set.
     */
    protected var Set<Reaction> unorderedReactions = null
    
    /**
     * A map of all resources to the set of resource they import
     */
    protected var importedResources = new HashMap<Resource, Set<Resource>>;
    
    ////////////////////////////////////////////
    //// Target properties, if they are included.
    
    /**
     * A list of federate instances or a list with a single empty string
     * if there are no federates specified.
     */
    protected var List<FederateInstance> federates = new LinkedList<FederateInstance>
    
    /**
     * A map from federate names to federate instances.
     */
    protected var Map<String,FederateInstance> federateByName
            = new HashMap<String,FederateInstance>()

    /**
     * A map from federate IDs to federate instances.
     */
    protected var Map<Integer,FederateInstance> federateByID
            = new HashMap<Integer,FederateInstance>()

    /**
     * A map from reactor names to the federate instance that contains the
     * reactor.
     */
    protected var Map<String,FederateInstance> federateByReactor

    /**
     * The federation RTI properties, which defaults to 'localhost: 15045'.
     */
    protected val federationRTIProperties = newLinkedHashMap(
        'host' -> 'localhost',
        'port' -> 15045
    )
    
    /**
     * For the top-level reactor (main), a list of reactions in each federate.
     *This will be null if there is only one federate.
     */
    protected var HashMap<FederateInstance,LinkedList<Reaction>> reactionsInFederate = null

    /**
     * The build-type target parameter, or null if there is none.
     */
    protected String targetBuildType

    /**
     * The cmake-include target parameter, or null if there is none.
     */
    protected String targetCmakeInclude
    
    /**
     * The compiler target parameter, or null if there is none.
     */
    protected String targetCompiler

    /**
     * The compiler flags target parameter, or null if there is none.
     */
    protected String targetCompilerFlags

    /**
     * The compiler target no-compile parameter, or false if there is none.
     */
    protected boolean targetNoCompile = false
    
    /**
     * The compiler target no-runtime-validation parameter, or false if there is none.
     */
    protected boolean targetNoRuntimeValidation = false
        
    /**
     * The fast target parameter, or false if there is none.
     */
    protected boolean targetFast = false
    
    /**
     * The value of the keepalive target parameter, or false if there is none.
     */
    protected boolean targetKeepalive
    
    /**
     * The level of logging or null if not given.
     */
    protected String targetLoggingLevel

    /**
     * The threads target parameter, or the default 0 if there is none.
     */
    protected int targetThreads = 0

    /**
     * The timeout parameter, or the default -1 if there is none.
     */
    protected int targetTimeout = -1

    /**
     * The threads timeout unit parameter, or the default null if there is none.
     */
    protected TimeUnit targetTimeoutUnit
    
    /**
     * The tracing target parameter, or false if there is none.
     */
    protected boolean targetTracing = false

    ////////////////////////////////////////////
    //// Private fields.

    /**
     * The RTI generator provides capabilities for
     * generating RTI code and compiling it. Null if
     * federates are not specified.
     */
    var RTIGenerator rtiGenerator = null

    /**
     * Recursion stack used to detect cycles in imports.
     */
    var importRecursionStack = new HashSet<Resource>();
    
    /**
     * A flag indicating whether a cycle was found while processing imports.
     */
    var cyclicImports = false;

    ////////////////////////////////////////////
    //// Code generation functions to override for a concrete code generator.
    
    /**
     * Analyze the model, setting target variables, filenames,
     * working directory, and federates. This also performs any
     * transformations that are needed on the AST of the model,
     * including handling delays on connections and communication
     * between federates.
     * @param resource The resource containing the source code.
     * @param fsa The file system access (used to write the result).
     * @param context Context relating to invocation of the code generator.
     * In stand alone mode, this object is also used to relay CLI arguments.
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
                    case "tracing":
                        if (param.value.literal == 'true') {
                            targetTracing = true
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
        
        // Create an RTIGenerator if federates have been specified.
        // This must be done after analyzeFederates, which sets up the
        // federates list.
        if (federates.length > 1) {
            rtiGenerator = new RTIGenerator(filename, directory, federates, targetThreads)
        } else {
            rtiGenerator = null
        }            
    }
    
    /**
     * Find connections that have a delay associated with them and reroute them via
     * a generated delay reactor.
     * @param resource The AST.
     */
    def void insertGeneratedDelays(Resource resource) {
        resource.insertGeneratedDelays(this)
    }
    
    /**
     * Generate code from the Lingua Franca model contained by the specified resource.
     * 
     * This is the main entry point for code generation. This base class finds all
     * reactor class definitions, including any reactors defined in imported .lf files
     * (except any main reactors in those imported files), and adds them to the 
     * {@link #GeneratorBase.reactors reactors} list. If errors occur during
     * generation, then a subsequent call to errorsOccurred() will return true.
     * @param resource The resource containing the source code.
     * @param fsa The file system access (used to write the result).
     * @param context Context relating to invocation of the code generator.
     * In stand alone mode, this object is also used to relay CLI arguments.
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
        
        // Next process all the imports to find reactors defined in the imports.
        processImports(resource)
        
        // Replace connections in this resources that are annotated with the 
        // "after" keyword by ones that go through a delay reactor. 
        resource.insertGeneratedDelays()
        
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
                reactors.add(reactor)
            }
        }
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

    /**
     * Return true if the reaction is unordered. An unordered reaction is one
     * that does not have any dependency on other reactions in the containing
     * reactor, and where no other reaction in the containing reactor depends
     * on it. There is currently no way in the syntax of LF to make a reaction
     * unordered, deliberately, because it can introduce unexpected 
     * nondeterminacy. However, certain automatically generated reactions are
     * known to be safe to be unordered because they do not interact with the
     * state of the containing reactor. To make a reaction unordered, when
     * the Reaction instance is created, add that instance to this set.
     * @return True if the reaction has been marked unordered.
     */
    def isUnordered(Reaction reaction) {
        if (unorderedReactions !== null) {
            unorderedReactions.contains(reaction)
        } else {
            false
        }
    }
    
    /**
     * Mark the reaction unordered. An unordered reaction is one that does not
     * have any dependency on other reactions in the containing reactor, and
     * where no other reaction in the containing reactor depends on it. There
     * is currently no way in the syntax of LF to make a reaction unordered,
     * deliberately, because it can introduce unexpected nondeterminacy. 
     * However, certain automatically generated reactions are known to be safe
     * to be unordered because they do not interact with the state of the 
     * containing reactor. To make a reaction unordered, when the Reaction
     * instance is created, add that instance to this set.
     * @param reaction The reaction to make unordered.
     */
    def makeUnordered(Reaction reaction) {
        if (unorderedReactions === null) {
            unorderedReactions = new HashSet<Reaction>()
        }
        unorderedReactions.add(reaction)
    }
    
    /**
     * Given a representation of time that may possibly include units, return
     * a string that the target language can recognize as a value. In this base
     * class, if units are given, e.g. "msec", then we convert the units to upper
     * case and return an expression of the form "MSEC(value)". Particular target
     * generators will need to either define functions or macros for each possible
     * time unit or override this method to return something acceptable to the
     * target language.
     * @param time A TimeValue that represents a time.
     * @return A string, such as "MSEC(100)" for 100 milliseconds.
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
    
    // //////////////////////////////////////////
    // Protected methods for code generation
    // of the RTI.
    
    // FIXME: Allow target code generators to specify the directory
    // structure for the generated C RTI?
    /** Create the runtime infrastructure (RTI) source file.
     */
    protected def generateFederateRTI() {
        if (rtiGenerator !== null) {
        rtiGenerator.generateFederateRTI(rtiSrcPath, rtiBinPath,
            targetFast, federationRTIProperties
        )
        } else {
            reportError("Cannot generateFederateRTI. "
                + "rtiGenerator is null, likeley because there are no "
                + "specified federates.")
        }
    }
    
    /** Invoke the compiler on the generated RTI */
    protected def compileRTI() {
       if (rtiGenerator !== null) {
           rtiGenerator.compileRTI(targetCompiler, targetCompilerFlags,
               compileAdditionalSources, compileLibraries)
       } else {
           reportError("Cannot compile RTI. "
               + "rtiGenerator is null, likeley because there are no "
                + "specified federates.")
       }
    }
    
    ////////////////////////////////////////////
    //// Protected methods.

    /**
     * Return a set of targets that are acceptable to this generator.
     * Imported files that are Lingua Franca files must specify targets in this
     * set or an error message will be reported and the import will be ignored.
     * The returned set is a set of case-insensitive strings specifying target
     * names. If any target is acceptable, return null.
     */
    protected abstract def Set<String> acceptableTargets()
    
    /**
     * Clear markers in the IDE if running in integrated mode.
     * This has the side effect of setting the iResource variable to point to
     * the IFile for the Lingua Franca program.
     */
    protected def clearMarkers() {
        if (mode == Mode.INTEGRATED) {
            val uri = resource.getURI()
            val platformResourceString = uri.toPlatformString(true);
            iResource = ResourcesPlugin.getWorkspace().getRoot().getFile(
                new Path(platformResourceString))
            try {
                // First argument can be null to delete all markers.
                // But will that delete xtext markers too?
                iResource.deleteMarkers(IMarker.PROBLEM, true,
                    IResource.DEPTH_INFINITE);
            } catch (Exception e) {
                // Ignore, but print a warning.
                println("Warning: Deleting markers in the IDE failed: " + e)
            }
        }
    } 
    
    /**
     * Return the target.
     */
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
    
    /**
     * Generate any preamble code that appears in the code generated
     * file before anything else.
     */
    protected def generatePreamble() {
        prComment("Code generated by the Lingua Franca compiler from file:")
        prComment(sourceFile)
    }

    /**
     * Open a non-Lingua Franca import file at the specified URI
     * in the specified resource set. Throw an exception if the
     * file import is not supported. This base class always throws
     * an exception because the only supported imports, by default,
     * are Lingua Franca files.
     * @param importStatement The original import statement (used for error reporting).
     * @param resourceSet The resource set in which to find the file.
     * @param resolvedURI The URI to import.
     */
    protected def openForeignImport(
        Import importStatement, ResourceSet resourceSet, URI resolvedURI
    ) {
        reportError(importStatement, "Unsupported imported file type: "
            + importStatement.importURI
        )
    }
    
    /**
     * Open an import at the Lingua Franca file at the specified URI in the
     * specified resource, find all non-main reactors, and add them to the
     * {@link #GeneratorBase.reactors reactors}.
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
                    // Add each reactor contained by the import to the list of reactors,
                    // unless it is a main reactor.
                    for (reactor : importResource.allContents.toIterable.filter(Reactor)) {
                        if (!reactor.isMain && !reactor.isFederated) {
                            println("Including imported reactor: " + reactor.name)
                            reactors.add(reactor)
                        }
                    }
                } finally {
                    sourceFile = previousSourceFile
                }
            }
        }
        return importResource
    }

    /**
     * Process any imports included in the resource defined by the specified
     * resource. This will open the import, check for compatibility, and find
     * and any reactors the import defines that are not main reactors. If the
     * target is not acceptable to this generator, as reported by
     * acceptableTargets, report an error, ignore the import and continue.
     * @param resource The resource (file) that may contain import statements.
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
        
        // Replace connections in this resources that are annotated with the 
        // "after" keyword by ones that go through a delay reactor. 
        resource.insertGeneratedDelays()
        
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
    
    /** Return a list of Reaction containing every reaction of the specified
     *  reactor that should be included in code generated for the specified
     *  federate. If the reaction is triggered by or sends data to a contained
     *  reactor that is not in the federate, then that reaction will not be
     *  included in the returned list.  This method assumes that analyzeFederates
     *  has been called.
     *  @param reactor The reactor
     *  @param federate The federate or null to include all reactions.
     */
    protected def List<Reaction> reactionsInFederate(Reactor reactor, FederateInstance federate) {
        if (!reactor.federated || federate === null || reactionsInFederate === null) {
            reactor.allReactions
        } else {
            // reactionsInFederate is a Map<FederateInstance,List<Reaction>>
            var result = reactionsInFederate.get(federate)
            if (result === null) reactor.allReactions
            else result
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
                // Get the user information, if specified.
                if (mainDef.reactorClass.host.user !== null) {
                    federationRTIProperties.put('user', mainDef.reactorClass.host.user)                
                }
                // Get the directory information, if specified.
                /* FIXME
                if (mainDef.reactorClass.host.dir !== null) {
                    federationRTIProperties.put('dir', mainDef.reactorClass.host.dir)                
                }
                */
            }
            
            // Create the cached list of reactions in federates.
            reactionsInFederate = new HashMap<FederateInstance,LinkedList<Reaction>>()
            
            // Create a FederateInstance for each top-level reactor.
            for (instantiation : mainDef.reactorClass.allInstantiations) {
                // Assign an integer ID to the federate.
                var federateID = federates.length
                // Add the federate name to the list of names.
                var federateInstance = new FederateInstance(instantiation, federateID, this)
                federates.add(federateInstance)
                federateByName.put(instantiation.name, federateInstance)
                federateByID.put(federateID, federateInstance)
                
                if (instantiation.host !== null) {
                    federateInstance.host = instantiation.host.addr
                    // The following could be 0.
                    federateInstance.port = instantiation.host.port
                    // The following could be null.
                    federateInstance.user = instantiation.host.user
                    /* FIXME
                    federateInstance.dir = instantiation.host.dir
                    */
                }

                if (federateByReactor === null) {
                    federateByReactor = new HashMap<String, FederateInstance>()
                }
                for (reactorName : federateInstance.containedReactorNames) {
                    federateByReactor.put(reactorName, federateInstance)
                }
            }
            
            // In a federated execution, we need keepalive to be true,
            // otherwise a federate could exit simply because it hasn't received
            // any messages.
            if (federates.size > 1) {
                targetKeepalive = true
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
                                val message = "Causality loop found between federates " + leftFederate.name + " and " +
                                    rightFederate.name
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
            // Construct the cached list of reactions in federates.
            for (federate : federates) {
                val reactions = new LinkedList<Reaction>()
                reactionsInFederate.put(federate, reactions)
                for (reaction : mainDef.reactorClass.allReactions) {
                    if (federate.containsReaction(mainDef.reactorClass, reaction)) {
                        reactions.add(reaction)
                    }
                }
            }
        }
    }
    
    /**
     * Analyze the resource (the .lf file) that is being parsed
     * to generate code to set the following variables:
     * directory, filename, mode, sourceFile.
     */
    private def analyzeResource(Resource resource) {
        sourceFile = resource.toPath;
        windowsSourceFile = sourceFile.replace("\\","\\\\");
        
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
}
