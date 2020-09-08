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

import java.io.ByteArrayOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStream
import java.nio.file.Files
import java.nio.file.Paths
import java.nio.file.StandardCopyOption
import java.util.ArrayList
import java.util.LinkedHashMap
import java.util.LinkedHashSet
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
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.InferredType
import org.icyphy.Targets.TargetProperties
import org.icyphy.TimeValue
import org.icyphy.graph.InstantiationGraph
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Element
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Model
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
import org.icyphy.linguaFranca.Variable
import org.icyphy.validation.AbstractLinguaFrancaValidator

import static extension org.icyphy.ASTUtils.*
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.xtext.validation.CheckMode

/**
 * Generator base class for shared code between code generators.
 * This extends AbstractLinguaFrancaValidator so that errors can be highlighted
 * in the XText-based IDE.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
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
        
    ////////////////////////////////////////////
    //// Protected fields.
        
    /**
     * All code goes into this string buffer.
     */
    protected var code = new StringBuilder
    
    /** Additional sources to add to the compile command if appropriate. */
    protected var List<String> compileAdditionalSources = newArrayList
    
    /** Additional libraries to add to the compile command using the "-l" command-line option. */
    protected var List<String> compileLibraries = newArrayList

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
     * Indicator of whether generator errors occurred.
     * This is set to true by the report() method and returned by the
     * errorsOccurred() method.
     */
    var generatorErrorsOccurred = false
    
    /**
     * If running in an Eclipse IDE, the iResource refers to the
     * IFile representing the Lingua Franca program.
     * This is the XText view of the file, which is distinct
     * from the Eclipse eCore view of the file and the OS view of the file.
     */
    protected var iResource = null as IResource
    
    /** 
     * The main (top-level) reactor instance.
     */
    protected ReactorInstance main
    
    /**
     * Definition of the main (top-level) reactor.
     * This is an automatically generated AST node for the top-level
     * reactor.
     */
    protected Instantiation mainDef
    
    /**
     * {@link #Mode.STANDALONE Mode.STANDALONE} if the code generator is being
     * called from the command line, {@link #Mode.INTEGRATED Mode.INTEGRATED}
     * if it is being called from the Eclipse IDE, and 
     * {@link #Mode.UNDEFINED Mode.UNDEFINED} otherwise.
     */
    protected var mode = Mode.UNDEFINED
    
    /**
     * A list of Reactor definitions in the main resource, including non-main 
     * reactors defined in imported resources.
     */
    protected var List<Reactor> reactors = newLinkedList // FIXME: derived from instantiationGraph 
    
    /**
     * The set of resources referenced reactor classes reside in.
     */
    protected var Set<Resource> resources = newLinkedHashSet // FIXME: derived from instantiationGraph
    
    /**
     * Graph that tracks dependencies between instantiations.
     */
    protected var InstantiationGraph instantiationGraph
    
    /**
     * The file containing the main source code.
     * This is the Eclipse eCore view of the file, which is distinct
     * from the XText view of the file and the OS view of the file.
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
            = new LinkedHashMap<String,FederateInstance>()

    /**
     * A map from federate IDs to federate instances.
     */
    protected var Map<Integer,FederateInstance> federateByID
            = new LinkedHashMap<Integer,FederateInstance>()

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
     * List of files to be copied to src-gen.
     */
    protected List<File> targetFiles = newLinkedList
    
    /**
     * List of proto files to be processed by the code generator.
     */
    protected List<File> protoFiles = newLinkedList
    
    /**
     * Contents of $LF_CLASSPATH, if it was set.
     */
    protected String classpath
    
    /**
     * The value of the keepalive target parameter, or false if there is none.
     */
    protected boolean targetKeepalive
    
    /**
     * The target name.
     */
    protected String targetName
    
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
     * Map from builder to its current indentation.
     */
    var indentation = new LinkedHashMap<StringBuilder, String>()
    

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
        
        this.resource = resource
        
        // Clear any markers that may have been created by a previous build.
        // Markers mark problems in the Eclipse IDE when running in integrated mode.
        clearMarkers()
        
        generatorErrorsOccurred = false // FIXME: do this in clearMarkers?
        
        // Figure out the file name for the target code from the source file name.
        resource.analyzeResource
        
        var target = resource.findTarget
        targetName = target.name
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
                    case TargetProperties.FILES.name:
                        this.targetFiles.addAll(this.collectFiles(param.value))
                    case TargetProperties.PROTOBUFS.name: 
                        this.protoFiles.addAll(this.collectFiles(param.value))
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
        
        
        // If federates are specified in the target, create a mapping
        // from Instantiations in the main reactor to federate names.
        // Also create a list of federate names or a list with a single
        // empty name if there are no federates specified.
        // This must be done before desugaring delays below.
        resource.analyzeFederates            
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
        
        // The following "analysis" has hidden in it AST transformations.
        // FIXME: We should factor them out and rename the following method
        // parseTargetProperties or something along those lines. 
        analyzeModel(resource, fsa, context)

        // Replace connections in this resources that are annotated with the 
        // "after" keyword by ones that go through a delay reactor. 
        resource.insertGeneratedDelays(this)
        
        
        // Collect a list of reactors defined in this resource and (non-main)
        // reactors defined in imported resources.
        instantiationGraph = new InstantiationGraph(resource, false)

        // Topologically sort the instantiations such that all of a reactor's
        // dependencies occur earlier in the sorted list or reactors.
        this.reactors = instantiationGraph.nodesInTopologicalOrder
        // Add to the known resources the resource that is the main file.
        this.resources.add(resource)
        // Add to the known resources all imported files.
        for (r : this.reactors) {
            this.resources.add(r.eResource)
        }
        // FIXME: This is just a proof of concept. What to do instead:
        // - use reportError
        // - report at node that represents import through which this resource
        // was (transitively) reached; we need an ImportGraph for this resource
        // reached.
        // Alternatively: only validate the individual reactors that are imported
        // This would allow importing a reactor from a file that has issues.
        // Probably not the best idea.
        for (r : this.resources) {
            if (r !== this.resource) {
                val issues = (r as XtextResource).resourceServiceProvider.
                    resourceValidator.validate(r, CheckMode.ALL, null)
                if (issues.size > 0) {
                    println('''Issues found in «r.URI.toFileString».''')
                    println(issues.join("\n"))
                    return
                }
            }
        }
        
        // First, produce any preamble code that the code generator needs
        // to produce before anything else goes into the code generated files.
        generatePreamble() // FIXME: Move this elsewhere.
    }
    
    /**
     * Return true if errors occurred in the last call to doGenerate().
     * This will return true if any of the reportError methods was called.
     * @return True if errors occurred.
     */
    def errorsOccurred() {
        return generatorErrorsOccurred;
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
            unorderedReactions = new LinkedHashSet<Reaction>()
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
    def createFederateRTI() {
        // Derive target filename from the .lf filename.
        var cFilename = filename + "_RTI.c"

        // Delete source previously produced by the LF compiler.
        var file = new File(directory + rtiSrcPath + File.separator + cFilename)
        if (file.exists) {
            file.delete
        }

        // Delete binary previously produced by the C compiler.
        file = new File(directory + rtiBinPath + File.separator + filename)
        if (file.exists) {
            file.delete
        }
        
        val rtiCode = new StringBuilder()
        pr(rtiCode, '''
            #ifdef NUMBER_OF_FEDERATES
            #undefine NUMBER_OF_FEDERATES
            #endif
            #define NUMBER_OF_FEDERATES «federates.length»
            #include "rti.c"
            int main(int argc, char* argv[]) {
        ''')
        indent(rtiCode)
        
        // Initialize the array of information that the RTI has about the
        // federates.
        // FIXME: No support below for some federates to be FAST and some REALTIME.
        pr(rtiCode, '''
            for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
                initialize_federate(i);
                «IF targetFast»
                    federates[i].mode = FAST;
                «ENDIF»
            }
        ''')
        // Initialize the arrays indicating connectivity to upstream and downstream federates.
        for(federate : federates) {
            if (!federate.dependsOn.keySet.isEmpty) {
                // Federate receives non-physical messages from other federates.
                // Initialize the upstream and upstream_delay arrays.
                val numUpstream = federate.dependsOn.keySet.size
                // Allocate memory for the arrays storing the connectivity information.
                pr(rtiCode, '''
                    federates[«federate.id»].upstream = (int*)malloc(sizeof(federate_t*) * «numUpstream»);
                    federates[«federate.id»].upstream_delay = (interval_t*)malloc(sizeof(interval_t*) * «numUpstream»);
                    federates[«federate.id»].num_upstream = «numUpstream»;
                ''')
                // Next, populate these arrays.
                // Find the minimum delay in the process.
                // FIXME: Zero delay is not really the same as a microstep delay.
                var count = 0;
                for (upstreamFederate : federate.dependsOn.keySet) {
                    pr(rtiCode, '''
                        federates[«federate.id»].upstream[«count»] = «upstreamFederate.id»;
                        federates[«federate.id»].upstream_delay[«count»] = 0LL;
                    ''')
                    // The minimum delay calculation needs to be made in the C code because it
                    // may depend on parameter values.
                    // FIXME: These would have to be top-level parameters, which don't really
                    // have any support yet. Ideally, they could be overridden on the command line.
                    // When that is done, they will need to be in scope here.
                    val delays = federate.dependsOn.get(upstreamFederate)
                    if (delays !== null) {
                        for (value : delays) {
                            pr(rtiCode, '''
                                if (federates[«federate.id»].upstream_delay[«count»] < «value.getRTITime») {
                                    federates[«federate.id»].upstream_delay[«count»] = «value.getRTITime»;
                                }
                            ''')
                        }
                    }
                    count++;
                }
            }
            // Next, set up the downstream array.
            if (!federate.sendsTo.keySet.isEmpty) {
                // Federate sends non-physical messages to other federates.
                // Initialize the downstream array.
                val numDownstream = federate.sendsTo.keySet.size
                // Allocate memory for the array.
                pr(rtiCode, '''
                    federates[«federate.id»].downstream = (int*)malloc(sizeof(federate_t*) * «numDownstream»);
                    federates[«federate.id»].num_downstream = «numDownstream»;
                ''')
                // Next, populate the array.
                // Find the minimum delay in the process.
                // FIXME: Zero delay is not really the same as a microstep delay.
                var count = 0;
                for (downstreamFederate : federate.sendsTo.keySet) {
                    pr(rtiCode, '''
                        federates[«federate.id»].downstream[«count»] = «downstreamFederate.id»;
                    ''')
                    count++;
                }
            }
        }
        
        // Start the RTI server before launching the federates because if it
        // fails, e.g. because the port is not available, then we don't want to
        // launch the federates.
        pr(rtiCode, '''
            int socket_descriptor = start_rti_server(«federationRTIProperties.get('port')»);
        ''')
        
        // Generate code that blocks until the federates resign.
        pr(rtiCode, "wait_for_federates(socket_descriptor);")
        
        unindent(rtiCode)
        pr(rtiCode, "}")
        
        var fOut = new FileOutputStream(
                new File(directory + rtiSrcPath + File.separator + cFilename));
        fOut.write(rtiCode.toString().getBytes())
        fOut.close()
    }
    
    /** Invoke the C compiler on the generated RTI 
     * 
     * The C RTI is used across targets. Thus we need to be able to compile 
     * it from GeneratorBase. 
     */
    def compileRTI() {
        var fileToCompile = filename + '_RTI'
        runCCompiler(directory, fileToCompile, false)
    }
    
    /** 
     * Run the C compiler.
     * 
     * This is required here in order to allow any target to compile the RTI.
     * 
     * @param directory the directory to run the compiler in
     * @param the source file to compile
     * param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     */
    def runCCompiler(String directory, String file, boolean doNotLinkIfNoMain) {
        val compile = compileCCommand(file, doNotLinkIfNoMain)
        if (compile === null) {
            return
        }

        val stderr = new ByteArrayOutputStream()
        val returnCode = compile.executeCommand(stderr)

        if (returnCode != 0 && mode !== Mode.INTEGRATED) {
            reportError('''«targetCompiler»r returns error code «returnCode»''')
        }
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (stderr.toString.length > 0 && mode === Mode.INTEGRATED) {
            reportCommandErrors(stderr.toString())
        }
    }
    
    /** Return a command to compile the specified C file.
     * 
     * This produces a C specific compile command. Since this command is
     * used across targets to build the RTI, it needs to be available in
     * GeneratorBase.
     * 
     *  @param fileToCompile The C filename without the .c extension.
     *  @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     */
    protected def compileCCommand(String fileToCompile, boolean doNotLinkIfNoMain) {
        val cFilename = getTargetFileName(fileToCompile);            
        val relativeSrcFilename = "src-gen" + File.separator + cFilename;
        val relativeBinFilename = "bin" + File.separator + fileToCompile;

        var compileArgs = newArrayList
        val flags = targetCompilerFlags.split(' ')
        compileArgs.addAll(flags)
        compileArgs.add(relativeSrcFilename)
        compileArgs.addAll(compileAdditionalSources)
        compileArgs.addAll(compileLibraries)
        
        // Only set the output file name if it hasn't already been set
        // using a target property or Args line flag.
        if (compileArgs.forall[it.trim != "-o"]) {
            compileArgs.addAll("-o", relativeBinFilename)
        }

        // If threaded computation is requested, add a -pthread option.
        if (targetThreads !== 0) {
            compileArgs.add("-pthread")
        }
        // If there is no main reactor, then use the -c flag to prevent linking from occurring.
        // FIXME: we could add a `-c` flag to `lfc` to make this explicit in stand-alone mode.
        // Then again, I think this only makes sense when we can do linking.
        // In any case, a warning is helpful to draw attention to the fact that no binary was produced.
        if (doNotLinkIfNoMain && main === null) {
            compileArgs.add("-c") // FIXME: revisit
            if (mode === Mode.STANDALONE) {
                reportError("ERROR: Did not output executable; no main reactor found.")
            }
        }
        return createCommand(targetCompiler, compileArgs)
    }

    ////////////////////////////////////////////
    //// Protected methods.
    
    /** Produces the filename including the target-specific extension */
    protected def getTargetFileName(String fileName)
    {
    	return fileName + ".c";
    }

    /**
     * Return a set of targets that are acceptable to this generator.
     * Imported files that are Lingua Franca files must specify targets in this
     * set or an error message will be reported and the import will be ignored.
     * The returned set is a set of case-insensitive strings specifying target
     * names. If any target is acceptable, return null.
     */
    protected abstract def Set<String> acceptableTargets()
    
    /**
     * Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
    }
    
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
     * Run a given command and record its output.
     * 
     * @param cmd the command to be executed
     * @param errStream a stream object to forward the commands error messages to
     * @param outStrram a stream object to forward the commands output messages to
     * @return the commands return code
     */
    protected def executeCommand(ProcessBuilder cmd, OutputStream errStream, OutputStream outStream) {
        println('''--- In directory: «cmd.directory.absolutePath»''')
        println('''--- Executing command: «cmd.command.join(" ")»''')

        var List<OutputStream> outStreams = newArrayList
        var List<OutputStream> errStreams = newArrayList
        outStreams.add(System.out)
        errStreams.add(System.err)
        if (outStream !== null) { outStreams.add(outStream) } 
        if (errStream !== null) { errStreams.add(errStream) }

        // Execute the command. Write output to the System output,
        // but also keep copies in outStream and errStream
        return cmd.runSubprocess(outStreams, errStreams)
    }

    /**
     * Run a given command and record its error messages.
     * 
     * @param cmd the command to be executed
     * @param errStream a stream object to forward the commands error messages to
     * @return the commands return code
     */
    protected def executeCommand(ProcessBuilder cmd) {
        return cmd.executeCommand(null, null)
    }
    
    /**
     * Run a given command.
     * 
     * @param cmd the command to be executed
     * @return the commands return code
     */
    protected def executeCommand(ProcessBuilder cmd, OutputStream errStream) {
        return cmd.executeCommand(errStream, null)
    }
    
    /**
     * Creates a ProcessBuilder for a given command.
     * 
     * This method makes sure that the given command is executable,
     * It first tries to find the command with 'which cmake'. If that
     * fails, it tries again with bash. In case this fails again,
     * it returns null. Otherwise, a correctly constructed ProcessBuilder
     * object is returned. 
     * 
     * A bit more context:
     * If the command cannot be found directly, then a second attempt is made using a
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
     * @param cmd The command to be executed
     * @return A ProcessBuilder object if the command was found or null otherwise.
     */
    protected def createCommand(String cmd) {
        return createCommand(cmd, #[])
    }
    
    /**
     * Creates a ProcessBuilder for a given command and its arguments.
     * 
     * This method ensures that the given command is executable,
     * It first tries to find the command with 'which cmake'. If that
     * fails, it tries again with bash. n case this fails again,
     * it returns null. Otherwise, a correctly constructed ProcessBuilder
     * object is returned. 
     * 
     * @param cmd The command to be executed
     * @param args A list of arguments for the given command
     * @return A ProcessBuilder object if the command was found or null otherwise.
     */
    protected def createCommand(String cmd, List<String> args) {
        // Make sure the command is found in the PATH.
        print('''--- Looking for command «cmd» ... ''')
        val whichBuilder = new ProcessBuilder(#["which", cmd])
        val whichReturn = whichBuilder.start().waitFor()
        if (whichReturn == 0) {
            println("SUCCESS")
            return new ProcessBuilder(#[cmd] + args)
        }
        println("FAILED")
        // Try running with bash.
        // The --login option forces bash to look for and load the first of
        // ~/.bash_profile, ~/.bash_login, and ~/.bashrc that it finds.
        print('''--- Trying again with bash ... ''')
        val bashCommand = #["bash", "--login", "-c", '''which «cmd»''']
        val bashBuilder = new ProcessBuilder(bashCommand)
        val bashReturn = bashBuilder.start().waitFor()
        if (bashReturn == 0) {
            println("SUCCESS")
            val builder = new ProcessBuilder(#["bash", "--login", "-c", '''«cmd» «args.join(" ")»'''])
            builder.directory(new File(directory))
            return builder
        }
        println("FAILED")
        reportError("The command " + cmd + " could not be found.\n" +
                    "Make sure that your PATH variable includes the directory where " + cmd + " is installed.\n" +
                    "You can set PATH in ~/.bash_profile on Linux or Mac.")
        return null as ProcessBuilder
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
    protected def void generatePreamble() {
        prComment("Code generated by the Lingua Franca compiler from file:")
        prComment(sourceFile)
        val models = new LinkedHashSet<Model>
        for (r : this.reactors ?: emptyList) {
            models.add(r.toDefinition.eContainer as Model)
        }
        // FIXME: preambles should also be printed in correct order. (see topological sort in doGenerate
        for (m : models) {
            for (p : m.preambles) {
                pr(p.code.toText)
            }
        }
    }

    /**
     * Get the code produced so far.
     * @return The code produced so far as a String.
     */
    protected def getCode() {
        code.toString()
    }
    
    /**
     * Depending on the target language, parts of code might be
     * generated that are only useful for the compiler and can
     * make the generated code less readable for the end user.
     * @return The cleaned up code so far as a String
     */
     protected def getReadableCode() {
     	code.toString()
     }
        
    /**
     * Increase the indentation of the output code produced.
     */
    protected def indent() {
        indent(code)
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
                // No line designator.
                if (message.length > 0) {
                    message.append("\n")
                } else {
                    if (line.toLowerCase.contains('warning:')) {
                        severity = IMarker.SEVERITY_WARNING
                    }
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
     * Given the right-hand side of a target property, return a list with all
     * the files that the property lists.
     * 
     * Arrays are traversed, so files are collected recursively. If elements
     * are found that do not denote files or denote files that cannot be found,
     * warnings are reported.
     * @param value The right-hand side of a target property.
     */
    def List<File> collectFiles(Element value) {
        var allFound = true;
        val files = newLinkedList
        var filename = ""
        if (value.array !== null) {
            for (element : value.array.elements) {
                files.addAll(element.collectFiles)
            }
            return files
        } else if (value.literal !== null) {
            filename = value.literal.withoutQuotes
        } else if (value.id !== "") {
            filename = value.id
        } else {
            this.
                reportWarning(
                    value, '''Expected a string, path, or array but found something else.''')
        }
        val file = filename.findFile
        if (file !== null) {
            files.add(file)
        } else {
            // Warn that file hasn't been found.
            allFound = false
            this.reportWarning(value, '''Could not find «filename».''')
        }

        if (!allFound && this.classpath.isNullOrEmpty) {
            // Report that LF_CLASSPATH hasn't been set 
            // and something hasn't been found.
            this.reportWarning(value,
                "LF_CLASSPATH environment variable is not set.")
        }

        return files
    }

    /**
     * Search for a given file name in the current directory.
     * If not found, search in directories in LF_CLASSPATH.
     * The first file found will be returned.
     * 
     * @param fileName The file name or relative path + file name
     * in plain string format
     * @return A Java file or null if not found
     */
     def File findFile(String fileName) {

        var File foundFile;

        // Check in local directory
        foundFile = new File(this.directory + '/' + fileName);
        if (foundFile.exists && foundFile.isFile) {
            return foundFile;
        }

        // Check in LF_CLASSPATH
        // Load all the resources in LF_CLASSPATH if it is set.
        this.classpath = System.getenv("LF_CLASSPATH");
        if (this.classpath !== null) {
            var String[] paths = this.classpath.split(
                System.getProperty("path.separator"));
            for (String path : paths) {
                foundFile = new File(path + '/' + fileName);
                if (foundFile.exists && foundFile.isFile) {
                    return foundFile;
                }
            }
        }
        // Not found.
        return null;
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
    
    //// Utility functions supporting multiports.
        
    /**
     * If the argument is a multiport, return a list of strings
     * describing the width of the port, and otherwise, return null.
     * If the list is empty, then the width is variable (specified
     * as '[]'). Otherwise, it is a list of integers and/or parameter
     * references obtained by getTargetReference().
     * @param variable The port.
     * @return The width specification for a multiport or null if it is
     *  not a multiport.
     */
    protected def List<String> multiportWidthSpec(Variable variable) {
        var result = null as LinkedList<String>
        if (variable instanceof Port) {
            if (variable.widthSpec !== null) {
                result = new LinkedList<String>()
                if (!variable.widthSpec.ofVariableLength) {
                    for (term : variable.widthSpec.terms) {
                        if (term.parameter !== null) {
                            result.add(getTargetReference(term.parameter))
                        } else {
                            result.add('' + term.width)
                        }
                    }
                }
            }
        }
        return result
    }
            
    /**
     * If the argument is a multiport, then return a string that
     * gives the width as an expression, and otherwise, return null.
     * The string will be empty if the width is variable (specified
     * as '[]'). Otherwise, if is a single term or a sum of terms
     * (separated by '+'), where each term is either an integer
     * or a parameter reference in the target language.
     */
    protected def String multiportWidthExpression(Variable variable) {
        val spec = multiportWidthSpec(variable)
        if (spec !== null) {
            return spec.join(' + ')
        }
        return null
    }
    
    /**
     * Return true if the specified port is a multiport.
     * @param port The port.
     * @return True if the port is a multiport.
     */
    protected def boolean isMultiport(Port port) {
        port.widthSpec !== null
    }
    
    ////////////////////////////////////////////////////
    //// Private functions
    
    /**
     * Get textual representation of a time in the target language.
     * This is a separate function from 
     * getTargetTime to avoid producing invalid RTI
     * code for targets that override timeInTargetLanguage
     * to return a C-incompatible time type.
     *
     * @param v A time AST node
     * @return An RTI-compatible (ie. C target) time string
     */
    protected def getRTITime(Value v) {  
        var TimeValue time 
        if (v.time !== null) {
            time = new TimeValue(v.time.interval, v.time.unit)
        } else if (v.isZero) {
            time = new TimeValue(0, TimeUnit.NONE)
        } else {
            return v.toText
        }
        if (time.unit != TimeUnit.NONE) {
            return time.unit.name() + '(' + time.time + ')'
        } else {
            return time.time.toString()
        }
    }
    
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
        val mainDefn = this.mainDef?.reactorClass.toDefinition
        
        if (this.mainDef === null || !mainDefn.isFederated) {
            // Ensure federates is never empty.
            var federateInstance = new FederateInstance(null, 0, this)
            federates.add(federateInstance)
            federateByName.put("", federateInstance)
            federateByID.put(0, federateInstance)
        } else {            
            if (mainDefn.host !== null) {
                // Get the host information, if specified.
                // If not specified, this defaults to 'localhost'
                if (mainDefn.host.addr !== null) {
                    federationRTIProperties.put('host', mainDefn.host.addr)                
                }
                // Get the port information, if specified.
                // If not specified, this defaults to 14045
                if (mainDefn.host.port !== 0) {
                    federationRTIProperties.put('port', mainDefn.host.port)                
                }
                // Get the user information, if specified.
                if (mainDefn.host.user !== null) {
                    federationRTIProperties.put('user', mainDefn.host.user)                
                }
                // Get the directory information, if specified.
                /* FIXME
                if (mainDef.reactorClass.host.dir !== null) {
                    federationRTIProperties.put('dir', mainDef.reactorClass.host.dir)                
                }
                */
            }
            
            // Create a FederateInstance for each top-level reactor.
            for (instantiation : mainDefn.allInstantiations) {
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
                    federateByReactor = new LinkedHashMap<String, FederateInstance>()
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
            for (connection : mainDefn.connections) {
                // FIXME: Connections between federates do not support parallel connections.
                if (connection.leftPorts.length !== 1 || connection.rightPorts.length !== 1) {
                    val message = "FIXME: Parallel connections between federates are not supported yet."
                    reportError(connection, message)
                }
                var leftFederate = federateByReactor.get(connection.leftPorts.get(0).container.name)
                var rightFederate = federateByReactor.get(connection.rightPorts.get(0).container.name)
                if (leftFederate !== rightFederate) {
                    // Connection spans federates.
                    // First, update the dependencies in the FederateInstances.
                    // Exclude physical connections because these do not create real dependencies.
                    if (leftFederate !== rightFederate && !connection.physical) {
                        var dependsOn = rightFederate.dependsOn.get(leftFederate)
                        if (dependsOn === null) {
                            dependsOn = new LinkedHashSet<Value>()
                            rightFederate.dependsOn.put(leftFederate, dependsOn)
                        }
                        if (connection.delay !== null) {
                            dependsOn.add(connection.delay)
                        }
                        var sendsTo = leftFederate.sendsTo.get(rightFederate)
                        if (sendsTo === null) {
                            sendsTo = new LinkedHashSet<Value>()
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
                mainDefn.connections.remove(connection)
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
    private def runSubprocess(ProcessBuilder processBuilder,
                              List<OutputStream> outStream,
                              List<OutputStream> errStream) {
        val process = processBuilder.start()

        var outThread = new Thread([|
                var buffer = newByteArrayOfSize(64)
                var len = process.getInputStream().read(buffer)
                while(len != -1) {
                    for (os : outStream) {
                        os.write(buffer, 0, len)
                    }
                    len = process.getInputStream().read(buffer)
                }
            ])
        outThread.start()

        var errThread = new Thread([|
                var buffer = newByteArrayOfSize(64)
                var len = process.getErrorStream().read(buffer)
                while(len != -1) {
                    for (es : errStream) {
                        es.write(buffer, 0, len)
                    }
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
    
    /**
     * Return a string representing the specified type in the target language.
     * @param type The type.
     */ 
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
