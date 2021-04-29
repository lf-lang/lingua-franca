/* Generator base class for shared code between code generators. */

/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.generator

import java.io.ByteArrayOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStream
import java.net.URI
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import java.nio.file.StandardCopyOption
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
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.xtext.validation.CheckMode
import org.lflang.ASTUtils
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.MainConflictChecker
import org.lflang.Mode
import org.lflang.Target
import org.lflang.TargetConfig
import org.lflang.TargetProperty
import org.lflang.TargetProperty.CoordinationType
import org.lflang.TimeValue
import org.lflang.graph.InstantiationGraph
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.Code
import org.lflang.lf.Connection
import org.lflang.lf.Delay
import org.lflang.lf.Instantiation
import org.lflang.lf.LfFactory
import org.lflang.lf.Model
import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar
import org.lflang.lf.TargetDecl
import org.lflang.lf.Time
import org.lflang.lf.TimeUnit
import org.lflang.lf.Type
import org.lflang.lf.Value
import org.lflang.lf.VarRef
import org.lflang.lf.Variable
import org.lflang.validation.AbstractLFValidator

import static extension org.lflang.ASTUtils.*

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
abstract class GeneratorBase extends AbstractLFValidator {

    ////////////////////////////////////////////
    //// Public fields.
    
    /**
     * Constant that specifies how to name generated delay reactors.
     */
    public static val GEN_DELAY_CLASS_NAME = "__GenDelay"
    
    ////////////////////////////////////////////
    //// Protected fields.
        
    /**
     * All code goes into this string buffer.
     */
    protected var code = new StringBuilder

    /**
     * The current target configuration.
     */
    protected var TargetConfig targetConfig = new TargetConfig()
    
    /**
     * The current file configuration. NOTE: not initialized until the
     * invocation of doGenerate, which calls setFileConfig.
     */
    protected var FileConfig fileConfig
    
    /**
     * {@link #Mode.STANDALONE Mode.STANDALONE} if the code generator is being
     * called from the command line, {@link #Mode.INTEGRATED Mode.INTEGRATED}
     * if it is being called from the Eclipse IDE, and 
     * {@link #Mode.UNDEFINED Mode.UNDEFINED} otherwise.
     */
    public var Mode mode = Mode.UNDEFINED

    /**
     * Collection of generated delay classes.
     */
    val delayClasses = new LinkedHashSet<Reactor>()
    
    /**
     * Set the fileConfig field to point to the specified resource using the specified
     * file-system access and context.
     * @param resource The resource (Eclipse-speak for a file).
     * @param fsa The Xtext abstraction for the file system.
     * @param context The generator context (whatever that is).
     */
    def void setFileConfig(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
        this.fileConfig = new FileConfig(resource, fsa, context);
        this.topLevelName = fileConfig.name
    }

    /**
     * Indicator of whether generator errors occurred.
     * This is set to true by the report() method and returned by the
     * errorsOccurred() method.
     */
    protected var generatorErrorsOccurred = false

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
     * A list of Reactor definitions in the main resource, including non-main 
     * reactors defined in imported resources. These are ordered in the list in
     * such a way that each reactor is preceded by any reactor that it instantiates
     * using a command like `foo = new Foo();`
     */
    protected var List<Reactor> reactors = newLinkedList
    
    /**
     * The set of resources referenced reactor classes reside in.
     */
    protected var Set<Resource> resources = newLinkedHashSet
    
    /**
     * Graph that tracks dependencies between instantiations. 
     * This is a graph where each node is a Reactor (not a ReactorInstance)
     * and an arc from Reactor A to Reactor B means that B contains an instance of A, constructed with a statement
     * like `a = new A();`  After creating the graph, 
     * sort the reactors in topological order and assign them to the reactors class variable. 
     * Hence, after this method returns, `this.reactors` will be a list of Reactors such that any
     * reactor is preceded in the list by reactors that it instantiates.
     */
    protected var InstantiationGraph instantiationGraph

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
     * Indicates whether or not the current Lingua Franca program
     * contains a federation.
     */
    protected var boolean isFederated = false

    // //////////////////////////////////////////
    // // Target properties, if they are included.
    /**
     * A list of federate instances or a list with a single empty string
     * if there are no federates specified. FIXME: Why put a single empty string there? It should be just empty...
     */
    protected var List<FederateInstance> federates = new LinkedList<FederateInstance>

    /**
     * A map from federate IDs to federate instances.
     */
    protected var Map<Integer, FederateInstance> federateByID = new LinkedHashMap<Integer, FederateInstance>()

    /**
     * A map from instantiations to the federate instances for that instantiation.
     * If the instantiation has a width, there may be more than one federate instance.
     */
    protected var Map<Instantiation, List<FederateInstance>> federatesByInstantiation

    /**
     * The federation RTI properties, which defaults to 'localhost: 15045'.
     */
    protected val federationRTIProperties = newLinkedHashMap(
        'host' -> 'localhost',
        'port' -> 0 // Indicator to use the default port, typically 15045.
    )

    /**
     * Contents of $LF_CLASSPATH, if it was set.
     */
    protected String classpathLF

    /**
     * The index available to user-generated reaction that delineates the index
     * of the reactor in a bank of reactors. The value must be set to zero
     * in generated code for reactors that are not in a bank
     */
    protected String targetBankIndex = "bank_index"

    /**
     * The type of the bank index, which must be an integer in the target language
     */
    protected String targetBankIndexType = "int"

    /**
     * The name of the top-level reactor.
     */
    protected var String topLevelName; // FIXME: remove and use fileConfig.name instead

    // //////////////////////////////////////////
    // // Private fields.
    /**
     * Map from builder to its current indentation.
     */
    var indentation = new LinkedHashMap<StringBuilder, String>()

    /**
     * Defines the execution environment that is used to execute binaries.
     * 
     * A given command may be directly executable on the host (NATIVE) 
     * or it may need to be executed within a bash shell (BASH). 
     * On Unix-like machines, this is typically determined by the PATH variable
     * which could be different in these two environments.
     */
    enum ExecutionEnvironment {
        NATIVE,
        BASH
    }

    // //////////////////////////////////////////
    // // Code generation functions to override for a concrete code generator.

    /**
     * Store the given reactor in the collection of generated delay classes
     * and insert it in the AST under the top-level reactors node.
     */
    def void addDelayClass(Reactor generatedDelay) {
        // Record this class, so it can be reused.
        this.delayClasses.add(generatedDelay)
        // And hook it into the AST.
        (fileConfig.resource.allContents.findFirst[it|it instanceof Model] as Model).reactors.add(generatedDelay)
    }

    /**
     * Return the generated delay reactor that corresponds to the given class
     * name if it had been created already, `null` otherwise.
     */
    def Reactor findDelayClass(String className) {
        return this.delayClasses.findFirst[it|it.name.equals(className)]
    }

    /**
     * 
     */
    def void setTargetConfig(IGeneratorContext context) {
        // If there are any physical actions, ensure the threaded engine is used.
        for (action : fileConfig.resource.allContents.toIterable.filter(Action)) {
            if (action.origin == ActionOrigin.PHYSICAL) {
                targetConfig.threads = 1
            }
        }

        val target = fileConfig.resource.findTarget
        if (target.config !== null) {
            // Update the configuration according to the set target properties.
            TargetProperty.update(this.targetConfig, target.config.pairs ?: emptyList)
        }

        // Override target properties if specified as command line arguments.
        if (context instanceof StandaloneContext) {
            if (context.args.containsKey("no-compile")) {
                targetConfig.noCompile = true
            }
            if (context.args.containsKey("threads")) {
                targetConfig.threads = Integer.parseInt(context.args.getProperty("threads"))
            }
            if (context.args.containsKey("target-compiler")) {
                targetConfig.compiler = context.args.getProperty("target-compiler")
            }
            if (context.args.containsKey("target-flags")) {
                targetConfig.compilerFlags.clear()
                if (!context.args.getProperty("target-flags").isEmpty) {
                    targetConfig.compilerFlags.addAll(context.args.getProperty("target-flags").split(' '))
                }
            }
            if (context.args.containsKey("runtime-version")) {
                targetConfig.runtimeVersion = context.args.getProperty("runtime-version")
            }
            if (context.args.containsKey("external-runtime-path")) {
                targetConfig.externalRuntimePath = context.args.getProperty("external-runtime-path")
            }
        }
    }

    /**
     * If there is a main or federated reactor, then create a synthetic Instantiation
     * for that top-level reactor and set the field mainDef to refer to it.
     */
    def createMainInstance() {
        // Find the main reactor and create an AST node for its instantiation.
        for (reactor : fileConfig.resource.allContents.toIterable.filter(Reactor)) {
            if (reactor.isMain || reactor.isFederated) {
                // Creating an definition for the main reactor because there isn't one.
                this.mainDef = LfFactory.eINSTANCE.createInstantiation()
                this.mainDef.setName(reactor.name)
                this.mainDef.setReactorClass(reactor)
            }
        }
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
    def void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
        
        setFileConfig(resource, fsa, context)
        
        setMode()
        
        printInfo()
        
        // Clear any markers that may have been created by a previous build.
        // Markers mark problems in the Eclipse IDE when running in integrated mode.
        clearMarkers()
        
        ASTUtils.setMainName(fileConfig.resource, fileConfig.name)
        
        createMainInstance()

        // Check if there are any conflicting main reactors elsewhere in the package.
        if (mainDef !== null) {
            for (String conflict : new MainConflictChecker(fileConfig).conflicts) {
                reportError(this.mainDef.reactorClass, "Conflicting main reactor in " + conflict);
            }
        }
        
        setTargetConfig(context)
        
        // If federates are specified in the target, create a mapping
        // from Instantiations in the main reactor to federate names.
        // Also create a list of federate names or a list with a single
        // empty name if there are no federates specified.
        // This must be done before desugaring delays below.
        analyzeFederates()
        
        // Process target files. Copy each of them into the src-gen dir.
        copyUserFiles()

        // Collect reactors and create an instantiation graph. These are needed to figure out which resources we need
        // to validate, which happens in setResources().
        setReactorsAndInstantiationGraph()

        // Collect the reactors defined in this resource (i.e., file in Eclipse speak) and (non-main)
        // reactors defined in imported resources.
        setResources(context)
        
        // Reroute connections that have delays associated with them via generated delay reactors.
        transformDelays()

        // Invoke this function a second time because transformations may have introduced new reactors!
        setReactorsAndInstantiationGraph()

        // First, produce any preamble code that the code generator needs
        // to produce before anything else goes into the code generated files.
        generatePreamble() // FIXME: Move this elsewhere. See awkwardness with CppGenerator because it will not even
        // use the result.
    }

    /**
     * Create a new instantiation graph. This is a graph where each node is a Reactor (not a ReactorInstance)
     * and an arc from Reactor A to Reactor B means that B contains an instance of A, constructed with a statement
     * like `a = new A();`  After creating the graph, 
     * sort the reactors in topological order and assign them to the reactors class variable. 
     * Hence, after this method returns, `this.reactors` will be a list of Reactors such that any
     * reactor is preceded in the list by reactors that it instantiates.
     */
    protected def setReactorsAndInstantiationGraph() {
        // Build the instantiation graph . 
        this.instantiationGraph = new InstantiationGraph(fileConfig.resource, false)

        // Topologically sort the reactors such that all of a reactor's instantiation dependencies occur earlier in 
        // the sorted list of reactors. This helps the code generator output code in the correct order.
        // For example if `reactor Foo {bar = new Bar()}` then the definition of `Bar` has to be generated before
        // the definition of `Foo`.
        this.reactors = this.instantiationGraph.nodesInTopologicalOrder

        // If there is no main reactor, then make sure the reactors list includes
        // even reactors that are not instantiated anywhere.
        if (mainDef === null) {
            for (r : fileConfig.resource.allContents.toIterable.filter(Reactor)) {
                if (!this.reactors.contains(r)) {
                    this.reactors.add(r);
                }
            }
        }
    }

    /**
     * For each involved resource, replace connections with delays with generated delay reactors.
     */
    protected def transformDelays() {
         for (r : this.resources) {
             r.insertGeneratedDelays(this)
        }
    }

    /**
     * Update the class variable that lists all the involved resources. Also report validation problems of imported 
     * resources at the import statements through those failing resources are reached.
     * 
     * @param context The context providing the cancel indicator used by the validator.
     */
    protected def setResources(IGeneratorContext context) {
        val validator = (this.fileConfig.resource as XtextResource).resourceServiceProvider.resourceValidator
        if (mainDef !== null) {
            reactors.add(mainDef.reactorClass as Reactor);
        }
        // Iterate over reactors and mark their resources as tainted if they import resources that are either marked
        // as tainted or fail to validate.
        val tainted = newHashSet
        for (r : this.reactors) {
            val res = r.eResource
            if (!this.resources.contains(res)) {
                if (res !== this.fileConfig.resource) {
                    if (tainted.contains(res) ||
                        (validator.validate(res, CheckMode.ALL, context.cancelIndicator)).size > 0) {
                        for (inst : this.instantiationGraph.getDownstreamAdjacentNodes(r)) {
                            for (imp : (inst.eContainer as Model).imports) {
                                for (decl : imp.reactorClasses) {
                                    if (decl.reactorClass.eResource === res) {
                                        reportError(imp, '''Unresolved compilation issues in '«imp.importURI»'.''')
                                        tainted.add(decl.eResource)
                                    }
                                }
                            }
                        }
                    }
                }
                this.resources.add(res)
            }
        }
    }

    /**
     * Copy all files listed in the target property `files` into the
     * specified directory.
     */
    protected def copyUserFiles() {
        // Make sure the target directory exists.
        val targetDir = this.fileConfig.getSrcGenPath.toFile
        targetDir.mkdirs

        for (filename : targetConfig.fileNames) {
            val file = FileConfig.findFile(filename, this.fileConfig.srcFile.parent)
            if (file !== null) {
                val target = new File(targetDir, file.name)
                if (target.exists) {
                    target.delete
                }
                Files.copy(file.toPath, target.toPath)
                targetConfig.filesNamesWithoutPath.add(file.name);
            } else {
                // Try to copy the file as a resource.
                // If this is missing, it should have been previously reported as an error.
                try {
                    var filenameWithoutPath = filename
                    val lastSeparator = filename.lastIndexOf(File.separator)
                    if (lastSeparator > 0) {
                        filenameWithoutPath = filename.substring(lastSeparator + 1) // FIXME: brittle. What if the file is in a subdirectory?
                    }
                    copyFileFromClassPath(filename, targetDir + File.separator + filenameWithoutPath)
                    targetConfig.filesNamesWithoutPath.add(filenameWithoutPath);
                } catch (IOException ex) {
                    // Ignore. Previously reported as a warning.
                    System.err.println('''WARNING: Failed to find file «filename».''')
                }
            }
        }
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
        var cFilename = fileConfig.name + "_RTI.c"

        // Delete source previously produced by the LF compiler.
        // 
        var file = fileConfig.RTISrcPath.resolve(cFilename).toFile
        if (file.exists) {
            file.delete
        }
        
        // Also make sure the directory exists.
        if (!file.parentFile.exists || !file.parentFile.isDirectory) {
            file.mkdirs
        }

        // Delete binary previously produced by the C compiler.
        file = fileConfig.RTIBinPath.resolve(fileConfig.name).toFile
        if (file.exists) {
            file.delete
        }

        val rtiCode = new StringBuilder()
        pr(rtiCode, '''
            #ifdef NUMBER_OF_FEDERATES
            #undefine NUMBER_OF_FEDERATES
            #endif
            #define NUMBER_OF_FEDERATES «federates.size»
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
                «IF targetConfig.fastMode»
                    federates[i].mode = FAST;
                «ENDIF»
            }
        ''')
        // Initialize the arrays indicating connectivity to upstream and downstream federates.
        for (federate : federates) {
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
                        for (delay : delays) {
                            pr(rtiCode, '''
                                if (federates[«federate.id»].upstream_delay[«count»] < «delay.getRTITime») {
                                    federates[«federate.id»].upstream_delay[«count»] = «delay.getRTITime»;
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
        // Also generate code that blocks until the federates resign.
        pr(rtiCode, '''
            int socket_descriptor = start_rti_server(«federationRTIProperties.get('port')»);
            wait_for_federates(socket_descriptor);
        ''')

        unindent(rtiCode)
        pr(rtiCode, "}")

        var fOut = new FileOutputStream(fileConfig.RTISrcPath.resolve(cFilename).toFile);
        fOut.write(rtiCode.toString().getBytes())
        fOut.close()
    }

    /** 
     * Invoke the C compiler on the generated RTI 
     * The C RTI is used across targets. Thus we need to be able to compile 
     * it from GeneratorBase. 
     */
    def compileRTI() {
        var fileToCompile = fileConfig.name + '_RTI'
        runCCompiler(fileToCompile, false)
    }

    /** 
     * Run the C compiler.
     * 
     * This is required here in order to allow any target to compile the RTI.
     * 
     * @param file The source file to compile without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     * 
     * @return true if compilation succeeds, false otherwise. 
     */
    def runCCompiler(String file, boolean doNotLinkIfNoMain) {
        val compile = compileCCommand(file, doNotLinkIfNoMain)
        if (compile === null) {
            return false
        }

        val stderr = new ByteArrayOutputStream()
        val returnCode = compile.executeCommand(stderr)

        if (returnCode != 0 && mode !== Mode.INTEGRATED) {
            reportError('''«targetConfig.compiler» returns error code «returnCode»''')
        }
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (stderr.toString.length > 0 && mode === Mode.INTEGRATED) {
            reportCommandErrors(stderr.toString())
        }
        return (returnCode == 0)
    }

    /**
     * Run the custom build command specified with the "build" parameter.
     * This command is executed in the same directory as the source file.
     */
    protected def runBuildCommand() {
        var commands = newLinkedList
        for (cmd : targetConfig.buildCommands) {
            val tokens = newArrayList(cmd.split("\\s+"))
            if (tokens.size > 1) {
                val buildCommand = createCommand(tokens.head, tokens.tail.toList, this.fileConfig.srcPath)
                // If the build command could not be found, abort.
                // An error has already been reported in createCommand.
                if (buildCommand === null) {
                    return
                }
                commands.add(buildCommand)
            }
        }

        for (cmd : commands) {
            val stderr = new ByteArrayOutputStream()
            val returnCode = cmd.executeCommand(stderr)

            if (returnCode != 0 && mode !== Mode.INTEGRATED) {
                reportError('''Build command "«targetConfig.buildCommands»" returns error code «returnCode»''')
                return
            }
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (stderr.toString.length > 0 && mode === Mode.INTEGRATED) {
                reportCommandErrors(stderr.toString())
                return
            }
        }
    }
    
    /**
     * Return a command to compile the specified C file.
     * This produces a C specific compile command. Since this command is
     * used across targets to build the RTI, it needs to be available in
     * GeneratorBase.
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     */
    protected def compileCCommand(String fileToCompile, boolean doNotLinkIfNoMain) {
        val env = findCommandEnv(targetConfig.compiler)
        
        val cFilename = getTargetFileName(fileToCompile);

        var relativeSrcPath = fileConfig.outPath.relativize(
            fileConfig.getSrcGenPath.resolve(Paths.get(cFilename)))
        var relativeBinPath = fileConfig.outPath.relativize(
            fileConfig.binPath.resolve(Paths.get(fileToCompile)))

        // NOTE: we assume that any C compiler takes Unix paths as arguments.
        var relSrcPathString = FileConfig.toUnixString(relativeSrcPath)
        var relBinPathString = FileConfig.toUnixString(relativeBinPath)
        
        // If there is no main reactor, then generate a .o file not an executable.
        if (mainDef === null) {
            relBinPathString += ".o";
        }
        
        var compileArgs = newArrayList
        compileArgs.add(relSrcPathString)
        compileArgs.addAll(targetConfig.compileAdditionalSources)
        compileArgs.addAll(targetConfig.compileLibraries)

        // Only set the output file name if it hasn't already been set
        // using a target property or Args line flag.
        if (compileArgs.forall[it.trim != "-o"]) {
            compileArgs.addAll("-o", relBinPathString)
        }

        // If threaded computation is requested, add a -pthread option.

        if (targetConfig.threads !== 0 || targetConfig.tracing !== null) {
            compileArgs.add("-pthread")
            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            compileArgs.add('''-DNUMBER_OF_WORKERS=«targetConfig.threads»''')
        }
        // Finally add the compiler flags in target parameters (if any)
        if (!targetConfig.compilerFlags.isEmpty()) {
            compileArgs.addAll(targetConfig.compilerFlags)
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
        return createCommand(targetConfig.compiler,compileArgs, fileConfig.outPath, env)
    }

    // //////////////////////////////////////////
    // // Protected methods.
    /** Produces the filename including the target-specific extension */
    protected def getTargetFileName(String fileName) {
        return fileName + ".c"; // FIXME: Does not belong in the base class.
    }

    /**
     * Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
    }
    
    /**
     * Get the specified file as an Eclipse IResource or, if it is not found, then
     * return the iResource for the main file.
     * For some inexplicable reason, Eclipse uses a mysterious parallel to the file
     * system, and when running in INTEGRATED mode, for some things, you cannot access
     * files by referring to their file system location. Instead, you have to refer
     * to them relative the workspace root. This is required, for example, when marking
     * the file with errors or warnings or when deleting those marks. 
     * 
     * @param uri A java.net.uri of the form "file://path".
     */
    protected def getEclipseResource(URI uri) {
        var resource = iResource // Default resource.
        // For some peculiar reason known only to Eclipse developers,
        // the resource cannot be used directly but has to be converted
        // a resource relative to the workspace root.
        val workspaceRoot = ResourcesPlugin.getWorkspace().getRoot();
        // The following uses a java.net.URI, which,
        // pathetically, cannot be distinguished in xtend from a org.eclipse.emf.common.util.URI.
        if (uri !== null) {
             // Pathetically, Eclipse requires a java.net.uri, not a org.eclipse.emf.common.util.URI.
             val files = workspaceRoot.findFilesForLocationURI(uri);
             if (files !== null && files.length > 0 && files.get(0) !== null) {
                 resource = files.get(0)
             }
        }
        return resource;
    }

    /**
     * Clear markers in the IDE if running in integrated mode.
     * This has the side effect of setting the iResource variable to point to
     * the IFile for the Lingua Franca program. 
     * Also reset the flag indicating that generator errors occurred.
     */
    protected def clearMarkers() {
        if (mode == Mode.INTEGRATED) {
            try {
                val resource = getEclipseResource(fileConfig.srcFile.toURI());
                // First argument can be null to delete all markers.
                // But will that delete xtext markers too?
                resource.deleteMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE);
            } catch (Exception e) {
                // Ignore, but print a warning.
                println("Warning: Deleting markers in the IDE failed: " + e)
            }
        }
        generatorErrorsOccurred = false
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
        println('''--- Current working directory: «cmd.directory.toString()»''')
        println('''--- Executing command: «cmd.command.join(" ")»''')

        var List<OutputStream> outStreams = newArrayList
        var List<OutputStream> errStreams = newArrayList
        outStreams.add(System.out)
        errStreams.add(System.err)
        if (outStream !== null) {
            outStreams.add(outStream)
        }
        if (errStream !== null) {
            errStreams.add(errStream)
        }

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
     * Create a ProcessBuilder for a given command.
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
        return createCommand(cmd, #[], fileConfig.outPath, findCommandEnv(cmd)) // FIXME: add argument to specify where to execute; there is no useful assumption that would work here
    }

    /** 
     * It tries to find the command with 'which <cmd>' (or 'where <cmd>' on Windows). 
     * If that fails, it tries again with bash. 
     * In case this fails again, raise an error.
     * 
     * Return ExecutionEnvironment.NATIVE 
     * if the specified command is directly executable on the current host
     * Returns ExecutionEnvironment.BASH 
     * if the command must be executed within a bash shell.
     * 
     * The latter occurs, for example, if the specified command 
     * is not in the native path but is in the path
     * specified by the user's bash configuration file.
     * If the specified command is not found in either the native environment 
     * nor the bash environment,
     * then this reports and error and returns null.
     * 
     * @param cmd The command to be find.
     * @return Returns an ExecutionEnvironment.
     */
    protected def findCommandEnv(String cmd) {
        // Make sure the command is found in the PATH.
        print('''--- Looking for command «cmd» ... ''')
        // Use 'where' on Windows, 'which' on other systems
        val which = System.getProperty("os.name").startsWith("Windows") ? "where" : "which"
        val whichBuilder = new ProcessBuilder(#[which, cmd])
        val whichReturn = whichBuilder.start().waitFor()
        if (whichReturn == 0) {
            println("SUCCESS")

            return ExecutionEnvironment.NATIVE
        }
        println("FAILED")
        // Try running with bash.
        // The --login option forces bash to look for and load the first of
        // ~/.bash_profile, ~/.bash_login, and ~/.bashrc that it finds.
        print('''--- Trying again with bash ... ''')
        val bashCommand = #["bash", "--login", "-c", '''which «cmd»''']
        val bashBuilder = new ProcessBuilder(bashCommand)
        val bashOut = new ByteArrayOutputStream()
        val bashReturn = bashBuilder.runSubprocess(#[bashOut], #[])
        if (bashReturn == 0) {
            println("SUCCESS")
            return ExecutionEnvironment.BASH
        }
        reportError(
            "The command " + cmd + " could not be found.\n" +
                "Make sure that your PATH variable includes the directory where " + cmd + " is installed.\n" +
                "You can set PATH in ~/.bash_profile on Linux or Mac.")
        return null as ExecutionEnvironment

    }

    /**
     * Creates a ProcessBuilder for a given command and its arguments.
     * 
     * This method returns correctly constructed ProcessBuilder object 
     * according to the Execution environment. Raise an error if the env is null.
     * 
     * @param cmd The command to be executed
     * @param args A list of arguments for the given command
     * @param dir the directory to change into before executing the command.
     * @param env is the type of the Execution Environment.
     * @return A ProcessBuilder object if the command was found or null otherwise.
     */
    protected def createCommand(String cmd, List<String> args, Path dir, ExecutionEnvironment env) {
        if (env == ExecutionEnvironment.NATIVE) {
            val builder = new ProcessBuilder(#[cmd] + args)
            builder.directory(dir.toFile)
            return builder
        } else if (env == ExecutionEnvironment.BASH) {

            val str_builder = new StringBuilder(cmd + " ")
            for (str : args) {
                str_builder.append(str + " ")
            }
            val bash_arg = str_builder.toString
            // val bash_arg = #[cmd + args]
            val newCmd = #["bash", "--login", "-c"]
            // use that command to build the process
            val builder = new ProcessBuilder(newCmd + #[bash_arg])
            builder.directory(dir.toFile)
            return builder
        }
        println("FAILED")
        reportError(
            "The command " + cmd + " could not be found.\n" +
                "Make sure that your PATH variable includes the directory where " + cmd + " is installed.\n" +
                "You can set PATH in ~/.bash_profile on Linux or Mac.")
        return null as ProcessBuilder
    }

    /**
     * Creates a ProcessBuilder for a given command and its arguments.
     * 
     * This method returns correctly constructed ProcessBuilder object 
     * according to the Execution environment. It finds the execution environment using findCommandEnv(). 
     * Raise an error if the env is null.
     * 
     * @param cmd The command to be executed
     * @param args A list of arguments for the given command
     * @return A ProcessBuilder object if the command was found or null otherwise.
     */
    protected def createCommand(String cmd, List<String> args, Path dir) {
        val env = findCommandEnv(cmd)
        return createCommand(cmd, args, dir, env)
    }

    /**
     * Return the target.
     */
    def findTarget(Resource resource) {
        var TargetDecl targetDecl
        for (t : resource.allContents.toIterable.filter(TargetDecl)) {
            if (targetDecl !== null) {
                throw new RuntimeException("There is more than one target!") // FIXME: check this in validator
            }
            targetDecl = t
        }
        if (targetDecl === null) {
            throw new RuntimeException("No target found!")
        }
        targetDecl
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
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
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
        int receivingBankIndex,
        int receivingChannelIndex,
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
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @throws UnsupportedOperationException If the target does not support this operation.
     */
    def String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        int sendingBankIndex,
        int sendingChannelIndex,
        FederateInstance receivingFed,
        InferredType type,
        boolean isPhysical,
        Delay delay
    ) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.")
    }

    /**
     * Generate any preamble code that appears in the code generated
     * file before anything else.
     */
    protected def void generatePreamble() {
        prComment("Code generated by the Lingua Franca compiler from:")
        prComment("file:/" +FileConfig.toUnixString(fileConfig.srcFile.toPath))
        val models = new LinkedHashSet<Model>

        for (r : this.reactors ?: emptyList) {
            // The following assumes all reactors have a container.
            // This means that generated reactors **have** to be
            // added to a resource; not doing so will result in a NPE.
            models.add(r.toDefinition.eContainer as Model)
        }
        // Add the main reactor if it is defined
        if (this.mainDef !== null) {
            models.add(this.mainDef.reactorClass.toDefinition.eContainer as Model)
        }
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
        prefix += "    ";
        indentation.put(builder, prefix)
    }

    /**
     * Append the specified text plus a final newline to the current
     * code buffer.
     * @param format A format string to be used by String.format or
     * the text to append if no further arguments are given.
     * @param args Additional arguments to pass to the formatter.
     */
    protected def pr(String format, Object... args) {
        pr(code, if (args !== null && args.length > 0)
            String.format(format, args)
        else
            format)
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
     * Leave a marker in the generated code that indicates the original line
     * number in the LF source.
     * @param eObject The node.
     */
    protected def prSourceLineNumber(EObject eObject) {
        if (eObject instanceof Code) {
            pr(code, '''// «NodeModelUtils.getNode(eObject).startLine +1»''')
        } else {
            pr(code, '''// «NodeModelUtils.getNode(eObject).startLine»''')
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
     * the first line of an error message. Otherwise, return null.
     * This base class simply returns null.
     * @param line A line of output from a compiler or other external
     * tool that might generate errors.
     * @return If the line is recognized as the start of an error message,
     * then return a class containing the path to the file on which the
     * error occurred (or null if there is none), the line number (or the
     * string "1" if there is none), the character position (or the string
     * "0" if there is none), and the message (or an empty string if there
     * is none).
     */
    protected def parseCommandOutput(String line) {
        return null as ErrorFileAndLine
    }

    /**
     * Parse the specified string for command errors that can be reported
     * using marks in the Eclipse IDE. In this class, we attempt to parse
     * the messages to look for file and line information, thereby generating
     * marks on the appropriate lines.  This should only be called if
     * mode == INTEGRATED.
     * 
     * @param stderr The output on standard error of executing a command.
     */
    protected def reportCommandErrors(String stderr) {
        // First, split the message into lines.
        val lines = stderr.split("\\r?\\n")
        var message = new StringBuilder()
        var lineNumber = null as Integer
        var resource = getEclipseResource(fileConfig.srcFile.toURI());
        // In case errors occur within an imported file, record the original resource.
        val originalResource = resource;
        
        var severity = IMarker.SEVERITY_ERROR
        for (line : lines) {
            val parsed = parseCommandOutput(line)
            if (parsed !== null) {
                // Found a new line number designator.
                // If there is a previously accumulated message, report it.
                if (message.length > 0) {
                    report(message.toString(), severity, lineNumber, resource)
                    if (originalResource != resource) {
                        // Report an error also in the top-level resource.
                        // FIXME: It should be possible to descend through the import
                        // statements to find which one matches and mark all the
                        // import statements down the chain. But what a pain!
                        report(
                            "Error in imported file: " + resource.fullPath,
                            IMarker.SEVERITY_ERROR,
                            null,
                            originalResource
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
                // Sadly, Eclipse defines an interface called "URI" that conflicts with the
                // Java one, so we have to give the full class name here.
                val uri = new URI(parsed.filepath);
                resource = getEclipseResource(uri);
            } else {
                // No line designator.
                if (message.length > 0) {
                    message.append("\n")
                } else {
                    if (line.toLowerCase.contains('warning:')) {
                        severity = IMarker.SEVERITY_WARNING
                    }
                }
                message.append(line);
            }
        }
        if (message.length > 0) {
            report(message.toString, severity, lineNumber, resource)
            if (originalResource != resource) {
                // Report an error also in the top-level resource.
                // FIXME: It should be possible to descend through the import
                // statements to find which one matches and mark all the
                // import statements down the chain. But what a pain!
                report(
                    "Error in imported file: " + resource.fullPath,
                    IMarker.SEVERITY_ERROR,
                    null,
                    originalResource
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
            throw new IOException(
                "A required target resource could not be found: " + source + "\n" +
                    "Perhaps a git submodule is missing or not up to date.\n" +
                    "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.\n" +
                    "Also try to refresh and clean the project explorer if working from eclipse.")
        }

        // Copy the file.
        try {
            // Make sure the directory exists
            val destFile = new File(destination);
            destFile.getParentFile().mkdirs();

            Files.copy(sourceStream, Paths.get(destination), StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException ex) {
            throw new IOException(
                "A required target resource could not be copied: " + source + "\n" +
                    "Perhaps a git submodule is missing or not up to date.\n" +
                    "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.",
                ex)
        } finally {
            sourceStream.close()
        }
    }

    /**
     * Copy a list of files from a given source directory to a given destination directory.
     * @param srcDir The directory to copy files from.
     * @param dstDir The directory to copy files to.
     * @param files The files to copy.
     */
    protected def copyFilesFromClassPath(String srcDir, String dstDir, List<String> files) {
        for (file : files) {
            copyFileFromClassPath(srcDir + '/' + file, dstDir + File.separator + file)
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
                    Pattern.compile("platform:" + File.separator + "resource" + File.separator + id + File.separator);
                } else { // Windows file separator
                    Pattern.compile(
                        "platform:" + File.separator + File.separator + "resource" + File.separator + File.separator +
                            id + File.separator + File.separator);
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
                    if (projName == "" || projName.equals(member.fullPath.toString.substring(1))) {
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
        val header = (severity === IMarker.SEVERITY_ERROR) ? "ERROR: " : "WARNING: "
        val lineAsString = (line === null) ? "" : "Line " + line
        var fullPath = resource?.fullPath?.toString
        if (fullPath === null) {
            if (object !== null && object.eResource !== null) {
                fullPath = FileConfig.toPath(object.eResource).toString()
            } 
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
                    val uri = FileConfig.toPath(eResource).toUri();
                    myResource = getEclipseResource(uri);
                }
            }
            // If the resource is still null, use the resource associated with
            // the top-level file.
            if (myResource === null) {
                myResource = iResource
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

        val assignments = i.parameters.filter[p|p.lhs === param]

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

    // // Utility functions supporting multiports.
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

    // //////////////////////////////////////////////////
    // // Private functions
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
    protected def getRTITime(Delay d) {
        var TimeValue time
        if (d.parameter !== null) {
            return d.toText
        }

        time = new TimeValue(d.interval, d.unit)

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
    private def analyzeFederates() {
        // Next, if there actually are federates, analyze the topology
        // interconnecting them and replace the connections between them
        // with an action and two reactions.
        val mainDefn = this.mainDef?.reactorClass.toDefinition

        if (this.mainDef === null || !mainDefn.isFederated) {
            // Ensure federates is never empty.
            var federateInstance = new FederateInstance(null, 0, 0, this)
            federates.add(federateInstance)
            federateByID.put(0, federateInstance)
        } else {
            // The Lingua Franca program is federated
            isFederated = true
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
             * if (mainDef.reactorClass.host.dir !== null) {
             *     federationRTIProperties.put('dir', mainDef.reactorClass.host.dir)                
             * }
             */
            }

            // Create a FederateInstance for each top-level reactor.
            for (instantiation : mainDefn.allInstantiations) {
                var bankWidth = ASTUtils.width(instantiation.widthSpec);
                if (bankWidth < 0) {
                    reportError(instantiation, "Cannot determine bank width!");
                    // Continue with a bank width of 1.
                    bankWidth = 1;
                }
                // Create one federate instance for each reactor instance in the bank of reactors.
                val federateInstances = new LinkedList<FederateInstance>();
                for (var i = 0; i < bankWidth; i++) {
                    // Assign an integer ID to the federate.
                    var federateID = federates.size
                    var federateInstance = new FederateInstance(instantiation, federateID, i, this)
                    federateInstance.bankIndex = i;
                    federates.add(federateInstance)
                    federateInstances.add(federateInstance)
                    federateByID.put(federateID, federateInstance)

                    if (instantiation.host !== null) {
                        federateInstance.host = instantiation.host.addr
                        // The following could be 0.
                        federateInstance.port = instantiation.host.port
                        // The following could be null.
                        federateInstance.user = instantiation.host.user
                        /* FIXME: The at keyword should support a directory component.
                         * federateInstance.dir = instantiation.host.dir
                         */
                    }
                }
                if (federatesByInstantiation === null) {
                    federatesByInstantiation = new LinkedHashMap<Instantiation, List<FederateInstance>>();
                }
                federatesByInstantiation.put(instantiation, federateInstances);
            }

            // In a federated execution, we need keepalive to be true,
            // otherwise a federate could exit simply because it hasn't received
            // any messages.
            if (isFederated) {
                targetConfig.keepalive = true
            }

            // Analyze the connection topology of federates.
            // First, find all the connections between federates.
            // For each connection between federates, replace it in the
            // AST with an action (which inherits the delay) and two reactions.
            // The action will be physical for physical connections and logical
            // for logical connections.
            var connectionsToRemove = new LinkedList<Connection>()
            for (connection : mainDefn.connections) {
                // Each connection object may represent more than one physical connection between
                // federates because of banks and multiports. We need to generate communciation
                // for each of these. This iteration assumes the balance of the connection has been
                // checked.
                var rightIndex = 0;
                var rightPort = connection.rightPorts.get(rightIndex++);
                var rightBankIndex = 0;
                var rightChannelIndex = 0;
                var rightPortWidth = width((rightPort.variable as Port).widthSpec);
                for (leftPort: connection.leftPorts) {
                    var leftPortWidth = width((leftPort.variable as Port).widthSpec);
                    for (var leftBankIndex = 0; leftBankIndex < width(leftPort.container.widthSpec); leftBankIndex++) {
                        var leftChannelIndex = 0;
                        while (rightPort !== null) {
                            var minWidth = (leftPortWidth - leftChannelIndex < rightPortWidth - rightChannelIndex)
                                    ? leftPortWidth - leftChannelIndex
                                    : rightPortWidth - rightChannelIndex;
                            for (var j = 0; j < minWidth; j++) {
                            
                                // Finally, we have a specific connection.
                                // Replace the connection in the AST with an action
                                // (which inherits the delay) and two reactions.
                                // The action will be physical if the connection physical and
                                // otherwise will be logical.
                                val leftFederate = federatesByInstantiation.get(leftPort.container).get(leftBankIndex);
                                val rightFederate = federatesByInstantiation.get(rightPort.container).get(rightBankIndex);

                                // Set up dependency information.
                                // FIXME: Maybe we don't need this any more?
                                if (
                                    leftFederate !== rightFederate
                                    && !connection.physical
                                    && targetConfig.coordination !== CoordinationType.DECENTRALIZED
                                ) {
                                    var dependsOn = rightFederate.dependsOn.get(leftFederate)
                                    if (dependsOn === null) {
                                        dependsOn = new LinkedHashSet<Delay>()
                                        rightFederate.dependsOn.put(leftFederate, dependsOn)
                                    }
                                    if (connection.delay !== null) {
                                        dependsOn.add(connection.delay)
                                    }
                                    var sendsTo = leftFederate.sendsTo.get(rightFederate)
                                    if (sendsTo === null) {
                                        sendsTo = new LinkedHashSet<Delay>()
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
                                            val message = "Causality loop found between federates " +
                                                leftFederate.name + " and " + rightFederate.name
                                            reportError(connection, message)
                                            // This is a fatal error, so throw an exception.
                                            throw new Exception(message)
                                        }
                                    }
                                }
                                                                
                                ASTUtils.makeCommunication(
                                    connection, 
                                    leftFederate, leftBankIndex, leftChannelIndex,
                                    rightFederate, rightBankIndex, rightChannelIndex,
                                    this, targetConfig.coordination
                                )
                            
                                leftChannelIndex++;
                                rightChannelIndex++;
                                if (rightChannelIndex >= rightPortWidth) {
                                    // Ran out of channels on the right.
                                    // First, check whether there is another bank reactor.
                                    if (rightBankIndex < width(rightPort.container.widthSpec) - 1) {
                                        rightBankIndex++;
                                        rightChannelIndex = 0;
                                    } else if (rightIndex >= connection.rightPorts.size()) {
                                        // We are done.
                                        rightPort = null;
                                        rightBankIndex = 0;
                                        rightChannelIndex = 0;
                                    } else {
                                        rightBankIndex = 0;
                                        rightPort = connection.rightPorts.get(rightIndex++);
                                        rightChannelIndex = 0;
                                        rightPortWidth = width((rightPort.variable as Port).widthSpec);
                                    }
                                }
                            }
                        }
                    }
                }
                // To avoid concurrent modification exception, collect a list
                // of connections to remove.
                connectionsToRemove.add(connection)
            }
            for (connection : connectionsToRemove) {
                // Remove the original connection for the parent.
                mainDefn.connections.remove(connection)
            }
        }
    }

    /**
     * Determine which mode the compiler is running in.
     * Integrated mode means that it is running within an Eclipse IDE.
     * Standalone mode means that it is running on the command line.
     */
    private def setMode() {
        val resource = fileConfig.resource
        if (resource.URI.isPlatform) {
            mode = Mode.INTEGRATED
        } else if (resource.URI.isFile) {
            mode = Mode.STANDALONE
        } else {
            mode = Mode.UNDEFINED
            System.err.println("ERROR: Source file protocol is not recognized: " + resource.URI);
        }
    }

    /**
     * Print to stdout information about what source file is being generated,
     * what mode the generator is in, and where the generated sources are to be put.
     */
    def printInfo() {
        println("Generating code for: " + fileConfig.resource.getURI.toString)
        println('******** mode: ' + mode)
        println('******** source file: ' + fileConfig.srcFile) // FIXME: redundant
        println('******** generated sources: ' + fileConfig.getSrcGenPath)
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
    private def runSubprocess(ProcessBuilder processBuilder, List<OutputStream> outStream,
        List<OutputStream> errStream) {
        val process = processBuilder.start()

        var outThread = new Thread([|
            var buffer = newByteArrayOfSize(64)
            var len = process.getInputStream().read(buffer)
            while (len != -1) {
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
            while (len != -1) {
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

    abstract def String getTargetTagType()

    abstract def String getTargetTagIntervalType()

    abstract def String getTargetUndefinedType()

    abstract def String getTargetFixedSizeListType(String baseType, Integer size)

    abstract def String getTargetVariableSizeListType(String baseType);

    /**
     * Return the Targets enum for the current target
     */
    abstract def Target getTarget()

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

    protected def getTargetTime(Delay d) {
        if (d.parameter !== null) {
            return d.toText
        } else {
            return new TimeValue(d.interval, d.unit).timeInTargetLanguage
        }
    }

    /**
     * Write the source code to file.
     * @param code The code to be written.
     * @param path The file to write the code to.
     */
    protected def writeSourceCodeToFile(byte[] code, String path) {
        // Write the generated code to the output file.
        var fOut = new FileOutputStream(new File(path), false);
        fOut.write(code)
        fOut.close()
    }
    
}
