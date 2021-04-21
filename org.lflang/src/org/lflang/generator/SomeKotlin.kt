/**
 * Generator base class for shared code between code generators.
 */
package org.lflang.generator

import com.google.common.collect.Iterables
import org.eclipse.core.resources.IMarker
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtend2.lib.StringConcatenation
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.xbase.lib.*
import org.lflang.*
import org.lflang.Target
import org.lflang.graph.InstantiationGraph
import org.lflang.lf.*
import org.lflang.validation.AbstractLFValidator
import java.io.*
import java.net.URI
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import java.nio.file.StandardCopyOption
import java.util.*
import java.util.regex.Pattern
import kotlin.collections.LinkedHashMap
import kotlin.collections.LinkedHashSet
import kotlin.math.max

/**
 * Generator base class for shared code between code generators.
 * This extends AbstractLinguaFrancaValidator so that errors can be highlighted
 * in the XText-based IDE.
 *
 * @author{Edward A. Lee <eal></eal>@berkeley.edu>}
 * @author{Marten Lohstroh <marten></marten>@berkeley.edu>}
 * @author{Christian Menard <christian.menard></christian.menard>@tu-dresden.de}
 * @author{Matt Weber <matt.weber></matt.weber>@berkeley.edu>}
 */
abstract class GeneratorBase(val target: Target) : AbstractLFValidator() {
    /**
     * Defines the execution environment that is used to execute binaries.
     *
     * A given command may be directly executable on the host (NATIVE)
     * or it may need to be executed within a bash shell (BASH).
     * On Unix-like machines, this is typically determined by the PATH variable
     * which could be different in these two environments.
     */
    enum class ExecutionEnvironment {
        NATIVE, BASH
    }

    /**
     * Parsed error message from a compiler is returned here.
     */
    protected class ErrorFileAndLine {
        var filepath = null as String?
        var line = "1"
        var character = "0"
        var message = ""
        var isError = true
    }

    /**
     * All code goes into this string buffer.
     */
    protected var code = StringBuilder()

    /**
     * The current target configuration.
     */
    protected var targetConfig = TargetConfig()

    /**
     * The current file configuration. NOTE: not initialized until the
     * invocation of doGenerate, which calls setFileConfig.
     */
    protected var fileConfig: FileConfig? = null

    /**
     * [Mode.STANDALONE][.Mode.STANDALONE] if the code generator is being
     * called from the command line, [Mode.INTEGRATED][.Mode.INTEGRATED]
     * if it is being called from the Eclipse IDE, and
     * [Mode.UNDEFINED][.Mode.UNDEFINED] otherwise.
     */
    var mode = Mode.UNDEFINED

    /**
     * Collection of generated delay classes.
     */
    private val delayClasses = LinkedHashSet<Reactor>()

    /**
     * Set the fileConfig field to point to the specified resource using the specified
     * file-system access and context.
     * @param resource The resource (Eclipse-speak for a file).
     * @param fsa The Xtext abstraction for the file system.
     * @param context The generator context (whatever that is).
     */
    open fun setFileConfig(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        fileConfig = FileConfig(resource, fsa, context)
        topLevelName = fileConfig!!.name
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
    protected var iResource = null as IResource?

    /**
     * The main (top-level) reactor instance.
     */
    protected var main: ReactorInstance? = null

    /**
     * Definition of the main (top-level) reactor.
     * This is an automatically generated AST node for the top-level
     * reactor.
     */
    protected var mainDef: Instantiation? = null

    /**
     * A list of Reactor definitions in the main resource, including non-main
     * reactors defined in imported resources. These are ordered in the list in
     * such a way that each reactor is preceded by any reactor that it instantiates
     * using a command like `foo = new Foo();`
     */
    protected var reactors: MutableList<Reactor> = mutableListOf()

    /**
     * The set of resources referenced reactor classes reside in.
     */
    protected var resources: MutableSet<Resource> = CollectionLiterals.newLinkedHashSet()

    /**
     * Graph that tracks dependencies between instantiations.
     * This is a graph where each node is a Reactor (not a ReactorInstance)
     * and an arc from Reactor A to Reactor B means that B contains an instance of A, constructed with a statement
     * like `a = new A();`  After creating the graph,
     * sort the reactors in topological order and assign them to the reactors class variable.
     * Hence, after this method returns, `this.reactors` will be a list of Reactors such that any
     * reactor is preceded in the list by reactors that it instantiates.
     */
    protected var instantiationGraph: InstantiationGraph? = null

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
    protected var unorderedReactions: MutableSet<Reaction>? = null

    /**
     * Indicates whether or not the current Lingua Franca program
     * contains a federation.
     */
    protected var isFederated = false

    /**
     * A list of federate instances or a list with a single empty string
     * if there are no federates specified. FIXME: Why put a single empty string there? It should be just empty...
     */
    protected var federates: MutableList<FederateInstance> = LinkedList()

    /**
     * A map from federate IDs to federate instances.
     */
    protected var federateByID: MutableMap<Int, FederateInstance> = LinkedHashMap()

    /**
     * A map from instantiations to the federate instances for that instantiation.
     * If the instantiation has a width, there may be more than one federate instance.
     */
    protected var federatesByInstantiation: MutableMap<Instantiation, List<FederateInstance>>? = null

    /**
     * The federation RTI properties, which defaults to 'localhost: 15045'.
     */
    protected val federationRTIProperties = mutableMapOf<String, Any>(
        "host" to "localhost",
        "port" to 0
    )

    /**
     * Contents of $LF_CLASSPATH, if it was set.
     */
    protected var classpathLF: String? = null

    /**
     * The index available to user-generated reaction that delineates the index
     * of the reactor in a bank of reactors. The value must be set to zero
     * in generated code for reactors that are not in a bank
     */
    protected var targetBankIndex = "bank_index"

    /**
     * The type of the bank index, which must be an integer in the target language
     */
    protected var targetBankIndexType = "int"

    /**
     * The name of the top-level reactor.
     */
    protected var topLevelName: String? = null

    /**
     * Map from builder to its current indentation.
     */
    private val indentation = LinkedHashMap<StringBuilder, String>()

    /**
     * Store the given reactor in the collection of generated delay classes
     * and insert it in the AST under the top-level reactors node.
     */
    fun addDelayClass(generatedDelay: Reactor) {
        delayClasses.add(generatedDelay)
        val model = fileConfig!!.resource.allContents.asSequence().mapNotNull { it as? Model }.firstOrNull()
        model?.reactors?.add(generatedDelay)
    }

    /**
     * Return the generated delay reactor that corresponds to the given class
     * name if it had been created already, `null` otherwise.
     */
    fun findDelayClass(className: String): Reactor? =
        delayClasses.firstOrNull { it.name == className }


    fun setTargetConfig(context: IGeneratorContext) {
        // If there are any physical actions, ensure the threaded engine is used.
        for (action in fileConfig.resource.allContents.toIterable.filter(Action)) {
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
        if (context is StandaloneContext) {
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
                if (context.args.getProperty("target-flags").isNotEmpty()) {
                    targetConfig.compilerFlags.addAll(context.args.getProperty("target-flags").split(' '))
                }
            }
        }
    }

    /**
     * If there is a main or federated reactor, then create a synthetic Instantiation
     * for that top-level reactor and set the field mainDef to refer to it.
     */
    fun createMainInstance() {
        // Find the main reactor and create an AST node for its instantiation.
        for (reactor in fileConfig!!.resource.allContents.asSequence().filterIsInstance<Reactor>()) {
            if (reactor.isMain || reactor.isFederated) {
                // Creating an definition for the main reactor because there isn't one.
                this.mainDef = LfFactory.eINSTANCE.createInstantiation().apply {
                    name = reactor.name
                    reactorClass = reactor
                }
            }
        }
    }

    /**
     * Generate code from the Lingua Franca model contained by the specified resource.
     *
     * This is the main entry point for code generation. This base class finds all
     * reactor class definitions, including any reactors defined in imported .lf files
     * (except any main reactors in those imported files), and adds them to the
     * [reactors][.GeneratorBase.reactors] list. If errors occur during
     * generation, then a subsequent call to errorsOccurred() will return true.
     * @param resource The resource containing the source code.
     * @param fsa The file system access (used to write the result).
     * @param context Context relating to invocation of the code generator.
     * In stand alone mode, this object is also used to relay CLI arguments.
     */
    open fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {

        setFileConfig(resource, fsa, context)

        setMode()

        printInfo()

        // Clear any markers that may have been created by a previous build.
        // Markers mark problems in the Eclipse IDE when running in integrated mode.
        clearMarkers()

        val fileConfig = fileConfig!!
        ASTUtils.setMainName(fileConfig.resource, fileConfig.name)

        createMainInstance()

        // Check if there are any conflicting main reactors elsewhere in the package.
        if (mainDef !== null) {
            for (conflict in MainConflictChecker(fileConfig).conflicts) {
                reportError(this.mainDef!!.reactorClass, "Conflicting main reactor in $conflict")
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
    protected fun setReactorsAndInstantiationGraph() {
        // Build the instantiation graph .
        val fileConfig = fileConfig!!
        val instantiationGraph = InstantiationGraph(fileConfig.resource, false)
        this.instantiationGraph = instantiationGraph

        // Topologically sort the reactors such that all of a reactor's instantiation dependencies occur earlier in
        // the sorted list of reactors. This helps the code generator output code in the correct order.
        // For example if `reactor Foo {bar = new Bar()}` then the definition of `Bar` has to be generated before
        // the definition of `Foo`.
        this.reactors = instantiationGraph.nodesInTopologicalOrder()

        // If there is no main reactor, then make sure the reactors list includes
        // even reactors that are not instantiated anywhere.
        if (mainDef === null) {
            for (r in fileConfig.resource.allContents.asSequence().filterIsInstance<Reactor>()) {
                if (r !in this.reactors) {
                    this.reactors.add(r)
                }
            }
        }
    }

    /**
     * For each involved resource, replace connections with delays with generated delay reactors.
     */
    protected fun transformDelays() {
        for (r in resources) {
            ASTUtils.insertGeneratedDelays(r, this)
        }
    }

    /**
     * Update the class variable that lists all the involved resources. Also report validation problems of imported
     * resources at the import statements through those failing resources are reached.
     *
     * @param context The context providing the cancel indicator used by the validator.
     */
    protected fun setResources(context: IGeneratorContext) {
        val fileConfig = this.fileConfig!!
        val validator = (fileConfig.resource as XtextResource).resourceServiceProvider.resourceValidator
        val mainDef = mainDef
        if (mainDef != null) {
            reactors.add(mainDef.reactorClass as Reactor)
        }
        // Iterate over reactors and mark their resources as tainted if they import resources that are either marked
        // as tainted or fail to validate.
        val tainted = mutableSetOf<Resource>()
        for (r in this.reactors) {
            val res = r.eResource()
            if (!this.resources.contains(res)) {
                if (res !== fileConfig.resource) {
                    if (tainted.contains(res)
                        || validator.validate(res, CheckMode.ALL, context.cancelIndicator).isNotEmpty()
                    ) {

                        for (inst in this.instantiationGraph!!.getDownstreamAdjacentNodes(r)) {
                            for (imp in (inst.eContainer() as Model).imports) {
                                for (decl in imp.reactorClasses) {
                                    if (decl.reactorClass.eResource() === res) {
                                        reportError(imp, """Unresolved compilation issues in "${imp.importURI}".""")
                                        tainted.add(decl.eResource())
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
    protected open fun copyUserFiles() {
// Make sure the target directory exists.
        val fileConfig = this.fileConfig!!
        val targetDir = fileConfig.srcGenPath!!
        Files.createDirectories(targetDir)

        for (filename in targetConfig.fileNames) {
            val file: File? = FileConfig.findFile(filename, fileConfig.srcFile.parent)
            if (file != null) {
                val target = targetDir.resolve(file.name)
                Files.deleteIfExists(target)
                Files.copy(file.toPath(), target)
                targetConfig.filesNamesWithoutPath.add(file.name)
            } else {
                // Try to copy the file as a resource.
                // If this is missing, it should have been previously reported as an error.
                try {
                    val filenameWithoutPath = filename.substringAfterLast(File.separatorChar)
                    copyFileFromClassPath(filename, targetDir.resolve(filenameWithoutPath).toString())
                    targetConfig.filesNamesWithoutPath.add(filenameWithoutPath)
                } catch (ex: IOException) {
                    // Ignore. Previously reported as a warning.
                    System.err.println("WARNING: Failed to find file $filename.")
                }
            }
        }
    }

    /**
     * Return true if errors occurred in the last call to doGenerate().
     * This will return true if any of the reportError methods was called.
     * @return True if errors occurred.
     */
    fun errorsOccurred(): Boolean {
        return generatorErrorsOccurred
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action the action to schedule
     * @param port the port to read from
     */
    abstract fun generateDelayBody(action: Action, port: VarRef): String?

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     * @param action the action that triggers the reaction
     * @param port the port to write to
     */
    abstract fun generateForwardBody(action: Action, port: VarRef): String?

    /**
     * Generate code for the generic type to be used in the class definition
     * of a generated delay reactor.
     */
    abstract fun generateDelayGeneric(): String?

    /**
     * Generate code for referencing a port, action, or timer.
     * @param reference The referenced variable.
     */
    fun generateVarRef(reference: VarRef): String {
        var prefix = ""
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
    fun isUnordered(reaction: Reaction): Boolean =
        unorderedReactions?.contains(reaction) == true

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
    fun makeUnordered(reaction: Reaction): Boolean =
        unorderedReactions?.add(reaction) ?: let {
            unorderedReactions = mutableSetOf(reaction)
            true
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
    open fun timeInTargetLanguage(time: TimeValue?): String =
        if (time !== null) {
            if (time.unit != TimeUnit.NONE) {
                time.unit.name + '(' + time.time + ')'
            } else {
                time.time.toString()
            }
        } else
            "0" // FIXME: do this or throw exception?

    /**
     * Create the runtime infrastructure (RTI) source file.
     */
    open fun createFederateRTI() {
        val fileConfig = fileConfig!!

        // Derive target filename from the .lf filename.
        val cFilename = fileConfig.name + "_RTI.c"

        // Delete source previously produced by the LF compiler.
        var file = fileConfig.rtiSrcPath.resolve(cFilename)
        Files.deleteIfExists(file)
        // Also make sure the directory exists.
        Files.createDirectories(file.parent)

        // Delete binary previously produced by the C compiler.
        file = fileConfig.rtiBinPath.resolve(fileConfig.name)
        Files.deleteIfExists(file)

        val rtiCode = StringBuilder()
        pr(
            rtiCode, """
            #ifdef NUMBER_OF_FEDERATES
        #undefine NUMBER_OF_FEDERATES
        #endif
        #define NUMBER_OF_FEDERATES ${federates.size}
        #include "rti.c"
        int main(int argc, char* argv[]) {
            """
        )
        indent(rtiCode)

        // Initialize the array of information that the RTI has about the
        // federates.
        // FIXME: No support below for some federates to be FAST and some REALTIME.
        pr(
            rtiCode, """
                for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
            initialize_federate(i);
            ${
                if (targetConfig.fastMode)
                    "federates[i].mode = FAST;"
                else ""
            }
        }
            """
        )
        // Initialize the arrays indicating connectivity to upstream and downstream federates.
        for (federate in federates) {
            if (federate.dependsOn.isNotEmpty()) {
                // Federate receives non-physical messages from other federates.
                // Initialize the upstream and upstream_delay arrays.
                val numUpstream = federate.dependsOn.size
                // Allocate memory for the arrays storing the connectivity information.
                pr(
                    rtiCode, """
                            federates[${federate.id}].upstream = (int*)malloc(sizeof(federate_t*) * ${numUpstream});
                    federates[${federate.id}].upstream_delay = (interval_t*)malloc(sizeof(interval_t*) * ${numUpstream});
                    federates[${federate.id}].num_upstream = ${numUpstream};
                    """
                )
                // Next, populate these arrays.
                // Find the minimum delay in the process.
                // FIXME: Zero delay is not really the same as a microstep delay.
                var count = 0
                for (upstreamFederate: federate.dependsOn.keySet) {
                    pr(
                        rtiCode, """
                                federates[${federate.id}].upstream[${count}] = ${upstreamFederate.id};
                        federates[${federate.id}].upstream_delay[${count}] = 0LL;
                        """
                    )
                    // The minimum delay calculation needs to be made in the C code because it
                    // may depend on parameter values.
                    // FIXME: These would have to be top-level parameters, which don't really
                    // have any support yet. Ideally, they could be overridden on the command line.
                    // When that is done, they will need to be in scope here.
                    val delays = federate.dependsOn.get(upstreamFederate)
                    if (delays !== null) {
                        for (delay in delays) {
                            pr(
                                rtiCode, """
                                    if (federates[${federate.id}].upstream_delay[${count}] < ${delay.getRTITime}) {
                                    federates[${federate.id}].upstream_delay[${count}] = ${delay.getRTITime};
                                }
                                """
                            )
                        }
                    }
                    count++
                }
            }
            // Next, set up the downstream array.
            if (federate.sendsTo.isNotEmpty()) {
                // Federate sends non-physical messages to other federates.
                // Initialize the downstream array.
                val numDownstream = federate.sendsTo.size
                // Allocate memory for the array.
                pr(
                    rtiCode, """
                            federates[${federate.id}].downstream = (int*)malloc(sizeof(federate_t*) * ${numDownstream});
                    federates[${federate.id}].num_downstream = ${numDownstream};
                    """
                )
                // Next, populate the array.
                // Find the minimum delay in the process.
                // FIXME: Zero delay is not really the same as a microstep delay.
                for ((count, downstreamFederate) in federate.sendsTo.keys.withIndex()) {
                    pr(
                        rtiCode, """
                                federates[${federate.id}].downstream[${count}] = ${downstreamFederate.id};
                        """
                    )
                }
            }
        }

        // Start the RTI server before launching the federates because if it
        // fails, e.g. because the port is not available, then we don't want to
        // launch the federates.
        // Also generate code that blocks until the federates resign.
        pr(
            rtiCode, """
                    int socket_descriptor = start_rti_server(${federationRTIProperties["port"]});
            wait_for_federates(socket_descriptor);
            """
        )

        unindent(rtiCode)
        pr(rtiCode, "}")

        Files.write(
            fileConfig.rtiSrcPath.resolve(cFilename),
            rtiCode.toString().toByteArray()
        )
    }

    /**
     * Invoke the C compiler on the generated RTI
     * The C RTI is used across targets. Thus we need to be able to compile
     * it from GeneratorBase.
     */
    fun compileRTI(): Boolean {
        val fileToCompile = fileConfig!!.name + "_RTI"
        return runCCompiler(fileToCompile, false)
    }

    /**
     * Run the C compiler.
     *
     * This is required here in order to allow any target to compile the RTI.
     *
     * @param file The source file to compile without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     * `-c` flag when there is no main reactor. If false, the compile command
     * will never have a `-c` flag.
     *
     * @return true if compilation succeeds, false otherwise.
     */
    open fun runCCompiler(file: String, doNotLinkIfNoMain: Boolean): Boolean {
        val compile = compileCCommand(file, doNotLinkIfNoMain) ?: return false

        val stderr = ByteArrayOutputStream()
        val returnCode = compile.executeCommand(stderr)

        if (returnCode != 0 && mode !== Mode.INTEGRATED) {
            reportError("""${targetConfig.compiler} returns error code $returnCode""")
        }
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (stderr.size() > 0 && mode === Mode.INTEGRATED) {
            reportCommandErrors(stderr.toString())
        }
        return (returnCode == 0)
    }

    /**
     * Run the custom build command specified with the "build" parameter.
     * This command is executed in the same directory as the source file.
     */
    protected fun runBuildCommand() {
        val commands = mutableListOf<ProcessBuilder>()
        for (cmd in targetConfig.buildCommands) {
            val tokens = cmd.split("\\s+")
            if (tokens.size > 1) {
                val buildCommand = createCommand(tokens.first(), tokens.tail(), this.fileConfig!!.srcPath)
                    ?: return
                // If the build command could not be found, abort.
                // An error has already been reported in createCommand.
                commands.add(buildCommand)
            }
        }

        for (cmd in commands) {
            val stderr = ByteArrayOutputStream()
            val returnCode = cmd.executeCommand(stderr)

            if (returnCode != 0 && mode !== Mode.INTEGRATED) {
                reportError("""Build command "${targetConfig.buildCommands}" returns error code $returnCode""")
                return
            }
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (stderr.size() > 0 && mode == Mode.INTEGRATED) {
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
     * `-c` flag when there is no main reactor. If false, the compile command
     * will never have a `-c` flag.
     */
    protected fun compileCCommand(fileToCompile: String, doNotLinkIfNoMain: Boolean): ProcessBuilder? {
        val env = findCommandEnv(targetConfig.compiler)

        val cFilename = getTargetFileName(fileToCompile)
        val fileConfig = fileConfig!!

        val relativeSrcPath = fileConfig.outPath.relativize(fileConfig.srcGenPath.resolve(cFilename))
        val relativeBinPath = fileConfig.outPath.relativize(fileConfig.binPath.resolve(fileToCompile))

        // NOTE: we assume that any C compiler takes Unix paths as arguments.
        val relSrcPathString = FileConfig.toUnixString(relativeSrcPath)
        var relBinPathString = FileConfig.toUnixString(relativeBinPath)

        // If there is no main reactor, then generate a .o file not an executable.
        if (mainDef === null) {
            relBinPathString += ".o"
        }

        val compileArgs = mutableListOf<String>()
        compileArgs.add(relSrcPathString)
        compileArgs.addAll(targetConfig.compileAdditionalSources)
        compileArgs.addAll(targetConfig.compileLibraries)

        // Only set the output file name if it hasn't already been set
        // using a target property or Args line flag.
        if (compileArgs.all { it.trim() != "-o" }) {
            compileArgs.addAll(listOf("-o", relBinPathString))
        }

        // If threaded computation is requested, add a -pthread option.

        if (targetConfig.threads != 0 || targetConfig.tracing !== null) {
            compileArgs.add("-pthread")
            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            compileArgs.add("-DNUMBER_OF_WORKERS=${targetConfig.threads}")
        }
        // Finally add the compiler flags in target parameters (if any)
        if (targetConfig.compilerFlags.isNotEmpty()) {
            compileArgs.addAll(targetConfig.compilerFlags)
        }
        // If there is no main reactor, then use the -c flag to prevent linking from occurring.
        // FIXME: we could add a `-c` flag to `lfc` to make this explicit in stand-alone mode.
        // Then again, I think this only makes sense when we can do linking.
        // In any case, a warning is helpful to draw attention to the fact that no binary was produced.
        if (doNotLinkIfNoMain && main == null) {
            compileArgs.add("-c") // FIXME: revisit
            if (mode == Mode.STANDALONE) {
                reportError("ERROR: Did not output executable; no main reactor found.")
            }
        }
        return createCommand(targetConfig.compiler, compileArgs, fileConfig.outPath, env)
    }

    /**
     * Produces the filename including the target-specific extension
     */
    protected open fun getTargetFileName(fileName: String): String = "$fileName.c"

    /**
     * Clear the buffer of generated code.
     */
    protected fun clearCode(): StringBuilder = StringBuilder().also { this.code = it }

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
    protected fun getEclipseResource(uri: URI?): IResource? {
        var resource = iResource // Default resource.
        // For some peculiar reason known only to Eclipse developers,
        // the resource cannot be used directly but has to be converted
        // a resource relative to the workspace root.
        val workspaceRoot = ResourcesPlugin.getWorkspace().root
        // The following uses a java.net.URI, which,
        // pathetically, cannot be distinguished in xtend from a org.eclipse.emf.common.util.URI.
        if (uri !== null) {
            // Pathetically, Eclipse requires a java.net.uri, not a org.eclipse.emf.common.util.URI.
            val files = workspaceRoot.findFilesForLocationURI(uri)
            if (files !== null && files.isNotEmpty() && files[0] !== null) {
                resource = files[0]
            }
        }
        return resource
    }

    /**
     * Clear markers in the IDE if running in integrated mode.
     * This has the side effect of setting the iResource variable to point to
     * the IFile for the Lingua Franca program.
     * Also reset the flag indicating that generator errors occurred.
     */
    protected fun clearMarkers(): Boolean {
        if (mode == Mode.INTEGRATED) {
            try {
                val resource = getEclipseResource(fileConfig!!.srcFile.toURI())
                // First argument can be null to delete all markers.
                // But will that delete xtext markers too?
                resource?.deleteMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE)
            } catch (e: Exception) {
                // Ignore, but print a warning.
                println("Warning: Deleting markers in the IDE failed: $e")
            }
        }
        generatorErrorsOccurred = false
        return false
    }

    /**
     * Run a given command and record its output.
     *
     * @param cmd the command to be executed
     * @param errStream a stream object to forward the commands error messages to
     * @param outStream a stream object to forward the commands output messages to
     * @return the commands return code
     */
    protected fun executeCommand(cmd: ProcessBuilder, errStream: OutputStream? = null, outStream: OutputStream? = null): Int {
        println("--- Current working directory: \${cmd.directory.toString()}")
        println("--- Executing command: ${cmd.command().joinToString(" ")}")

        val outStreams = mutableListOf<OutputStream>()
        val errStreams = mutableListOf<OutputStream>()
        outStreams.add(System.out)
        errStreams.add(System.err)
        if (outStream != null) {
            outStreams.add(outStream)
        }
        if (errStream != null) {
            errStreams.add(errStream)
        }

        // Execute the command. Write output to the System output,
        // but also keep copies in outStream and errStream
        return cmd.runSubprocess(outStreams, errStreams)

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
    </cmd></cmd> */
    protected fun findCommandEnv(cmd: String): ExecutionEnvironment? {
        // Make sure the command is found in the PATH.
        print("--- Looking for command $cmd ...")
        // Use 'where' on Windows, 'which' on other systems
        val which = if (System.getProperty("os.name").startsWith("Windows")) "where" else "which"
        val whichBuilder = ProcessBuilder(which, cmd)
        val whichReturn = whichBuilder.start().waitFor()
        if (whichReturn == 0) {
            println("SUCCESS")

            return ExecutionEnvironment.NATIVE
        }
        println("FAILED")
        // Try running with bash.
        // The --login option forces bash to look for and load the first of
        // ~/.bash_profile, ~/.bash_login, and ~/.bashrc that it finds.
        print("--- Trying again with bash ... ")
        val bashCommand = listOf("bash", "--login", "-c", "which $cmd")
        val bashBuilder = ProcessBuilder(bashCommand)
        val bashOut = ByteArrayOutputStream()
        val bashReturn = bashBuilder.runSubprocess(#[bashOut], #[])
        if (bashReturn == 0) {
            println("SUCCESS")
            return ExecutionEnvironment.BASH
        }
        reportError(
            """The command $cmd could not be found.
Make sure that your PATH variable includes the directory where $cmd is installed.
You can set PATH in ~/.bash_profile on Linux or Mac."""
        )
        return null

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
     * @param args A list of arguments for the given command
     * @param dir the directory to change into before executing the command.
     * @param env is the type of the Execution Environment.
     * @return A ProcessBuilder object if the command was found or null otherwise.
     */
    protected fun createCommand(
        cmd: String,
        args: List<String> = emptyList(),
        dir: Path = fileConfig!!.outPath,
        env: ExecutionEnvironment? = findCommandEnv(cmd)
    ): ProcessBuilder? {
        if (env == ExecutionEnvironment.NATIVE) {
            val builder = ProcessBuilder(cmd, *args.toTypedArray())
            builder.directory(dir.toFile())
            return builder
        } else if (env == ExecutionEnvironment.BASH) {
            val bashArg = args.joinToString(separator = " ", prefix = "$cmd ")
            // use that command to build the process
            val builder = ProcessBuilder("bash", "--login", "-c", bashArg)
            builder.directory(dir.toFile())
            return builder
        }
        println("FAILED")
        reportError(
            """The command $cmd could not be found.
Make sure that your PATH variable includes the directory where $cmd is installed.
You can set PATH in ~/.bash_profile on Linux or Mac."""
        )
        return null
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
    protected fun createCommand(cmd: String, args: List<String>, dir: Path): ProcessBuilder? {
        val env = findCommandEnv(cmd)
        return this.createCommand(cmd, args, dir, env)
    }

    /**
     * Return the target.
     */
    fun findTarget(resource: Resource): TargetDecl? {
        val targets = resource.allContents.asSequence().filterIsInstance<TargetDecl>().toList()
        when {
            targets.isEmpty() -> throw RuntimeException("No target found!")
            targets.size > 1  -> throw RuntimeException("There is more than one target!") // FIXME: check this in validator
            else              -> return targets[0]
        }
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
    open fun generateNetworkReceiverBody(
        action: Action?,
        sendingPort: VarRef?,
        receivingPort: VarRef?,
        receivingPortID: Int,
        sendingFed: FederateInstance?,
        receivingFed: FederateInstance?,
        receivingBankIndex: Int,
        receivingChannelIndex: Int,
        type: InferredType?
    ): String? {
        throw UnsupportedOperationException("This target does not support direct connections between federates.")
    }

    // ---ported until HERE---

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
    open fun generateNetworkSenderBody(
        sendingPort: VarRef?,
        receivingPort: VarRef?,
        receivingPortID: Int,
        sendingFed: FederateInstance?,
        sendingBankIndex: Int,
        sendingChannelIndex: Int,
        receivingFed: FederateInstance?,
        type: InferredType?,
        isPhysical: Boolean,
        delay: Delay?
    ): String? {
        throw UnsupportedOperationException("This target does not support direct connections between federates.")
    }

    /**
     * Generate any preamble code that appears in the code generated
     * file before anything else.
     */
    protected open fun generatePreamble() {
        prComment("Code generated by the Lingua Franca compiler from:")
        val _unixString = FileConfig.toUnixString(fileConfig!!.srcFile.toPath())
        val _plus = ("file:/$_unixString")
        prComment(_plus)
        val models = LinkedHashSet<Model>()
        var _elvis: List<Reactor>? = null
        if (reactors != null) {
            _elvis = reactors
        } else {
            val _emptyList = CollectionLiterals.emptyList<Reactor>()
            _elvis = _emptyList
        }
        for (r: Reactor? in _elvis!!) {
            val _eContainer = ASTUtils.toDefinition(r).eContainer()
            models.add((_eContainer as Model))
        }
        if ((mainDef != null)) {
            val _eContainer_1 = ASTUtils.toDefinition(mainDef!!.reactorClass).eContainer()
            models.add((_eContainer_1 as Model))
        }
        for (m: Model in models) {
            val _preambles = m.preambles
            for (p: Preamble in _preambles) {
                this.pr(ASTUtils.toText(p.code))
            }
        }
    }

    /**
     * Get the code produced so far.
     * @return The code produced so far as a String.
     */
    protected fun getCode(): String {
        return this.code.toString()
    }
    /**
     * Increase the indentation of the output code produced
     * on the specified builder.
     * @param The builder to indent.
     */
    /**
     * Increase the indentation of the output code produced.
     */
    protected fun indent(builder: StringBuilder = this.code): String? {
        var _xblockexpression: String? = null
        run {
            var prefix: String? = this.indentation[builder]
            if ((prefix == null)) {
                prefix = ""
            }
            val _prefix: String = prefix
            prefix = (_prefix + "    ")
            _xblockexpression = this.indentation.put(builder, prefix)
        }
        return _xblockexpression
    }

    /**
     * Append the specified text plus a final newline to the current
     * code buffer.
     * @param format A format string to be used by String.format or
     * the text to append if no further arguments are given.
     * @param args Additional arguments to pass to the formatter.
     */
    protected fun pr(format: String?, vararg args: Any?): StringBuilder? {
        var _xifexpression: String? = null
        if (((args != null) && (args.size > 0))) {
            _xifexpression = String.format((format)!!, *args)
        } else {
            _xifexpression = format
        }
        return this.pr(this.code, _xifexpression)
    }

    /**
     * Append the specified text plus a final newline to the specified
     * code buffer.
     * @param builder The code buffer.
     * @param text The text to append.
     */
    protected fun pr(builder: StringBuilder, text: Any?): StringBuilder? {
        var _xblockexpression: StringBuilder? = null
        run {
            var string: String = text.toString()
            var indent: String? = this.indentation[builder]
            if ((indent == null)) {
                indent = ""
            }
            var _xifexpression: StringBuilder? = null
            val _contains: Boolean = string.contains("\n")
            if (_contains) {
                string = string.replace("\t".toRegex(), "    ")
                val split: Array<String> = string.split("\n").toTypedArray()
                var offset: Int = Int.MAX_VALUE
                var firstLine: Boolean = true
                for (line: String in split) {
                    if (firstLine) {
                        firstLine = false
                    } else {
                        val numLeadingSpaces: Int = line.indexOf(line.trim { it <= ' ' })
                        if ((numLeadingSpaces < offset)) {
                            offset = numLeadingSpaces
                        }
                    }
                }
                firstLine = true
                for (line_1: String? in split) {
                    {
                        builder.append(indent)
                        if (firstLine) {
                            builder.append(line_1)
                            firstLine = false
                        } else {
                            builder.append(line_1.substring(offset))
                        }
                        builder.append("\n")
                    }
                }
            } else {
                var _xblockexpression_1: StringBuilder? = null
                {
                    builder.append(indent)
                    builder.append(text)
                    _xblockexpression_1 = builder.append("\n")
                }
                _xifexpression = _xblockexpression_1
            }
            _xblockexpression = _xifexpression
        }
        return _xblockexpression
    }

    /**
     * Prints an indented block of text with the given begin and end markers,
     * but only if the actions print any text at all.
     * This is helpful to avoid the production of empty blocks.
     * @param begin The prologue of the block.
     * @param end The epilogue of the block.
     * @param actions Actions that print the interior of the block.
     */
    protected fun prBlock(begin: String?, end: String?, vararg actions: Runnable): StringBuilder? {
        var _xblockexpression: StringBuilder? = null
        run {
            val i: Int = this.code.length
            this.indent()
            for (action: Runnable in actions) {
                action.run()
            }
            this.unindent()
            var _xifexpression: StringBuilder? = null
            val _length: Int = this.code.length
            val _lessThan: Boolean = (i < _length)
            if (_lessThan) {
                var _xblockexpression_1: StringBuilder? = null
                {
                    val inserted: String = this.code.substring(i, this.code.length)
                    this.code.delete(i, this.code.length)
                    this.pr(begin)
                    this.code.append(inserted)
                    _xblockexpression_1 = this.pr(end)
                }
                _xifexpression = _xblockexpression_1
            }
            _xblockexpression = _xifexpression
        }
        return _xblockexpression
    }

    /**
     * Leave a marker in the generated code that indicates the original line
     * number in the LF source.
     * @param eObject The node.
     */
    protected open fun prSourceLineNumber(eObject: EObject?): StringBuilder? {
        var _xifexpression: StringBuilder? = null
        if ((eObject is Code)) {
            val _builder = StringConcatenation()
            _builder.append("// ")
            val _startLine = NodeModelUtils.getNode(eObject).startLine
            val _plus = (_startLine + 1)
            _builder.append(_plus)
            _xifexpression = this.pr(this.code, _builder)
        } else {
            val _builder_1 = StringConcatenation()
            _builder_1.append("// ")
            val _startLine_1 = NodeModelUtils.getNode(eObject).startLine
            _builder_1.append(_startLine_1)
            _xifexpression = this.pr(this.code, _builder_1)
        }
        return _xifexpression
    }

    /**
     * Print a comment to the generated file.
     * Particular targets will need to override this if comments
     * start with something other than '//'.
     * @param comment The comment.
     */
    protected fun prComment(comment: String): StringBuilder? {
        return this.pr(this.code, ("// $comment"))
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
    protected open fun parseCommandOutput(line: String?): ErrorFileAndLine? {
        return null
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
    protected fun reportCommandErrors(stderr: String): String? {
        val message = StringBuilder()
        var lineNumber: Int? = null
        var resource: IResource? = this.getEclipseResource(this.fileConfig!!.srcFile.toURI())
        val originalResource: IResource? = resource
        var severity: Int = IMarker.SEVERITY_ERROR

        for (line in stderr.lines()) {
            val parsed = parseCommandOutput(line)
            if (parsed != null) {
                // Found a new line number designator.
                // If there is a previously accumulated message, report it.

                if (message.isNotEmpty()) {
                    this.report(message.toString(), severity, lineNumber, resource)
                    if (originalResource != resource) {
                        // Report an error also in the top-level resource.
                        // FIXME: It should be possible to descend through the import
                        // statements to find which one matches and mark all the
                        // import statements down the chain. But what a pain!
                        this.report(
                            "Error in imported file: " + resource?.fullPath,
                            IMarker.SEVERITY_ERROR,
                            null, originalResource
                        )
                    }
                }
                severity = if (parsed.isError) {
                    IMarker.SEVERITY_ERROR
                } else {
                    IMarker.SEVERITY_WARNING
                }

                // Start accumulating a new message.
                message.setLength(0)
                // Append the message on the line number designator line.
                message.append(parsed.message)

                // Set the new line number.
                lineNumber = try {
                    Integer.decode(parsed.line)
                } catch (_t: Exception) {
                    null
                }

                // FIXME: Ignoring the position within the line.
                // Determine the resource within which the error occurred.
                // Sadly, Eclipse defines an interface called "URI" that conflicts with the
                // Java one, so we have to give the full class name here.
                resource = this.getEclipseResource(URI(parsed.filepath))
            } else {
                // No line designator.
                if (message.isNotEmpty()) {
                    message.append("\n")
                } else {
                    if ("warning:" in line.toLowerCase()) {
                        severity = IMarker.SEVERITY_WARNING
                    }
                }
                message.append(line)
            }
        }

        if (message.isNotEmpty()) {
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
    }

    /**
     * Lookup a file in the classpath and copy its contents to a destination path
     * in the filesystem.
     *
     * This also creates new directories for any directories on the destination
     * path that do not yet exist.
     *
     * @param source The source file as a path relative to the classpath.
     * @param destination The file system path that the source file is copied to.
     */
    protected fun copyFileFromClassPath(source: String, destination: String): Long {
        val sourceStream = this::class.java.getResourceAsStream(source)

        if (sourceStream == null) {
            throw IOException(
                "A required target resource could not be found: " + source + "\n" +
                        "Perhaps a git submodule is missing or not up to date.\n" +
                        "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.\n" +
                        "Also try to refresh and clean the project explorer if working from eclipse."
            )
        }

        // Copy the file.
        try {
            // Make sure the directory exists
            val destFile = File(destination)
            destFile.parentFile.mkdirs()

            Files.copy(sourceStream, Paths.get(destination), StandardCopyOption.REPLACE_EXISTING)
        } catch (ex: IOException) {
            throw IOException(
                "A required target resource could not be copied: " + source + "\n" +
                        "Perhaps a git submodule is missing or not up to date.\n" +
                        "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository.",
                ex
            )
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
    protected fun copyFilesFromClassPath(srcDir: String, dstDir: String, files: List<String>) {
        for (file: String in files) {
            copyFileFromClassPath("$srcDir/$file", dstDir + File.separator + file)
        }
    }

    /**
     * If the mode is INTEGRATED (the code generator is running in an
     * an Eclipse IDE), then refresh the project. This will ensure that
     * any generated files become visible in the project.
     */
    protected fun refreshProject(): String? {
        if (mode == Mode.INTEGRATED) {
            // Find name of current project
            val id = "((:?[a-z]|[A-Z]|_\\w)*)"
            val pattern: Pattern = if (File.separator.equals("/")) { // Linux/Mac file separator
                Pattern.compile("platform:" + File.separator + "resource" + File.separator + id + File.separator)
            } else { // Windows file separator
                Pattern.compile(
                    "platform:" + File.separator + File.separator + "resource" + File.separator + File.separator +
                            id + File.separator + File.separator
                )
            }
            val matcher = pattern.matcher(code)
            var projName = ""
            if (matcher.find()) {
                projName = matcher.group(1)
            }
            try {
                val members = ResourcesPlugin.getWorkspace().root.members
                for (member in members) {
                    // Refresh current project, or simply entire workspace if project name was not found
                    if (projName == "" || projName.equals(member.fullPath.toString().substring(1))) {
                        member.refreshLocal(IResource.DEPTH_INFINITE, null)
                        println("Refreshed " + member.fullPath)
                    }
                }
            } catch (e: IllegalStateException) {
                println("Unable to refresh workspace: $e")
            }
        }
    }

    /**
     * Report a warning or error on the specified line of the specified resource.
     * The caller should not throw an exception so execution can continue.
     * This will print the error message to stderr.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     * @param message The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param line The line number or null if it is not known.
     * @param object The Ecore object, or null if it is not known.
     * @param resource The resource, or null if it is not known.
     */
    protected fun report(message: String, severity: Int, line: Int?, `object`: EObject?, resource: IResource?): String {
        if (severity == IMarker.SEVERITY_ERROR) {
            generatorErrorsOccurred = true
        }
        val header = if (severity == IMarker.SEVERITY_ERROR) "ERROR: " else "WARNING: "
        val lineAsString = if (line === null) "" else "Line $line"

        val fullPath: String = let {
            var p = resource?.fullPath?.toString()
            if (p == null) {
                if (`object` != null && `object`.eResource() !== null)
                    p = FileConfig.toPath(`object`.eResource()).toString()
                if (p == null) {
                    p = if (line == null) "" else "path unknown"
                }
            }
            p
        }


        System.err.println("$header$fullPath $lineAsString\n$message")

        // If running in INTEGRATED mode, create a marker in the IDE for the error.
        // See: https://help.eclipse.org/2020-03/index.jsp?topic=%2Forg.eclipse.platform.doc.isv%2Fguide%2FresAdv_markers.htm
        if (mode === Mode.INTEGRATED) {
            var myResource = resource
            if (myResource === null && `object` != null) {
                // Attempt to identify the IResource from the object.
                val eResource = `object`.eResource()
                if (eResource !== null) {
                    val uri = FileConfig.toPath(eResource).toUri()
                    myResource = getEclipseResource(uri)
                }
            }
            // If the resource is still null, use the resource associated with
            // the top-level file.
            if (myResource === null) {
                myResource = iResource
            }
            if (myResource !== null) {
                val marker = myResource.createMarker(IMarker.PROBLEM)
                marker.setAttribute(IMarker.MESSAGE, message)
                if (line !== null) {
                    marker.setAttribute(IMarker.LINE_NUMBER, line)
                } else {
                    marker.setAttribute(IMarker.LINE_NUMBER, 1)
                }
                // Human-readable line number information.
                marker.setAttribute(IMarker.LOCATION, lineAsString)
                // Mark as an error or warning.
                marker.setAttribute(IMarker.SEVERITY, severity)
                marker.setAttribute(IMarker.PRIORITY, IMarker.PRIORITY_HIGH)

                marker.setAttribute(IMarker.USER_EDITABLE, false)

                // NOTE: It might be useful to set a start and end.
                // marker.setAttribute(IMarker.CHAR_START, 0);
                // marker.setAttribute(IMarker.CHAR_END, 5);
            }
        }

        // Return a string that can be inserted into the generated code.
        if (severity == IMarker.SEVERITY_ERROR) {
            return "[[ERROR: $message]]"
        }
        return ""
    }

    /**
     * Report a warning or error on the specified parse tree object in the
     * current resource.
     * The caller should not throw an exception so execution can continue.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     * @param message The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param `object` The parse tree object or null if not known.
     */
    protected fun report(message: String, severity: Int, obj: EObject?): String {
        var line: Int? = null
        if (obj != null) {
            val node = NodeModelUtils.getNode(obj)
            if ((node != null)) {
                line = Integer.valueOf(node.startLine)
            }
        }
        return this.report(message, severity, line, obj, null)
    }

    /**
     * Report a warning or error on the specified parse tree object in the
     * current resource.
     * The caller should not throw an exception so execution can continue.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     * @param message The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param resource The resource.
     */
    protected fun report(message: String, severity: Int, line: Int?, resource: IResource?): String {
        return this.report(message, severity, line, null, resource)
    }

    /**
     * Report an error.
     * @param message The error message.
     */
    protected fun reportError(message: String): String {
        return this.report(message, IMarker.SEVERITY_ERROR, null)
    }

    /**
     * Report an error on the specified parse tree object.
     * @param object The parse tree object.
     * @param message The error message.
     */
    protected fun reportError(`object`: EObject?, message: String): String {
        return this.report(message, IMarker.SEVERITY_ERROR, `object`)
    }

    /**
     * Report a warning on the specified parse tree object.
     * @param object The parse tree object.
     * @param message The error message.
     */
    protected fun reportWarning(`object`: EObject?, message: String): String {
        return this.report(message, IMarker.SEVERITY_WARNING, `object`)
    }

    /**
     * Reduce the indentation by one level for generated code
     * in the default code buffer.
     */
    protected fun unindent(builder: StringBuilder = this.code) {
        var indent = indentation[builder] ?: return
        val end = indent.length - 4
        indent = indent.substring(0, max(0, end))
        indentation[builder] = indent
    }

    /**
     * Create a list of default parameter initializers in target code.
     *
     * @param param The parameter to create initializers for
     * @return A list of initializers in target code
     */
    protected fun getInitializerList(param: Parameter?): LinkedList<String> {
        val list = LinkedList<String>()
        for (i in param?.init.orEmpty()) {
            if (ASTUtils.isOfTimeType(param)) {
                list.add(this.getTargetTime(i))
            } else {
                list.add(getTargetValue(i))
            }
        }
        return list
    }

    /**
     * Create a list of state initializers in target code.
     *
     * @param this@getInitializerList The state variable to create initializers for
     * @return A list of initializers in target code
     */
    protected val StateVar?.initializerList: List<String>?
        get() = if (ASTUtils.isInitialized(this)) {
            this?.init.orEmpty().map {
                when {
                    it.parameter != null        -> (getTargetReference(it.parameter))
                    ASTUtils.isOfTimeType(this) -> (this@GeneratorBase.getTargetTime(it))
                    else                        -> (getTargetValue(it))
                }
            }
        } else {
            null
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
    protected fun getInitializerList(param: Parameter?, i: Instantiation?): List<String>? {
        if (i === null || param === null) {
            return null
        }

        val assignments = i.parameters.filter { it.lhs === param }

        return if (assignments.isEmpty()) {
            // the parameter was not overwritten in the instantiation
            param.initializerList()
        } else {
            // the parameter was overwritten in the instantiation
            val ofTimeType = param.isOfTimeType
            assignments.firstOrNull()?.rhs.orEmpty().map {
                if (ofTimeType) init.targetTime
                else init.targetValue
            }
        }
    }

    /**
     * Generate target code for a parameter reference.
     *
     * @param param The parameter to generate code for
     * @return Parameter reference in target code
     */
    protected open fun getTargetReference(param: Parameter): String {
        return param.name
    }

    /**
     * If the argument is a multiport, return a list of strings
     * describing the width of the port, and otherwise, return null.
     * If the list is empty, then the width is variable (specified
     * as '[]'). Otherwise, it is a list of integers and/or parameter
     * references obtained by getTargetReference().
     * @param variable The port.
     * @return The width specification for a multiport or null if it is
     * not a multiport.
     */
    protected fun multiportWidthSpec(variable: Variable?): List<String>? {
        if (variable !is Port || variable.widthSpec == null) return null
        val result = LinkedList<String>()
        if (!variable.widthSpec.isOfVariableLength) {
            for (term in variable.widthSpec.terms) {
                if (term.parameter !== null) {
                    result.add(getTargetReference(term.parameter))
                } else {
                    result.add(term.width.toString())
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
    protected fun multiportWidthExpression(variable: Variable?): String? =
        multiportWidthSpec(variable)?.joinToString(separator = " + ")


    /**
     * Return true if the specified port is a multiport.
     * @param port The port.
     * @return True if the port is a multiport.
     */
    protected fun isMultiport(port: Port): Boolean = port.widthSpec != null

    /**
     * Get textual representation of a time in the target language.
     * This is a separate function from
     * getTargetTime to avoid producing invalid RTI
     * code for targets that override timeInTargetLanguage
     * to return a C-incompatible time type.
     *
     * @param d A time AST node
     * @return An RTI-compatible (ie. C target) time string
     */
    protected fun getRTITime(d: Delay): String {
        if (d.parameter !== null) {
            return d.toText
        }

        val time = TimeValue(d.interval.toLong(), d.unit)

        return if (time.unit != TimeUnit.NONE) {
            time.unit.name + '(' + time.time + ')'
        } else {
            time.time.toString()
        }
    }

    /**
     * Analyze the resource (the .lf file) that is being parsed
     * to determine whether code is being mapped to single or to
     * multiple target machines. If it is being mapped to multiple
     * machines, then set the 'federates' list, the 'federateIDs'
     * map, and the 'federationRTIHost' and 'federationRTIPort'
     * variables.
     *
     * In addition, analyze the connections between federates.
     * Ensure that every cycle has a non-zero delay (microstep
     * delays will not be sufficient). Construct the dependency
     * graph between federates. And replace connections between
     * federates with a pair of reactions, one triggered by
     * the sender's output port, and the other triggered by
     * an action.
     *
     * This class is target independent, so the target code
     * generator still has quite a bit of work to do.
     * It needs to provide the body of the sending and
     * receiving reactions. It also needs to provide the
     * runtime infrastructure that uses the dependency
     * information between federates. See the C target
     * for a reference implementation.
     */
    private fun analyzeFederates(): FederateInstance? {
        // Next, if there actually are federates, analyze the topology
        // interconnecting them and replace the connections between them
        // with an action and two reactions.
        val mainDefn = this.mainDef?.reactorClass.toDefinition()

        if (this.mainDef === null || !mainDefn.isFederated) {
            // Ensure federates is never empty.
            val federateInstance = FederateInstance(null, 0, 0, this)
            federates.add(federateInstance)
            federateByID[0] = federateInstance
        } else {
            // The Lingua Franca program is federated
            isFederated = true
            if (mainDefn?.host != null) {
                // Get the host information, if specified.
                // If not specified, this defaults to 'localhost'
                if (mainDefn.host.addr != null) {
                    federationRTIProperties["host"] = mainDefn.host.addr
                }
                // Get the port information, if specified.
                // If not specified, this defaults to 14045
                if (mainDefn.host.port != 0) {
                    federationRTIProperties["port"] = mainDefn.host.port
                }
                // Get the user information, if specified.
                if (mainDefn.host.user !== null) {
                    federationRTIProperties["user"] = mainDefn.host.user
                }
                // Get the directory information, if specified.
                /* FIXME
                 * if (mainDef.reactorClass.host.dir !== null) {
                 *     federationRTIProperties.put('dir', mainDef.reactorClass.host.dir)
                 * }
                 */
            }

            // Create a FederateInstance for each top-level reactor.
            for (instantiation in mainDefn.allInstantiations) {
                var bankWidth = ASTUtils.width(instantiation.widthSpec)
                if (bankWidth < 0) {
                    reportError(instantiation, "Cannot determine bank width!")
                    // Continue with a bank width of 1.
                    bankWidth = 1
                }
                // Create one federate instance for each reactor instance in the bank of reactors.
                val federateInstances = LinkedList<FederateInstance>()
                for (i in 0..bankWidth) {
                    // Assign an integer ID to the federate.
                    val federateID = federates.size
                    val federateInstance = FederateInstance(instantiation, federateID, i, this)
                    federateInstance.bankIndex = i
                    federates.add(federateInstance)
                    federateInstances.add(federateInstance)
                    federateByID[federateID] = federateInstance

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
                    federatesByInstantiation = LinkedHashMap()
                }
                federatesByInstantiation!![instantiation] = federateInstances
            }

            // In a federated execution, we need keepalive to be true,
            // otherwise a federate could exit simply because it hasn't received
            // any messages.
            if (federates.size > 1) {
                targetConfig.keepalive = true
            }

            // Analyze the connection topology of federates.
            // First, find all the connections between federates.
            // For each connection between federates, replace it in the
            // AST with an action (which inherits the delay) and two reactions.
            // The action will be physical for physical connections and logical
            // for logical connections.
            val connectionsToRemove = LinkedList<Connection>()
            for (connection in mainDefn.connections) {
                // Each connection object may represent more than one physical connection between
                // federates because of banks and multiports. We need to generate communciation
                // for each of these. This iteration assumes the balance of the connection has been
                // checked.
                var rightIndex = 0
                var rightPort = connection.rightPorts[rightIndex++]
                var rightBankIndex = 0
                var rightChannelIndex = 0
                var rightPortWidth = ASTUtils.width((rightPort.variable as Port).widthSpec)
                for (leftPort in connection.leftPorts) {
                    val leftPortWidth = ASTUtils.width((leftPort.variable as Port).widthSpec)
                    for (leftBankIndex in 0..ASTUtils.width(leftPort.container.widthSpec)) {
                        var leftChannelIndex = 0
                        while (rightPort !== null) {
                            val minWidth =
                                if (leftPortWidth - leftChannelIndex < rightPortWidth - rightChannelIndex)
                                    leftPortWidth - leftChannelIndex
                                else rightPortWidth - rightChannelIndex
                            for (j in 0..minWidth) {

                                // Finally, we have a specific connection.
                                // Replace the connection in the AST with an action
                                // (which inherits the delay) and two reactions.
                                // The action will be physical if the connection physical and
                                // otherwise will be logical.
                                val leftFederate = federatesByInstantiation!!.get(leftPort.container)!!.get(leftBankIndex)
                                val rightFederate = federatesByInstantiation!!.get(rightPort.container)!!.get(rightBankIndex)

                                // Set up dependency information.
                                // FIXME: Maybe we don't need this any more?
                                if (
                                    leftFederate !== rightFederate
                                    && !connection.isPhysical
                                    && targetConfig.coordination !== TargetProperty.CoordinationType.DECENTRALIZED
                                ) {
                                    var dependsOn = rightFederate.dependsOn[leftFederate]
                                    if (dependsOn === null) {
                                        dependsOn = LinkedHashSet<Delay>()
                                        rightFederate.dependsOn[leftFederate] = dependsOn
                                    }
                                    if (connection.delay !== null) {
                                        dependsOn.add(connection.delay)
                                    }
                                    var sendsTo = leftFederate.sendsTo[rightFederate]
                                    if (sendsTo === null) {
                                        sendsTo = LinkedHashSet<Delay>()
                                        leftFederate.sendsTo[rightFederate] = sendsTo
                                    }
                                    if (connection.delay !== null) {
                                        sendsTo.add(connection.delay)
                                    }
                                    // Check for causality loops between federates.
                                    // FIXME: This does not detect cycles involving more than one federate.
                                    val reverseDependency = leftFederate.dependsOn[rightFederate]
                                    if (reverseDependency !== null) {
                                        // Check that at least one direction has a delay.
                                        if (reverseDependency.size == 0 && dependsOn.size == 0) {
                                            // Found a causality loop.
                                            val message = "Causality loop found between federates " +
                                                    leftFederate.name + " and " + rightFederate.name
                                            reportError(connection, message)
                                            // This is a fatal error, so throw an exception.
                                            throw AssertionError(message)
                                        }
                                    }
                                }

                                ASTUtils.makeCommunication(
                                    connection,
                                    leftFederate, leftBankIndex, leftChannelIndex,
                                    rightFederate, rightBankIndex, rightChannelIndex,
                                    this, targetConfig.coordination
                                )

                                leftChannelIndex++
                                rightChannelIndex++
                                if (rightChannelIndex >= rightPortWidth) {
                                    // Ran out of channels on the right.
                                    // First, check whether there is another bank reactor.
                                    when {
                                        rightBankIndex < ASTUtils.width(rightPort.container.widthSpec) - 1 -> {
                                            rightBankIndex++
                                            rightChannelIndex = 0
                                        }
                                        rightIndex >= connection.rightPorts.size()                         -> {
                                            // We are done.
                                            rightPort = null
                                            rightBankIndex = 0
                                            rightChannelIndex = 0
                                        }
                                        else                                                               -> {
                                            rightBankIndex = 0
                                            rightPort = connection.rightPorts[rightIndex++]
                                            rightChannelIndex = 0
                                            rightPortWidth = ASTUtils.width((rightPort.variable as Port).widthSpec)
                                        }
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
            for (connection in connectionsToRemove) {
                // Remove the original connection for the parent.
                mainDefn?.connections?.remove(connection)
            }
        }
    }

    /**
     * Determine which mode the compiler is running in.
     * Integrated mode means that it is running within an Eclipse IDE.
     * Standalone mode means that it is running on the command line.
     */
    private fun setMode(): Mode? {
        val resource = fileConfig?.resource
        mode = when {
            resource?.uri?.isPlatform == true -> Mode.INTEGRATED
            resource?.uri?.isFile == true     -> Mode.STANDALONE
            else                              -> Mode.UNDEFINED.also {
                System.err.println("ERROR: Source file protocol is not recognized: " + resource.uri)
            }
        }
        return mode
    }

    /**
     * Print to stdout information about what source file is being generated,
     * what mode the generator is in, and where the generated sources are to be put.
     */
    open fun printInfo(): String? {
        println("Generating code for: " + fileConfig!!.resource.uri)
        println("******** mode: $mode")
        println("******** source file: " + fileConfig!!.srcFile) // FIXME: redundant
        println("******** generated sources: " + fileConfig!!.srcGenPath)
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
     * @author{Christian Menard <christian.menard></christian.menard>@tu-dresden.de}
     */
    private fun runSubprocess(processBuilder: ProcessBuilder, outStream: List<OutputStream>, errStream: List<OutputStream>): Int {
        val process = processBuilder.start()

        val outThread = Thread {
            val buffer = ByteArray(64)
            var len = process.inputStream.read(buffer)
            while (len != -1) {
                for (os in outStream) {
                    os.write(buffer, 0, len)
                }
                len = process.inputStream.read(buffer)
            }
        }
        outThread.start()

        val errThread = Thread {
            val buffer = ByteArray(64)
            var len = process.errorStream.read(buffer)
            while (len != -1) {
                for (es in errStream) {
                    es.write(buffer, 0, len)
                }
                len = process.errorStream.read(buffer)
            }
        }
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
    abstract fun supportsGenerics(): Boolean
    abstract val targetTimeType: String
    abstract val targetTagType: String
    abstract val targetTagIntervalType: String
    abstract val targetUndefinedType: String

    abstract fun getTargetFixedSizeListType(baseType: String?, size: Int?): String
    abstract fun getTargetVariableSizeListType(baseType: String?): String

    /** Return a string representing the specified type in the target language. */
    protected fun getTargetType(type: InferredType): String = when {
        type.isUndefined        -> this.targetUndefinedType
        type.isTime             -> when {
            type.isFixedSizeList    -> this.getTargetFixedSizeListType(this.targetTimeType, type.listSize)
            type.isVariableSizeList -> this.getTargetVariableSizeListType(this.targetTimeType)
            else                    -> this.targetTimeType
        }
        type.isFixedSizeList    -> this.getTargetFixedSizeListType(type.baseType(), type.listSize)
        type.isVariableSizeList -> this.getTargetVariableSizeListType(type.baseType())
        else                    -> type.toText()
    }

    val StateVar.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Action.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Port.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Type.targetType: String get() = getTargetType(InferredType.fromAST(this))
    val Parameter.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))

    /**
     * Get textual representation of a time in the target language.
     *
     * @param t A time AST node
     * @return A time string in the target language
     */
    protected fun getTargetTime(t: Time): String = timeInTargetLanguage(TimeValue(t.interval.toLong(), t.unit))

    /**
     * Get textual representation of a value in the target language.
     *
     * If the value evaluates to 0, it is interpreted as a normal value.
     *
     * @param v A time AST node
     * @return A time string in the target language
     */
    protected fun getTargetValue(v: Value): String =
        if ((v.time != null)) {
            this.getTargetTime(v.time)
        } else ASTUtils.toText(v)

    /**
     * Get textual representation of a value in the target language.
     *
     * If the value evaluates to 0, it is interpreted as a time.
     *
     * @param v A time AST node
     * @return A time string in the target language
     */
    protected fun getTargetTime(v: Value): String = when {
        v.time != null     -> this.getTargetTime(v.time)
        ASTUtils.isZero(v) -> timeInTargetLanguage(TimeValue(0, TimeUnit.NONE))
        else               -> ASTUtils.toText(v)
    }

    protected fun getTargetTime(d: Delay): String =
        if (d.parameter != null) {
            ASTUtils.toText(d)
        } else {
            timeInTargetLanguage(TimeValue(d.interval.toLong(), d.unit))
        }

    /**
     * Write the source code to file.
     * @param code The code to be written.
     * @param path The file to write the code to.
     */
    protected fun writeSourceCodeToFile(code: ByteArray, path: String) {
        Files.write(Paths.get(path), code)
    }

    companion object {
        /**
         * Constant that specifies how to name generated delay reactors.
         */
        val GEN_DELAY_CLASS_NAME = "__GenDelay"
    }
}
