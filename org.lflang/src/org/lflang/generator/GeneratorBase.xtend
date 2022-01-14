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

import java.io.File
import java.nio.file.Files
import java.nio.file.Paths
import java.util.ArrayList
import java.util.HashSet
import java.util.LinkedHashMap
import java.util.LinkedHashSet
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
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.validation.CheckMode
import org.lflang.ASTUtils
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.MainConflictChecker
import org.lflang.Target
import org.lflang.TargetConfig
import org.lflang.TargetConfig.Mode
import org.lflang.TargetProperty
import org.lflang.TargetProperty.CoordinationType
import org.lflang.TimeUnit
import org.lflang.TimeValue
import org.lflang.federated.FedASTUtils
import org.lflang.federated.FederateInstance
import org.lflang.federated.serialization.SupportedSerializers
import org.lflang.graph.InstantiationGraph
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.Code
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
import org.lflang.lf.Value
import org.lflang.lf.VarRef
import org.lflang.lf.Variable
import org.lflang.validation.AbstractLFValidator

import static extension org.lflang.ASTUtils.*
import static extension org.lflang.JavaAstUtils.*

/**
 * Generator base class for shared code between code generators.
 * This extends AbstractLinguaFrancaValidator so that errors can be highlighted
 * in the XText-based IDE.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */
abstract class GeneratorBase extends AbstractLFValidator implements TargetTypes {

    ////////////////////////////////////////////
    //// Public fields.
    
    /**
     * Constant that specifies how to name generated delay reactors.
     */
    public static val GEN_DELAY_CLASS_NAME = "_lf_GenDelay"

    /** 
     * The main (top-level) reactor instance.
     */
    public ReactorInstance main
    
    /** A error reporter for reporting any errors or warnings during the code generation */
    public ErrorReporter errorReporter
    
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
    def TargetConfig getTargetConfig() { return this.targetConfig;}
    
    /**
     * The current file configuration.
     */
    protected var FileConfig fileConfig
    
    /**
     * A factory for compiler commands.
     */
    protected var GeneratorCommandFactory commandFactory   

    /**
     * Collection of generated delay classes.
     */
    val delayClasses = new LinkedHashSet<Reactor>()

    /**
     * Definition of the main (top-level) reactor.
     * This is an automatically generated AST node for the top-level
     * reactor.
     */
    protected Instantiation mainDef
    def getMainDef() { return mainDef; }

    /**
     * A list of Reactor definitions in the main resource, including non-main 
     * reactors defined in imported resources. These are ordered in the list in
     * such a way that each reactor is preceded by any reactor that it instantiates
     * using a command like `foo = new Foo();`
     */
    protected var List<Reactor> reactors = new ArrayList
    
    /**
     * The set of resources referenced reactor classes reside in.
     */
    protected var Set<LFResource> resources = newLinkedHashSet
    
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
     * Map from reactions to bank indices
     */
    protected var Map<Reaction,Integer> reactionBankIndices = null

    /**
     * Keep a unique list of enabled serializers
     */
    public var HashSet<SupportedSerializers> enabledSerializers = new HashSet<SupportedSerializers>();

    /**
     * Indicates whether or not the current Lingua Franca program
     * contains a federation.
     */
    public var boolean isFederated = false

    // //////////////////////////////////////////
    // // Target properties, if they are included.
    /**
     * A list of federate instances or a list with a single empty string
     * if there are no federates specified. FIXME: Why put a single empty string there? It should be just empty...
     */
    public var List<FederateInstance> federates = new ArrayList<FederateInstance>

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
     * Create a new GeneratorBase object.
     */
    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        this.fileConfig = fileConfig
        this.topLevelName = fileConfig.name
        this.errorReporter = errorReporter
        this.commandFactory = new GeneratorCommandFactory(errorReporter, fileConfig)
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
     * Set the appropriate target properties based on the target properties of
     * the main .lf file.
     */
    protected def void setTargetConfig(LFGeneratorContext context) {

        val target = fileConfig.resource.findTarget
        if (target.config !== null) {
            // Update the configuration according to the set target properties.
            TargetProperty.set(this.targetConfig, target.config.pairs ?: emptyList, errorReporter)
        }

        // Accommodate the physical actions in the main .lf file
        accommodatePhysicalActionsIfPresent(fileConfig.resource);

        // Override target properties if specified, e.g. as command line arguments.
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
        if (context.args.containsKey(TargetProperty.KEEPALIVE.description)) {
            targetConfig.keepalive = Boolean.parseBoolean(
                context.args.getProperty(TargetProperty.KEEPALIVE.description));
        }
    }

    /**
     * Look for physical actions in 'resource'.
     * If found, take appropriate actions to accommodate.
     *
     * Set keepalive to true.
     */
    protected def void accommodatePhysicalActionsIfPresent(Resource resource) {
        if (!target.setsKeepAliveOptionAutomatically) {
            return; // nothing to do
        }

        // If there are any physical actions, ensure the threaded engine is used and that
        // keepalive is set to true, unless the user has explicitly set it to false.
        for (action : resource.allContents.toIterable.filter(Action)) {
            if (action.origin == ActionOrigin.PHYSICAL) {
                // Check if the user has explicitly set keepalive to false or true
                if (!targetConfig.setByUser.contains(TargetProperty.KEEPALIVE)
                    && targetConfig.keepalive == false
                ) {
                    // If not, set it to true
                    targetConfig.keepalive = true
                    errorReporter.reportWarning(
                        action,
                        '''Setting «TargetProperty.KEEPALIVE.displayName» to true because of «action.name».«
                        » This can be overridden by setting the «TargetProperty.KEEPALIVE.description»«
                        » target property manually.'''
                    );
                }
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
    def void doGenerate(Resource resource, IFileSystemAccess2 fsa, LFGeneratorContext context) {
        
        setTargetConfig(context)

        fileConfig.cleanIfNeeded()

        printInfo()

        // Clear any IDE markers that may have been created by a previous build.
        // Markers mark problems in the Eclipse IDE when running in integrated mode.
        if (errorReporter instanceof EclipseErrorReporter) {
            errorReporter.clearMarkers()
        }
        
        ASTUtils.setMainName(fileConfig.resource, fileConfig.name)
        
        createMainInstance()

        // Check if there are any conflicting main reactors elsewhere in the package.
        if (context.mode == Mode.STANDALONE && mainDef !== null) {
            for (String conflict : new MainConflictChecker(fileConfig).conflicts) {
                errorReporter.reportError(this.mainDef.reactorClass, "Conflicting main reactor in " + conflict);
            }
        }
        
        // If federates are specified in the target, create a mapping
        // from Instantiations in the main reactor to federate names.
        // Also create a list of federate names or a list with a single
        // empty name if there are no federates specified.
        // This must be done before desugaring delays below.
        analyzeFederates(context)
        
        // Process target files. Copy each of them into the src-gen dir.
        // FIXME: Should we do this here? I think the Cpp target doesn't support
        // the files property and this doesn't make sense for federates the way it is
        // done here.
        copyUserFiles(this.targetConfig, this.fileConfig);

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

        if (!enabledSerializers.isNullOrEmpty) {
            // If serialization support is
            // requested by the programmer
            // enable support for them.
            enableSupportForSerialization(context.cancelIndicator);
        }


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
             r.eResource.insertGeneratedDelays(this)
        }
    }

    /**
     * Update the class variable that lists all the involved resources. Also report validation problems of imported 
     * resources at the import statements through those failing resources are reached.
     * 
     * @param context The context providing the cancel indicator used by the validator.
     */
    protected def setResources(LFGeneratorContext context) {
        val fsa = this.fileConfig.fsa;
        val validator = (this.fileConfig.resource as XtextResource).resourceServiceProvider.resourceValidator
        if (mainDef !== null) {
            reactors.add(mainDef.reactorClass as Reactor);
            this.resources.add(
                new LFResource(
                    mainDef.reactorClass.eResource,
                    this.fileConfig,
                    this.targetConfig));
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
                                        errorReporter.reportError(imp, '''Unresolved compilation issues in '«imp.importURI»'.''')
                                        tainted.add(decl.eResource)
                                    }
                                }
                            }
                        }
                    }
                    // Read the target property of the imported file
                    val target = res.findTarget
                    var targetConfig = new TargetConfig();
                    if (target.config !== null) {
                        TargetProperty.set(targetConfig, target.config.pairs ?: emptyList, errorReporter);
                    }
                    val fileConfig = new FileConfig(res, fsa, context);
                    // Add it to the list of LFResources
                    this.resources.add(
                        new LFResource(
                            res,
                            fileConfig,
                            targetConfig)
                    );

                    // Accommodate the physical actions in the imported .lf file
                    accommodatePhysicalActionsIfPresent(res);
                    // FIXME: Should the GeneratorBase pull in `files` from imported
                    // resources? If so, uncomment the following line.
                    // copyUserFiles(targetConfig, fileConfig);
                }
            }
        }
    }

    /**
     * Copy all files listed in the target property `files` into the
     * src-gen folder of the main .lf file.
     *
     * @param targetConfig The targetConfig to read the `files` from.
     * @param fileConfig The fileConfig used to make the copy and resolve paths.
     */
    protected def copyUserFiles(TargetConfig targetConfig, FileConfig fileConfig) {
        // Make sure the target directory exists.
        val targetDir = this.fileConfig.getSrcGenPath
        Files.createDirectories(targetDir)

        for (filename : targetConfig.fileNames) {
            val relativeFileName = fileConfig.copyFileOrResource(
                    filename,
                    fileConfig.srcFile.parent,
                    targetDir);
            if (relativeFileName.isNullOrEmpty) {
                errorReporter.reportError(
                    "Failed to find file " + filename + " specified in the" +
                    " files target property."
                )
            } else {
                this.targetConfig.filesNamesWithoutPath.add(
                    relativeFileName
                );
            }
        }
    }

    /**
     * Return true if errors occurred in the last call to doGenerate().
     * This will return true if any of the reportError methods was called.
     * @return True if errors occurred.
     */
    def errorsOccurred() {
        return errorReporter.getErrorsOccurred();
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
     * @param reference The reference to the variable.
     */
    def String generateVarRef(VarRef reference) {
        var prefix = "";
        if (reference.container !== null) {
            prefix = reference.container.name + "."
        }
        return prefix + reference.variable.name
    }

    /**
     * Generate code for referencing a port possibly indexed by
     * a bank index and/or a multiport index. This assumes the target language uses
     * the usual array indexing [n] for both cases. If not, this needs to be overridden
     * by the target code generator.  If the provided reference is not a port, then
     * this return the string "ERROR: not a port.".
     * @param reference The reference to the port.
     * @param bankIndex A bank index or null or negative if not in a bank.
     * @param multiportIndex A multiport index or null or negative if not in a multiport.
     */
    def String generatePortRef(VarRef reference, Integer bankIndex, Integer multiportIndex) {
        if (!(reference.variable instanceof Port)) {
            return "ERROR: not a port.";
        }
        var prefix = "";
        if (reference.container !== null) {
            var bank = "";
            if (reference.container.widthSpec !== null && bankIndex !== null && bankIndex >= 0) {
                bank = "[" + bankIndex + "]";
            }
            prefix = reference.container.name + bank + "."
        }
        var multiport = "";
        if ((reference.variable as Port).widthSpec !== null && multiportIndex !== null && multiportIndex >= 0) {
            multiport = "[" + multiportIndex + "]";
        }
        return prefix + reference.variable.name + multiport;
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
     * Mark the specified reaction to belong to only the specified
     * bank index. This is needed because reactions cannot declare
     * a specific bank index as an effect or trigger. Reactions that
     * send messages between federates, including absent messages,
     * need to be specific to a bank member.
     * @param The reaction.
     * @param bankIndex The bank index, or -1 if there is no bank.
     */
    def setReactionBankIndex(Reaction reaction, int bankIndex) {
        if (bankIndex >= 0) {
            if (reactionBankIndices === null) {
                reactionBankIndices = new LinkedHashMap<Reaction,Integer>()
            }  
            reactionBankIndices.put(reaction, bankIndex)
        }
    }

    /**
     * Return the reaction bank index.
     * @see setReactionBankIndex(Reaction reaction, int bankIndex)
     * @param The reaction.
     * @return The reaction bank index, if one has been set, and -1 otherwise.
     */
    def int getReactionBankIndex(Reaction reaction) {
        if (reactionBankIndices === null) return -1
        if (reactionBankIndices.get(reaction) === null) return -1
        return reactionBankIndices.get(reaction)
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
            if (time.unit !== null) {
                return time.unit.cMacroName + '(' + time.magnitude + ')'
            } else {
                return time.magnitude.toString()
            }
        }
        return "0" // FIXME: do this or throw exception?
    }

    // note that this is moved out by #544
    final def String cMacroName(TimeUnit unit) {
        return unit.canonicalName.toUpperCase
    }

    /**
     * Run the custom build command specified with the "build" parameter.
     * This command is executed in the same directory as the source file.
     * 
     * The following environment variables will be available to the command:
     * 
     * * LF_CURRENT_WORKING_DIRECTORY: The directory in which the command is invoked.
     * * LF_SOURCE_DIRECTORY: The directory containing the .lf file being compiled.
     * * LF_SOURCE_GEN_DIRECTORY: The directory in which generated files are placed.
     * * LF_BIN_DIRECTORY: The directory into which to put binaries.
     * 
     */
    protected def runBuildCommand() {
        var commands = new ArrayList
        for (cmd : targetConfig.buildCommands) {
            val tokens = newArrayList(cmd.split("\\s+"))
            if (tokens.size > 0) {
                val buildCommand = commandFactory.createCommand(
                    tokens.head,
                    tokens.tail.toList,
                    this.fileConfig.srcPath
                )
                // If the build command could not be found, abort.
                // An error has already been reported in createCommand.
                if (buildCommand === null) {
                    return
                }
                commands.add(buildCommand)
            }
        }

        for (cmd : commands) {
            // execute the command
            val returnCode = cmd.run()

            if (returnCode != 0 && fileConfig.context.mode === Mode.STANDALONE) {
                errorReporter.reportError('''Build command "«targetConfig.buildCommands»" returns error code «returnCode»''')
                return
            }
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (cmd.errors.toString.length > 0 && fileConfig.context.mode !== Mode.STANDALONE) {
                reportCommandErrors(cmd.errors.toString())
                return
            }
        }
    }

    // //////////////////////////////////////////
    // // Protected methods.

    /**
     * Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
    }

    /**
     * Return the target.
     */
    def findTarget(Resource resource) {
        var TargetDecl targetDecl
        for (t : resource.allContents.toIterable.filter(TargetDecl)) {
            if (targetDecl !== null) {
                throw new InvalidSourceException("There is more than one target!") // FIXME: check this in validator
            }
            targetDecl = t
        }
        if (targetDecl === null) {
            throw new InvalidSourceException("No target found!")
        }
        targetDecl
    }

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type The type.
     * @param isPhysical Indicates whether or not the connection is physical
     * @param serializer The serializer used on the connection.
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
        InferredType type,
        boolean isPhysical,
        SupportedSerializers serializer
    ) {
        throw new UnsupportedOperationException("This target does not support network connections between federates.")
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
     * @param serializer The serializer used on the connection.
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
        Delay delay,
        SupportedSerializers serializer
    ) {
        throw new UnsupportedOperationException("This target does not support network connections between federates.")
    }
    
    /**
     * Generate code for the body of a reaction that waits long enough so that the status
     * of the trigger for the given port becomes known for the current logical time.
     * 
     * @param port The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     *  that have port as their trigger or source
     */
    def String generateNetworkInputControlReactionBody(
        int receivingPortID,
        TimeValue maxSTP
    ) {
        throw new UnsupportedOperationException("This target does not support network connections between federates.")
    }    
    
    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     * 
     * @param port The port to generate the control reaction for
     * @param portID The ID assigned to the port in the AST transformation
     * @param receivingFederateID The ID of the receiving federate
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel if a multiport
     * @param delay The delay value imposed on the connection using after
     */
    def String generateNetworkOutputControlReactionBody(
        VarRef port,
        int portID,
        int receivingFederateID,
        int sendingBankIndex,
        int sendingChannelIndex,
        Delay delay
    ) {
        throw new UnsupportedOperationException("This target does not support network connections between federates.")
    }

    /**
     * Add necessary code to the source and necessary build support to
     * enable the requested serializations in 'enabledSerializations'
     */
    def void enableSupportForSerialization(CancelIndicator cancelIndicator) {
        throw new UnsupportedOperationException(
            "Serialization is target-specific "+
            " and is not implemented for the "+target.toString+" target."
        );
    }
    
    /**
     * Returns true if the program is federated and uses the decentralized
     * coordination mechanism.
     */
    def isFederatedAndDecentralized() {
        if (isFederated &&
            targetConfig.coordination === CoordinationType.DECENTRALIZED) {
            return true
        }
        return false
    }
    
    /**
     * Returns true if the program is federated and uses the centralized
     * coordination mechanism.
     */
    def isFederatedAndCentralized() {
        if (isFederated &&
            targetConfig.coordination === CoordinationType.CENTRALIZED) {
            return true
        }
        return false
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param The name of the docker file.
     * @param The name of the federate.
     */
    def writeDockerFile(String dockerFileName, String federateName) {
        throw new UnsupportedOperationException("This target does not support docker file generation.")
    }


    /**
     * Parsed error message from a compiler is returned here.
     */
    static class ErrorFileAndLine {
        public var filepath = null as String
        public var line = "1"
        public var character = "0"
        public var message = ""
        public var isError = true // false for a warning.
        override String toString() {
          return (isError ? "Error" : "Non-error") + " at " + line + ":" + character + " of file " + filepath + ": " + message;
        }
    }

    /**
     * Generate any preamble code that appears in the code generated
     * file before anything else.
     */
    protected def void generatePreamble() {
        prComment("Code generated by the Lingua Franca compiler from:")
        prComment("file:/" +FileConfig.toUnixString(fileConfig.srcFile))
        val models = new LinkedHashSet<Model>

        for (r : this.reactors ?: emptyList) {
            // The following assumes all reactors have a container.
            // This means that generated reactors **have** to be
            // added to a resource; not doing so will result in a NPE.
            models.add(r.toDefinition.eContainer as Model)
        }
        // Add the main reactor if it is defined
        if (this.mainDef !== null) {
            val mainModel = this.mainDef.reactorClass.toDefinition.eContainer as Model
            models.add(mainModel)
            for (p : mainModel.preambles) {
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
     * marks on the appropriate lines. This should not be called in standalone
     * mode.
     * 
     * @param stderr The output on standard error of executing a command.
     */
    def reportCommandErrors(String stderr) {
        // First, split the message into lines.
        val lines = stderr.split("\\r?\\n")
        var message = new StringBuilder()
        var lineNumber = null as Integer
        var path = fileConfig.srcFile
        // In case errors occur within an imported file, record the original path.
        val originalPath = path;
        
        var severity = IMarker.SEVERITY_ERROR
        for (line : lines) {
            val parsed = parseCommandOutput(line)
            if (parsed !== null) {
                // Found a new line number designator.
                // If there is a previously accumulated message, report it.
                if (message.length > 0) {
                    if (severity == IMarker.SEVERITY_ERROR)
                        errorReporter.reportError(path, lineNumber, message.toString())
                    else
                        errorReporter.reportWarning(path, lineNumber, message.toString())

                    if (originalPath.toFile != path.toFile) {
                        // Report an error also in the top-level resource.
                        // FIXME: It should be possible to descend through the import
                        // statements to find which one matches and mark all the
                        // import statements down the chain. But what a pain!
                        if (severity == IMarker.SEVERITY_ERROR) {
                            errorReporter.reportError(originalPath, 0, "Error in imported file: " + path)
                        } else {
                            errorReporter.reportWarning(originalPath, 0, "Warning in imported file: " + path)
                        }
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
                // Determine the path within which the error occurred.
                path = Paths.get(parsed.filepath)
            } else {
                // No line designator.
                if (message.length > 0) {
                    message.append("\n")
                } else {
                    if (!line.toLowerCase.contains('error:')) {
                        severity = IMarker.SEVERITY_WARNING
                    }
                }
                message.append(line);
            }
        }
        if (message.length > 0) {
            if (severity == IMarker.SEVERITY_ERROR) {
                errorReporter.reportError(path, lineNumber, message.toString())
            } else {
                errorReporter.reportWarning(path, lineNumber, message.toString())
            }

            if (originalPath.toFile != path.toFile) {
                // Report an error also in the top-level resource.
                // FIXME: It should be possible to descend through the import
                // statements to find which one matches and mark all the
                // import statements down the chain. But what a pain!
                if (severity == IMarker.SEVERITY_ERROR) {
                    errorReporter.reportError(originalPath, 0, "Error in imported file: " + path)
                } else {
                    errorReporter.reportWarning(originalPath, 0, "Warning in imported file: " + path)
                }
            }
        }
    }

    /** If the mode is EPOCH (the code generator is running in an
     *  an Eclipse IDE), then refresh the project. This will ensure that
     *  any generated files become visible in the project.
     */
    protected def refreshProject() {
        if (fileConfig.context.mode == Mode.EPOCH) {
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
        var list = new ArrayList<String>();

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

        var list = new ArrayList<String>();

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
            var list = new ArrayList<String>();
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
        var result = null as List<String>
        if (variable instanceof Port) {
            if (variable.widthSpec !== null) {
                result = new ArrayList<String>()
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
    def boolean isMultiport(Port port) {
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
        if (d.parameter !== null) {
            return d.toText
        }

        return d.time.toTimeValue.timeInTargetLanguage
    }



    /**
     * Remove triggers in each federates' network reactions that are defined in remote federates.
     *
     * This must be done in code generators after the dependency graphs
     * are built and levels are assigned. Otherwise, these disconnected ports
     * might reference data structures in remote federates and cause compile errors.
     *
     * @param instance The reactor instance to remove these ports from if any.
     *  Can be null.
     */
    protected def void removeRemoteFederateConnectionPorts(ReactorInstance instance) {
        if (isFederated) {
            for (federate: federates) {
                // Remove disconnected network triggers from the AST
                federate.removeRemoteFederateConnectionPorts();
                if (instance !== null) {
                    // If passed a reactor instance, also purge the disconnected network triggers
                    // from the reactor instance graph
                    for (reaction: federate.networkReactions) {
                        val networkReaction = instance.lookupReactionInstance(reaction)
                        if (networkReaction !== null) {
                            for (port: federate.remoteNetworkReactionTriggers) {
                                val disconnectedPortInstance = instance.lookupPortInstance(port);
                                if (disconnectedPortInstance !== null) {
                                    networkReaction.removePortInstance(disconnectedPortInstance);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /** Sets the RTI hostname, port and username if given as compiler arguments
     */
    private def setFederationRTIProperties(LFGeneratorContext context) {
        val rtiAddr = context.args.getProperty("rti").toString()
        val pattern = Pattern.compile("([a-zA-Z0-9]+@)?([a-zA-Z0-9]+\\.?[a-z]{2,}|[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+):?([0-9]+)?")
        val matcher = pattern.matcher(rtiAddr)

        if (!matcher.find()) {
            return;
        }

        // the user match group contains a trailing "@" which needs to be removed.
        val userWithAt = matcher.group(1)
        val user = userWithAt === null ? null : userWithAt.substring(0, userWithAt.length() - 1)
        val host = matcher.group(2)
        val port = matcher.group(3)

        if (host !== null) {
            federationRTIProperties.put("host", host)
        } 
        if (port !== null) {
            federationRTIProperties.put("port", port)
        }
        if (user !== null) {
            federationRTIProperties.put("user", user)
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
    private def analyzeFederates(LFGeneratorContext context) {
        // Next, if there actually are federates, analyze the topology
        // interconnecting them and replace the connections between them
        // with an action and two reactions.
        val mainReactor = this.mainDef?.reactorClass.toDefinition

        if (this.mainDef === null || !mainReactor.isFederated) {
            // The program is not federated.
            // Ensure federates is never empty.
            var federateInstance = new FederateInstance(null, 0, 0, this, errorReporter)
            federates.add(federateInstance)
            federateByID.put(0, federateInstance)
        } else {
            // The Lingua Franca program is federated
            isFederated = true
            
            // If the "--rti" flag is given to the compiler, use the argument from the flag.
            if (context.args.containsKey("rti")) {
                setFederationRTIProperties(context)
            } else if (mainReactor.host !== null) {
                // Get the host information, if specified.
                // If not specified, this defaults to 'localhost'
                if (mainReactor.host.addr !== null) {
                    federationRTIProperties.put("host", mainReactor.host.addr)
                }
                // Get the port information, if specified.
                // If not specified, this defaults to 14045
                if (mainReactor.host.port !== 0) {
                    federationRTIProperties.put("port", mainReactor.host.port)
                }
                // Get the user information, if specified.
                if (mainReactor.host.user !== null) {
                    federationRTIProperties.put("user", mainReactor.host.user)
                }
            }

            // Since federates are always within the main (federated) reactor,
            // create a list containing just that one containing instantiation.
            // This will be used to look up parameter values.
            val mainReactorContext = new ArrayList<Instantiation>();
            mainReactorContext.add(mainDef);

            // Create a FederateInstance for each top-level reactor.
            for (instantiation : mainReactor.allInstantiations) {
                var bankWidth = ASTUtils.width(instantiation.widthSpec, mainReactorContext);
                if (bankWidth < 0) {
                    errorReporter.reportError(instantiation, "Cannot determine bank width!");
                    // Continue with a bank width of 1.
                    bankWidth = 1;
                }
                // Create one federate instance for each reactor instance in the bank of reactors.
                val federateInstances = new ArrayList<FederateInstance>();
                for (var i = 0; i < bankWidth; i++) {
                    // Assign an integer ID to the federate.
                    var federateID = federates.size
                    var federateInstance = new FederateInstance(instantiation, federateID, i, this, errorReporter)
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
                        if (federateInstance.host !== null &&
                            federateInstance.host != 'localhost' &&
                            federateInstance.host != '0.0.0.0'
                        ) {
                            federateInstance.isRemote = true;
                        }
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
            targetConfig.keepalive = true

            // Analyze the connection topology of federates.
            // First, find all the connections between federates.
            // For each connection between federates, replace it in the
            // AST with an action (which inherits the delay) and two reactions.
            // The action will be physical for physical connections and logical
            // for logical connections.
            replaceFederateConnectionsWithActions()

            // Remove the connections at the top level
            mainReactor.connections.clear()
        }
    }
    
    /**
     * Replace connections between federates in the AST with actions that
     * handle sending and receiving data.
     */
    private def replaceFederateConnectionsWithActions() {
        val mainReactor = this.mainDef?.reactorClass.toDefinition

        // Each connection in the AST may represent more than one connection between
        // federate instances because of banks and multiports. We need to generate communication
        // for each of these. To do this, we create a ReactorInstance so that we don't have
        // to duplicate the rather complicated logic in that class. We specify a depth of 1,
        // so it only creates the reactors immediately within the top level, not reactors
        // that those contain.
        val mainInstance = new ReactorInstance(mainReactor, errorReporter, 1)

        for (federateReactor : mainInstance.children) {
            // Skip banks and just process the individual instances.
            if (federateReactor.bankIndex > -2) {
                val bankIndex = (federateReactor.bankIndex >= 0)? federateReactor.bankIndex : 0
                val federateInstance = federatesByInstantiation.get(federateReactor.definition).get(bankIndex);
                for (input : federateReactor.inputs) {
                    replaceConnectionFromSource(input, federateInstance, federateReactor, mainInstance)
                }
            }
        }
    }
    
    /**
     * Replace the connections to the specified input port for the specified federate reactor.
     * @param input The input port instance.
     * @param destinationFederate The federate for which this port is an input.
     * @param federateReactor The reactor instance for that federate.
     * @param mainInstance The main reactor instance.
     */
    def void replaceConnectionFromSource(
        PortInstance input, FederateInstance destinationFederate, ReactorInstance federateReactor, ReactorInstance mainInstance
    ) {
        var channel = 0; // Next input channel to be replaced.
        // If the port is not an input, ignore it.
        if (input.isInput) {
            for (source : input.immediateSources()) {
                val sourceBankIndex = (source.getPortInstance().parent.bankIndex >= 0) ? source.getPortInstance().parent.bankIndex : 0
                val sourceFederate = federatesByInstantiation.get(source.getPortInstance().parent.definition).get(sourceBankIndex);

                // Set up dependency information.
                var connection = mainInstance.getConnection(source.getPortInstance(), input)
                if (connection === null) {
                    // This should not happen.
                    errorReporter.reportError(input.definition, "Unexpected error. Cannot find input connection for port")
                } else {
                    if (sourceFederate !== destinationFederate
                            && !connection.physical 
                            && targetConfig.coordination !== CoordinationType.DECENTRALIZED) {
                        // Map the delays on connections between federates.
                        // First see if the cache has been created.
                        var dependsOnDelays = destinationFederate.dependsOn.get(sourceFederate)
                        if (dependsOnDelays === null) {
                            // If not, create it.
                            dependsOnDelays = new LinkedHashSet<Delay>()
                            destinationFederate.dependsOn.put(sourceFederate, dependsOnDelays)
                        }
                        // Put the delay on the cache.
                        if (connection.delay !== null) {
                            dependsOnDelays.add(connection.delay)
                        } else {
                            // To indicate that at least one connection has no delay, add a null entry.
                            dependsOnDelays.add(null)
                        }
                        // Map the connections between federates.
                        var sendsToDelays = sourceFederate.sendsTo.get(destinationFederate)
                        if (sendsToDelays === null) {
                            sendsToDelays = new LinkedHashSet<Delay>()
                            sourceFederate.sendsTo.put(destinationFederate, sendsToDelays)
                        }
                        if (connection.delay !== null) {
                            sendsToDelays.add(connection.delay)
                        } else {
                            // To indicate that at least one connection has no delay, add a null entry.
                            sendsToDelays.add(null)
                        }
                    }

                    // Make one communication for each channel.
                    // FIXME: There is an opportunity for optimization here by aggregating channels.
                    for (var i = 0; i < source.channelWidth; i++) {
                        FedASTUtils.makeCommunication(
                            source.getPortInstance(),
                            input,
                            connection,
                            sourceFederate,
                            source.getPortInstance().parent.bankIndex,
                            source.startChannel + i,
                            destinationFederate,
                            input.parent.bankIndex,
                            channel + i,
                            this,
                            targetConfig.coordination
                        );
                    }
                    channel += source.channelWidth;
                }
            }
        }
    }

    /**
     * Print to stdout information about what source file is being generated,
     * what mode the generator is in, and where the generated sources are to be put.
     */
    def printInfo() {
        println("Generating code for: " + fileConfig.resource.getURI.toString)
        println('******** mode: ' + fileConfig.context.mode)
        println('******** source file: ' + fileConfig.srcFile) // FIXME: redundant
        println('******** generated sources: ' + fileConfig.getSrcGenPath)
    }

    /**
     * Indicates whether delay banks generated from after delays should have a variable length width.
     * 
     * If this is true, any delay reactors that are inserted for after delays on multiport connections 
     * will have a unspecified variable length width. The code generator is then responsible for inferring the
     * correct width of the delay bank, which is only possible if the precise connection width is known at compile time.
     * 
     * If this is false, the width specification of the generated bank will list all the ports listed on the right
     * side of the connection. This gives the code generator the information needed to infer the correct width at 
     * runtime.
     */
    def boolean generateAfterDelaysWithVariableWidth() { return true }

    /**
     * Get the buffer type used for network messages
     */
    def String getNetworkBufferType() ''''''

    /**
     * Return the Targets enum for the current target
     */
    abstract def Target getTarget()

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

    /**
     * Get textual representation of a time in the target language.
     * 
     * @param t A time AST node
     * @return A time string in the target language
     */
    protected def getTargetTime(Time t) {
        val value = new TimeValue(t.interval, TimeUnit.fromName(t.unit))
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
            val value = TimeValue.ZERO
            return value.timeInTargetLanguage
        }
        return v.toText
    }

    protected def getTargetTime(Delay d) {
        if (d.parameter !== null) {
            return d.toText
        } else {
            return d.time.toTimeValue.timeInTargetLanguage
        }
    }
}
