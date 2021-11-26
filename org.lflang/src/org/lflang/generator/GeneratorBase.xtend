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
import java.util.stream.Collectors
import java.util.regex.Pattern
import org.eclipse.core.resources.IMarker
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ASTUtils
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.MainConflictChecker
import org.lflang.Target
import org.lflang.TargetConfig
import org.lflang.TargetConfig.Mode
import org.lflang.TargetProperty.CoordinationType
import org.lflang.TimeValue
import org.lflang.federated.FedASTUtils
import org.lflang.federated.FederateInstance
import org.lflang.federated.serialization.SupportedSerializers
import org.lflang.graph.InstantiationGraph
import org.lflang.lf.Action
import org.lflang.lf.Delay
import org.lflang.lf.Instantiation
import org.lflang.lf.LfFactory
import org.lflang.lf.Model
import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar
import org.lflang.lf.TimeUnit
import org.lflang.lf.VarRef

import static extension org.lflang.ASTUtils.*
import static extension org.lflang.JavaAstUtils.*

/**
 * Generator base class for specifying core functionality
 * that all code generators should have.
 *
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 */
abstract class GeneratorBase extends JavaGeneratorBase {

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
        
        JavaGeneratorUtils.setTargetConfig(
            context, JavaGeneratorUtils.findTarget(fileConfig.resource), targetConfig, errorReporter
        )

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
        if (mainDef !== null) {
            for (String conflict : new MainConflictChecker(fileConfig).conflicts) {
                errorReporter.reportError(this.mainDef.reactorClass, "Conflicting main reactor in " + conflict);
            }
        }
        
        // If federates are specified in the target, create a mapping
        // from Instantiations in the main reactor to federate names.
        // Also create a list of federate names or a list with a single
        // empty name if there are no federates specified.
        // This must be done before desugaring delays below.
        analyzeFederates()
        
        // Process target files. Copy each of them into the src-gen dir.
        // FIXME: Should we do this here? I think the Cpp target doesn't support
        // the files property and this doesn't make sense for federates the way it is
        // done here.
        copyUserFiles(this.targetConfig, this.fileConfig);

        // Collect reactors and create an instantiation graph. These are needed to figure out which resources we need
        // to validate, which happens in setResources().
        setReactorsAndInstantiationGraph()

        JavaGeneratorUtils.validateImports(context, fileConfig, instantiationGraph, errorReporter)
        val allResources = JavaGeneratorUtils.getResources(reactors)
        resources.addAll(allResources.stream()
            .filter [it | it != fileConfig.resource]
            .map [it | JavaGeneratorUtils.getLFResource(it, fsa, context, errorReporter)]
            .collect(Collectors.toList())
        )
        JavaGeneratorUtils.accommodatePhysicalActionsIfPresent(allResources, target, targetConfig, errorReporter);
        // FIXME: Should the GeneratorBase pull in `files` from imported
        // resources?
        
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
    private def transformDelays() {
         for (r : this.resources) {
             r.eResource.insertGeneratedDelays(this)
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

    // //////////////////////////////////////////
    // // Protected methods.

    /**
     * Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
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
        return isFederated && targetConfig.coordination === CoordinationType.DECENTRALIZED
    }
    
    /**
     * Returns true if the program is federated and uses the centralized
     * coordination mechanism.
     */
    def isFederatedAndCentralized() {
        return isFederated && targetConfig.coordination === CoordinationType.CENTRALIZED
    }

    /**
     * Copy the core files needed to build the RTI within a container.
     *
     * @param the directory where rti.Dockerfile is located.
     * @param the core files used for code generation in the current target.
     */
    def copyRtiFiles(File rtiDir, ArrayList<String> coreFiles) {
        var rtiFiles = newArrayList()
        rtiFiles.addAll(coreFiles)

        // add the RTI files on top of the coreFiles
        rtiFiles.addAll(
            "federated/RTI/rti.h",
            "federated/RTI/rti.c",
            "federated/RTI/CMakeLists.txt"
        )
        fileConfig.copyFilesFromClassPath("/lib/c/reactor-c/core", rtiDir + File.separator + "core", rtiFiles)
    }

    /**
     * Write a Dockerfile for the current federate as given by filename.
     * @param the name given to the docker file (without any extension).
     */
    def writeDockerFile(String dockerFileName) {
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
    def reportCommandErrors(String stderr) {
        // NOTE: If the VS Code branch passes code review, then this function,
        //  parseCommandOutput, and ErrorFileAndLine will be deleted soon after.
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
                      
                    if (originalPath.compareTo(path) != 0) {
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

            if (originalPath.compareTo(path) != 0) {
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

    // //////////////////////////////////////////////////
    // // Private functions
    
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
            if (mainReactor.host !== null) {
                // Get the host information, if specified.
                // If not specified, this defaults to 'localhost'
                if (mainReactor.host.addr !== null) {
                    federationRTIProperties.put('host', mainReactor.host.addr)
                }
                // Get the port information, if specified.
                // If not specified, this defaults to 14045
                if (mainReactor.host.port !== 0) {
                    federationRTIProperties.put('port', mainReactor.host.port)
                }
                // Get the user information, if specified.
                if (mainReactor.host.user !== null) {
                    federationRTIProperties.put('user', mainReactor.host.user)
                }
            }

            // Since federates are always within the main (federated) reactor,
            // create a list containing just that one containing instantiation.
            // This will be used to look up parameter values.
            val context = new ArrayList<Instantiation>();
            context.add(mainDef);

            // Create a FederateInstance for each top-level reactor.
            for (instantiation : mainReactor.allInstantiations) {
                var bankWidth = ASTUtils.width(instantiation.widthSpec, context);
                if (bankWidth < 0) {
                    errorReporter.reportError(instantiation, "Cannot determine bank width! Assuming width of 1.");
                    // Continue with a bank width of 1.
                    bankWidth = 1;
                }
                // Create one federate instance for each instance in a bank of reactors.
                val federateInstances = new ArrayList<FederateInstance>(bankWidth);
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
            // FIXME: This used to Skip banks and just process the individual instances.
            val federateInstances = federatesByInstantiation.get(federateReactor.definition);
            for (federateInstance : federateInstances) {
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
                // FIXME: How to determine the bank index of the source?
                // Need ReactorInstance support for ranges.
                // val sourceBankIndex = (source.getPortInstance().parent.bankIndex >= 0) ? source.getPortInstance().parent.bankIndex : 0
                val sourceBankIndex = 0
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
                    // FIXME: bank indices not handled yet.
                    for (var i = 0; i < source.channelWidth; i++) {
                        FedASTUtils.makeCommunication(
                            source.getPortInstance(),
                            input,
                            connection,
                            sourceFederate,
                            0, // FIXME: source.getPortInstance().parent.bankIndex,
                            source.startChannel + i,
                            destinationFederate,
                            0, // FIXME: input.parent.bankIndex,
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
        println('******** mode: ' + fileConfig.compilerMode)
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
}
