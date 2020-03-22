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

import java.io.BufferedReader
import java.io.File
import java.io.IOException
import java.io.InputStream
import java.io.InputStreamReader
import java.net.URL
import java.nio.file.Paths
import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.List
import java.util.Map
import java.util.Set
import java.util.regex.Pattern
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.core.runtime.FileLocator
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Delay
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.VarRef

/** Generator base class for shared code between code generators.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Chris Gill, <cdgill@wustl.edu>}
 */
abstract class GeneratorBase {

    ////////////////////////////////////////////
    //// Public fields.

    // Map from time units to an expression that can convert a number in
    // the specified time unit into nanoseconds. This expression may need
    // to have a suffix like 'LL' or 'L' appended to it, depending on the
    // target language, to ensure that the result is a 64-bit long.            
    static public var timeUnitsToNs = #{TimeUnit.NSEC -> 1L,
        TimeUnit.NSECS -> 1L, TimeUnit.USEC -> 1000L, TimeUnit.USECS -> 1000L,
        TimeUnit.MSEC -> 1000000L, TimeUnit.MSECS -> 1000000L,
        TimeUnit.SEC -> 1000000000L, TimeUnit.SECS -> 1000000000L,
        TimeUnit.SECOND -> 1000000000L, TimeUnit.SECONDS -> 1000000000L,
        TimeUnit.MIN -> 60000000000L, TimeUnit.MINS -> 60000000000L,
        TimeUnit.MINUTE -> 60000000000L, TimeUnit.MINUTES -> 60000000000L,
        TimeUnit.HOUR -> 3600000000000L, TimeUnit.HOURS -> 3600000000000L,
        TimeUnit.DAY -> 86400000000000L, TimeUnit.DAYS -> 86400000000000L,
        TimeUnit.WEEK -> 604800000000000L, TimeUnit.WEEKS -> 604800000000000L}
        
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
     *  full filename with the .lf extension.
     */
    protected var String sourceFile
    
    ////////////////////////////////////////////
    //// Target properties, if they are included.
    
    /** A list of federate names or a list with a single empty string
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

    /** The federation RTI host, which defaults to "localhost". */
    protected var federationRTIHost = "localhost"
    
    /** The federation RTI port, which defaults to "15045". */
    protected var federationRTIPort = "15045"

    /** The cmake_include target parameter, or null if there is none. */
    protected String targetCmakeInclude
    
    /** The compile target parameter, or null if there is none. */
    protected String targetCompile
    
    /** The fast target parameter, or false if there is none. */
    protected boolean targetFast = false
    
    /** The value of the keepalive target parameter, or false if there is none. */
    protected boolean targetKeepalive
    
    /** The level of logging or null if not given. */
    protected String targetLoggingLevel

    /** The run target parameter, or null if there is none. */
    protected String targetRun

    /** The threads target parameter, or the default 0 if there is none. */
    protected int targetThreads = 0

    /** The threads timeout parameter, or the default -1 if there is none. */
    protected int targetTimeout = -1

    /** The threads timeout unit parameter, or the default null if there is none. */
    protected TimeUnit targetTimeoutUnit

    ////////////////////////////////////////////
    //// Private fields.

    /** Map from builder to its current indentation. */
    var indentation = new HashMap<StringBuilder, String>()

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
                    case "cmake_include":
                        targetCmakeInclude = param.value.literal.withoutQuotes
                    case "compile":
                        targetCompile = param.value.literal.withoutQuotes
                    case "fast":
                        if (param.value.id.equals('true')) {
                            targetFast = true
                        } else {
                            targetFast = false
                        }
                    case "keepalive":
                        if (param.value.id.equals('true')) {
                            targetKeepalive = true
                        } else {
                            targetKeepalive = false
                        }
                    case "logging":
                        targetLoggingLevel = param.value.id
                    case "run":
                        targetRun = param.value.literal.withoutQuotes
                    case "threads":
                        targetThreads = Integer.decode(param.value.literal)
                    case "timeout": {
                        targetTimeout = param.value.time
                        targetTimeoutUnit = param.value.unit
                    }
                }
            }
        }
        
        this.resource = resource
        // Figure out the file name for the target code from the source file name.
        analyzeResource(resource)
        
        // If federates are specified in the target, create a mapping
        // from Instantiations in the main reactor to federate names.
        // Also create a list of federate names or a list with a single
        // empty name if there are no federates specified.
        // This must be done before desugaring delays below.
        analyzeFederates(resource)

        // Find connections, and see whether they have a delay associated with them.
        // For those that do, remove the connection, and replace it with two reactions
        // and an action. Removal of the connection must occur after iterating to avoid
        // concurrent modification problems.
        val toRemove = new LinkedList<Connection>()
        for (connection : resource.allContents.toIterable.filter(Connection)) {
            if (connection.delay !== null) {
                desugarDelay(connection, connection.delay)
                toRemove.add(connection)
            }
        }
        for (connection : toRemove) {
            val parent = (connection.eContainer as Reactor)
            parent.connections.remove(connection)
        }
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

        if (context instanceof StandaloneContext) {
            println("Standlone>>>>>>>>>>>>>>>>>>")
            if (context.args.containsKey("no-target")) {
                println("No target!")
            }
            if (context.args.containsKey("target_args")) {
                println("Target args:" + context.args.get("target_args"))
            }
        }

        println("Generating code for: " + resource.getURI.toString)
        
        // Find the main reactor and create an AST node for its instantiation.
        for (reactor : resource.allContents.toIterable.filter(Reactor)) {
            if (reactor.isMain) {
                // Creating an definition for the main reactor because there isn't one.
                this.mainDef = LinguaFrancaFactory.eINSTANCE.createInstantiation()
                this.mainDef.setName(reactor.name)
                this.mainDef.setReactorClass(reactor)
            }
        }

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
            if (!reactor.isMain) {
                generateReactor(reactor)
            }
        }
    }
    
    /**
     * Produce a unique identifier within a reactor based on a
     * given based name. If the name does not exists, it is returned;
     * if does exist, an index is appended that makes the name unique.
     * @param reactor The reactor to find a unique identifier within.
     * @param name The name to base the returned identifier on.
     */
    def getUniqueIdentifier(Reactor reactor, String name) {
        val vars = new HashSet<String>();
        reactor.actions.forEach[it | vars.add(it.name)]
        reactor.timers.forEach[it | vars.add(it.name)]
        reactor.parameters.forEach[it | vars.add(it.name)]
        reactor.inputs.forEach[it | vars.add(it.name)]
        reactor.outputs.forEach[it | vars.add(it.name)]
        reactor.states.forEach[it | vars.add(it.name)]
        reactor.instantiations.forEach[it | vars.add(it.name)]
        
        var index = 0;
        var suffix = ""
        var exists = true
        while(exists) {
            val id = name + suffix
            if (vars.exists[it | it.equals(id)]) {
                suffix = "_" + index
                index++
            } else {
                exists = false;
            }
        }
        return name + suffix
    }

    /**
     * Take a connection and replace it with an action and two reactions
     * that implement a delayed transfer between the end points of the 
     * given connection.
     * @param connection The connection to replace.
     * @param delay The delay associated with the connection.
     */
    def desugarDelay(Connection connection, Delay delay) {
        val factory = LinguaFrancaFactory.eINSTANCE
        var type = (connection.rightPort.variable as Port).type
        val action = factory.createAction
        val triggerRef = factory.createVarRef
        val effectRef = factory.createVarRef
        val inRef = factory.createVarRef
        val outRef = factory.createVarRef
        val parent = (connection.eContainer as Reactor)
        val r1 = factory.createReaction
        val r2 = factory.createReaction

        // Name the newly created action; set its delay and type.
        action.name = getUniqueIdentifier(parent, "delay")
        action.minDelay = connection.delay.time
        action.type = type
        action.origin = ActionOrigin.LOGICAL

        // Establish references to the action.
        triggerRef.variable = action
        effectRef.variable = action

        // Establish references to the involved ports.
        inRef.container = connection.leftPort.container
        inRef.variable = connection.leftPort.variable
        outRef.container = connection.rightPort.container
        outRef.variable = connection.rightPort.variable

        // Add the action to the reactor.
        parent.actions.add(action)

        // Configure the first reaction.
        r1.triggers.add(inRef)
        r1.effects.add(effectRef)
        // FIXME: Why the extra semicolon?
        r1.code = generateDelayBody(action, inRef) + ";\n"

        // Configure the second reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        // FIXME: Why the extra semicolon?
        r2.code = generateForwardBody(action, outRef) + ";\n"

        // Add the reactions to the parent.
        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
        parent.reactions.add(r2)
        parent.reactions.add(r1)
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
        
    /** If the argument starts with '{=', then remove it and the last two characters.
     *  @return The body without the code delimiter or the unmodified argument if it
     *  is not delimited.
     */
    static def String removeCodeDelimiter(String code) {
        if (code === null) {
            ""
        } else if (code.startsWith("{=")) {
            code.substring(2, code.length - 2).trim();
        } else {
            code
        }
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
        if (time.unit != TimeUnit.NONE) {
            time.unit.name() + '(' + time.time + ')'
        } else {
            time.time.toString()
        }
    }

    /** Return a string that the target language can recognize as a type
     *  for a time value. This base class returns "instant_t".
     *  Particular target generators will likely need to override
     *  this method to return something acceptable to the target language.
     *  @return The string "instant_t"
     */
    def timeTypeInTargetLanguage() {
        "instant_t"
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
     *  hack-y approach of just trying to execute using a bash shel.
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
            var process = builder.start()
            var returnCode = process.waitFor()
            var stdout = readStream(process.getInputStream())
            if (stdout.length() > 0) {
                println("--- Standard output from command:")
                println(stdout)
                println("--- End of standard output.")
            }
            var stderr = readStream(process.getErrorStream())
            if (stderr.length() > 0) {
                reportError("---Command reports errors:\n" + stderr.toString
                        + "\n--- End of standard error.")
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
            var process = builder.start()
            var returnCode = process.waitFor()
            var stdout = readStream(process.getInputStream())
            if (stdout.length() > 0) {
                println("--- Standard output from bash command:")
                println(stdout)
                println("--- End of standard output.")
            }
            if (returnCode !== 0) {
                reportError("Bash command returns error code " + returnCode)
            }
            var stderr = readStream(process.getErrorStream())
            if (stderr.length() > 0) {
                reportError("---Bash command reports errors:\n" + stderr.toString
                        + "\n--- End of standard standard error.")
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
        String type
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
        String type
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
     *  @param resourceSet The resource set in which to find the file.
     *  @param resolvedURI The URI to import.
     */
    protected def openLFImport(ResourceSet resourceSet, URI resolvedURI) {
        val importResource = resourceSet?.getResource(resolvedURI, true);
        if (importResource === null) {
            throw new Exception("Failed to load resource.")
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
                throw new Exception("Import target " + offendingTarget
                    + " is not an acceptable target in import "
                    + importResource.getURI
                    + ". Acceptable targets are: "
                    + acceptableTargets.join(", ")
                )
            } else {
                // Process any imports that the import has.
                processImports(importResource)
                // Call generateReactor for each reactor contained by the import
                // that is not a main reactor.
                for (reactor : importResource.allContents.toIterable.filter(Reactor)) {
                    if (!reactor.isMain) {
                        println("Including imported reactor: " + reactor.name)
                        generateReactor(reactor)
                    }
                }
            }
        }
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
        for (import : resource.allContents.toIterable.filter(Import)) {
            // Resolve the import as a URI relative to the current resource's URI.
            val URI currentURI = resource?.getURI;
            val URI importedURI = URI?.createFileURI(import.importURI);
            val URI resolvedURI = importedURI?.resolve(currentURI);
            val ResourceSet resourceSet = resource?.resourceSet;
            
            // Check for self import.
            if (resolvedURI.equals(currentURI)) {
                reportError(import,
                    "Recursive imports are not permitted: " + import.importURI)
                return
            }
            try {
                if (import.importURI.endsWith(".lf")) {
                    // Handle Lingua Franca imports.
                    openLFImport(resourceSet, resolvedURI)
                } else {
                    // Handle other supported imports (if any).
                    openForeignImport(import, resourceSet, resolvedURI)
                }
            } catch (Exception ex) {
                reportError(
                    import,
                    "Import error: " + import.importURI +
                    "\nException message: " + ex.message
                )
            }
        }
    }

    /** Read a text file in the classpath and return its contents as a string.
     *  @param filename The file name as a path relative to the classpath.
     *  @return The contents of the file as a String or null if the file cannot be opened.
     */
    protected def readFileInClasspath(String filename) throws IOException {
        var inputStream = this.class.getResourceAsStream(filename)

        if (inputStream === null) {
            return null
        }
        try {
            var resultStringBuilder = new StringBuilder()
            // The following reads a file relative to the classpath.
            // The file needs to be in the src directory.
            var reader = new BufferedReader(new InputStreamReader(inputStream))
            var line = ""
            while ((line = reader.readLine()) !== null) {
                resultStringBuilder.append(line).append("\n");
            }
            return resultStringBuilder.toString();
        } finally {
            inputStream.close
        }
    }

    /** Read the specified input stream until an end of file is encountered
     *  and return the result as a StringBuilder.
     *  @param stream The stream to read.
     *  @return The result as a string.
     */
    protected def readStream(InputStream stream) {
        var reader = new BufferedReader(new InputStreamReader(stream))
        var result = new StringBuilder();
        var line = "";
        while ((line = reader.readLine()) !== null) {
            result.append(line);
            result.append(System.getProperty("line.separator"));
        }
        stream.close()
        reader.close()
        result
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

    /** Report an error.
     *  @param message The error message.
     */
    protected def reportError(String message) {
        System.err.println("ERROR: " + message)
    }

    /** Report an error on the specified parse tree object.
     *  @param object The parse tree object.
     *  @param message The error message.
     */
    protected def reportError(EObject object, String message) {
        generatorErrorsOccurred = true;
        // FIXME: All calls to this should also be checked by the validator (See LinguaFrancaValidator.xtend).
        // In case we are using a command-line tool, we report the line number.
        // The caller should not throw an exception so compilation can continue.
        var node = NodeModelUtils.getNode(object)
        val line = (node === null) ? "unknown" : node.getStartLine
        System.err.println("ERROR: Line " + line + ": " + message)
        // Return a string that can be inserted into the generated code.
        "[[ERROR: " + message + "]]"
    }

    /** Report a warning on the specified parse tree object.
     *  @param object The parse tree object.
     *  @param message The error message.
     */
    protected def reportWarning(EObject object, String message) {
        // FIXME: All calls to this should also be checked by the validator (See LinguaFrancaValidator.xtend).
        // In case we are using a command-line tool, we report the line number.
        // The caller should not throw an exception so compilation can continue.
        var node = NodeModelUtils.getNode(object)
        System.err.println("WARNING: Line " + node.getStartLine() + ": " +
            message)
        // Return an empty string that can be inserted into the generated code.
        ""
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

    /** Given a representation of time that may possibly include units,
     *  return a string for the same amount of time
     *  in terms of the specified baseUnit. If the two units are the
     *  same, or if no time unit is given, return the number unmodified.
     *  @param time The source time.
     *  @param baseUnit The target unit.
     */
    protected def unitAdjustment(TimeOrValue timeOrValue, TimeUnit baseUnit) { // FIXME: likelt needs revision
        if (timeOrValue === null) {
            return '0'
        }
        var timeValue = timeOrValue.time
        var timeUnit = timeOrValue.unit

        if (timeOrValue.parameter !== null) {
            timeUnit = timeOrValue.parameter.unit
            if (timeOrValue.parameter.unit != TimeUnit.NONE) {
                timeValue = timeOrValue.parameter.time
            } else {
                try {
                    timeValue = Integer.parseInt(timeOrValue.parameter.value)
                } catch (NumberFormatException e) {
                    reportError(timeOrValue,
                        "Invalid time value: " + timeOrValue)
                }
            }
        }

        if (timeUnit === TimeUnit.NONE || baseUnit.equals(timeUnit)) {
            return timeValue
        }
        // Convert time to nanoseconds, then divide by base scale.
        return ((timeValue * timeUnitsToNs.get(timeUnit)) /
            timeUnitsToNs.get(baseUnit)).toString

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
        var target = resource.findTarget
        var foundOne = false
        // First, collect the properties of the RTI, if there is one,
        // and create a FederateInstance for each federate.
        for (param : target.config?.pairs ?: emptyList) {
            if (param.name.equals("federates")) {
                for (federate : param.value.keyvalue.pairs) {
                    if (federate.name == "RTI") {
                        for (property : federate.value.keyvalue.pairs) {
                            switch property.name {
                            case "host": 
                                federationRTIHost = 
                                    if (property.value.literal !== null) {
                                        property.value.literal.withoutQuotes
                                    } else {
                                        property.value.id
                                    }
                            case "port":
                                federationRTIPort
                                    = property.value.literal.withoutQuotes
                            }
                        }
                    } else {
                        // Assign an integer ID to the federate.
                        var federateID = federates.length
                        // Add the federate name to the list of names.
                        var federateInstance = new FederateInstance(federate, federateID, this)
                        federates.add(federateInstance)
                        federateByName.put(federate.name, federateInstance)
                        federateByID.put(federateID, federateInstance)
                        foundOne = true
                        
                        if (federateByReactor === null) {
                            federateByReactor = new HashMap<String,FederateInstance>()
                        }
                        for (reactorName : federateInstance.containedReactorNames) {
                            federateByReactor.put(reactorName, federateInstance)
                        }
                    }
                }
            }
        }
        // Next, if there actually are federates, analyze the topology
        // interconnecting them and replace the connections between them
        // with an action and two reactions.
        if (!foundOne) {
            // Ensure federates is never empty.
            var federateInstance = new FederateInstance(null, 0, this)
            federates.add(federateInstance)
            federateByName.put("", federateInstance)
            federateByID.put(0, federateInstance)
        } else {
            // Analyze the connection topology of federates.
            // First, find all the connections between federates.
            // Those that are labeled "physical" create no dependency.
            // Otherwise, there is a dependency. This may have a delay
            // which corresponds to the "lookahead"
            // of HLA.
            // FIXME: If there is no delay, we may have to transmit
            // the microstep, not just the timestamp.
            
            // For each connection between federates, replace it in the
            // AST with an action (which inherits the delay) and two reactions.
            // The action will be physical if the connection physical and
            // otherwise will be logical.
            if (mainDef !== null) {
                var connectionsToRemove = new LinkedList<Connection>()
                for (connection : mainDef.reactorClass.connections) {
                    var leftFederate = federateByReactor.get(connection.leftPort.container.name)
                    var rightFederate = federateByReactor.get(connection.rightPort.container.name)
                    if (leftFederate !== rightFederate) {
                        // Connection spans federates.
                        // First, update the dependencies in the FederateInstances.
                        var dependsOn = rightFederate.dependsOn.get(leftFederate)
                        if (dependsOn === null) {
                            dependsOn = new HashSet<TimeOrValue>()
                            rightFederate.dependsOn.put(leftFederate, dependsOn)
                        }
                        if (connection.delay !== null) {
                            dependsOn.add(connection.delay.time)
                        }
                        // Check for causality loops between federates.
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
                        
                        // Next, replace the connection in the AST with an action
                        // (which inherits the delay) and two reactions.
                        // The action will be physical if the connection physical and
                        // otherwise will be logical.
                        makeCommunication(connection,  leftFederate, rightFederate)
                        
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
    
    /** Analyze the resource (the .lf file) that is being parsed
     *  to generate code to set the following variables:
     *  directory, filename, mode, sourceFile.
     */
    private def analyzeResource(Resource resource) {
        var path = resource.getURI.toString
        if (path.startsWith('platform:')) {
            mode = Mode.INTEGRATED
            var fileURL = FileLocator.toFileURL(new URL(path)).toString
            sourceFile = Paths.get(fileURL.substring(5)).normalize.toString
        } else if (path.startsWith('file:')) {
            mode = Mode.STANDALONE
            sourceFile = Paths.get(path.substring(5)).normalize.toString
        } else {
            System.err.println(
                "ERROR: Source file protocol is not recognized: " + path);
        }
        
        // Strip the filename of the extension.
        var File f = new File(sourceFile);
        filename = f.getName();
        directory = f.getParent();
       
        if (filename.endsWith('.lf')) {
            filename = filename.substring(0, filename.length - 3)
        }
        println('******** filename: ' + filename)
        println('******** sourceFile: ' + sourceFile)
        println('******** directory: ' + directory)
        println('******** mode: ' + mode)
    }
    
    /** Replace the specified connection with a communication between federates.
     *  @param connection The connection.
     *  @param leftFederate The source federate.
     *  @param rightFederate The destination federate.
     */
    private def makeCommunication(
        Connection connection, 
        FederateInstance leftFederate,
        FederateInstance rightFederate
    ) {
        val factory = LinguaFrancaFactory.eINSTANCE
        var type = (connection.rightPort.variable as Port).type
        val action = factory.createAction
        val triggerRef = factory.createVarRef
        val effectRef = factory.createVarRef
        val inRef = factory.createVarRef
        val outRef = factory.createVarRef
        val parent = (connection.eContainer as Reactor)
        val r1 = factory.createReaction
        val r2 = factory.createReaction

        // Name the newly created action; set its delay and type.
        action.name = getUniqueIdentifier(parent, "networkMessage")
        // FIXME: Handle connection delay. E.g.:  
        // action.minTime = connection.delay.time
        action.type = type
        // FIXME: For now, only handling physical connections.
        action.origin = ActionOrigin.PHYSICAL
        
        // Record this action in the right federate.
        // The ID of the receiving port (rightPort) is the position
        // of the action in this list.
        val receivingPortID = rightFederate.networkMessageActions.length
        rightFederate.networkMessageActions.add(action)

        // Establish references to the action.
        triggerRef.variable = action
        effectRef.variable = action

        // Establish references to the involved ports.
        inRef.container = connection.leftPort.container
        inRef.variable = connection.leftPort.variable
        outRef.container = connection.rightPort.container
        outRef.variable = connection.rightPort.variable

        // Add the action to the reactor.
        parent.actions.add(action)

        // Configure the sending reaction.
        r1.triggers.add(inRef)
        r1.effects.add(effectRef)
        r1.code = generateNetworkSenderBody(
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            type
        )

        // Configure the receiving reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        r2.code = generateNetworkReceiverBody(
            action,
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            type
        )

        // Add the reactions to the parent.
        parent.reactions.add(r1)
        parent.reactions.add(r2)
    }

    enum Mode {
        STANDALONE,
        INTEGRATED,
        UNDEFINED
    }
}
