/*
 * Generator for C target.
 */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.io.BufferedReader
import java.io.File
import java.io.FileOutputStream
import java.io.InputStream
import java.io.InputStreamReader
import java.net.URL
import java.nio.file.Paths
import java.util.Collection
import java.util.HashSet
import java.util.Hashtable
import java.util.LinkedList
import java.util.regex.Pattern
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.core.runtime.FileLocator
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionModifier
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

/**
 * Generator for C target.
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill, Mehrdad Niknami
 */
class CGenerator extends GeneratorBase {

    // For each reactor, we collect a set of input and parameter names.
    var reactionCount = 0
    var triggerCount = 0
    var tmpVariableCount = 0

    // Indicator of whether to generate multithreaded code and how many by default.
    var numberOfThreads = 0

    // Place to collect code to initialize the trigger objects for all reactors.
    var initializeTriggerObjects = new StringBuilder()

    // List of deferred assignments to perform in initialize_trigger_objects.
    var deferredInitialize = new LinkedList<InitializeRemoteTriggersTable>();

    // Place to collect code to execute at the start of a time step.
    var startTimeStep = new StringBuilder()

    // Place to collect code to initialize timers for all reactors.
    var startTimers = new StringBuilder()

    /** Generate C code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     *  @param importTable The mapping given by import statements.
     */
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context,
        Hashtable<String, String> importTable) {

        pr(includes)
        _resource = resource

        println("Generating code for: " + resource.getURI.toString)

        var runCommand = newArrayList("bin/" + _filename, "-timeout", "3", "secs")
        var runCommandOverridden = false
        var compileCommand = newArrayList()

        for (target : resource.allContents.toIterable.filter(Target)) {
            if (target.parameters !== null) {
                for (parameter : target.parameters.assignments) {
                    if (parameter.name.equals("threads")) {
                        // This has been checked by the validator.
                        numberOfThreads = Integer.decode(parameter.value)
                        // Set this as the default in the generated code,
                        // but only if it has not been overridden on the command line.
                        pr(startTimers, "if (number_of_threads == 0) {")
                        indent(startTimers)
                        pr(startTimers, "number_of_threads = " + numberOfThreads + ";")
                        unindent(startTimers)
                        pr(startTimers, "}")
                    } else if (parameter.name.equals("run")) {
                        // Strip off enclosing quotation marks and split at spaces.
                        val command = parameter.value.substring(1, parameter.value.length - 1).split(' ')
                        runCommand.clear
                        runCommand.addAll(command)
                        runCommandOverridden = true
                    } else if (parameter.name.equals("compile")) {
                        // Strip off enclosing quotation marks and split at spaces.
                        val command = parameter.value.substring(1, parameter.value.length - 1).split(' ')
                        compileCommand.clear
                        compileCommand.addAll(command)
                    }
                }
            }
        }
        if (numberOfThreads === 0) {
            pr("#include \"reactor.c\"")
        } else {
            pr("#include \"reactor_threaded.c\"")
            if (!runCommandOverridden) {
                runCommand.add("-threads")
                runCommand.add(numberOfThreads.toString())
            }
        }

        // First process all the imports.
        processImports(importTable)

        super.doGenerate(resource, fsa, context, importTable)

        // Generate main instance, if there is one.
        if (this.main !== null) {
            generateReactorInstance(this.main)
        }
    
        // Determine path to generated code
        val cFilename = _filename + ".c";
        var srcFile = resource.getURI.toString;
        var mode = Mode.UNDEFINED;
        
        if (srcFile.startsWith("file:")) { // Called from command line
            srcFile = Paths.get(srcFile.substring(5)).normalize.toString
            mode = Mode.STANDALONE;
        } else if (srcFile.startsWith("platform:")) { // Called from Eclipse
            srcFile = FileLocator.toFileURL(new URL(srcFile)).toString
            srcFile = Paths.get(srcFile.substring(5)).normalize.toString
            mode = Mode.INTEGRATED;
        } else {
            System.err.println("ERROR: Source file protocol is not recognized: " + srcFile);
        }
        
        // Any main reactors in imported files are ignored.        
        if (main !== null) {
            // Generate function to initialize the trigger objects for all reactors.
            pr('void __initialize_trigger_objects() {\n')
            indent()
            pr(initializeTriggerObjects.toString)
            doDeferredInitialize()
            setReactionPriorities(main)
            unindent()
            pr('}\n')

            // Generate function to start timers for all reactors.
            pr("void __start_timers() {")
            indent()
            pr(startTimers.toString)
            unindent()
            pr("}")

            // Generate function to execute at the start of a time step.
            pr('void __start_time_step() {\n')
            indent()
            pr(startTimeStep.toString)
            unindent()
            pr('}\n')
        }

        val srcPath = srcFile.substring(0, srcFile.lastIndexOf(File.separator))
        var srcGenPath = srcPath + File.separator + "src-gen"
        var outPath = srcPath + File.separator + "bin"
        
        // Create output directories if they don't yet exist
        var dir = new File(srcGenPath)
        if (!dir.exists()) dir.mkdirs()
        dir = new File(outPath)
        if (!dir.exists()) dir.mkdirs()
        
        
        // Delete source previous output the LF compiler
        var file = new File(srcPath + File.separator + cFilename)
        if (file.exists) {
            file.delete
        }
        
        // Delete binary previously output by C compiler
        file = new File(outPath + File.separator + _filename)
        if (file.exists) {
            file.delete
        }
        
        // Copy the required library files into the target filesystem.
        var fOut = new FileOutputStream(new File(srcGenPath + File.separator + cFilename));
        fOut.write(getCode().getBytes())

        fOut = new FileOutputStream(new File(srcGenPath + File.separator + "reactor_common.c"));
        fOut.write(readFileInClasspath("/lib/C/reactor_common.c").getBytes())

        fOut = new FileOutputStream(new File(srcGenPath + File.separator + "reactor.h"));
        fOut.write(readFileInClasspath("/lib/C/reactor.h").getBytes())

        if (numberOfThreads === 0) {
            fOut = new FileOutputStream(new File(srcGenPath + File.separator + "reactor.c"));
            fOut.write(readFileInClasspath("/lib/C/reactor.c").getBytes())
        } else {
            fOut = new FileOutputStream(new File(srcGenPath + File.separator + "reactor_threaded.c"));
            fOut.write(readFileInClasspath("/lib/C/reactor_threaded.c").getBytes())
        }

        fOut = new FileOutputStream(new File(srcGenPath + File.separator + "pqueue.c"));
        fOut.write(readFileInClasspath("/lib/C/pqueue.c").getBytes())

        fOut = new FileOutputStream(new File(srcGenPath + File.separator + "pqueue.h"));
        fOut.write(readFileInClasspath("/lib/C/pqueue.h").getBytes())
        
        // Trigger a refresh so Eclipse also sees the generated files in the package explorer.
        if (mode == Mode.INTEGRATED) {
            // Find name of current project
            val id = "((:?[a-z]|[A-Z]|_\\w)*)";
            val pattern = Pattern.compile("platform:" + File.separator + "resource" + File.separator + id +
                File.separator);
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
        
        // Invoke the compiler on the generated code.
        val relativeSrcFilename = "src-gen" + File.separator + cFilename;
        val relativeBinFilename = "bin" + File.separator + _filename;
        // FIXME: Do we want to keep the compileCommand option?
        if (compileCommand.isEmpty()) {
            compileCommand.addAll("gcc", "-O2", relativeSrcFilename, "-o", relativeBinFilename)
            // If threaded computation is requested, add a -pthread option.
            if (numberOfThreads !== 0) {
                compileCommand.add("-pthread")
            }
            // If there is no main reactor, then use the -c flag to prevent linking from occurring.
            // FIXME: we could add a `-c` flag to `lfc` to make this explicit in stand-alone mode.
            // Then again, I think this only makes sense when we can do linking.
            // In any case, a warning is helpful to draw attention to the fact that no binary was produced.
            if (main === null) {
                compileCommand.add("-c")
                if (mode === Mode.STANDALONE) {
                    reportError("ERROR: Did not output executable; no main reactor found.")
                }
            }
        }
        println("In directory: " + srcPath)
        println("Compiling with command: " + compileCommand.join(" "))
        var builder = new ProcessBuilder(compileCommand);
        builder.directory(new File(srcPath));
        var process = builder.start()
        // FIXME: The following doesn't work. Somehow, the command to
        // run the generated code gets executed before the following is printed!
        val code = process.waitFor()
        var stdout = readStream(process.getInputStream())
        var stderr = readStream(process.getErrorStream())
        if (stdout.length() > 0) {
            println("--- Standard output from C compiler:")
            println(stdout)
        }
        if (code !== 0) {
            reportError("Compiler returns error code " + code)
        }
        if (stderr.length() > 0) {
            reportError("Compiler reports errors:\n" + stderr.toString)
        } else {
            println("SUCCESS (compiling generated C code)")
        }
    }

    /** Read the specified input stream until an end of file is encountered
     *  and return the result as a StringBuilder.
     *  @param stream The stream to read.
     *  @return The result as a string.
     */
    private def readStream(InputStream stream) {
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

    // //////////////////////////////////////////
    // // Code generators.
    /** Generate a reactor class definition.
     *  @param reactor The parsed reactor data structure.
     *  @param importTable Substitution table for class names (from import statements).
     */
    override generateReactor(Reactor reactor, Hashtable<String, String> importTable) {
        super.generateReactor(reactor, importTable)

        pr("// =============== START reactor class " + reactor.name)

        // Scan reactions
        var savedReactionCount = reactionCount;

        // Preamble code contains state declarations with static initializers.
        if (reactor.preamble !== null) {
            pr("// *********** From the preamble, verbatim:")
            pr(removeCodeDelimiter(reactor.preamble.code))
            pr("\n// *********** End of preamble.")
        }

        var classInfo = ReactorInfo.get(reactor)

        // Put parameters into a struct and construct the code to go
        // into the preamble of any reaction function to extract the
        // parameters from the struct.
        //val argType = "reactor_instance_" + (reactorClassCount++) + "_self_t"
        val argType = reactor.name.toLowerCase + "_self_t" 
        // Construct the typedef for the "self" struct.
        var body = new StringBuilder()
        // Start with parameters.
        for (parameter : reactor.parameters) {
            prSourceLineNumber(parameter)
            if (parameter.type === null) {
                reportError(parameter, "Parameter is required to have a type: " + parameter.name)
            } else {
                pr(body, getParameterType(parameter) + ' ' + parameter.name + ';');
            }
        }
        // Next handle states.
        for (state : reactor.states) {
            prSourceLineNumber(state)
            if (state.type === null) {
                reportError(state, "State is required to have a type: " + state.name)
            } else {
                pr(body, removeCodeDelimiter(state.type) + ' ' + state.name + ';');
            }
        }
        // Next handle actions.
        for (action : reactor.actions) {
            prSourceLineNumber(action)
            // NOTE: Slightly obfuscate output name to help prevent accidental use.
            pr(body, "trigger_t* __" + action.name + ";")
        }
        // Next handle inputs.
        for (input : reactor.inputs) {
            prSourceLineNumber(input)
            if (input.type === null) {
                reportError(input, "Input is required to have a type: " + input.name)
            } else {
                // NOTE: Slightly obfuscate input name to help prevent accidental use.
                pr(body, removeCodeDelimiter(input.type) + '* __' + input.name + ';');
                pr(body, 'bool* __' + input.name + '_is_present;');
            }
        }
        // Next handle outputs.
        for (output : reactor.outputs) {
            prSourceLineNumber(output)
            if (output.type === null) {
                reportError(output, "Output is required to have a type: " + output.name)
            } else {
                // NOTE: Slightly obfuscate output name to help prevent accidental use.
                pr(body, removeCodeDelimiter(output.type) + ' __' + output.name + ';')
                pr(body, 'bool __' + output.name + '_is_present;')
                // If there are contained reactors that send data via this output,
                // then create a place to put the pointers to the sources of that data.
                var containedSource = classInfo.outputToContainedOutput.get(output)
                if (containedSource !== null) { // FIXME: outputToContainedOutput result appears to be always null
                    pr(body, removeCodeDelimiter(output.type) + '* __' + output.name + '_inside;')
                    pr(body, 'bool* __' + output.name + '_inside_is_present;')
                }
            }
        }
        // Finally, handle reactions that produce outputs sent to inputs
        // of contained reactions.
        for (reaction : reactor.reactions) {
            if (reaction.effects !== null) {
                for (effect : reaction.effects) {
                    if (effect.variable instanceof Input) {
                        val port = effect.variable as Input
                        pr(
                            body,
                            removeCodeDelimiter(port.type) + ' __' + effect.container.name + '_' + port.name + ';'
                        )
                        pr(
                            body,
                            'bool __' + effect.container.name + '_' + port.name + '_is_present;'
                        )
                    }
                }
            }
        }
        if (body.length > 0) {
            selfStructType(reactor)
            pr("typedef struct {")
            indent()
            pr(body.toString)
            unindent()
            pr("} " + argType + ";")
        }

        // Generate reactions
        // For this second pass, restart the reaction count where the first pass started.
        reactionCount = savedReactionCount;
        generateReactions(reactor)
        generateTransferOutputs(reactor)
        pr("// =============== END reactor class " + reactor.name)
        pr("")
    }

    /** Generate reaction functions definition for a reactor.
     *  These functions have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reactor The reactor.
     */
    def generateReactions(Reactor reactor) {
        var reactions = reactor.reactions
        var classInfo = ReactorInfo.get(reactor)
        var reactionIndex = 0;
        for (reaction : reactions) {
            // Create a unique function name for each reaction.
            val functionName = reactor.name.toLowerCase + "_rfunc_" + reactionIndex
            classInfo.targetProperties.put(reaction, functionName) // FIXME

            // Construct the reactionInitialization code to go into
            // the body of the function before the verbatim code.
            // This defines the "self" struct.
            var StringBuilder reactionInitialization = new StringBuilder()
            var structType = selfStructType(reactor)
            if (!hasEmptySelfStruct(reactor)) {
                // A null structType means there are no inputs, state,
                // or anything else. No need to declare it.
                pr(reactionInitialization, structType + "* self = (" + structType + "*)instance_args;")
            }

            // Actions may appear twice, first as a trigger, then with the outputs.
            // But we need to declare it only once. Collect in this data structure
            // the actions that are declared as triggered so that if they appear
            // again with the outputs, they are not defined a second time.
            // That second redefinition would trigger a compile error.
            var actionsAsTriggers = new HashSet<Action>();

            // Next, add the triggers (input and actions; timers are not needed).
            // This defines a local variable in the reaction function whose
            // name matches that of the trigger. If the trigger is an input
            // (not an action), then it also defines a local variable whose
            // name is the input name with suffix "_is_present", a boolean
            // that indicates whether the input is present.
            if (reaction.triggers !== null && reaction.triggers.length > 0) {
                for (VarRef trigger : reaction.triggers) {
                    // val input = getInput(reactor, trigger.variable.name)
                    if (trigger.variable instanceof Input) {
                        generateInputVariablesInReaction(reactionInitialization, trigger.variable as Input)
                    } else if (trigger.variable instanceof Action) {
                        pr(
                            reactionInitialization,
                            "trigger_t* " + trigger.variable.name + ' = self->__' + trigger.variable.name + ';'
                        );
                        actionsAsTriggers.add(trigger.variable as Action);
                        // If the action has a type, create variables for accessing the payload.
                        val type = (trigger.variable as Action).type
                        val payloadPointer = trigger.variable.name + '->payload'
                        // Create the _has_payload variable.
                        pr(reactionInitialization, 'bool ' + trigger.variable.name + '_has_payload = (' + payloadPointer + ' != NULL);')
                        // Create the _payload variable if there is a type.
                        if (type !== null) {
                            // Create the value variable, but initialize it only if the pointer is not null.
                            pr(reactionInitialization, type + ' ' + trigger.variable.name + '_payload;')
                            pr(reactionInitialization, 'if (' + trigger.variable.name + '_has_payload) '
                                + trigger.variable.name + '_payload = *(' + '(' + type + '*)' + payloadPointer + ');'
                            );
                        }
                    } else if (trigger.variable instanceof Output) {
                        // FIXME: triggered by contained output
                        reportError(trigger, "(FIXME) Failed to handle hierarchical reference: " + trigger)
                    }
                }
            } else {
                // No triggers are given, which means react to any input.
                // Declare an argument for every input.
                // NOTE: this does not include contained outputs. Should it? 
                for (input : reactor.inputs) {
                    generateInputVariablesInReaction(reactionInitialization, input)
                }
            }
            // Define argument for non-triggering inputs.
            for (VarRef src : reaction.sources ?: emptyList) {
                // FIXME: handle hierarchical references
                if (src instanceof Input) {
                    generateInputVariablesInReaction(reactionInitialization, src.variable as Input)
                } else {
                    reportError(src, "(FIXME) Failed to handle hierarchical reference: " + src)
                }
            }

            // Define variables for each declared output or action.
            if (reaction.effects !== null) {
                for (effect : reaction.effects) {
                    // val action = getAction(reactor, output)
                    if (effect.variable instanceof Action) {
                        // It is an action, not an output.
                        // If it has already appeared as trigger, do not redefine it.
                        if (!actionsAsTriggers.contains(effect.variable.name)) {
                            pr(reactionInitialization,
                                "trigger_t* " + effect.variable.name + ' = self->__' + effect.variable.name + ';');
                        }
                    } else {
                        if (effect.variable instanceof Output) {
                            generateOutputVariablesInReaction(reactionInitialization, effect.variable as Output)
                        } else if (effect.variable instanceof Input) {
                            // It is the input of a contained reactor.
                            generateVariablesForSendingToContainedReactors(
                                reactionInitialization,
                                effect.container,
                                effect.variable as Input
                            )
                        } else {
                            reportError(reaction, "In generateReactor(): "
                                + effect.variable.name + " is neither an input nor an output."
                            )
                        }
                    }
                }
            }
            pr('void ' + functionName + '(void* instance_args) {')
            indent()
            pr(reactionInitialization.toString)
            // Code verbatim from 'reaction'
            prSourceLineNumber(reaction)
            pr(removeCodeDelimiter(reaction.code))
            unindent()
            pr("}")
            
            // Now generate code for the deadline violation function, if there is one.
            if (reaction.localDeadline !== null) {
                val deadlineFunctionName = 'deadline_function' + reactionCount
                ReactorInfo.get(reactor).targetProperties.put(reaction.localDeadline, deadlineFunctionName) // FIXME

                pr('void ' + deadlineFunctionName + '(void* instance_args) {')
                indent();
                pr(reactionInitialization.toString)
                // Code verbatim from 'deadline'
                prSourceLineNumber(reaction.localDeadline.time)
                pr(removeCodeDelimiter(reaction.localDeadline.deadlineCode))
                unindent()
                pr("}")
            }
            reactionCount++
            reactionIndex++
        }
    }

    /** Generate reaction_t structs, one for each reaction in the
     *  specified reactor instance. The name of the struct will be
     *  uniqueID of the reaction instance.
     *  @param reactorIntance The reactor instance.
     */
    def generateReactionStructs(ReactorInstance reactorInstance) {
        val result = new StringBuilder()
        var reactionIndex = 0;
        for (reaction : reactorInstance.reactions) {
            
            val reactionInstanceName = reaction.uniqueID
            
            var presentPredicates = new LinkedList<String>()
            var triggeredSizesContents = new LinkedList<String>()
            var triggersContents = new LinkedList<String>()
            var destinations = null as Collection<PortInstance>

            // Generate entries for the reaction_t struct that specify how
            // to handle outputs.
            for (port : reaction.dependentPorts) {
                if (port.definition instanceof Output) {
                    // Reaction sends to an output.
                    // First create the array of pointers to booleans indicating
                    // whether an output is produced.
                    presentPredicates.add('&' + selfStructName(reactorInstance)
                        + '.__' + port.name + '_is_present')

                    // Next, create an array trigger_t objects, which are
                    // the triggers that fire if this output is produced.
                    // For each output, obtain the destinations from the parent.
                    var parent = reactorInstance.parent
                    if (parent !== null) {
                        destinations = parent.transitiveClosure(port)
                    } else {
                        // At the top level, where there cannot be any destinations
                        // for an output port.
                        destinations = new LinkedList<PortInstance>()
                    }
               } else {
                    // The reaction is sending data to the input of a contained reactor.
                    // First create the array of pointers to booleans indicating whether
                    // an output is produced.
                    presentPredicates.add('&' + selfStructName(reactorInstance)
                        + '.__' + port.parent.name + '_' + port.name + '_is_present')
                        
                    // Since the port is the input port of a contained reactor,
                    // use that reactor instance to compute the transitive closure.
                    destinations = port.parent.transitiveClosure(port)
                }
                // Append to the array that records the sizes of the trigger arrays.
                triggeredSizesContents.add("" + destinations.size)

                // Then, for each destination connected to this output,
                // create an array of pointers to its trigger_t structs,
                // and collect pointers to each of these arrays.
                if (destinations.size === 0) {
                    triggersContents.add("NULL")
                } else {
                    // FIXME: This ID may exceed some maximum length.
                    var remoteTriggersArrayName = reactionInstanceName + '_'
                            + presentPredicates.size + '_remote_triggers'
                    var inputCount = 0;
                    for (destination : destinations) {
                        deferredInitialize.add(
                            new InitializeRemoteTriggersTable(
                                reactorInstance,
                                remoteTriggersArrayName,
                                (inputCount++),
                                destination
                            )
                        )
                    }
                    pr(result,
                        'trigger_t* ' + remoteTriggersArrayName + '[' + inputCount + '];'
                    )
                    triggersContents.add('&' + remoteTriggersArrayName + '[0]')
                }
            }
            var outputProducedArray = "NULL"
            var triggeredSizesArray = "NULL"
            var triggersArray = "NULL"
            val outputCount = presentPredicates.size
            if (outputCount > 0) {
                outputProducedArray = reactionInstanceName + '_outputs_are_present'
                // Create a array with booleans indicating whether an output has been produced.
                pr(result,
                    'bool* ' + reactionInstanceName + '_outputs_are_present[]' + ' = {'
                    + presentPredicates.join(", ") + '};'
                )
                // Create a array with ints indicating these
                // numbers and assign it to triggered_reactions_sizes
                // field of the reaction_t object.
                triggeredSizesArray = '&' + reactionInstanceName + '_triggered_sizes[0]'
                pr(result,
                    'int ' + reactionInstanceName + '_triggered_sizes' + '[] = {'
                    + triggeredSizesContents.join(", ") + '};'
                )
                // Create an array with pointers to arrays of pointers to trigger_t
                // structs for each input triggered by an output.
                triggersArray = '&' + reactionInstanceName + '_triggers[0]'
                pr(result,
                    'trigger_t** ' + reactionInstanceName + '_triggers' + '[] = {'
                    + triggersContents.join(', ') + '};'
                )
            }
            // Finally, produce the reaction_t struct.          
            // The argument specifying the self struct may be NULL if there
            // is no self struct.
            var selfStructArgument = ", &" + selfStructName(reactorInstance)
            var reactorClass = reactorInstance.definition.reactorClass
            if (hasEmptySelfStruct(reactorClass)) {
                selfStructArgument = ", NULL"
            }
            var deadlineFunctionPointer = ", NULL"
            if (reaction.definition.localDeadline !== null) {
                deadlineFunctionPointer = ", &" + ReactorInfo.get(reactorClass).targetProperties.get(reaction.definition.localDeadline)
            }
            
            // Use the same function name as in generateReactions.
            // FIXME: Fragile!  Find a better way to get agreement on function name.
            val functionName = reactorClass.name.toLowerCase + "_rfunc_" + reactionIndex
            reactionIndex++
            
            // First 0 is an index that specifies priorities based on precedences.
            // It will be set later.
            pr(
                result,
                "reaction_t " + reactionInstanceName + " = {&" + functionName + selfStructArgument + ", 0" // index: index from the topological sort.
                + ", 0" // pos: position used by the pqueue implementation for sorting.
                + ", " + outputCount // num_outputs: number of outputs produced by this reaction.
                + ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
                + ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
                + ", " + triggersArray       // triggered: array of pointers to arrays of triggers.
                + ", false"    // Indicator that the reaction is not running.
                + ", 0LL"      // Local deadline.
                + deadlineFunctionPointer // deadline_violation_handler: Pointer to local handler function.
                + ", 0LL"      // Container deadline.
                + ", NULL"     // Pointer to container deadline violation trigger.
                + ", -1LL"     // violation_handled for container deadlines.
                + "};"
            )
        }
        // This goes directly out to the generated code.
        pr(result.toString())
    }

    /** Generate one reaction function definition for each output of
     *  a reactor that relays data from the output of a contained reactor.
     *  This reaction function transfers the data from the output of the
     *  contained reactor (in the self struct of this reactor labeled as
     *  "inside") to the output of this reactor (also in its self struct).
     *  There needs to be one reaction function
     *  for each such output because these reaction functions have to be
     *  individually invoked after each contained reactor produces an
     *  output that must be relayed.
     *  @param reactor The reactor.
     */
    def generateTransferOutputs(Reactor reactor) {
        var classInfo = ReactorInfo.get(reactor)
        for (output : classInfo.outputToContainedOutput.keySet()) {
            // The following function name will be unique, assuming that
            // reactor class names are unique and within each reactor class,
            // output names are unique.
            // This has to match what's in generateTriggerForTransferOutputs
            val functionName = reactor.name.toLowerCase + "_xfer_outs_" + "_" + output.name
            
            pr('void ' + functionName + '(void* instance_args) {')
            indent()

            // Define the "self" struct. First get its name. Note that this
            // must not be null because there is at least one output.
            var structType = selfStructType(reactor)
            pr(structType + "* self = (" + structType + "*)instance_args;")

            // Transfer the output value from the inside value.
            pr("self->__" + output.name + " = *(self->__" + output.name + "_inside);")
            // Transfer the presence flag from the inside value.
            pr("self->__" + output.name + "_is_present = *(self->__" + output.name + "_inside_is_present);")
            unindent()
            pr("}")
        }
    }

    /** Generate trigger_t objects for transferring outputs from inside a composite
     *  to the outside.  Each trigger_t object is a struct that contains an
     *  array of pointers to reaction_t objects representing
     *  the transfer output.
     *  This also creates the reaction_t object for each transfer outputs.
     *  This object has a pointer to the function to invoke for that
     *  reaction.
     *  @param reactorInstance The instance for which we are generating trigger objects.
     */
    def generateTriggerForTransferOutputs(ReactorInstance reactorInstance) {        
        var outputCount = 0
        val result = new StringBuilder()
        var triggersContents = new LinkedList<String>()
        var triggeredSizesContents = new LinkedList<String>()
        var nameOfSelfStruct = selfStructName(reactorInstance)

        for (output : reactorInstance.outputs) {
            if (output.dependsOnPort !== null) {
                // The output is connected on the inside.
                // Create the reaction and trigger structs for this port.
                
                // The function name for the transfer outputs function:
                // This has to match what's in generateTransferOutputs
                val functionName = 
                    output.parent.definition.reactorClass.name.toLowerCase
                    + "_xfer_outs_" + "_" + output.name

                pr(result, "// --- Reaction and trigger objects for transfer outputs for "
                    + output.getFullName
                )

                // Figure out how many inputs are connected to the output.
                // This is obtained via the container.
                var parent = reactorInstance.parent
                var destinations = null as Collection<PortInstance>
                if (parent !== null) {
                    destinations = parent.transitiveClosure(output)
                } else {
                    // At the top level, where there cannot be any destinations.
                    destinations = new LinkedList<PortInstance>()
                }

                // Append to the array that records the length of each trigger_t array.            
                triggeredSizesContents.add("" + destinations.size)
            
                val structName = triggerStructName(output)

                // Then, for each input connected to this output,
                // find its trigger_t struct. Create an array of pointers
                // to these trigger_t structs, and collect pointers to
                // each of these arrays.
                if (destinations.size === 0) {
                    triggersContents.add("NULL")
                } else {
                    var inputCount = 0;
                    for (destination : destinations) {
                        deferredInitialize.add(
                            new InitializeRemoteTriggersTable(
                                parent,
                                output.uniqueID + '_remote_triggers',
                                (inputCount++),
                                destination
                            )
                        )
                    }
                    pr(result,
                        'trigger_t* ' + output.uniqueID + '_remote_triggers[' + inputCount + '];'
                    )
                    triggersContents.add('&' + output.uniqueID + '_remote_triggers[0]')
                }
                // Next generate the array of booleans which indicates whether outputs are present.
                var outputProducedArray = output.uniqueID + '_outputs_are_present'
                pr(result,
                    'bool* ' + outputProducedArray + '[]' + ' = {' + '&'
                    + nameOfSelfStruct + '.__' + output.name + '_is_present' + '};'
                )
                // Create a array with ints indicating these
                // numbers and assign it to triggered_reactions_sizes
                // field of the reaction_t object.
                var triggeredSizesArray = '&' + output.uniqueID + '_triggered_sizes[0]'
                pr(result,
                    'int ' + output.uniqueID + '_triggered_sizes' + '[] = {' + triggeredSizesContents.join(', ') + '};'
                )
                // Create an array with pointers to arrays of pointers to trigger_t
                // structs for each input triggered by an output.
                var triggersArray = '&' + output.uniqueID + '_triggers[0]'
                pr(result,
                    'trigger_t** ' + output.uniqueID + '_triggers' + '[] = {' + triggersContents.join(', ') + '};'
                )
                // First 0 is an index that specifies priorities based on precedences.
                // It will be set later.
                var reactionInstanceName = output.uniqueID + "_reaction"
                pr(
                    result,
                    "reaction_t " + reactionInstanceName 
                    + " = {&" + functionName + ", &" + nameOfSelfStruct // Function
                    + ", 0" // index: index from the topological sort.
                    + ", 0" // pos: position used by the pqueue implementation for sorting.
                    + ", 1" // num_outputs: number of outputs produced by this reaction. This is just one.
                    + ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
                    + ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
                    + ", " + triggersArray       // triggered: array of pointers to arrays of triggers.
                    + ", false" // Indicator that the reaction is not running.
                    + ", 0LL"   // Local deadline.
                    + ", NULL"  // Pointer to local deadline_violation_handler.
                    + ", 0LL"   // Container deadline.
                    + ", NULL"  // Pointer to container deadline violation trigger.
                    + ", -1LL"  // violation_handled for container deadline.
                    + "};"
                )
                pr(result, 'reaction_t* ' + structName + '_reactions[1] = {&' + reactionInstanceName + '};')

                pr(result, 'trigger_t ' + structName + ' = {')
                indent(result)
                pr(result, structName + '_reactions, 1, 0LL, 0LL, NULL, false')
                unindent(result)
                pr(result, '};')
            
                triggerCount++
            }
            outputCount++
        }        
        // This goes directly out to the generated code.
        if (result.length > 0) {
            pr("// *********** Transfer outputs structures for " + reactorInstance.definition.name)
            pr(result.toString())
        }
    }
    
    /** Generate trigger_t objects, one for
     *  each input, clock, and action of the reactor instance.
     *  Each trigger_t object is a struct that contains an
     *  array of pointers to reaction_t objects representing
     *  reactions triggered by this trigger. The trigger_t object
     *  also provides the length of the array, and if the trigger
     *  is a timer or an action, the offset,
     *  and the period. (The offset and period are zero if the trigger
     *  is not an action or a timer. The period is zero for an action).
     * 
     *  This also creates the reaction_t object for each reaction.
     *  This object has a pointer to the function to invoke for that
     *  reaction.
     *  @param reactorInstance The instance for which we are generating trigger objects.
     *  @return A map of trigger names to the name of the trigger struct.
     */
    def generateTriggerObjects(ReactorInstance reactorInstance) {
        val result = new StringBuilder()

        var count = 0
        // Iterate over triggers (input ports, actions, and timers that trigger reactions).
        for (triggerInstance : reactorInstance.triggers) {
            var trigger = triggerInstance.definition
            var numberOfReactionsTriggered = triggerInstance.dependentReactions.length

            // Collect names of the reaction_t objects that are triggered together.
            var reactionTNames = new LinkedList<String>();

            // Generate reaction_t struct.
            // Along the way, we need to generate its contents, including trigger_t structs.
            for (reactionInstance : triggerInstance.dependentReactions) {                
                pr(result,
                    '// --- Reaction and trigger objects for reaction to trigger ' 
                    + trigger.name + ' of instance ' +    reactorInstance.fullName
                )
                
                val reactionInstanceName = reactionInstance.uniqueID

                // Collect the reaction instance names to initialize the
                // reaction pointer array for the trigger.
                reactionTNames.add('&' + reactionInstanceName)
            }
            // Trigger could be a Timer, Action, or Input
            var triggerStructName = triggerStructName(triggerInstance)

            pr(result,
                'reaction_t* ' + triggerStructName + '_reactions[' + numberOfReactionsTriggered + '] = {' +
                    reactionTNames.join(", ") + '};')
            // Declare a variable with the name of the trigger whose
            // value is a struct.
            pr(result, 'trigger_t ' + triggerStructName + ' = {')
            indent(result)
            if (trigger instanceof Timer || trigger instanceof Input) {
                pr(result,
                    triggerStructName + '_reactions, ' + numberOfReactionsTriggered + ', ' + '0LL, 0LL, NULL, false'
                )
            } else if (trigger instanceof Action) {
                var isPhysical = "true";
                var delay = (trigger as Action).delay
                if (delay === null) {
                    delay = "0LL"
                }
                if ((trigger as Action).modifier == ActionModifier.LOGICAL) {
                    isPhysical = "false";
                }
                pr(result,
                    triggerStructName + '_reactions, ' + numberOfReactionsTriggered + ', ' +
                        delay + ', 0LL, NULL, ' + isPhysical // 0 is ignored since actions don't have a period.
                )
            } else {
                reportError(reactorInstance.definition, "Internal error: Seems to not be an input, timer, or action: "
                    + trigger.name
                )
            }
            unindent(result)
            pr(result, '};')
            // Assignment of the offset and period have to occur after creating
            // the struct because the value assigned may not be a compile-time constant.
            if (trigger instanceof Timer) {
                val timing = (trigger as Timer).timing
                var offset = if (timing === null) { null } else {timing.offset}
                var period = if (timing === null) { null } else {timing.period}

                pr(initializeTriggerObjects, triggerStructName + '.offset = ' + timeInTargetLanguage(offset) + ';')
                pr(initializeTriggerObjects, triggerStructName + '.period = ' + timeInTargetLanguage(period) + ';')

                // Generate a line to go into the __start_timers() function.
                // Note that the delay, the second argument, is zero because the
                // offset is already in the trigger struct.
                pr(startTimers, "__schedule(&" + triggerStructName + ", 0LL, NULL);")
            }
            count++
            triggerCount++
        }
        // This goes directly out to the generated code.
        pr(result.toString())
    }

    /** Return the unique name for the "self" struct of the specified
     *  reactor instance from the instance ID.
     *  @param instance The reactor instance.
     *  @return The name of the self struct.
     */
    static def selfStructName(ReactorInstance instance) {
        return instance.uniqueID + "_self"
    }

    /** Construct a unique type for the "self" struct of the specified
     *  reactor class from the reactor class.
     *  @param instance The reactor instance.
     *  @return The name of the self struct.
     */
    def selfStructType(Reactor reactor) {
        return reactor.name.toLowerCase + "_self_t"
    }

    /** Return the unique name for the trigger_t struct of the specified
     *  trigger instance (input port or action).
     *  @param instance The port or action instance.
     *  @return The name of the trigger struct.
     */
    static def triggerStructName(TriggerInstance<Variable> instance) {
        return instance.uniqueID + "_trigger"
    }

    /** Return true of the given reactor has an empty self struct, false otherwise.
     *  @param reactor A reactor class
     */    
    def hasEmptySelfStruct(Reactor reactor) {
        if (!reactor.parameters.isEmpty || !reactor.states.isEmpty || !reactor.actions.isEmpty ||
            !reactor.inputs.isEmpty || !reactor.outputs.isEmpty) {
            return false
        }
        for (reaction : reactor.reactions ?: emptyList) {
            for (effect : reaction.effects ?: emptyList) {
                // Sending to input of contained reactor
                if (effect.variable instanceof Input) {
                    return false
                }
            }
        }
        return true
    }

    /** Traverse the runtime hierarchy of reaction instances and generate code.
     *  @param instance A reactor instance.
     */
    def void generateReactorInstance(ReactorInstance instance) {
        var reactorClass = instance.definition.reactorClass
        var fullName = instance.fullName
        pr('// ************* Instance ' + fullName + ' of class ' + reactorClass.name)
        
        // Generate the instance struct containing parameters, state variables,
        // and outputs (the "self" struct).
        var nameOfSelfStruct = selfStructName(instance)
        var structType = selfStructType(reactorClass)
        if (!hasEmptySelfStruct(reactorClass)) {
            pr('// --- "self" struct for instance ' + fullName)
            pr(structType + " " + nameOfSelfStruct + ";")
        }

        // Generate code to initialize the "self" struct in the
        // __initialize_trigger_objects function.
        // Create a scope for the parameters in case the names collide with other instances.
        pr(initializeTriggerObjects, "{ // Scope for " + fullName)
        indent(initializeTriggerObjects)
        // Start with parameters.
        for (parameter : instance.parameters) {
            // In case the parameter value refers to a container parameter with the same name,
            // we have to first store the value in a temporary variable, then in the
            // parameter variable.
            var tmpVariableName = '__tmp' + tmpVariableCount++
            pr(initializeTriggerObjects,
                parameter.type + ' ' + tmpVariableName + ' = ' + parameter.value + ';'
            )
            pr(initializeTriggerObjects,
                parameter.type + ' ' + parameter.name + ' = ' + tmpVariableName + ';'
            )
            pr(initializeTriggerObjects,
                nameOfSelfStruct + "." + parameter.name + " = " + parameter.value + ";"
            )
        }
        
        // Next, initialize the "self" struct with state variables.
        // These values may be expressions that refer to the parameter values defined above.
        for (state : reactorClass.states) {
            var value = removeCodeDelimiter(state.value)
            pr(initializeTriggerObjects, nameOfSelfStruct + "." + state.name + " = " + value + ";")
        }
        
        // Generate reaction structs for the instance.
        generateReactionStructs(instance)

        // Generate trigger objects for the instance.
        generateTriggerObjects(instance)

        // Generate trigger objects for transferring outputs of a composite.
        generateTriggerForTransferOutputs(instance)

        // Next, initialize the struct with actions.
        for (action : instance.actions) {
            var triggerStruct = triggerStructName(action)
            pr(initializeTriggerObjects,
                nameOfSelfStruct + '.__' + action.name + ' = &' + triggerStruct + ';'
            )
        }
        // Next, generate the code to initialize outputs and inputs at the start
        // of a time step to be absent.
        for (output : reactorClass.outputs) {
            pr(
                startTimeStep,
                nameOfSelfStruct + '.__' + output.name + '_is_present = false;'
            )
        }
        // Handle reaction local deadlines.
        for (reaction: instance.reactions) {
            if (reaction.definition.localDeadline !== null) {
                pr(initializeTriggerObjects, reactionStructName(reaction) + '.local_deadline
                    = ' + timeInTargetLanguage(reaction.definition.localDeadline.time) + ';')
            }
        }

        // Finally, handle container deadline commands.
        // FIXME: Need deadline instances.
        /* FIXME: Remove this for now. Should we bury this?
        for (deadline : reactorClass.deadlines) {
            if (deadline.port.container !== null) { // Form is x.y
                var triggerToReactions = ReactorInfo.get(deadline.port.container.reactorClass).triggerToReactions
                var reactions = triggerToReactions.get(deadline.port.variable.name)
                if (reactions !== null) {
                    for (reaction : reactions) {
                        var childReactorInstance = instance.getChildReactorInstance(deadline.port.container)
                        var reactionInstance = childReactorInstance.getReactionInstance(reaction)
                        var reactionTName = reactionStructName(reactionInstance)
                        pr(initializeTriggerObjects, reactionTName + '.deadline = ' + timeInTargetLanguage(deadline.delay) + ';')
    
                        // Next, set the deadline_violation field to point to the trigger_t struct.
                        var triggerStructName = triggerStructName(instance.getActionInstance(deadline.action))
                        if (triggerStructName === null) {
                            reportError(
                                instance.definition.getReactorClass(),
                                "Internal error: failed to find trigger struct for action " + deadline.action.name +
                                " in reactor " + instance.getFullName()
                            )
                        } else {
                            pr(initializeTriggerObjects,
                                reactionTName + '.deadline_violation = &' + triggerStructName + ';')
                        }
                    }
                }
            } else { // x
                reportError(deadline, 'Malformed input port specification: ' + deadline.port)
            }
        }
        * 
        */

        for (child : instance.children) {
            generateReactorInstance(child)
        }
        unindent(initializeTriggerObjects)
        pr(initializeTriggerObjects, "} // End of scope for " + instance.fullName)
    }

    /** Set the reaction priorities based on dependency analysis.
     *  @param reactor The reactor on which to do this.
     */
    def void setReactionPriorities(ReactorInstance reactor) {
        // Use "reactionToReactionTName" property of reactionInstance
        // to set the levels.
        for (reactionInstance : reactor.reactions) {
            pr(reactionStructName(reactionInstance) + ".index = " + reactionInstance.level + ";")
        }
        for (child : reactor.children) {
            setReactionPriorities(child)
        }
    }
    
    ////////////////////////////////////////////
    //// Protected methods.

    /** Return a unique name for the reaction_t struct for the
     *  specified reaction instance.
     *  @param reaction The reaction instance.
     *  @return A name for the reaction_t struct.
     */
    protected def reactionStructName(ReactionInstance reaction) {
        reaction.uniqueID
    }
    
    ////////////////////////////////////////////
    //// Private methods.
    
    /** Perform deferred initializations in initialize_trigger_objects. */
    private def doDeferredInitialize() {
        // First, populate the trigger tables for each output.
        // The entries point to the trigger_t structs for the destination inputs.
        pr('// doDeferredInitialize')
        for (init : deferredInitialize) {
            var triggerStructName = triggerStructName(init.input)
            // If the destination of a connection is an input
            // port of a reactor that has no reactions to that input,
            // then this trigger struct will not have been created.
            // In that case, we want NULL.
            // If the destination is an output port, however, then
            // the dependentReactions.size will be zero, but we nevertheless
            // want to set up the trigger.
            if (init.input.dependentReactions.size === 0 && !init.input.isOutput) {
                pr(init.remoteTriggersArrayName + '[' + init.arrayIndex + '] = NULL;')
            } else {
                pr(init.remoteTriggersArrayName + '[' + init.arrayIndex + '] = &' + triggerStructName + ';')
            }
        }
        // Set all inputs _is_present variables to point to False by default.
        setInputsAbsentByDefault(main)

        // Next, for every input port, populate its "self" struct
        // fields with pointers to the output port that send it data.
        connectInputsToOutputs(main)
    }

    // Generate assignments of pointers in the "self" struct of a destination
    // port's reactor to the appropriate entries in the "self" struct of the
    // source reactor.
    private def void connectInputsToOutputs(ReactorInstance instance) {
        for (source : instance.destinations.keySet) {
            var eventualSource = source
            // If the source is an input port, find the ultimate source,
            // which has to be an output port. Or it may be a dangling
            // connection, in which case the result will still be an
            // input port.
            while (eventualSource.isInput && eventualSource.dependsOnPort !== null) {
                eventualSource = eventualSource.dependsOnPort
            }
            // If the eventual source is still an input, then this is a dangling
            // connection and we don't need to do anything.
            if (eventualSource.isOutput) {
                var sourceStruct = selfStructName(eventualSource.parent)
                // Use the source, not the eventualSource here to find the destinations.
                // If .parent.parent is null, then the source is an input port belonging
                // the top-level reactor, in which case, it cannot receive any inputs
                // and there is nothing to do.
                // FIXME: If the source is an input, we want to use just parent!
                val destinations = if (source.isInput) {
                    source.parent.destinations.get(source)
                } else if (source.parent.parent !== null) {
                    source.parent.parent.destinations.get(source)
                }
                if (destinations !== null) {
                    for (destination : destinations) {
                        var destStruct = selfStructName(destination.parent)
                
                        if (destination.isInput) {
                            pr(
                                destStruct + '.__' + destination.name + ' = &' + sourceStruct + '.__' +
                                    eventualSource.name + ';'
                            )
                            pr(
                                destStruct + '.__' + destination.name + '_is_present = &' + sourceStruct + '.__' +
                                    eventualSource.name + '_is_present;'
                            )
                        } else {
                            // Destination is an output.
                            var containerSelfStructName = selfStructName(destination.parent)
                            pr(
                                containerSelfStructName + '.__' + destination.name + '_inside = &' +
                                    sourceStruct + '.__' + eventualSource.name + ';'
                            )
                            pr(
                                containerSelfStructName + '.__' + destination.name + '_inside_is_present = &' +
                                    sourceStruct + '.__' + eventualSource.name + '_is_present;'
                            )
                        }
                    }
                }
            }
        }
        
        for (child : instance.children) {
            // In case this is a composite, recurse.
            connectInputsToOutputs(child)
        }

        // Handle inputs that get sent data from a reaction rather than from
        // another contained reactor.
        for (reaction : instance.reactions) {
            for (port : reaction.dependentPorts) {
                if (port.definition instanceof Input) {
                    // This reaction is sending to an input. Must be
                    // the input of a contained reactor.
                    var inputSelfStructName = selfStructName(port.parent)
                    var containerSelfStructName = selfStructName(instance)
                    pr(
                        inputSelfStructName + '.__' + port.definition.name + ' = &' + containerSelfStructName +
                            '.__' + port.parent.definition.name + '_' + port.definition.name + ';'
                    )
                    pr(
                        inputSelfStructName + '.__' + port.definition.name + '_is_present = &' + containerSelfStructName +
                            '.__' + port.parent.definition.name + '_' + port.definition.name + '_is_present;'
                    )
                    pr(
                        startTimeStep,
                        containerSelfStructName + '.__' + port.parent.definition.name + '_' + port.definition.name +
                            '_is_present = false;'
                    )
                }
            }
        }
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for inputs in a reaction function
     *  from the "self" struct.
     *  @param builder The string builder.
     *  @param input The input statement from the AST.
     */
    private def generateInputVariablesInReaction(
        StringBuilder builder,
        Input input
    ) {
        var present = input.name + '_is_present'
        pr(builder, 'bool ' + present + ' = *(self->__' + input.name + '_is_present);')
        pr(builder, removeCodeDelimiter(input.type) + ' ' + input.name + ';')
        pr(builder, 'if(' + present + ') {')
        indent(builder)
        pr(builder, input.name + ' = *(self->__' + input.name + ');')
        unindent(builder)
        pr(builder, '}')
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for outputs in a reaction function
     *  from the "self" struct.
     *  @param builder The string builder.
     *  @param output The output statement from the AST.
     */
    private def generateOutputVariablesInReaction(
        StringBuilder builder,
        Output output
    ) {
        if (output.type === null) {
            reportError(output, "Output is required to have a type: " + output.name)
        }
        // Slightly obfuscate the name to help prevent accidental use.
        pr(
            builder,
            removeCodeDelimiter(output.type) + '* ' + output.name + ' = &(self->__' + output.name + ');'
        )
        pr(
            builder,
            'bool* ' + output.name + '_is_present = &(self->__' + output.name + '_is_present);'
        )
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for sending data to an input
     *  of a contained reaction (e.g. for a deadline violation).
     *  @param builder The string builder.
     *  @param definition AST node defining the reactor within which this occurs
     *  @param input Input of the contained reactor.
     */
    private def generateVariablesForSendingToContainedReactors(StringBuilder builder, Instantiation definition, Input input) {
        // Need to create a struct so that the port can be referenced in C code
        // as reactorName.portName.
        // FIXME: This means that the destination instance name cannot match
        // any input port, output port, or action, because we will get a name collision.
        pr(
            builder,
            'struct ' + definition.name + ' {' + removeCodeDelimiter(input.type) + '* ' + input.name + '; ' + 'bool* ' +
                input.name + '_is_present;} ' + definition.name + ';'
        )
        pr(builder, definition.name + '.' + input.name + ' = &(self->__' + definition.name + '_' + input.name + ');')
        pr(builder,
            definition.name + '.' + input.name + '_is_present' + ' = &(self->__' + definition.name + '_' + input.name +
                '_is_present);')
    }

    /** Return a C type for the type of the specified parameter.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "interval_t".
     *  @param parameter The parameter.
     *  @return The C type.
     */
    private def getParameterType(Param parameter) {
        var type = removeCodeDelimiter(parameter.type)
        if (parameter.type.equals('time')) {
            type = 'interval_t'
        }
        type
    }

    /** Process any imports included in the resource defined by _resource.
     *  @param importTable The import table.
     */
    private def void processImports(Hashtable<String, String> importTable) {
        for (import : _resource.allContents.toIterable.filter(Import)) {
            val importResource = openImport(_resource, import)
            if (importResource !== null) {
                // Make sure the target of the import is C.
                var targetOK = false
                for (target : importResource.allContents.toIterable.filter(Target)) {
                    if ("C".equalsIgnoreCase(target.name)) {
                        targetOK = true
                    }
                }
                if (!targetOK) {
                    reportError(import, "Import does not have a C target.")
                } else {
                    val oldResource = _resource
                    _resource = importResource
                    // Process any imports that the import has.
                    processImports(importTable)
                    for (reactor : importResource.allContents.toIterable.filter(Reactor)) {
                        if (!reactor.isMain) {
                            println("Including imported reactor: " + reactor.name)
                            generateReactor(reactor, importTable)
                        }
                    }
                    _resource = oldResource
                }
            } else {
                pr("Unable to open import: " + import.name)
            }
        }
    }

    // Print the #line compiler directive with the line number of
    // the most recently used node.
    private def prSourceLineNumber(EObject reaction) {
        var node = NodeModelUtils.getNode(reaction)
        pr("#line " + node.getStartLine() + ' "' + _resource.getURI() + '"')

    }

    // Set inputs _is_present variables to the default to point to False.
    private def void setInputsAbsentByDefault(ReactorInstance parent) {
        // For all inputs, set a default where their _is_present variable points to False.
        // This handles dangling input ports that are not connected to anything
        // even if they are connected locally in the hierarchy, but not globally.
        for (containedReactor : parent.children) {
            for (input : containedReactor.inputs) {
                var inputSelfStructName = selfStructName(containedReactor)
                pr(inputSelfStructName + '.__' + input.definition.name + '_is_present = &False;')
            }
        }
        for (containedReactor : parent.children) {
            // In case this is a composite, handle its assignments.
            setInputsAbsentByDefault(containedReactor)
        }
    }

    val static includes = '''
        #include "pqueue.c"
    '''
    
}
