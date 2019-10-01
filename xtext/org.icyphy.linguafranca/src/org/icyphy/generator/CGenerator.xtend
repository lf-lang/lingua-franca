/*
 * Generator for C target.
 */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.io.BufferedReader
import java.io.File
import java.io.InputStream
import java.io.InputStreamReader
import java.net.URL
import java.nio.file.Paths
import java.util.HashMap
import java.util.HashSet
import java.util.Hashtable
import java.util.LinkedList
import org.eclipse.core.runtime.FileLocator
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.generator.ReactionGraph.ReactionInstance
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.SourceRef
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.TriggerRef

/**
 * Generator for C target.
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill, Mehrdad Niknami
 */
class CGenerator extends GeneratorBase {

	// For each reactor, we collect a set of input and parameter names.
	var reactorClassCount = 0
	var reactionCount = 0
	var reactionInstanceCount = 0
	var triggerCount = 0
	var instanceCount = 0
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

		var runCommand = newArrayList("./" + _filename, "-timeout", "3", "secs")
		var runCommandOverridden = false
		var compileCommand = newArrayList()

		var errors = new LinkedList<String>()

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
//				 else if (parameter.name.equals("threads")) {
//   					threads = parameter.value
//				}
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

		// Determine path to generated code
		val cFilename = _filename + ".c";
		var srcFile = resource.getURI.toString;
		
		if (srcFile.startsWith("file:")) { // Called from command line
			srcFile = Paths.get(srcFile.substring(5)).normalize.toString
		} else if (srcFile.startsWith("platform:")) { // Called from Eclipse
			srcFile = FileLocator.toFileURL(new URL(srcFile)).toString
			srcFile = Paths.get(srcFile.substring(5)).normalize.toString
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
			setReactionPriorities()
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
		// NOTE: IFileSystemAccess2 fsa for some reason does not update the last modified
		// date on the file. Hence, we delete the file here. This also ensures that no
		// older file gets mistaken for the result of this code generation in case code
		// generation failed.
		fsa.deleteFile(cFilename)
		fsa.generateFile(cFilename, getCode())

		// Copy the required library files into the target filesystem.
		var reactorCCommon = readFileInClasspath("/lib/C/reactor_common.c")
		fsa.deleteFile("reactor_common.c")
		fsa.generateFile("reactor_common.c", reactorCCommon)
		if (numberOfThreads === 0) {
			var reactorC = readFileInClasspath("/lib/C/reactor.c")
			fsa.deleteFile("reactor.c")
			fsa.generateFile("reactor.c", reactorC)
		} else {
			var reactorC = readFileInClasspath("/lib/C/reactor_threaded.c")
			fsa.deleteFile("reactor_threaded.c")
			fsa.generateFile("reactor_threaded.c", reactorC)
		}
		var reactorH = readFileInClasspath("/lib/C/reactor.h")
		fsa.deleteFile("reactor.h")
		fsa.generateFile("reactor.h", reactorH)
		var pqueueC = readFileInClasspath("/lib/C/pqueue.c")
		fsa.deleteFile("pqueue.c")
		fsa.generateFile("pqueue.c", pqueueC)
		var pqueueH = readFileInClasspath("/lib/C/pqueue.h")
        fsa.deleteFile("pqueue.h")
		fsa.generateFile("pqueue.h", pqueueH)

        val srcPath = srcFile.substring(0, srcFile.lastIndexOf(File.separator))
        var srcGenPath = srcPath + File.separator + "src-gen"
        var outPath = srcPath + File.separator + "bin"
        
        // Create directories for generated source and binary if they do not exist.
        var directory = new File(outPath)
        if (! directory.exists()){
            directory.mkdir()
        }
        directory = new File(srcGenPath)
        if (! directory.exists()){
            directory.mkdir()
        }
        
		// Invoke the compiler on the generated code.
		val relativeSrcFilename = "src-gen" + File.separator + cFilename;
		val relativeBinFilename = "bin" + File.separator + _filename;
		// FIXME: compileCommand is obsolete.
		if (compileCommand.isEmpty()) {
			if (numberOfThreads === 0) {
				// Non-threaded version.
				// compileCommand.addAll("pwd");
				compileCommand.addAll("gcc", "-O2", relativeSrcFilename, "-o", relativeBinFilename)
			} else {
				// Threaded version.
				// compileCommand.addAll("pwd");
				compileCommand.addAll("gcc", "-O2", "-pthread", relativeSrcFilename, "-o", relativeBinFilename)
			}
		}
		// println("Filename: " + cFilename);
		println("In directory: " + srcPath)
		println("Compiling with command: " + compileCommand.join(" "))
		var builder = new ProcessBuilder(compileCommand);
		builder.directory(new File(srcPath));
		var process = builder.start()
		var stdout = readStream(process.getInputStream())
		var stderr = readStream(process.getErrorStream())
		if (stdout.length() > 0) {
			println("--- Standard output:")
			println(stdout)
		}
		if (stderr.length() > 0) {
			errors.add(stderr.toString)
			println("ERRORS")
			println("--- Standard error:")
			println(stderr)
		} else {
			println("SUCCESS")
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

		var properties = reactorToProperties.get(reactor)

		// Put parameters into a struct and construct the code to go
		// into the preamble of any reaction function to extract the
		// parameters from the struct.
		val argType = "reactor_instance_" + (reactorClassCount++) + "_self_t"

		// Construct the typedef for the "self" struct.
		// NOTE: The struct cannot be empty in C; should we make a dummy field or suppress the struct?
		var body = new StringBuilder()
		// Start with parameters.
		for (parameter : getParameters(reactor)) {
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
				var containedSource = properties.outputNameToContainedOutputName.get(output.name)
				if (containedSource !== null) {
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
							removeCodeDelimiter(port.type) + ' __' + effect.instance.name + '_' + port.name + ';'
						)
						pr(
							body,
							'bool __' + effect.instance.name + '_' + port.name + '_is_present;'
						)
					}
//					var split = ref.effect.name.split('\\.')
//					if (split.length === 2) {
//						// Get a port matching the portName.
//						var destinationPort = getInputPortOfContainedReactor(
//							reactor, split.get(0), split.get(1), reaction
//						)
//						if (destinationPort !== null) {
//							pr(body, removeCodeDelimiter(destinationPort.type) + ' __'
//								+ split.get(0) + '_' + split.get(1) + ';'
//							)
//							pr(body, 'bool __'
//								+ split.get(0) + '_' + split.get(1) + '_is_present;'
//							)
//						}
//					}
				}
			}
		}
		if (body.length > 0) {
			properties.targetProperties.put("structType", argType)
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
		var properties = reactorToProperties.get(reactor)
		for (reaction : reactions) {
			// Create a unique function name for each reaction.
			val functionName = "reaction_function" + reactionCount++

			properties.targetProperties.put(reaction, functionName)

			// Construct the reactionInitialization code to go into
			// the body of the function before the verbatim code.
			// This defines the "self" struct.
			var StringBuilder reactionInitialization = new StringBuilder()
			var structType = properties.targetProperties.get("structType")
			if (structType !== null) {
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
				for (TriggerRef trigger : reaction.triggers) {
					// val input = getInput(reactor, trigger.variable.name)
					if (trigger.variable instanceof Input) {
						generateInputVariablesInReaction(reactionInitialization, trigger.variable as Input)
					} else if (trigger.variable instanceof Action) {
						pr(
							reactionInitialization,
							"trigger_t* " + trigger.variable.name + ' = self->__' + trigger.variable.name + ';'
						);
						actionsAsTriggers.add(trigger.variable as Action);
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
			for (SourceRef src : reaction.sources ?: emptyList) {
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
						} //						var split = output.split('\\.') // FIXME
//						if (split.length === 1) {
//							// It is an output.
//							var out = getOutput(reactor, output)
//							
						// }
						else if (effect.variable instanceof Input) {
							// It is the input of a contained reactor.
							generateVariablesForSendingToContainedReactors(
								reactionInitialization,
								effect.instance,
								effect.variable as Input
							)
						} else {
							// ERROR
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
		}
	}

	/** Generate one reaction function definitions for each output of
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
		var properties = reactorToProperties.get(reactor)
		for (output : properties.outputNameToContainedOutputName.keySet()) {
			// The following function name will be unique, assuming that
			// reactor class names are unique and within each reactor class,
			// output names are unique.
			val functionName = "transfer_output_" + reactor.name + "_" + output

			pr('void ' + functionName + '(void* instance_args) {')
			indent()

			// Define the "self" struct. First get its name. Note that this
			// must not be null because there is at least one output.
			var structType = properties.targetProperties.get("structType")
			pr(structType + "* self = (" + structType + "*)instance_args;")

			// Transfer the output value from the inside value.
			pr("self->__" + output + " = *(self->__" + output + "_inside);")
			// Transfer the presence flag from the inside value.
			pr("self->__" + output + "_is_present = *(self->__" + output + "_inside_is_present);")
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
	 *  @param nameOfSelfStruct The name of the instance of "self" for this instance or
	 *   null if there isn't one.
	 */
	def generateTriggerForTransferOutputs(
		ReactorInstance reactorInstance,
		String nameOfSelfStruct,
		HashMap<String, String> triggerNameToTriggerStruct
	) {
		// FIXME: This code is rather similar to that in generateTriggerObjects(). Refactor?
		var reactor = reactorInstance.reactor
		var properties = reactorToProperties.get(reactor)
		var triggeredSizesContents = new StringBuilder()
		var triggersContents = new StringBuilder()
		var outputCount = 0
		val result = new StringBuilder()
		for (output : properties.outputNameToContainedOutputName.keySet()) {
			// The function name for the transfer outputs function:
			val functionName = "transfer_output_" + reactor.name + "_" + output

			pr(result, "// --- Reaction and trigger objects for transfer outputs for output " + output)

			outputCount++

			// For each output, figure out how many
			// inputs are connected to it. This is obtained via the container.
			var container = reactorInstance.container
			// If there is no container, then the output cannot be connected
			// to anything, so default to empty set.
			var inputNames = new HashSet<String>()
			if (container !== null) {
				var parentReactor = container.reactor
				var parentProperties = reactorToProperties.get(parentReactor)
				var outputName = reactorInstance.getName() + "." + output
				inputNames = parentProperties.outputNameToInputNames.get(outputName)
			}
			// Insert a comma if needed.
			if (triggeredSizesContents.length > 0) {
				triggeredSizesContents.append(", ")
			}
			if (inputNames === null) {
				triggeredSizesContents.append("0")
			} else {
				triggeredSizesContents.append(inputNames.size)
			}
			// Then, for each input connected to this output,
			// find its trigger_t struct. Create an array of pointers
			// to these trigger_t structs, and collect pointers to
			// each of these arrays.
			// Insert a comma if needed.
			if (triggersContents.length > 0) {
				triggersContents.append(", ")
			}
			if (inputNames === null || inputNames.size === 0) {
				triggersContents.append("NULL")
			} else {
				var remoteTriggersArrayName = '__' + functionName + '_' + outputCount + '_remote_triggers'
				var inputCount = 0;
				for (inputName : inputNames) {
					deferredInitialize.add(
						new InitializeRemoteTriggersTable(
							container,
							remoteTriggersArrayName,
							(inputCount++),
							inputName
						)
					)
				}
				pr(
					result,
					'trigger_t* ' + remoteTriggersArrayName + '[' + inputCount + '];'
				)
				triggersContents.append('&' + remoteTriggersArrayName + '[0]')
			}
			// Next generate the array of booleans which indicates whether outputs are present.
			var outputProducedArray = '__' + functionName + '_outputs_are_present'
			pr(
				result,
				'bool* ' + outputProducedArray + '[]' + ' = {' + '&' + nameOfSelfStruct + '.__' + output +
					'_is_present' + '};'
			)
			// Create a array with ints indicating these
			// numbers and assign it to triggered_reactions_sizes
			// field of the reaction_t object.
			var triggeredSizesArray = '&__' + functionName + '_triggered_sizes[0]'
			pr(
				result,
				'int __' + functionName + '_triggered_sizes' + '[] = {' + triggeredSizesContents + '};'
			)
			// Create an array with pointers to arrays of pointers to trigger_t
			// structs for each input triggered by an output.
			var triggersArray = '&__' + functionName + '_triggers[0]'
			pr(
				result,
				'trigger_t** __' + functionName + '_triggers' + '[] = {' + triggersContents + '};'
			)
			// First 0 is an index that specifies priorities based on precedences.
			// It will be set later.
			var reactionInstanceName = "__reaction" + reactionInstanceCount++
			pr(
				result,
				"reaction_t " + reactionInstanceName + " = {&" + functionName + ", &" + nameOfSelfStruct + ", 0" // index: index from the topological sort.
				+ ", 0" // pos: position used by the pqueue implementation for sorting.
				+ ", 1" // num_outputs: number of outputs produced by this reaction. This is just one.
				+ ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
				+ ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
				+ ", " + triggersArray // triggered: array of pointers to arrays of triggers.
				+ ", 0LL" // Deadline.
				+ ", NULL" // Pointer to deadline violation trigger.
				+ ", false" // Indicator that the reaction is not running.
				+ "};"
			)
			pr(result, 'reaction_t* ' + output + triggerCount + '_reactions[1] = {&' + reactionInstanceName + '};')

			pr(result, 'trigger_t ' + output + triggerCount + ' = {')
			indent(result)
			pr(result, output + triggerCount + '_reactions, 1, 0LL, 0LL')
			unindent(result)
			pr(result, '};')

			// name output + triggerCount has to be recorded here because
			// doDeferredInitialize needs it to initialize the remote_triggers
			// array of the gain reaction that produces the output.            
			triggerNameToTriggerStruct.put(output, output + triggerCount)

			triggerCount++
		}
		// This goes directly out to the generated code.
		if (result.length > 0) {
			pr("// *********** Transfer outputs structures for " + reactor.name)
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
	 *  @param nameOfSelfStruct The name of the instance of "self" for this instance or
	 *   null if there isn't one.
	 *  @return A map of trigger names to the name of the trigger struct.
	 */
	def generateTriggerObjects(ReactorInstance reactorInstance, String nameOfSelfStruct) {
		var triggerNameToTriggerStruct = new HashMap<String, String>()
		var instance = reactorInstance.instanceStatement
		// If there is no instance statement, then this is main.
		var reactor = getReactor(instance.reactorClass.name)
		var properties = reactorToProperties.get(reactor)
		if (reactor === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return triggerNameToTriggerStruct
		}
		var triggerToReactions = getTriggerToReactions(reactor)
		if (triggerToReactions === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return triggerNameToTriggerStruct
		}
		val result = new StringBuilder()

		// Create a place to store reaction_t object names, indexed by Reaction.
		val reactionToReactionTName = new HashMap<Reaction, String>()
		reactorInstance.properties.put("reactionToReactionTName", reactionToReactionTName)

		var count = 0
		for (triggerName : triggerToReactions.keySet) {
			val numberOfReactionsTriggered = triggerToReactions.get(triggerName).length
			// Collect names of the reaction_t objects that are triggered together.
			var reactionTNames = new StringBuffer();

			// Generate reaction_t struct.
			// Along the way, we need to generate its contents, including trigger_t structs.
			for (reaction : triggerToReactions.get(triggerName)) {
				var functionName = properties.targetProperties.get(reaction)
				pr(
					result,
					'// --- Reaction and trigger objects for reaction to trigger ' + triggerName + ' of instance ' +
						instance.name
				)
				// First check to see whether the reaction_t object has already been
				// created for this reaction.
				var reactionInstanceName = reactionToReactionTName.get(reaction)
				if (reactionInstanceName === null) {
					// There isn't already a reaction_t object for this reaction.
					// Generate a reaction_t object for an instance of a reaction.
					reactionInstanceName = "__reaction" + reactionInstanceCount++

					// Store the reaction_t object name for future use, indexed by the Reaction.
					reactionToReactionTName.put(reaction, reactionInstanceName)

					// Generate entries for the reaction_t struct that specify how
					// to handle outputs.
					var presentPredicates = new StringBuilder()
					var triggeredSizesContents = new StringBuilder()
					var triggersContents = new StringBuilder()
					var outputCount = 0
					if (reaction.effects !== null) {
						for (effect : reaction.effects) {
							if (effect.variable instanceof Output) {
								// It is an output, not an action.
								// First create the array of pointers to booleans indicating whether
								// an output is produced.
								// Insert a comma if needed.
								if (presentPredicates.length > 0) {
									presentPredicates.append(", ")
								}
								presentPredicates.append('&' + nameOfSelfStruct + '.__' + effect.variable.name +
									'_is_present')
								outputCount++

								// For each output, figure out how many
								// inputs are connected to it. This is obtained via the container.
								var container = reactorInstance.container
								// If there is no container, then the output cannot be connected
								// to anything, so default to empty set.
								var inputNames = new HashSet<String>()
								if (container !== null) {
									var parentReactor = container.reactor
									var parentProperties = reactorToProperties.get(parentReactor)
									var outputName = instance.getName() + "." + effect.variable.name
									inputNames = parentProperties.outputNameToInputNames.get(outputName)
								}
								// Insert a comma if needed.
								if (triggeredSizesContents.length > 0) {
									triggeredSizesContents.append(", ")
								}
								if (inputNames === null) {
									triggeredSizesContents.append("0")
								} else {
									triggeredSizesContents.append(inputNames.size)
								}
								// Then, for each input connected to this output,
								// find its trigger_t struct. Create an array of pointers
								// to these trigger_t structs, and collect pointers to
								// each of these arrays.
								// Insert a comma if needed.
								if (triggersContents.length > 0) {
									triggersContents.append(", ")
								}
								if (inputNames === null || inputNames.size === 0) {
									triggersContents.append("NULL")
								} else {
									var remoteTriggersArrayName = reactionInstanceName + '_' + outputCount +
										'_remote_triggers'
									var inputCount = 0;
									for (inputName : inputNames) {
										deferredInitialize.add(
											new InitializeRemoteTriggersTable(
												container,
												remoteTriggersArrayName,
												(inputCount++),
												inputName
											)
										)
									}
									pr(
										result,
										'trigger_t* ' + remoteTriggersArrayName + '[' + inputCount + '];'
									)
									triggersContents.append('&' + remoteTriggersArrayName + '[0]')
								}
							} else if (effect.variable instanceof Input) {
								val input = effect.variable as Input
								val _instance = effect.instance
								// It is not an output, but the reaction may be sending data
								// to the input of a contained reactor. Check first to see whether
								// it has the right form.
								// var portSpec = output.split('\\.')
								// if (portSpec.length === 2) {
								// The form is right.
								// First create the array of pointers to booleans indicating whether
								// an output is produced.
								// Insert a comma if needed.
								if (presentPredicates.length > 0) {
									presentPredicates.append(", ")
								}
								presentPredicates.append(
									'&' + nameOfSelfStruct + '.__' + _instance.name + '_' + input.name + '_is_present'
								)
								outputCount++

//									var destinationPort = getInputPortOfContainedReactor(
//										reactor, _instance.name, input.name, reaction.produces
//									)
								// Insert a comma if needed.
								if (triggeredSizesContents.length > 0) {
									triggeredSizesContents.append(", ")
								}
								triggeredSizesContents.append("1")
								if (input === null) {
									triggersContents.append("NULL")
								} else {
									var remoteTriggersArrayName = reactionInstanceName + '_' + outputCount +
										'_remote_triggers'
									var destinationInstance = reactorInstance.getContainedInstance(_instance.name)
									if (destinationInstance !== null) {
										// Null destinationInstance is an error, but it should have been caught
										// before this, so no need to report it now.
										deferredInitialize.add(
											new InitializeRemoteTriggersTable(
												destinationInstance,
												remoteTriggersArrayName,
												0,
												input.name
											)
										)
									}
									pr(
										result,
										'trigger_t* ' + remoteTriggersArrayName + '[1];'
									)
									triggersContents.append('&' + remoteTriggersArrayName + '[0]')
								}
							// }
							}
						}
					}
					var outputProducedArray = "NULL"
					var triggeredSizesArray = "NULL"
					var triggersArray = "NULL"
					if (outputCount > 0) {
						outputProducedArray = reactionInstanceName + '_outputs_are_present'
						// Create a array with booleans indicating whether an output has been produced.
						pr(
							result,
							'bool* ' + reactionInstanceName + '_outputs_are_present[]' + ' = {' +
								presentPredicates.toString + '};'
						)
						// Create a array with ints indicating these
						// numbers and assign it to triggered_reactions_sizes
						// field of the reaction_t object.
						triggeredSizesArray = '&' + reactionInstanceName + '_triggered_sizes[0]'
						pr(
							result,
							'int ' + reactionInstanceName + '_triggered_sizes' + '[] = {' + triggeredSizesContents +
								'};'
						)
						// Create an array with pointers to arrays of pointers to trigger_t
						// structs for each input triggered by an output.
						triggersArray = '&' + reactionInstanceName + '_triggers[0]'
						pr(
							result,
							'trigger_t** ' + reactionInstanceName + '_triggers' + '[] = {' + triggersContents + '};'
						)
					}
					// Finally, produce the reaction_t struct.			
					// The argument specifying the self struct may be NULL if there
					// is no self struct.
					var selfStructArgument = ", &" + nameOfSelfStruct
					if (nameOfSelfStruct === null) {
						selfStructArgument = ", NULL"
					}
					// First 0 is an index that specifies priorities based on precedences.
					// It will be set later.
					pr(
						result,
						"reaction_t " + reactionInstanceName + " = {&" + functionName + selfStructArgument + ", 0" // index: index from the topological sort.
						+ ", 0" // pos: position used by the pqueue implementation for sorting.
						+ ", " + outputCount // num_outputs: number of outputs produced by this reaction.
						+ ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
						+ ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
						+ ", " + triggersArray // triggered: array of pointers to arrays of triggers.
						+ ", 0LL" // Deadline.
						+ ", NULL" // Pointer to deadline violation trigger.
						+ ", false" // Indicator that the reaction is not running.
						+ "};"
					)
				}
				// Collect the reaction instance names to initialize the
				// reaction pointer array for the trigger.
				if (reactionTNames.length != 0) {
					reactionTNames.append(", ")
				}
				reactionTNames.append('&' + reactionInstanceName)
			}
			var triggerStructName = triggerName + triggerCount

			// Record the triggerStructName.
			triggerNameToTriggerStruct.put(triggerName, triggerStructName)

			pr(result,
				'reaction_t* ' + triggerStructName + '_reactions[' + numberOfReactionsTriggered + '] = {' +
					reactionTNames + '};')
			// Declare a variable with the name of the trigger whose
			// value is a struct.
			pr(result, 'trigger_t ' + triggerStructName + ' = {')
			indent(result)
			var timing = getTiming(reactor, triggerName)
			if (timing !== null || getInput(reactor, triggerName) !== null) {
				pr(
					result,
					triggerStructName + '_reactions, ' + numberOfReactionsTriggered + ', ' + '0LL, 0LL'
				)
			} else if (getAction(reactor, triggerName) !== null) {
				var modifier = getAction(reactor, triggerName).getModifier();
				var isPhysical = "true";
				if (modifier == "logical") {
					isPhysical = "false";
				}
				pr(
					result,
					triggerStructName + '_reactions, ' + numberOfReactionsTriggered + ', ' +
						getAction(reactor, triggerName).getDelay() + ', 0LL, NULL, ' + isPhysical // 0 is ignored since actions don't have a period.
				)
			} else {
				reportError(reactor, "Internal error: Seems to not be an input, timer, or action: " + triggerName)
			}
			unindent(result)
			pr(result, '};')
			// Assignment of the offset and period have to occur after creating
			// the struct because the value assigned may not be a compile-time constant.
			if (timing !== null) {
				pr(initializeTriggerObjects, triggerStructName + '.offset = ' + timeMacro(timing.offset) + ';')
				pr(initializeTriggerObjects, triggerStructName + '.period = ' + timeMacro(timing.period) + ';')

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
		return triggerNameToTriggerStruct
	}

	/** Instantiate a reactor.
	 *  @param instance The instance declaration in the AST.
	 *  @param container The instance that is the container.
	 *  @param importTable Substitution table for class names (from import statements).
	 *  @return The reactor instance, or null if an error occurs.
	 */
	override instantiate(
		Instance instance,
		ReactorInstance container,
		Hashtable<String, String> importTable
	) {
		var className = importTable.get(instance.reactorClass);
		if (className === null) {
			className = instance.reactorClass.name
		}
		pr('// ************* Instance ' + instance.name + ' of class ' + className)
		var reactor = getReactor(instance.reactorClass.name)
		if (reactor === null) {
			reportError(instance, "No such reactor: " + instance.reactorClass)
			return null
		}

		// Generate the instance struct containing parameters, state variables,
		// and outputs (the "self" struct).
		var properties = reactorToProperties.get(reactor)
		var nameOfSelfStruct = "__self_" + instanceCount++ + "_" + instance.name
		var structType = properties.targetProperties.get("structType")
		if (structType !== null) {
			pr('// --- "self" struct for instance ' + instance.name)
			pr(structType + " " + nameOfSelfStruct + ";")
		} else {
			// Nullify the name, indicating that there is no self struct.
			nameOfSelfStruct = null;
		}

		// Generate code to initialize the "self" struct in the
		// __initialize_trigger_objects function.
		// Create a scope for the parameters in case the names collide with other instances.
		pr(initializeTriggerObjects, "{ // Scope for " + instance.name)
		indent(initializeTriggerObjects)
		// Start with parameters.
		// First, collect the overrides.
		var overrides = new HashMap<String, String>()
		var parameters = instance.parameters
		if (parameters !== null) {
			for (assignment : parameters.assignments) {
				var value = assignment.value;
				if (assignment.unit !== null) {
					var time = LinguaFrancaFactory.eINSTANCE.createTime()
					time.setTime(value)
					time.setUnit(assignment.unit)
					value = timeMacro(time)
				}
				overrides.put(assignment.name, removeCodeDelimiter(value))
			}
		}
		// Next, initialize parameter with either the override or the defaults.
		for (parameter : getParameters(reactor)) {
			var value = overrides.get(parameter.name)
			if (value === null) {
				value = removeCodeDelimiter(parameter.value)
				if (parameter.time !== null) {
					value = timeMacro(parameter.time)
				}
			}
			// In case the parameter value refers to a container parameter with the same name,
			// we have to first store the value in a temporary variable, then in the
			// parameter variable.
			var tmpVariableName = '__tmp' + tmpVariableCount++
			pr(initializeTriggerObjects, getParameterType(parameter) + ' ' + tmpVariableName + ' = ' + value + ';')
			pr(initializeTriggerObjects,
				getParameterType(parameter) + ' ' + parameter.name + ' = ' + tmpVariableName + ';')
			pr(initializeTriggerObjects, nameOfSelfStruct + "." + parameter.name + " = " + value + ";")
		}
		// Next, initialize the "self" struct with state variables.
		for (state : reactor.states) {
			var value = removeCodeDelimiter(state.value)
			pr(initializeTriggerObjects, nameOfSelfStruct + "." + state.name + " = " + value + ";")
		}

		// Call superclass here so that parameters of this reactor
		// are in scope for contained instances.
		var reactorInstance = super.instantiate(instance, container, importTable)
		if (reactorInstance === null) {
			// An error occurred.
			return null
		}
		// Store the name of the "self" struct as a property of the instance
		// so that it can be used when establishing connections.
		// Only store it if the structType was actually created, however.
		if (nameOfSelfStruct !== null) {
			reactorInstance.properties.put("selfStructName", nameOfSelfStruct)
		}

		// Generate trigger objects for the instance.
		var triggerNameToTriggerStruct = generateTriggerObjects(reactorInstance, nameOfSelfStruct)
		reactorInstance.properties.put("triggerNameToTriggerStruct", triggerNameToTriggerStruct)

		// Generate trigger objects for transferring outputs of a composite.
		generateTriggerForTransferOutputs(reactorInstance, nameOfSelfStruct, triggerNameToTriggerStruct)

		// Next, initialize the struct with actions.
		for (action : reactor.actions) {
			var triggerStruct = triggerNameToTriggerStruct.get(action.name)
			if (triggerStruct === null) {
				reportError(reactor, "Internal error: No trigger struct found for action " + action.name)
			}
			// FIXME: Actions may have payloads.
			pr(
				initializeTriggerObjects,
				nameOfSelfStruct + '.__' + action.name + ' = &' + triggerStruct + ';'
			)
		}
		// Next, generate the code to initialize outputs at the start
		// of a time step to be absent.
		for (output : reactor.outputs) {
			pr(
				startTimeStep,
				nameOfSelfStruct + '.__' + output.name + '_is_present = false;'
			)
		}

		// Finally, handle deadline commands.
		for (deadline : reactor.deadlines) {
			if (deadline.port.instance !== null) { // x.y
				var deadlineReactor = reactorInstance.getContainedInstance(deadline.port.instance.name)
				var triggerToReactions = getTriggerToReactions(deadline.port.instance.reactorClass)
				var reactions = triggerToReactions.get(deadline.port.variable.name)
				for (reaction : reactions) {
					var reactionToReactionTName = deadlineReactor.properties.get("reactionToReactionTName")
					var reactionTName = (reactionToReactionTName as HashMap<Reaction, String>).get(reaction)
					if (reactionTName === null) {
						reportError(deadline, "Internal error: No reaction_t object found for reaction.")
					} else {
						pr(initializeTriggerObjects, reactionTName + '.deadline = ' + timeMacro(deadline.delay) + ';')

						// Next, set the deadline_violation field to point to the trigger_t struct.
						var triggerMap = reactorInstance.properties.get("triggerNameToTriggerStruct")
						if (triggerMap === null) {
							reportError(
								deadline,
								"Internal error: failed to map from name to trigger struct for " +
									reactorInstance.getFullName()
							)
						} else {
							var triggerStructName = (triggerMap as HashMap<String, String>).get(deadline.action.name)
							if (triggerStructName === null) {
								reportError(
									reactorInstance.reactor,
									"Internal error: failed to find trigger struct for action " + deadline.action.name +
										" in reactor " + reactorInstance.getFullName()
								)
							} else {
								pr(initializeTriggerObjects,
									reactionTName + '.deadline_violation = &' + triggerStructName + ';')
							}
						}
					}
				}

			} else { // x
				reportError(deadline, 'Malformed input port specification: ' + deadline.port)
			}
		}

		unindent(initializeTriggerObjects)
		pr(initializeTriggerObjects, "} // End of scope for " + instance.name)

		reactorInstance
	}

	/** Set the reaction priorities based on dependency analysis. */
	def setReactionPriorities() {
		var graph = new ReactionGraph(this)
		// Calculate levels for the graph.		
		graph.calculateLevels(main)

		// Use "reactionToReactionTName" property of reactionInstance
		// to set the levels.
		for (ReactionInstance instance : graph.nodes) {
			val reactorInstance = instance.reactorInstance;
			val map = reactorInstance.properties.get("reactionToReactionTName") as HashMap<Reaction, String>;
			val reactionTName = map.get(instance.reactionSpec);
			pr(reactionTName + ".index = " + instance.level + ";")
		}
	}

	// //////////////////////////////////////////
	// // Utility functions for generating code.
	/** Perform deferred initializations in initialize_trigger_objects. */
	private def doDeferredInitialize() {
		// First, populate the trigger tables for each output.
		// The entries point to the trigger_t structs for the destination inputs.
		pr('// doDeferredInitialize')
		for (init : deferredInitialize) {
			// The reactor containing the specified input may be a contained reactor.
			var reactor = init.reactor
			var insideReactor = null as ReactorInstance
			var port = init.inputName
			var split = init.inputName.split('\\.')
			if (split.length === 2 || split.length === 3) {
				reactor = init.reactor.getContainedInstance(split.get(0))
				if (reactor === null) {
					reportError(
						init.reactor.reactor,
						"No reactor named: " + split.get(0) + " in container " + init.reactor.getFullName()
					)
				}
				if (split.length === 2) {
					port = split.get(1)
				} else {
					// The destination of a connection is inside a contained reactor.
					// The form of the inputName is:
					// reactorInstanceName.containedReactorInstanceName.inputName
					insideReactor = reactor.getContainedInstance(split.get(1))
					if (insideReactor === null) {
						reportError(
							init.reactor.reactor,
							"No reactor named: " + split.get(1) + " in container " + split.get(0) + " in " +
								init.reactor.getFullName()
						)
					}
					port = split.get(2)
				}
			} else if (split.length !== 1) {
				reportError(init.reactor.reactor, "Invalid input specification: " + init.inputName)
			}
			if (reactor !== null) {
				var triggerMap = if (insideReactor !== null) {
						insideReactor.properties.get("triggerNameToTriggerStruct")
					} else {
						reactor.properties.get("triggerNameToTriggerStruct")
					}
				if (triggerMap === null) {
					reportError(
						init.reactor.reactor,
						"Internal error: failed to find map from name to trigger struct for " +
							init.reactor.getFullName()
					)
				} else {
					var triggerStructName = (triggerMap as HashMap<String, String>).get(port)
					// Note that triggerStructName will be null if the destination of a
					// connection is the input port of a reactor that has no reactions to
					// that input.
					if (triggerStructName !== null) {
						pr(
							init.remoteTriggersArrayName + '[' + init.arrayIndex + '] = &' + triggerStructName + ';'
						)
					} else {
						// Destination port has no reactions, but we need to fill in an
						// entry in the table anyway.
						pr(
							init.remoteTriggersArrayName + '[' + init.arrayIndex + '] = NULL;'
						)
					}
				}
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
	private def void connectInputsToOutputs(ReactorInstance container) {
		// Collect the set of inputs that have connections so that we can
		// report as an error inputs that are connected to more than one output.
		var connectedInputs = new HashSet<String>()

		// Include in connectedInputs all inputs of contained
		// reactors that are connected to an input of the container.
		// This is useful for detecting errors where multiple conenctions are
		// made to an input.
		for (connection : container.reactor.connections) {
			var split = connection.leftPort.split('\\.')
			if (split.length === 1) {
				connectedInputs.add(connection.rightPort)
			}
		}

		for (containedReactor : container.containedInstances.values()) {
			// In case this is a composite, handle its assignments.
			connectInputsToOutputs(containedReactor)
			var containerProperties = reactorToProperties.get(containedReactor.container.reactor)
			for (output : containedReactor.reactor.outputs) {
				var outputSelfStructName = containedReactor.properties.get("selfStructName")
				var inputNames = containerProperties.outputNameToInputNames.get(containedReactor.name + '.' +
					output.name)
				if (inputNames !== null) {
					for (input : inputNames) {
						if (connectedInputs.contains(input)) {
							reportError(output, "Connecting to an input that already has a connection: " + input)
						}
						connectedInputs.add(input)
						var split = input.split('\\.')
						if (split.length > 1) {
							var inputReactor = containedReactor.container.getContainedInstance(split.get(0))
							if (inputReactor !== null) {
								// It is an error to be null, but it should have been caught earlier.
								var port = split.get(1)
								if (split.length > 2) {
									port = split.get(2)
									inputReactor = inputReactor.getContainedInstance(split.get(1))
								}
								if (inputReactor !== null) {
									// It is an error to be null, but it should have been caught earlier.
									var inputSelfStructName = inputReactor.properties.get("selfStructName")
									pr(
										inputSelfStructName + '.__' + port + ' = &' + outputSelfStructName + '.__' +
											output.name + ';'
									)
									pr(
										inputSelfStructName + '.__' + port + '_is_present = &' + outputSelfStructName +
											'.__' + output.name + '_is_present;'
									)
								}
							}
						} else {
							// Destination is the inside of an output port rather the input of
							// another reactor. Hence, "input" here is the name of the output port
							// of container.
							var containerSelfStructName = container.properties.get("selfStructName")
							pr(
								containerSelfStructName + '.__' + input + '_inside = &' + outputSelfStructName + '.__' +
									output.name + ';'
							)
							pr(
								containerSelfStructName + '.__' + input + '_inside_is_present = &' +
									outputSelfStructName + '.__' + output.name + '_is_present;'
							)
						}
					}
				}
			}
		}
		// Handle inputs that get sent data from a reaction rather than from
		// another contained reactor.
		for (reaction : container.reactor.reactions) {
			if (reaction.effects !== null) {
				for (effect : reaction.effects) {

					// var split = produces.split('\\.')
					if (effect.variable instanceof Input) {
						// Found an input that is sent data from a reaction.
						if (connectedInputs.contains(effect.variable.name)) {
							reportError(reaction,
								"Sending to an input that already has a connection: " + effect.variable.name)
						}
						connectedInputs.add(effect.variable.name)
						var inputReactor = container.getContainedInstance(effect.instance.name)
						if (inputReactor === null) {
							reportError(reaction, "No such destination reactor: " + effect.instance.name)
						} else {
							var inputSelfStructName = inputReactor.properties.get("selfStructName")
							var containerSelfStructName = container.properties.get("selfStructName")
							pr(
								inputSelfStructName + '.__' + effect.variable.name + ' = &' + containerSelfStructName +
									'.__' + effect.instance.name + '_' + effect.variable.name + ';'
							)
							pr(
								inputSelfStructName + '.__' + effect.variable.name + '_is_present = &' +
									containerSelfStructName + '.__' + effect.instance.name + '_' +
									effect.variable.name + '_is_present;'
							)
							pr(
								startTimeStep,
								containerSelfStructName + '.__' + effect.instance.name + '_' + effect.variable.name +
									'_is_present = false;'
							)
						}
					}
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
	 *  @param produces The output statement from the AST.
	 *  @param reactor The reactor within which this occurs.
	 *  @param portSpec The output statement split into reactorName and portName.
	 */
	private def generateVariablesForSendingToContainedReactors(StringBuilder builder, Instance instance, Input input) {
		// Need to create a struct so that the port can be referenced in C code
		// as reactorName.portName.
		// FIXME: This means that the destination instance name cannot match
		// any input port, output port, or action, because we will get a name collision.
		pr(
			builder,
			'struct ' + instance.name + ' {' + removeCodeDelimiter(input.type) + '* ' + input.name + '; ' + 'bool* ' +
				input.name + '_is_present;} ' + instance.name + ';'
		)
		pr(builder, instance.name + '.' + input.name + ' = &(self->__' + instance.name + '_' + input.name + ');')
		pr(builder,
			instance.name + '.' + input.name + '_is_present' + ' = &(self->__' + instance.name + '_' + input.name +
				'_is_present);')
	}

//	/** Given a container reactor, a reactor name, and a port name, return
//	 *  the Input statement that it corresponds to, or report an error and
//	 *  return null if there is no such input.
//	 *  @param container A composite reactor.
//	 *  @param reactorName The name of a contained reactor.
//	 *  @param portName The name of an input port of the contained reactor.
//	 *  @param report The AST object on which to report an error.
//	 */
//	private def getInputPortOfContainedReactor(
//		Reactor container, String reactorName, String portName, EObject report
//	) {
//		// First, find an instance whose name matches the reactorName.
//		var instance = container.getInstance(reactorName)
//		if (instance === null) {
//			reportError(report, "No instance named: " + reactorName)
//			return null as Input
//		}
//		
//		// Next, need to find the reactor definition referenced.
//		var containedReactor = getReactor(instance.reactorClass.name)
//		if (containedReactor === null) {
//			reportError(report, "Cannot find reactor definition for: "
//				+ instance.reactorClass
//			)
//			return null as Input
//		}
//		
//		// Next, get a port matching the portName.
//		var destinationPort = containedReactor.getInput(portName)
//		if (destinationPort === null) {
//			reportError(report, "Destination port does not have an input named: " 
//				+ portName
//			)
//			return null as Input
//		}
//		destinationPort
//	}
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
						if (!reactor.name.equalsIgnoreCase("main")) {
							println("Including imported reactor: " + reactor.name)
							generateReactor(reactor, importTable)
						}
					}
					_resource = oldResource
				}
			} else {
				pr("Unable to open import...")
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
	private def void setInputsAbsentByDefault(ReactorInstance container) {
		// For all inputs, set a default where their _is_present variable points to False.
		// This handles dangling input ports that are not connected to anything
		// even if they are connected locally in the hierarchy, but not globally.
		for (containedReactor : container.containedInstances.values()) {
			for (input : containedReactor.reactor.inputs) {
				var inputReactor = containedReactor.container.getContainedInstance(containedReactor.name)
				var inputSelfStructName = inputReactor.properties.get("selfStructName")
				pr(inputSelfStructName + '.__' + input.name + '_is_present = &False;')
			}
		}
		for (containedReactor : container.containedInstances.values()) {
			// In case this is a composite, handle its assignments.
			setInputsAbsentByDefault(containedReactor)
		}
	}

	/** Given a representation of time that may possibly include units,
	 *  return a string that invokes a macro to convert to nanoseconds.
	 *  @param time The time to convert.
	 *  @return A string, such as "MSEC(100)" for 100 milliseconds.
	 */
	protected def timeMacro(Time time) {
		if (time === null || time.time === null) {
			'0LL'
		} else if (time.unit === null) {
			// Assume the literal is correct.
			time.time
		} else {
			time.unit.toUpperCase + '(' + time.time + ')'
		}
	}

	val static includes = '''
		#include "pqueue.c"
	'''
}
