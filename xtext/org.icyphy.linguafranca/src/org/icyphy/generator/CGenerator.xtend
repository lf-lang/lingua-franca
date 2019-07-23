/*
 * Generator for C target.
 */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import java.util.Hashtable
import java.util.LinkedList
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.generator.ReactionGraph.ReactionInstance
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Produces
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.Time

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
	override void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
		
		pr(includes)
		_resource = resource
		
		var uriAsString = resource.getURI().toString()
		println("Generating code for: " + uriAsString)
				
		for (target: resource.allContents.toIterable.filter(Target)) {
			if (target.parameters !== null) {
   				for (parameter: target.parameters.assignments) {
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
   					}
   				}
   			}
		}
		if (numberOfThreads === 0) {
			pr("#include \"reactor.c\"")
		} else {
			pr("#include \"reactor_threaded.c\"")			
		}

		// First process all the imports.
		processImports(importTable)

		super.doGenerate(resource, fsa, context, importTable)
		
		// Any main reactors in imported files are ignored.		
		if (main !== null) {
			// Generate function to initialize the trigger objects for all reactors.
			pr('void __initialize_trigger_objects() {\n')
			indent()
			pr(initializeTriggerObjects)
			doDeferredInitialize()
			setReactionPriorities()
			unindent()
			pr('}\n')
		
			// Generate function to start timers for all reactors.
			pr("void __start_timers() {")
			indent()
			pr(startTimers)
			unindent()
			pr("}")
		
			// Generate function to execute at the start of a time step.
			pr('void __start_time_step() {\n')
			indent()
			pr(startTimeStep)
			unindent()
			pr('}\n')
		}
		fsa.generateFile(_filename + ".c", getCode())
		
		// Copy the required library files into the target filesystem.
		var reactorCCommon = readFileInClasspath("/lib/C/reactor_common.c")
		fsa.generateFile("reactor_common.c", reactorCCommon)
		if (numberOfThreads === 0) {
			var reactorC = readFileInClasspath("/lib/C/reactor.c")
			fsa.generateFile("reactor.c", reactorC)
		} else {
			var reactorC = readFileInClasspath("/lib/C/reactor_threaded.c")
			fsa.generateFile("reactor_threaded.c", reactorC)			
		}
		var reactorH = readFileInClasspath("/lib/C/reactor.h")
		fsa.generateFile("reactor.h", reactorH)
		var pqueueC = readFileInClasspath("/lib/C/pqueue.c")
		fsa.generateFile("pqueue.c", pqueueC)
		var pqueueH = readFileInClasspath("/lib/C/pqueue.h")
		fsa.generateFile("pqueue.h", pqueueH)		
	}
	
	////////////////////////////////////////////
	//// Code generators.
	
	/** Generate a reactor class definition.
	 *  @param reactor The parsed reactor data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	override generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
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
		for(parameter: getParameters(reactor)) {
			prSourceLineNumber(parameter)
			if (parameter.type === null) {
				reportError(parameter, "Parameter is required to have a type: " + parameter.name)
			} else {
				pr(body, getParameterType(parameter) + ' ' + parameter.name + ';');
			}
		}
		// Next handle states.
		for(state: reactor.states) {
			prSourceLineNumber(state)
			if (state.type === null) {
				reportError(state, "State is required to have a type: " + state.name)
			} else {
				pr(body, removeCodeDelimiter(state.type) + ' ' + state.name + ';');
			}
		}
		// Next handle actions.
		for(action: reactor.actions) {
			prSourceLineNumber(action)
			// NOTE: Slightly obfuscate output name to help prevent accidental use.
			pr(body, "trigger_t* __" + action.name + ";")
		}
		// Next handle inputs.
		for(input: reactor.inputs) {
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
		for(output: reactor.outputs) {
			prSourceLineNumber(output)
			if (output.type === null) {
				reportError(output, "Output is required to have a type: " + output.name)
			} else {
				// NOTE: Slightly obfuscate output name to help prevent accidental use.
				pr(body, removeCodeDelimiter(output.type) + ' __' + output.name + ';')
				pr(body, 'bool __' + output.name + '_is_present;')
			}
		}
		// Finally, handle reactions that produce outputs sent to inputs
		// of contained reactions.
		for(reaction: reactor.reactions) {
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					var split = output.split('\\.')
					if (split.length === 2) {
						// Get a port matching the portName.
						var destinationPort = getInputPortOfContainedReactor(
							reactor, split.get(0), split.get(1), reaction
						)
						if (destinationPort !== null) {
							pr(body, removeCodeDelimiter(destinationPort.type) + ' __'
								+ split.get(0) + '_' + split.get(1) + ';'
							)
							pr(body, 'bool __'
								+ split.get(0) + '_' + split.get(1) + '_is_present;'
							)
						}
					}
				}				
			}
		}
		if (body.length > 0) {
			properties.targetProperties.put("structType", argType)
			pr("typedef struct {")
			indent()
			pr(body)
			unindent()
			pr("} " + argType + ";")
		}
		
		// Generate reactions
		// For this second pass, restart the reaction count where the first pass started.
		reactionCount = savedReactionCount;
		generateReactions(reactor)	
		pr("// =============== END reactor class " + reactor.name)
		pr("")
	}

	/** Generate reaction functions definition for a reactor.
	 *  These functions have a single argument that is a void* pointing to
	 *  a struct that contains parameters, inputs (triggering or not),
	 *  actions (triggering or produced), and outputs.
	 *  @param reactor The reactor.
	 */
	def generateReactions(Reactor reactor) {
		var reactions = reactor.reactions
		var properties = reactorToProperties.get(reactor)
		for (reaction: reactions) {
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
				pr(reactionInitialization, structType
						+ "* self = (" + structType + "*)instance_args;")
			}
			
			// Next, add the triggers (input and actions; timers are not needed).
			// This defines a local variable in the reaction function whose
			// name matches that of the trigger. If the trigger is an input
			// (not an action), then it also defines a local variable whose
			// name is the input name with suffix "_is_present", a boolean
			// that indicates whether the input is present.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					val input = getInput(reactor, trigger)
					if (input !== null) {
						generateInputVariablesInReaction(reactionInitialization, input)
					}
					val action = getAction(reactor, trigger)
					if (action !== null) {
						// FIXME: Actions may have payloads.
						pr(reactionInitialization, "trigger_t* " 
							+ action.name + ' = self->__' + action.name + ';'
						);
					}
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare an argument for every input.
				for (input: reactor.inputs) {
					generateInputVariablesInReaction(reactionInitialization, input)
				}
			}
			// Define argument for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					val input = getInput(reactor, get)
					generateInputVariablesInReaction(reactionInitialization, input)
				}
			}
			// Define variables for each declared output or action.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					val action = getAction(reactor, output)
					if (action !== null) {
						// It is an action, not an output.
						// FIXME: Actions may have payloads.
						pr(reactionInitialization, "trigger_t* " + action.name + ' = self->__' + action.name + ';');
					} else {
						var split = output.split('\\.')
						if (split.length === 1) {
							// It is an output.
							var out = getOutput(reactor, output)
							generateOutputVariablesInReaction(reactionInitialization, out)
						} else {
							// It is the input of a contained reactor.
							generateVariablesForSendingToContainedReactors(
								reactionInitialization, reaction.produces, reactor, split
							)
						}
					}
				}
			}
			pr('void ' + functionName + '(void* instance_args) {')
			indent()
			pr(reactionInitialization)
			// Code verbatim from 'reaction'
			prSourceLineNumber(reaction)
			pr(removeCodeDelimiter(reaction.code))
			unindent()
			pr("}")
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
		var triggerNameToTriggerStruct = new HashMap<String,String>()
		var instance = reactorInstance.instanceStatement
		// If there is no instance statement, then this is main.
		var reactor = getReactor(instance.reactorClass)
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
		val reactionToReactionTName = new HashMap<Reaction,String>()
		reactorInstance.properties.put(
				"reactionToReactionTName", reactionToReactionTName)

		var count = 0
		for (triggerName: triggerToReactions.keySet) {
			val numberOfReactionsTriggered = triggerToReactions.get(triggerName).length
			var names = new StringBuffer();
			
			// Generate reaction_t object
			for (reaction : triggerToReactions.get(triggerName)) {
				var functionName = properties.targetProperties.get(reaction)
				pr(result, '// --- Reaction and trigger objects for reaction to trigger '+ triggerName
					+ ' of instance ' + instance.name
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
					if (reaction.produces !== null) {
						for(output: reaction.produces.produces) {
							if (getOutput(reactor, output) !== null) {
								// It is an output, not an action.
								// First create the array of pointers to booleans indicating whether
								// an output is produced.
								// Insert a comma if needed.
								if (presentPredicates.length > 0) {
									presentPredicates.append(", ")
								}
								presentPredicates.append('&' + nameOfSelfStruct + '.__' + output + '_is_present')
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
									var outputName = instance.getName() + "." + output
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
									var remoteTriggersArrayName = reactionInstanceName + '_' + outputCount + '_remote_triggers'
									var inputCount = 0;
									for (inputName: inputNames) {
										deferredInitialize.add(
											new InitializeRemoteTriggersTable(
												container, remoteTriggersArrayName, (inputCount++), inputName
											)
										)
									}
									pr(result, 'trigger_t* '
										+ remoteTriggersArrayName
										+ '['
										+ inputCount
										+ '];'
									)
									triggersContents.append('&' + remoteTriggersArrayName + '[0]')
								}
							} else {
								// It is not an output, but the reaction may be sending data
								// to the input of a contained reactor. Check first to see whether
								// it has the right form.
								var portSpec = output.split('\\.')
								if (portSpec.length === 2) {
									// The form is right.
									// First create the array of pointers to booleans indicating whether
									// an output is produced.
									// Insert a comma if needed.
									if (presentPredicates.length > 0) {
										presentPredicates.append(", ")
									}
									presentPredicates.append('&' + nameOfSelfStruct + '.__' 
										+ portSpec.get(0) + '_' + portSpec.get(1) + '_is_present'
									)
									outputCount++
									
									var destinationPort = getInputPortOfContainedReactor(
										reactor, portSpec.get(0), portSpec.get(1), reaction.produces
									)
									// Insert a comma if needed.
									if (triggeredSizesContents.length > 0) {
										triggeredSizesContents.append(", ")
									}
									triggeredSizesContents.append("1")
									if (destinationPort === null) {
										triggersContents.append("NULL")										
									} else {
										var remoteTriggersArrayName = reactionInstanceName + '_' + outputCount + '_remote_triggers'
										var destinationInstance = reactorInstance.getContainedInstance(portSpec.get(0))
										if (destinationInstance !== null) {
											// Null destinationInstance is an error, but it should have been caught
											// before this, so no need to report it now.
											deferredInitialize.add(
												new InitializeRemoteTriggersTable(
													destinationInstance, remoteTriggersArrayName, 0, destinationPort.name
												)
											)
										}
										pr(result, 'trigger_t* '
											+ remoteTriggersArrayName
											+ '[1];'
										)
										triggersContents.append('&' + remoteTriggersArrayName + '[0]')
									}
								}
							}
						}
					}
					var outputProducedArray = "NULL"
					var triggeredSizesArray = "NULL"
					var triggersArray = "NULL"
					if (outputCount > 0) {
						outputProducedArray = '&' + reactionInstanceName + '_outputs_are_present'
						// Create a array with booleans indicating whether an output has been produced.
						pr(result, 'bool* ' + reactionInstanceName
							+ '_outputs_are_present' 
							+ ' = {' 
							+ presentPredicates.toString
							+ '};'
						)
						// Create a array with ints indicating these
						// numbers and assign it to triggered_reactions_sizes
						// field of the reaction_t object.
						triggeredSizesArray = '&' + reactionInstanceName + '_triggered_sizes[0]'
						pr(result, 'int ' + reactionInstanceName
							+ '_triggered_sizes' 
							+ '[] = {'
							+ triggeredSizesContents
							+ '};'
						)
						// Create an array with pointers to arrays of pointers to trigger_t
						// structs for each input triggered by an output.
						triggersArray = '&' + reactionInstanceName + '_triggers[0]'
						pr(result, 'trigger_t** ' + reactionInstanceName + '_triggers'
							+ '[] = {'
							+ triggersContents
							+ '};'
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
					pr(result, "reaction_t " + reactionInstanceName 
						+ " = {&" + functionName 
						+ selfStructArgument
						+ ", 0"  // index: index from the topological sort.
						+ ", 0"  // pos: position used by the pqueue implementation for sorting.
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
				if (names.length != 0) {
					names.append(", ")
				}
				names.append('&' + reactionInstanceName)
			}
			var triggerStructName = triggerName + triggerCount
			
			// Record the triggerStructName.
			triggerNameToTriggerStruct.put(triggerName, triggerStructName)
			
			pr(result, 'reaction_t* ' + triggerStructName 
					+ '_reactions[' + numberOfReactionsTriggered + '] = {' + names + '};')
			// Declare a variable with the name of the trigger whose
			// value is a struct.
			pr(result, 'trigger_t ' + triggerStructName + ' = {')
			indent(result)
			var timing = getTiming(reactor, triggerName)
			if (timing !== null || getInput(reactor, triggerName) !== null) {
				pr(result, triggerStructName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ '0LL, 0LL'
				)
			} else if (getAction(reactor, triggerName) !== null) {
				var modifier = getAction(reactor, triggerName).getModifier();
				var isPhysical = "false";
				if (modifier == "physical") {
					isPhysical = "true";
				}
				pr(result, triggerStructName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ getAction(reactor, triggerName).getDelay()
					+ ', 0LL, NULL, ' 
					+ isPhysical // 0 is ignored since actions don't have a period.
				)
			} else {
				reportError(reactor,
					"Internal error: Seems to not be an input, timer, or action: "
					+ triggerName)
			}
			unindent(result)
			pr(result,'};')
			// Assignment of the offset and period have to occur after creating
			// the struct because the value assigned may not be a compile-time constant.
			if (timing !== null) {
				pr(initializeTriggerObjects, triggerStructName + '.offset = ' + timeMacro(timing.offset) + ';')
				pr(initializeTriggerObjects, triggerStructName + '.period = ' + timeMacro(timing.period) + ';')
				
				// Generate a line to go into the __start_timers() function.
				// Note that the delay, the second argument, is zero because the
				// offset is already in the trigger struct.
				pr(startTimers,"__schedule(&" + triggerStructName + ", 0LL, NULL);")
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
		Hashtable<String,String> importTable
	) {
		var className = importTable.get(instance.reactorClass);
		if (className === null) {
			className = instance.reactorClass
		}
		pr('// ************* Instance ' + instance.name + ' of class ' + className)
		var reactor = getReactor(instance.reactorClass)
		if (reactor === null) {
			reportError(instance, "No such reactor: " + instance.reactorClass)
			return null
		}

		// Generate the instance struct containing parameters and state variables.
		// (the "self" struct).
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
		var overrides = new HashMap<String,String>()
		var parameters = instance.parameters
		if (parameters !== null) {
			for (assignment: parameters.assignments) {
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
		for(parameter: getParameters(reactor)) {
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
			pr(initializeTriggerObjects, getParameterType(parameter) + ' ' + parameter.name + ' = ' + tmpVariableName + ';')
			pr(initializeTriggerObjects, nameOfSelfStruct + "." + parameter.name + " = " + value + ";")
		}
		// Next, initialize the "self" struct with state variables.
		for(state: reactor.states) {
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
				
		// Next, initialize the struct with actions.
		for(action: reactor.actions) {
			var triggerStruct = triggerNameToTriggerStruct.get(action.name)
			if (triggerStruct === null) {
				reportError(reactor, 
					"Internal error: No trigger struct found for action "
					+ action.name)
			}
			// FIXME: Actions may have payloads.
			pr(initializeTriggerObjects, nameOfSelfStruct + '.__' 
				+ action.name + ' = &' + triggerStruct + ';'
			)
		}
		// Next, generate the code to initialize outputs at the start
		// of a time step to be absent.
		for(output: reactor.outputs) {
			pr(startTimeStep, nameOfSelfStruct
				+ '.__' + output.name + '_is_present = false;'
			)
		}
		
		// Finally, handle deadline commands.
		for(deadline: reactor.deadlines) {
			var split = deadline.port.split('\\.')
			if (split.length !== 2) {
				reportError(deadline, 'Malformed input port specification: ' + deadline.port)
			} else {
				var deadlineReactor = reactorInstance.getContainedInstance(split.get(0))
				if (deadlineReactor === null) {
					reportError(deadline, "No such reactor: " + split.get(0))
				} else {
					var triggerToReactions = getTriggerToReactions(deadlineReactor.reactor)
					var reactions = triggerToReactions.get(split.get(1))
					if (reactions === null) {
						reportError(deadline, "No such port: " + deadline.port)
					} else {
						for (reaction: reactions) {
							var reactionToReactionTName =
									deadlineReactor.properties.get("reactionToReactionTName")
							var reactionTName = (reactionToReactionTName as HashMap<Reaction,String>).get(reaction)
							if (reactionTName === null) {
								reportError(deadline, "Internal error: No reaction_t object found for reaction.")
							} else {
								pr(initializeTriggerObjects,
										reactionTName + '.deadline = ' + timeMacro(deadline.delay) + ';')
								
								// Next, set the deadline_violation field to point to the trigger_t struct.
								var triggerMap = reactorInstance.properties.get("triggerNameToTriggerStruct")
								if (triggerMap === null) {
									reportError(deadline,
											"Internal error: failed to map from name to trigger struct for "
											+ reactorInstance.getFullName()
									)
								} else {
									var triggerStructName = (triggerMap as HashMap<String,String>).get(deadline.action)
									if (triggerStructName === null) {
										reportError(reactorInstance.reactor,
											"Internal error: failed to find trigger struct for action "
											+ deadline.action
											+ " in reactor "
											+ reactorInstance.getFullName()
										)
									} else {
										pr(initializeTriggerObjects,
											reactionTName + '.deadline_violation = &' + triggerStructName + ';')
									}
								}
							}
						}	
					}					
				}
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
		for (ReactionInstance instance: graph.nodes) {
			val reactorInstance = instance.reactorInstance;
			val map = reactorInstance.properties.get("reactionToReactionTName") as HashMap<Reaction,String>;
			val reactionTName = map.get(instance.reactionSpec);
			pr(reactionTName + ".index = " + instance.level + ";")			
		}
	}
	
	////////////////////////////////////////////
	//// Utility functions for generating code.
	
	/** Perform deferred initializations in initialize_trigger_objects. */
	private def doDeferredInitialize() {
		// First, populate the trigger tables for each output.
		// The entries point to the trigger_t structs for the destination inputs.
		pr('// doDeferredInitialize')
		for (init: deferredInitialize) {
			// The reactor containing the specified input may be a contained reactor.
			var reactor = init.reactor
			var port = init.inputName
			var split = init.inputName.split('\\.')
			if (split.length === 2) {
				reactor = init.reactor.getContainedInstance(split.get(0))
				if (reactor === null) {
					reportError(init.reactor.reactor, "No reactor named: "
						+ split.get(0)
						+ " in container "
						+ init.reactor.getFullName()
					)
				}
				port = split.get(1)
			} else if (split.length !== 1) {
				reportError(init.reactor.reactor, "Invalid input specification: " + init.inputName)
			}
			if (reactor !== null) {
				var triggerMap = reactor.properties.get("triggerNameToTriggerStruct")
				if (triggerMap === null) {
					reportError(init.reactor.reactor,
						"Internal error: failed to map from name to trigger struct for "
						+ init.reactor.getFullName()
					)
				} else {
					var triggerStructName = (triggerMap as HashMap<String,String>).get(port)
					if (triggerStructName === null) {
						reportError(init.reactor.reactor,
							"Internal error: failed to find trigger struct for input "
							+ port
							+ " in reactor "
							+ reactor.getFullName()
						)
					}
					pr(init.remoteTriggersArrayName + '['
						+ init.arrayIndex
						+ '] = &'
						+ triggerStructName
						+ ';'
					)
				}
			}
		}
		// Next, for every input port, populate its "self" struct
		// fields with pointers to the output port that send it data.
		connectInputsToOutputs(main)
	}
	
	// Generate assignments of pointers in the "self" struct of a destination
	// port's reactor to the appropriate entries in the "self" struct of the
	// source reactor.
	private def void connectInputsToOutputs(ReactorInstance container) {
		// Collect the set of inputs that have connections so that we can
		// later handle dangling inputs, ensuring that they are always "absent".
		var connectedInputs = new HashSet<String>()
		
		for (containedReactor: container.containedInstances.values()) {
			// In case this is a composite, handle its assignments.
			connectInputsToOutputs(containedReactor)
			var containerProperties = reactorToProperties.get(containedReactor.container.reactor)
			for (output: containedReactor.reactor.outputs) {
				var outputSelfStructName = containedReactor.properties.get("selfStructName")
				var inputNames = containerProperties.outputNameToInputNames.get(containedReactor.name + '.' + output.name)
				if (inputNames !== null) {
					for(input: inputNames) {
						if (connectedInputs.contains(input)) {
							reportError(output, "Connecting to an input that already has a connection: " + input)
						}
						connectedInputs.add(input)
						var split = input.split('\\.')
						if (split.length === 2) {
							var inputReactor = containedReactor.container.getContainedInstance(split.get(0))
							if (inputReactor !== null) {
								// It is an error to be null, but it should have been caught earlier.
								var inputSelfStructName = inputReactor.properties.get("selfStructName")
								pr(inputSelfStructName + '.__' + split.get(1) + ' = &'
									+ outputSelfStructName + '.__' + output.name + ';'
								)
								pr(inputSelfStructName + '.__' + split.get(1) + '_is_present = &'
									+ outputSelfStructName + '.__' + output.name + '_is_present;'
								)
							}
						} else {
						    println("************ FIXME")
							// FIXME: Handle case where size is not 2 (communication across hierarchy).
							reportError(container.reactor,
								"FIXME: Communication across hierarchy is not yet supported"
							)
							// var outputProperties = reactorToProperties.get(containedReactor.reactor)
						}
					}
				}
			}
		}
		// Handle inputs that get sent data from a reaction rather than from
		// another contained reactor.
		for (reaction: container.reactor.reactions) {
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for (produces: reaction.produces.produces) {
					var split = produces.split('\\.')
					if (split.length === 2) {
						// Found an input that is sent data from a reaction.
						if (connectedInputs.contains(produces)) {
							reportError(reaction, "Sending to an input that already has a connection: " + produces)
						}
						connectedInputs.add(produces)
						var inputReactor = container.getContainedInstance(split.get(0))
						if (inputReactor === null) {
							reportError(reaction, "No such destination reactor: " + split.get(0))
						} else {
							var inputSelfStructName = inputReactor.properties.get("selfStructName")
							var containerSelfStructName = container.properties.get("selfStructName")
							pr(inputSelfStructName + '.__' + split.get(1) + ' = &'
								+ containerSelfStructName + '.__' + split.get(0) + '_' + split.get(1) + ';'
							)
							pr(inputSelfStructName + '.__' + split.get(1) + '_is_present = &'
								+ containerSelfStructName + '.__' + split.get(0) + '_' + split.get(1) + '_is_present;'
							)
							pr(startTimeStep, containerSelfStructName + '.__' + split.get(0) + '_' + split.get(1) 
								+ '_is_present = false;'
							)
						}						
					}
				}
			}
		}
		
		// Handle dangling input ports that are not connected to anything.
		for (containedReactor: container.containedInstances.values()) {		
			for (input: containedReactor.reactor.inputs) {
				var inputName = containedReactor.name + '.' + input.name
				if (!connectedInputs.contains(inputName)) {
					// Input is dangling.
					var inputReactor = containedReactor.container.getContainedInstance(containedReactor.name)
					var inputSelfStructName = inputReactor.properties.get("selfStructName")
					pr(inputSelfStructName + '.__' + input.name + '_is_present = &False;')
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
		StringBuilder builder, Input input
	) {
		var present = input.name + '_is_present'
		pr(builder, 'bool ' + present + ' = *(self->__' + input.name + '_is_present);')
		pr(builder, removeCodeDelimiter(input.type)	+ ' ' + input.name + ';')
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
		StringBuilder builder, Output output
	) {
		if (output.type === null) {
			reportError(output, "Output is required to have a type: " + output.name)
		}		
		// Slightly obfuscate the name to help prevent accidental use.
		pr(builder, removeCodeDelimiter(output.type)
			+ '* ' + output.name + ' = &(self->__' + output.name + ');'
		)
		pr(builder, 'bool* ' + output.name
			+ '_is_present = &(self->__' + output.name + '_is_present);'
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
	private def generateVariablesForSendingToContainedReactors(
		StringBuilder builder, Produces produces, Reactor reactor, String[] portSpec
	) {
		// Get a port matching the portName.
		var destinationPort = getInputPortOfContainedReactor(
			reactor, portSpec.get(0), portSpec.get(1), produces
		)
		
		if (destinationPort === null) {
			reportError(produces, "Destination port not found: " + portSpec.get(0) + "." + portSpec.get(1))
			return
		}
				
		// Need to create a struct so that the port can be referenced in C code
		// as reactorName.portName.
		// FIXME: This means that the destination instance name cannot match
		// any input port, output port, or action, because we will get a name collision.
		pr(builder, 'struct ' + portSpec.get(0) + ' {'
			+ removeCodeDelimiter(destinationPort.type)
			+ '* ' + portSpec.get(1) + '; '
			+ 'bool* ' + portSpec.get(1) + '_is_present;} '
			+ portSpec.get(0)
			+ ';'
		)
		pr(builder, portSpec.get(0) + '.' + portSpec.get(1)
					+ ' = &(self->__' + portSpec.get(0) + '_' + portSpec.get(1) + ');')
		pr(builder, portSpec.get(0) + '.' + portSpec.get(1)	+ '_is_present'				
					+ ' = &(self->__' + portSpec.get(0) + '_' + portSpec.get(1) + '_is_present);')
	}
	
	/** Given a container reactor, a reactor name, and a port name, return
	 *  the Input statement that it corresponds to, or report an error and
	 *  return null if there is no such input.
	 *  @param container A composite reactor.
	 *  @param reactorName The name of a contained reactor.
	 *  @param portName The name of an input port of the contained reactor.
	 *  @param report The AST object on which to report an error.
	 */
	private def getInputPortOfContainedReactor(
		Reactor container, String reactorName, String portName, EObject report
	) {
		// First, find an instance whose name matches the reactorName.
		var instance = container.getInstance(reactorName)
		if (instance === null) {
			reportError(report, "No instance named: " + reactorName)
			return null as Input
		}
		
		// Next, need to find the reactor definition referenced.
		var containedReactor = getReactor(instance.reactorClass)
		if (containedReactor === null) {
			reportError(report, "Cannot find reactor definition for: "
				+ instance.reactorClass
			)
			return null as Input
		}
		
		// Next, get a port matching the portName.
		var destinationPort = containedReactor.getInput(portName)
		if (destinationPort === null) {
			reportError(report, "Destination port does not have an input named: " 
				+ portName
			)
			return null as Input
		}
		destinationPort
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
	private def void processImports(Hashtable<String,String> importTable) {
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
			}
		}
	}

	// Print the #line compiler directive with the line number of
	// the most recently used node.
	private def prSourceLineNumber(EObject reaction) {
		var node = NodeModelUtils.getNode(reaction)
		pr("#line " + node.getStartLine() + ' "' + _resource.getURI() + '"')
		
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
