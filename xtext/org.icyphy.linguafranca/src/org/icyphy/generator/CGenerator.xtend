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
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
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
			
	// Place to collect code to initialize the trigger objects for all reactors.
	var initializeTriggerObjects = new StringBuilder()
	
	// List of deferred assignments to perform in initialize_trigger_objects.
	var deferredInitialize = new LinkedList<InitiatlizeRemoteTriggersTable>();
	
	// Place to collect code to execute at the start of a time step.
	var startTimeStep = new StringBuilder()
	
	// Place to collect code to initialize timers for all reactors.
	var startTimers = new StringBuilder()
			
	var Resource _resource;
	
	/** Generate C code from the Lingua Franca model contained by the
	 *  specified resource. This is the main entry point for code
	 *  generation.
	 *  @param resource The resource containing the source code.
	 *  @param fsa The file system access (used to write the result).
	 *  @param context FIXME
	 *  @param importTable The mapping given by import statements.
	 */
	override void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
				
		_resource = resource
		// Figure out the file name for the target code from the source file name.
		var filename = extractFilename(_resource.getURI.toString)
		
		pr(includes)
		
		super.doGenerate(resource, fsa, context, importTable)
		
		// Generate function to initialize the trigger objects for all reactors.
		pr('void __initialize_trigger_objects() {\n')
		indent()
		pr(initializeTriggerObjects)
		doDeferredInitialize()
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
		
		fsa.generateFile(filename + ".c", getCode())		
	}
	
	////////////////////////////////////////////
	//// Code generators.
	
	/** Generate a reactor or composite class definition.
	 *  @param component The parsed component data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	override generateComponent(Component component, Hashtable<String,String> importTable) {
		super.generateComponent(component, importTable)
		
		pr("// =============== START reactor class " + component.componentBody.name)
				
		// Scan reactions
		var savedReactionCount = reactionCount;
				
		// Preamble code contains state declarations with static initializers.
		if (component.componentBody.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(component.componentBody.preamble.code))
			pr("\n// *********** End of preamble.")
		}
		
		var properties = componentToProperties.get(component)
		
		// Put parameters into a struct and construct the code to go
		// into the preamble of any reaction function to extract the
		// parameters from the struct.
		val argType = "reactor_instance_" + (reactorClassCount++) + "_this_t"
			
		// Construct the typedef for the "this" struct.
		// NOTE: The struct cannot be empty in C; should we make a dummy field or suppress the struct?
		var body = new StringBuilder()
		// Start with parameters.
		for(parameter: getParameters(component)) {
			prSourceLineNumber(parameter)
			if (parameter.type === null) {
				reportError(parameter, "Parameter is required to have a type: " + parameter.name)
			} else {
				pr(body, getParameterType(parameter) + ' ' + parameter.name + ';');
			}
		}
		// Next handle states.
		for(state: component.componentBody.states) {
			prSourceLineNumber(state)
			if (state.type === null) {
				reportError(state, "State is required to have a type: " + state.name)
			} else {
				pr(body, removeCodeDelimiter(state.type) + ' ' + state.name + ';');
			}
		}
		// Next handle actions.
		for(action: component.componentBody.actions) {
			prSourceLineNumber(action)
			// NOTE: Slightly obfuscate output name to help prevent accidental use.
			pr(body, "trigger_t* __" + action.name + ";")
		}
		// Next handle inputs.
		for(input: component.componentBody.inputs) {
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
		for(output: component.componentBody.outputs) {
			prSourceLineNumber(output)
			if (output.type === null) {
				reportError(output, "Output is required to have a type: " + output.name)
			} else {
				// NOTE: Slightly obfuscate output name to help prevent accidental use.
				pr(body, removeCodeDelimiter(output.type) + ' __' + output.name + ';');
				pr(body, 'bool __' + output.name + '_is_present;');
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
		generateReactions(component)	
		pr("// =============== END reactor class " + component.componentBody.name)
		pr("")
	}

	/** Generate reaction functions definition for a reactor or a composite.
	 *  These functions have a single argument that is a void* pointing to
	 *  a struct that contains parameters, inputs (triggering or not),
	 *  actions (triggering or produced), and outputs.
	 *  @param component The component (reactor or composite).
	 */
	def generateReactions(Component component) {
		var reactions = component.componentBody.reactions
		var properties = componentToProperties.get(component)
		for (reaction: reactions) {
			// Create a unique function name for each reaction.
			val functionName = "reaction_function" + reactionCount++
			
			properties.targetProperties.put(reaction, functionName)
			
			// Construct the reactionInitialization code to go into
			// the body of the function before the verbatim code.
			// This defines the "this" struct.
			var StringBuilder reactionInitialization = new StringBuilder()
			var structType = properties.targetProperties.get("structType")
			pr(reactionInitialization, structType
					+ "* this = (" + structType + "*)instance_args;")
			
			// Next, add the triggers (input and actions; timers are not needed).
			// This defines a local variable in the reaction function whose
			// name matches that of the trigger. If the trigger is an input
			// (not an action), then it also defines a local variable whose
			// name is the input name with suffix "_is_present", a boolean
			// that indicates whether the input is present.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					val input = getInput(component, trigger)
					if (input !== null) {
						generateInputVariablesInReaction(reactionInitialization, input)
					}
					val action = getAction(component, trigger)
					if (action !== null) {
						// FIXME: Actions may have payloads.
						pr(reactionInitialization, "trigger_t* " 
							+ action.name + ' = this->__' + action.name + ';'
						);
					}
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare an argument for every input.
				for (input: component.componentBody.inputs) {
					generateInputVariablesInReaction(reactionInitialization, input)
				}
			}
			// Define argument for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					val input = getInput(component, get)
					generateInputVariablesInReaction(reactionInitialization, input)
				}
			}
			// Define variables for each declared output or action.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					val action = getAction(component, output)
					if (action !== null) {
						// It is an action, not an output.
						// FIXME: Actions may have payloads.
						pr(reactionInitialization, "trigger_t* " + action.name + ' = this->__' + action.name + ';');
					} else {
						// It is an output.
						var out = getOutput(component, output)
						generateOutputVariablesInReaction(reactionInitialization, out)
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
	 *  each input, clock, and action of the reactor.
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
	 *  @param nameOfThisStruct The name of the instance of "this" for this instance.
	 *  @return A map of trigger names to the name of the trigger struct.
	 */
	def generateTriggerObjects(ReactorInstance reactorInstance, String nameOfThisStruct) {
		var triggerNameToTriggerStruct = new HashMap<String,String>()
		var instance = reactorInstance.instanceStatement
		// If there is no instance statement, then this is main.
		var component = getComponent(instance.reactorClass)
		var properties = componentToProperties.get(component)
		if (component === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return triggerNameToTriggerStruct
		}
		var triggerToReactions = getTriggerToReactions(component)
		if (triggerToReactions === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return triggerNameToTriggerStruct
		}
		val result = new StringBuilder()

		var count = 0
		for (triggerName: triggerToReactions.keySet) {
			val numberOfReactionsTriggered = triggerToReactions.get(triggerName).length
			var names = new StringBuffer();
			for (reaction : triggerToReactions.get(triggerName)) {
				var functionName = properties.targetProperties.get(reaction)
				pr(result, '// --- Reaction and trigger objects for reaction to trigger '+ triggerName
					+ ' of instance ' + instance.name
				)
				// Generate a reaction_t object for an instance of a reaction.
				val reactionInstanceName = "__reaction" + reactionInstanceCount++
				
				// Generate entries for the reaction_t struct that specify how
				// to handle outputs.
				var presentPredicates = new StringBuilder()
				var triggeredSizesContents = new StringBuilder()
				var triggersContents = new StringBuilder()
				var outputCount = 0
				if (reaction.produces !== null) {
					for(output: reaction.produces.produces) {
						if (getOutput(component, output) !== null) {
							// It is an output, not an action.
							// First create the array of pointers to booleans indicating whether
							// an output is produced.
							// Insert a comma if needed.
							if (presentPredicates.length > 0) {
								presentPredicates.append(", ")
							}
							presentPredicates.append('&' + nameOfThisStruct + '.__' + output + '_is_present')
							outputCount++
							
							// For each output, figure out how many
							// inputs are connected to it. This is obtained via the container.
							var container = reactorInstance.container
							// If there is no container, then the output cannot be connected
							// to anything, so default to empty set.
							var inputNames = new HashSet<String>()
							if (container !== null) {
								var parentComponent = container.component
								var parentProperties = componentToProperties.get(parentComponent)
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
								var inputTriggerStructPointers = new StringBuilder()
								var remoteTriggersArrayName = reactionInstanceName + '_' + outputCount + '_remote_triggers'
								var inputCount = 0;
								for (inputName: inputNames) {
									// Insert a comma if needed.
									if (inputTriggerStructPointers.length > 0) {
										inputTriggerStructPointers.append(', ')
									}
									deferredInitialize.add(
										new InitiatlizeRemoteTriggersTable(
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
								triggersContents.append('&' + remoteTriggersArrayName)
							}
							
							// Then, generate code to run in the __initialize_trigger_objects
							// function that initializes each of these blank
							// arrays with pointers to the reaction_t objects
							// for each of the destination inputs.
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
					triggeredSizesArray = '&(' + reactionInstanceName + '_triggered_sizes[0])'
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
						+ '[0]};'
					)
				}
								
				// FIXME: first 0 is an index that should come from the topological sort.
				pr(result, "reaction_t " + reactionInstanceName 
					+ " = {&" + functionName 
					+ ", &" + nameOfThisStruct
					+ ", 0"  // index: index from the topological sort.
					+ ", 0"  // pos: position used by the pqueue implementation for sorting.
					+ ", " + outputCount // num_outputs: number of outputs produced by this reaction.
					+ ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
					+ ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
					+ ", " + triggersArray // triggered: array of pointers to arrays of triggers.
					+ "};"
				)
				
				// Position is a label to be written by the priority queue as a side effect of inserting.
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
			var timing = getTiming(component, triggerName)
			if (timing !== null || getInput(component, triggerName) !== null) {
				pr(result, triggerStructName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ '0LL, 0LL'
				)
			} else if (getAction(component, triggerName) !== null) {
				pr(result, triggerStructName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ getAction(component, triggerName).getDelay()
					+ ', 0LL' // 0 is ignored since actions don't have a period.
				)
			} else {
				reportError(component,
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
				pr(startTimers,"__schedule(&" + triggerStructName + ", "
		 			+ timeMacro(timing.offset) + ");")
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
		var component = getComponent(instance.reactorClass)
				
		// Generate the instance struct containing parameters and state variables.
		// (the "this" struct).
		var properties = componentToProperties.get(component)
		var nameOfThisStruct = "__this_" + instanceCount + "_" + instance.name
		var structType = properties.targetProperties.get("structType")
		if (structType !== null) {
			pr('// --- "this" struct for instance ' + instance.name)
			pr(structType + " " + nameOfThisStruct + ";")
		}
		
		// Generate code to initialize the "this" struct in the
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
		for(parameter: getParameters(component)) {
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
			pr(initializeTriggerObjects, nameOfThisStruct + "." + parameter.name + " = " + value + ";")
		}
		// Next, initialize the struct with state variables.
		for(state: component.componentBody.states) {
			var value = removeCodeDelimiter(state.value)
			pr(initializeTriggerObjects, nameOfThisStruct + "." + state.name + " = " + value + ";")
		}
		
		// Call superclass here so that parameters of this composite
		// are in scope for contained instances.
		var reactorInstance = super.instantiate(instance, container, importTable)
		// Store the name of the "this" struct as a property of the instance
		// so that it can be used when establishing connections.
		reactorInstance.properties.put("thisStructName", nameOfThisStruct)
		
		// Generate trigger objects for the instance.
		var triggerNameToTriggerStruct = generateTriggerObjects(reactorInstance, nameOfThisStruct)
		reactorInstance.properties.put("triggerNameToTriggerStruct", triggerNameToTriggerStruct)
				
		// Next, initialize the struct with actions.
		for(action: component.componentBody.actions) {
			var triggerStruct = triggerNameToTriggerStruct.get(action.name)
			if (triggerStruct === null) {
				reportError(component, 
					"Internal error: No trigger struct found for action "
					+ action.name)
			}
			// FIXME: Actions may have payloads.
			pr(initializeTriggerObjects, nameOfThisStruct + '.__' 
				+ action.name + ' = &' + triggerStruct + ';'
			)
		}
		// Next, generate the code to initialize outputs at the start
		// of a time step to be absent.
		for(output: component.componentBody.outputs) {
			pr(startTimeStep, nameOfThisStruct
				+ '.__' + output.name + '_is_present = false;'
			)
		}
		
		unindent(initializeTriggerObjects)
		pr(initializeTriggerObjects, "} // End of scope for " + instance.name)
		
		instanceCount++
		
		reactorInstance
	}
	
	////////////////////////////////////////////
	//// Utility functions for generating code.
	
	/** Perform deferred initializations in initialize_trigger_objects. */
	private def doDeferredInitialize() {
		// First, populate the trigger tables for each output.
		// The entries point to the trigger_t structs for the destination inputs.
		for (init: deferredInitialize) {
			// The reactor containing the specified input may be the
			// composite or a reactor contained by the composite.
			var reactor = init.composite
			var port = init.inputName
			var split = init.inputName.split('\\.')
			if (split.length === 2) {
				reactor = init.composite.getContainedInstance(split.get(0))
				if (reactor === null) {
					reportError(init.composite.component, "No reactor named: "
						+ split.get(0)
						+ " in container "
						+ init.composite.getFullName()
					)
				}
				port = split.get(1)
			} else if (split.length !== 1) {
				reportError(init.composite.component, "Invalid input specification: " + init.inputName)
			}
				
			var triggerMap = reactor.properties.get("triggerNameToTriggerStruct")
			if (triggerMap === null) {
				reportError(init.composite.component,
					"Internal error: failed to map from name to trigger struct for "
					+ init.composite.getFullName()
				)
			} else {
				var triggerStructName = (triggerMap as HashMap<String,String>).get(port)
				if (triggerStructName === null) {
					reportError(init.composite.component,
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
		// Next, for every input port, populate its "this" struct
		// fields with pointers to the output port that send it data.
		connectInputsToOutputs(main)
	}
	
	// Generate assignments of pointers in the "this" struct of a destination
	// port's reactor to the appropriate entries in the "this" struct of the
	// source reactor.
	private def void connectInputsToOutputs(ReactorInstance container) {
		// FIXME: What to do with dangling input ports that are not connected to anything?
		for (containedReactor: container.containedInstances.values()) {
			// In case this is a composite, handle its assignments.
			connectInputsToOutputs(containedReactor)
			var outputProperties = componentToProperties.get(containedReactor.component)
			var containerProperties = componentToProperties.get(containedReactor.container.component)
			for (output: containedReactor.component.componentBody.outputs) {
				var outputThisStructName = containedReactor.properties.get("thisStructName")
				var inputNames = containerProperties.outputNameToInputNames.get(containedReactor.name + '.' + output.name)
				if (inputNames !== null) {
					for(input: inputNames) {
						var split = input.split('\\.')
						if (split.length === 2) {
							var inputReactor = containedReactor.container.getContainedInstance(split.get(0))
							var inputThisStructName = inputReactor.properties.get("thisStructName")
							pr(inputThisStructName + '.__' + split.get(1) + ' = &'
								+ outputThisStructName + '.__' + output.name + ';'
							)
							pr(inputThisStructName + '.__' + split.get(1) + '_is_present = &'
								+ outputThisStructName + '.__' + output.name + '_is_present;'
							)
						} else {
							// FIXME: Handle case where size is not 2 (communication across hierarchy).
							reportError(container.component,
								"FIXME: Communication across hierarchy is not yet supported"
							)
						}
					}
				}
			}
		}
	}

	// Extract a filename from a path.
	private def extractFilename(String path) {
		var result = path
		if (path.startsWith('platform:')) {
			result = result.substring(9)
		}
		var lastSlash = result.lastIndexOf('/')
		if (lastSlash >= 0) {
			result = result.substring(lastSlash + 1)
		}
		if (result.endsWith('.lf')) {
			result = result.substring(0, result.length - 3)
		}
		return result
	}
	
	/** Generate into the specified string builder the code to
	 *  initialize local variables for inputs in a reaction function
	 *  from the "this" struct.
	 *  @param builder The string builder.
	 *  @param input The input statement from the AST.
	 */
	private def generateInputVariablesInReaction(
		StringBuilder builder, Input input
	) {
		// Slightly obfuscate the name to help prevent accidental use.
		pr(builder, removeCodeDelimiter(input.type)
			+ ' ' + input.name + ' = *(this->__' + input.name + ');'
		)
		pr(builder, removeCodeDelimiter(input.type)
			+ ' ' + input.name + '_is_present = *(this->__' + input.name + '_is_present);'
		)
	}
	
	/** Generate into the specified string builder the code to
	 *  initialize local variables for outputs in a reaction function
	 *  from the "this" struct.
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
			+ '* ' + output.name + ' = &(this->__' + output.name + ');'
		)
		pr(builder, 'bool* ' + output.name
			+ '_is_present = &(this->__' + output.name + '_is_present);'
		)
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
	
	/** Given a string of form either "xx" or "xx.yy", return either
	 *  "'xx'" or "xx, 'yy'".
	 */
	// FIXME: Not used.
	def portSpec(String port) {
        val a = port.split('\\.');
        if (a.length == 1) {
            "'" + a.get(0) + "'"
        } else if (a.length > 1) {
            a.get(0) + ", '" + a.get(1) + "'"
        } else {
            "INVALID_PORT_SPEC:" + port
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
	
	// FIXME: pqueue.h and pqueue.c need to be copied to target directory.
	val static includes = '''
		#include "reactor.h"
	'''
}
