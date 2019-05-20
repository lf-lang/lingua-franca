/*
 * Generator for C target.
 */
package org.icyphy.generator

import java.util.HashMap
import java.util.Hashtable
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.LinguaFrancaFactory
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
	
	// Place to collect code to execute at the start of a time step.
	var startTimeStep = new StringBuilder()
	
	// Place to collect code to initialize timers for all reactors.
	var startTimers = new StringBuilder()
			
	var Resource _resource;
	
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
				
		_resource = resource
		// Figure out the file name for the target code from the source file name.
		var filename = extractFilename(_resource.getURI.toString)
		
		pr(includes)
			
		// Handle reactors and composites.
		for (component : resource.allContents.toIterable.filter(Component)) {
			generateComponent(component, importTable)
		}
		
		// Generate function to initialize the trigger objects for all reactors.
		pr('void __initialize_trigger_objects() {\n')
		indent()
		pr(initializeTriggerObjects)
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
		properties.targetProperties.put("structType", argType)
			
		// Construct the typedef for the "this" struct.
		pr("typedef struct {")
		indent()
		// Start with parameters.
		for(parameter: getParameters(component)) {
			prSourceLineNumber(parameter)
			if (parameter.type === null) {
				reportError(parameter, "Parameter is required to have a type: " + parameter.name)
			} else {
				pr(getParameterType(parameter) + ' ' + parameter.name + ';');
			}
		}
		// Next handle states.
		for(state: component.componentBody.states) {
			prSourceLineNumber(state)
			if (state.type === null) {
				reportError(state, "State is required to have a type: " + state.name)
			} else {
				pr(removeCodeDelimiter(state.type) + ' ' + state.name + ';');
			}
		}
		// Next handle actions.
		for(action: component.componentBody.actions) {
			prSourceLineNumber(action)
			// NOTE: Slightly obfuscate output name to help prevent accidental use.
			pr("trigger_t* __" + action.name + ";")
		}
		// Next handle inputs.
		for(input: component.componentBody.inputs) {
			prSourceLineNumber(input)
			if (input.type === null) {
				reportError(input, "Input is required to have a type: " + input.name)
			} else {
				// NOTE: Slightly obfuscate input name to help prevent accidental use.
				pr(removeCodeDelimiter(input.type) + '* __' + input.name + ';');
				pr('bool* __' + input.name + '_is_present;');
			}
		}
		// Next handle outputs.
		for(output: component.componentBody.outputs) {
			prSourceLineNumber(output)
			if (output.type === null) {
				reportError(output, "Output is required to have a type: " + output.name)
			} else {
				// NOTE: Slightly obfuscate output name to help prevent accidental use.
				pr(removeCodeDelimiter(output.type) + ' __' + output.name + ';');
				pr('bool __' + output.name + '_is_present;');
			}
		}
		
		unindent()
		pr("} " + argType + ";")
		
		// Generate reactions
		// For this second pass, restart the reaction count where the first pass started.
		reactionCount = savedReactionCount;
		generateReactions(component)	
		pr("// =============== END reactor class " + component.componentBody.name)
	}

	/** Generate reaction functions definition for a reactor or a composite.
	 *  These functions have a single argument that is a void* pointing to
	 *  a struct that contains parameters, inputs (triggering or not),
	 *  actions (triggering or produced), and references to outputs.
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
			var StringBuilder reactionInitialization = new StringBuilder()
			var structType = properties.targetProperties.get("structType")
			pr(reactionInitialization, structType
					+ "* this = (" + structType + "*)instance_args;")
			
			// Next, add the triggers.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					val input = getInput(component, trigger)
					if (input !== null) {
						pr(reactionInitialization, removeCodeDelimiter(input.type)
							+ ' ' + input.name + ' = *(this->__' + input.name + ');'
						);
						pr(reactionInitialization, removeCodeDelimiter(input.type)
							+ ' ' + input.name + '_is_present = *(this->__' + input.name + '_is_present);'
						);
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
					// FIXME: This should point to the output from which the input comes.
					pr(reactionInitialization, input.type + ' ' + input.name + ' = this->__' + input.name + ';');
				}
			}
			// Define argument for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					val input = getInput(component, get)
					pr(reactionInitialization, removeCodeDelimiter(input.type)
						+ ' ' + input.name + ' = *(this->__' + input.name + ');'
					);
					pr(reactionInitialization, removeCodeDelimiter(input.type)
						+ ' ' + input.name + '_is_present = *(this->__' + input.name + '_is_present);'
					);
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
						if (out.type === null) {
							reportError(out, "Output is required to have a type: " + output)
						}
						// NOTE: Slightly obfuscated the field name in the struct
						// to prevent the user from bypassing the declared outputs and
						// writing directly to the output using this->outputName.
						pr(reactionInitialization, removeCodeDelimiter(out.type)
							+ '* ' + output + ' = &(this->__' + output + ');'
						)
						pr(reactionInitialization, 'bool* ' + output 
							+ '_is_present = &(this->__' + output + '_is_present);'
						)
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

		val triggerTable = new StringBuffer()
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
								
				// FIXME: 0, 0 are index and position. Index comes from topological sort.
				pr(result, "reaction_t " + reactionInstanceName 
					+ " = {&" + functionName 
					+ ", &" + nameOfThisStruct
					+ ", 0, 0};"
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
			if (timing !== null) {
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
			if (triggerTable.length != 0) {
				triggerTable.append(', ')
			}
			triggerTable.append("&")
			triggerTable.append(triggerStructName)
			count++
			triggerCount++
		}
		// No longer used:
		// pr(result, '// --- Trigger table for instance '+ instance.name)
		// pr(result, 'trigger_t* trigger_table' + instanceCount + '[' + count + '] = {' + triggerTable + '};')
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
		pr('// --- "this" struct for instance ' + instance.name)
		var properties = componentToProperties.get(component)
		var nameOfThisStruct = "__this_" + instanceCount + "_" + instance.name
		var structType = properties.targetProperties.get("structType")
		pr(structType + " " + nameOfThisStruct + ";")
		
		// Generate code to initialize the instance struct in the
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
		// This also creates a local variable for each parameter.
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
			var type = state.type
			if (type === null) {
				// Types are optional in Lingua Franca, but required here.
				reportError(state, "No type given for state " + state.name)
			}
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
				
		// Next, initialize the struct with actions.
		for(action: component.componentBody.actions) {
			var triggerStruct = triggerNameToTriggerStruct.get(action.name)
			if (triggerNameToTriggerStruct === null) {
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

	/** Given a string of form either "xx" or "xx.yy", return either
	 *  "'xx'" or "xx, 'yy'".
	 */
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
	
	////////////////////////////////////////////
	//// Utility functions for generating code.
	
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
