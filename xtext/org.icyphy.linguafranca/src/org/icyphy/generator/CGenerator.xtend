/*
 * Generator for C target.
 */
package org.icyphy.generator

import java.util.HashMap
import java.util.Hashtable
import java.util.StringJoiner
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Composite
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
	
	// Map from action name to index of the trigger in the trigger table.
	var actionToTriggerTableIndex = new HashMap<String,Integer>()
	
	// Map from Instance to the name of the "this" struct for that instance.
	var instanceToNameOfThisStruct = new HashMap<Instance,String>()
			
	// Place to collect code to initialize the trigger table for all reactors.
	var initializeTriggerTable = new StringBuilder()
	
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
		
		// Generate function to initialize the trigger table for all reactors.
		pr('void __initialize_trigger_table() {\n')
		indent()
		pr(initializeTriggerTable)
		unindent()
		pr('}\n')
		
		// Generate function to start timers for all reactors.
		pr("void __start_timers() {")
		indent()
		pr(startTimers)
		unindent()
		pr("}")
		
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
		
		actionToTriggerTableIndex.clear()
		
		pr("// =============== START reactor class" + component.componentBody.name)
				
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
		properties.structType = argType
			
		// Construct the typedef for the "this" struct.
		pr("typedef struct {")
		indent()
		// Start with parameters.
		for(parameter: getParameters(component)) {
			prSourceLineNumber(parameter)
			if (parameter.type === null) {
				reportError(parameter, "Parameter must have a type.")
			} else {
				var type = removeCodeDelimiter(parameter.type)
				if (parameter.type.equals('time')) {
					type = 'interval_t'
				}
				pr(type + ' ' + parameter.name + ';');
			}
		}
		unindent()
		pr("} " + argType + ";")
		
		if (component instanceof Composite) {
			// Generated instances
			for (instance: component.instances) {
				// FIXME: instantiate (recursively) only from the component Main.
				instantiate(instance, importTable)
			}
			// Handle connections
			for (connection: component.connections) {
				// FIXME
			}
		}

		// Generate reactions
		// For this second pass, restart the reaction count where the first pass started.
		reactionCount = savedReactionCount;
		generateReactions(component)	
		pr("// =============== END reactor class " + component.componentBody.name)
	}
				
	def generateParameter(Param param) {
		var options = new StringJoiner(", ", "{", "}")
		var foundOptions = false
		if (param.type !== null) {
			options.add('''"type": "«removeCodeDelimiter(param.type)»"''')
			foundOptions = true
		}
		if (param.value !== null) {
			options.add('''"value": «removeCodeDelimiter(param.value)»''')
			foundOptions = true
		}
		// FIXME
		// pr('''this.parameter("«param.name»"«IF foundOptions», «options»«ENDIF»);''')
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
			val argType = functionName + "_args_t"
			
			properties.reactionToFunctionName.put(reaction, functionName)
			
			// Construct the typedef for the argument struct.
			// At the same time, construct the reactionInitialization code to go into
			// the body of the function before the verbatim code.
			var StringBuilder reactionInitialization = new StringBuilder()
			pr(reactionInitialization, componentToProperties.get(component).structType
					+ "* this = (" + componentToProperties.get(component).structType + "*)instance_args;")
			pr(reactionInitialization, argType + "* __cast_args = (" + argType + "*)reaction_args;")
			
			pr("typedef struct {")
			indent()
			// Next, add the triggers.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					val input = getInput(component, trigger)
					if (input !== null) {
						pr(input.type + " " + input.name + ";")
						pr(reactionInitialization, input.type + ' ' + input.name + ' = __cast_args->' + input.name + ';');
					}
					val action = getAction(component, trigger)
					if (action !== null) {
						// FIXME: Actions may have payloads.
						pr("trigger_t* " + action.name + ";")
						pr(reactionInitialization, "trigger_t* " + action.name + ' = __cast_args->' + action.name + ';');
					}
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare an argument for every input.
				for (input: component.componentBody.inputs) {
					pr(input.type + " " + input.name + ";")
					pr(reactionInitialization, input.type + ' ' + input.name + ' = __cast_args->' + input.name + ';');
				}
			}
			// Define argument for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					val input = getInput(component, get)
					pr(input.type + " " + input.name + ";")
					pr(reactionInitialization, input.type + ' ' + input.name + ' = __cast_args->' + input.name + ';');
				}
			}
			// Define variables for each declared output or action.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					val action = getAction(component, output)
					if (action !== null) {
						// It is an action, not an output.
						// FIXME: Actions may have payloads.
						pr("trigger_t* " + action.name + ";")
						pr(reactionInitialization, "trigger_t* " + action.name + ' = __cast_args->' + action.name + ';');
					} else {
						// FIXME: Handle output.
					}
				}
			}
			unindent()
			pr('} ' + argType + ';')	
			pr('void ' + functionName + '(void* instance_args, void* reaction_args) {')
			indent()
			pr(reactionInitialization)
			// Code verbatim from 'reaction'
			prSourceLineNumber(reaction)
			pr(removeCodeDelimiter(reaction.code))
			unindent()
			pr("}")
		}
	}
	
	/** Generate the trigger table for a reactor instance.
	 *  A trigger table is an array of trigger_t objects, one for
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
	 */
	def generateTriggerTable(Instance instance) {
		var component = getComponent(instance.reactorClass)
		var properties = componentToProperties.get(component)
		if (component === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return
		}
		var triggerToReactions = getTriggerToReactions(component)
		if (triggerToReactions === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return
		}
		val result = new StringBuilder()

		val triggerTable = new StringBuffer()
		var count = 0
		for (triggerName: triggerToReactions.keySet) {
			val numberOfReactionsTriggered = triggerToReactions.get(triggerName).length
			var names = new StringBuffer();
			for (reaction : triggerToReactions.get(triggerName)) {
				var functionName = properties.reactionToFunctionName.get(reaction)
				// Generate a reaction_t object for an instance of a reaction.
				val reactionInstanceName = "__reaction" + reactionInstanceCount++
				
				// FIXME: 0, 0 are index and position. Index comes from topological sort.
				pr(result, "reaction_t " + reactionInstanceName + " = {NULL, NULL, NULL, 0, 0};")
				pr(initializeTriggerTable, reactionInstanceName + ".function = &" + functionName + ";")
				pr(initializeTriggerTable, reactionInstanceName + ".this = &" + instanceToNameOfThisStruct.get(instance) + ";")
				
				// Position is a label to be written by the priority queue as a side effect of inserting.
				if (names.length != 0) {
					names.append(", ")
				}
				names.append('&' + reactionInstanceName)
			}
			var triggerStructName = triggerName + triggerCount
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
				actionToTriggerTableIndex.put(triggerName, count)
			}
			unindent(result)
			pr(result,'};')
			// Assignment of the offset and period have to occur after creating
			// the struct because the value assigned may not be a compile-time constant.
			if (timing !== null) {
				pr(initializeTriggerTable, triggerStructName + '.offset = ' + timeMacro(timing.offset) + ';')
				pr(initializeTriggerTable, triggerStructName + '.period = ' + timeMacro(timing.period) + ';')
				
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
		pr(result, 'trigger_t* trigger_table' + instanceCount + '[' + count + '] = {' + triggerTable + '};')
		// This goes directly out to the generated code.
		pr(result.toString())
	}
		
	/** Instantiate a reactor.
	 *  @param component The reactor or composite that this is instantiating.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def instantiate(Instance instance, Hashtable<String,String> importTable) {
		var className = importTable.get(instance.reactorClass);
		if (className === null) {
			className = instance.reactorClass
		}
		pr('// *** Instance ' + instance.name + ' of class ' + className)
		var component = getComponent(instance.reactorClass)
				
		// Generate the instance struct containing parameters and state variables.
		// (the "this" struct).
		var properties = componentToProperties.get(component)
		var nameOfThisStruct = "__this_" + instanceCount + "_" + instance.name
		instanceToNameOfThisStruct.put(instance, nameOfThisStruct)
		pr(properties.structType + " " + nameOfThisStruct + ";")
		
		// Generate code to initialize the instance struct in the
		// __initialize_trigger_table function.
		// FIXME: Parameter of the enclosing Component need to be in scope!!  Generate code here for that.
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
			pr(initializeTriggerTable, nameOfThisStruct + "." + parameter.name + " = " + value + ";")
		}
		
		// Generate trigger table for the instance.
		generateTriggerTable(instance)
		instanceCount++
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
	
	// Append to a string with preceding comma if needed.
	private def argListAppend(StringBuffer buffer, String item) {
		if (buffer.length > 0) {
			buffer.append(', ')
		}
		buffer.append(item)
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
