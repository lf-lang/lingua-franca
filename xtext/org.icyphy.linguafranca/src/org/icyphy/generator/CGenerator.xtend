/*
 * Generator for C target.
 */
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import java.util.Hashtable
import java.util.LinkedHashMap
import java.util.LinkedList
import java.util.StringJoiner
import org.eclipse.emf.common.util.EList
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Composite
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Time

/**
 * Generator for C target.
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill, Mehrdad Niknami
 */
class CGenerator extends GeneratorBase {
		
	// For each reactor, we collect a set of input and parameter names.
	var inputs = new HashSet<String>()
	var reactionCount = 0
	var reactionInstanceCount = 0
	var triggerCount = 0
	var reactorCount = 0
	var instanceCount = 0
		
	// Map from reactor or composite class name to the
	// map from timer or action name to reaction function name(s) triggered by it.
	var classToTriggerToReactions
			= new LinkedHashMap<String,LinkedHashMap<String,LinkedList<String>>>()
	
	// Map from action name to index of the trigger in the trigger table.
	var actionToTriggerTableIndex = new HashMap<String,Integer>()
	
	// Place to collect code to initialize the trigger table for all reactors.
	var initializeTriggerTable = new StringBuffer()
	
	// Place to collect code to initialize timers for all reactors.
	var startTimers = new StringBuffer()
			
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
	 *  This 
	 *  @param component The parsed component data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	override generateComponent(Component component, Hashtable<String,String> importTable) {
		super.generateComponent(component, importTable)
		
		inputs.clear()      // Reset set of inputs.
		
		var triggerToReactions = new LinkedHashMap<String,LinkedList<String>>()
		classToTriggerToReactions.put(component.componentBody.name, triggerToReactions)
		
		actionToTriggerTableIndex.clear()
		
		pr("// =============== START reactor class" + component.componentBody.name)
				
		// Scan reactions
		var savedReactionCount = reactionCount;
		scanReactions(component, triggerToReactions)
		
		// Define variables for each parameter.
		for(parameter: parameters) {
			prSourceLineNumber(parameter)
			if (parameter.type === null) {
				reportError(parameter, "Parameter must have a type.")
			} else {
				var type = removeCodeDelimiter(parameter.type)
				var value = removeCodeDelimiter(parameter.value)
				if (parameter.type.equals('time')) {
					type = 'interval_t'
					value = timeMacro(parameter.time)
				}
				pr(type + ' ' + parameter.name + ' = ' + value + ';');
			}
		}
		
		// Preamble code contains state declarations with static initializers.
		if (component.componentBody.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(component.componentBody.preamble.code))
			pr("\n// *********** End of preamble.")
		}
		
		// Handle Inputs, if any.
		for (input: component.componentBody.inputs) {
			generateInput(input)
		}
		// Handle outputs, if any
		for (output: component.componentBody.outputs) {
			generateOutput(output)
		}
		// Handle parameters, if any (FIXME: reconcile with above)
		if (component.componentBody.parameters !== null) {
			for (param : component.componentBody.parameters.params) {
				generateParameter(param)
			}
		}
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
		reactorCount++
		pr("// =============== END reactor class " + component.componentBody.name)
	}
			
	def generateInput(Input input) {
		inputs.add(input.name);
		pr('''this.input("«input.name»"«IF input.type !== null», { 'type': '«removeCodeDelimiter(input.type)»'}«ENDIF»);''')
	}
		
	def generateOutput(Output output) {
		pr('''this.output("«output.name»"«IF output.type !== null», { 'type': '«removeCodeDelimiter(output.type)»'}«ENDIF»);''')
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
	 */
	def generateReactions(Component component) {
		var reactions = component.componentBody.reactions
		for (reaction: reactions) {
			var argumentList = new StringBuffer()
			// Add arguments for inputs.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
						val input = getInput(trigger)
						argListAppend(argumentList, input.type + ' ' + input.name)
					}
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare an argument for every input.
				for (inputName: inputs) {
					val input = getInput(inputName)
					argListAppend(argumentList, input.type + ' ' + input.name)
				}
			}
			// Define argument for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					val input = getInput(get)
					argListAppend(argumentList, input.type + ' ' + input.name)
				}
			}
			// Define variables for each declared output or action.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					val action = getAction(component, output)
					if (action !== null) {
						// It is an action, not an output.
						// FIXME: Action should be able to carry an argument.
						argListAppend(argumentList, 'trigger_t* ' + action.name)
					} else {
						// FIXME: Handle output.
					}
				}
			}			
			val functionName = "reaction_function" + reactionCount++
			pr('void ' + functionName + '(' + argumentList + ') {')
			indent()
			// Code verbatim from 'reaction'
			prSourceLineNumber(reaction)
			pr(removeCodeDelimiter(reaction.code))
			unindent()
			pr("}")
		}
	}

	/** Scan reaction declarations and print the reaction function
	 *  definitions to the generated code. As a side effect, populate
	 *  the specified triggerToReactions map that maps trigger names
	 *  that trigger each reaction to the reactions triggered.
	 *  @param component The component whose reactions are being defined.
	 *  @param triggerToReactions The map from trigger names to reactions triggered
	 *   for this component.
	 */
	def scanReactions(Component component, LinkedHashMap<String,LinkedList<String>> triggerToReactions) {
		val reactions = component.componentBody.reactions		
		for (reaction: reactions) {
			val reactionFunctionName = "reaction_function" + reactionCount++;
			// Iterate over the reaction's triggers
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
                        // FIXME: handle inputs.
                    } else if (getTiming(component, trigger) !== null) {
                        // The trigger is a timer.
                        // Record this so we can schedule this reaction in initialize
                        // and initialize the trigger table.
                        var list = triggerToReactions.get(trigger)
                        if (list === null) {
                            list = new LinkedList<String>()
                            triggerToReactions.put(trigger, list)
                        }
                        list.add(reactionFunctionName)
                    } else if (getAction(component, trigger) !== null) {
                        // The trigger is an action.
                        // Record this so we can initialize the trigger table.
                        var list = triggerToReactions.get(trigger)
                        if (list === null) {
                            list = new LinkedList<String>()
                            triggerToReactions.put(trigger, list)
                        }
                        list.add(reactionFunctionName)
                    } else {
                        reportError(reaction,
                        		"Trigger '" + trigger + "' is neither an input, a timer, nor an action.")
                    }		
				}	
			}
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
	 *  reaction, which is a wrapper function that is also generated
	 *  here. The wrapper function constructs the appropriate argument
	 *  list for the reaction function.
	 */
	def generateTriggerTable(Instance instance) {
		var component = getComponent(instance.reactorClass)
		if (component === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return
		}
		var triggerToReactions = classToTriggerToReactions.get(instance.reactorClass)
		if (triggerToReactions === null) {
			reportError(instance, "Undefined reactor class: " + instance.reactorClass)
			return
		}
		val result = new StringBuffer()
		val wrappers = new StringBuffer()
		// For each trigger, create a reaction_t struct for each reaction it
		// triggers.
		for (String triggerName: triggerToReactions.keySet()) {
			val reactions = triggerToReactions.get(triggerName)
			for (reaction: reactions) {
			}
		}
		// timers // map from timer name to timer properties.
		// triggerReactions // map from timer name to a list of function names
		val triggerTable = new StringBuffer()
		var count = 0
		for (triggerName: triggerToReactions.keySet) {
			val numberOfReactionsTriggered = triggerToReactions.get(triggerName).length
			var names = new StringBuffer();
			for (functionName : triggerToReactions.get(triggerName)) {
				// Generate a reaction_t object for an instance of a reaction.
				val reactionInstanceName = "__reaction" + reactionInstanceCount++
				// FIXME: 0, 0 are index and position. Index comes from topological sort.
				result.append("reaction_t " + reactionInstanceName + " = {NULL, 0, 0};\n")
				
				wrappers.append("void " + functionName + "_wrapper() {\n")
				// FIXME: Create parameters and parameter list.
				wrappers.append("    " + functionName + "(FIXME);\n")
				wrappers.append("}\n")
				
				initializeTriggerTable.append(reactionInstanceName + ".function = " + functionName + "_wrapper;\n")
				
				// Position is a label to be written by the priority queue as a side effect of inserting.
				if (names.length != 0) {
					names.append(", ")
				}
				names.append('&' + reactionInstanceName)
			}
			result.append('reaction_t* ' + triggerName + triggerCount 
					+ '_reactions[' + numberOfReactionsTriggered + '] = {' + names + '};')
			result.append('\n')
			// Declare a variable with the name of the trigger whose
			// value is a struct.
			result.append('trigger_t ' + triggerName + triggerCount + ' = {\n')
			var timing = getTiming(component, triggerName)
			if (timing !== null) {
				result.append(triggerName + triggerCount + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ '0LL, 0LL'
				)
			} else if (getAction(component, triggerName) !== null) {
				result.append(triggerName + triggerCount + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ getAction(component, triggerName).getDelay()
					+ ', 0' // 0 is ignored since actions don't have a period.
				)
				actionToTriggerTableIndex.put(triggerName, count)
			}
			result.append('\n};\n')
			// Assignment of the offset and period have to occur after creating
			// the struct because the value assigned may not be a compile-time constant.
			if (timing !== null) {
				initializeTriggerTable.append(triggerName + triggerCount + '.offset = ' + timeMacro(timing.offset) + ';\n')
				initializeTriggerTable.append(triggerName + triggerCount + '.period = ' + timeMacro(timing.period) + ';\n')
				
				// Generate a line to go into the __start_timers() function.
				startTimers.append("__schedule(&" + triggerName + triggerCount + ", "
		 			+ timeMacro(timing.offset) + ");\n")
			}
			if (triggerTable.length != 0) {
				triggerTable.append(', ')
			}
			triggerTable.append("&")
			triggerTable.append(triggerName + triggerCount)
			count++
			triggerCount++
		}
		result.append('trigger_t* trigger_table' + instanceCount + '[' + count + '] = {' + triggerTable + '};\n')
		result.append(wrappers)
		// This goes directly out to the generated code.
		pr(result.toString())
	}
		
	/** Instantiate a reactor.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def instantiate(Instance instance, Hashtable<String,String> importTable) {
		var className = importTable.get(instance.reactorClass);
		if (className === null) {
			className = instance.reactorClass
		}
		pr('// *** Instance ' + instance.name + ' of class ' + className)
		// Generate trigger table for the instance.
		generateTriggerTable(instance)
		
		// FIXME: Handle parameters.
		if (instance.parameters !== null) {
			for (param: instance.parameters.assignments) {
				// pr('''«instance.name».setParameter('«param.name»', «removeCodeDelimiter(param.value)»);''')
			}
		}
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
