/*
 * Generator for C target.
 */
package org.icyphy.generator

import java.util.HashMap
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
	var inputs = newHashSet()
	var reactionCount = 0
	var triggerCount = 0
	var reactorCount = 0
	
	// Map from timer or action name to reaction name(s) triggered by it.
	var triggerToReactions = new LinkedHashMap<String,LinkedList<String>>()
	
	// Map from action name to index of the trigger in the trigger table.
	var actionToTriggerTableIndex = new HashMap<String,Integer>()
	
	// Place to collect code to initialize the trigger table for all reactors.
	var intializeTriggerTable = new StringBuffer()
	
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
		pr(intializeTriggerTable)
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
	
	/** Generate a reactor or composite definition.
	 *  @param component The parsed component data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	override generateComponent(Component component, Hashtable<String,String> importTable) {
		super.generateComponent(component, importTable)
		
		inputs.clear()      // Reset set of inputs.
		triggerToReactions.clear()
		actionToTriggerTableIndex.clear()
		
		pr("// =============== START " + component.componentBody.name)
				
		// Scan reactions
		var savedReactionCount = reactionCount;
		scanReactions(component.componentBody.reactions)
		
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
		
		// Generate trigger table
		generateTriggerTable()

		// Preamble code contains state declarations with static initializers.
		if (component.componentBody.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(component.componentBody.preamble.code))
			pr("\n// *********** End of preamble.")
		}

		// Generate reactions
		// For this second pass, restart the reaction count where the first pass started.
		reactionCount = savedReactionCount;
		generateReactions(component.componentBody.reactions)	
		reactorCount++
		pr("// =============== END " + component.componentBody.name)
	}
	
	/** Generate the setup function definition for a reactor or composite.
	 */
	def componentSetup(Component component, Hashtable<String,String> importTable) {
		pr("exports.setup = function () {")
		indent()
		// Generate Inputs, if any.
		for (input: component.componentBody.inputs) {
			generateInput(input)
		}
		// Generate outputs, if any
		for (output: component.componentBody.outputs) {
			generateOutput(output)
		}
		// Generate parameters, if any
		if (component.componentBody.parameters !== null) {
			for (param : component.componentBody.parameters.params) {
				generateParameter(param)
			}
		}
		if (component instanceof Composite) {
			// Generated instances
			for (instance: component.instances) {
				instantiate(instance, importTable)
			}
			// Generated connections
			for (connection: component.connections) {
				pr('''this.connect(«portSpec(connection.leftPort)», «portSpec(connection.rightPort)»);''')
			}
		}
		unindent()
		pr("}")
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
		pr('''this.parameter("«param.name»"«IF foundOptions», «options»«ENDIF»);''')
	}

	/** Generate reaction functions definition for a reactor or a composite.
	 */
	def generateReactions(EList<Reaction> reactions) {
		var triggerTable = new StringBuffer()
		for (reaction: reactions) {
			val functionName = "reaction_function" + reactionCount++
			pr('''void «functionName»() {''')
			indent()
			// Add variable declarations for inputs.
			// Meanwhile, record the mapping from triggers to handlers.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
						// The trigger is an input.
						// Declare a variable in the generated code.
						// NOTE: Here we are not using get() because null works
						// in JavaScript.
						// FIXME: Convert to C.
						pr('''var «trigger» = get("«trigger»");''')
					}
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare a variable for every input.
				// NOTE: Here we are not using get() because null works in JavaScript.
				for (input: inputs) {
					pr('''var «input» = get("«input»");''')
				}
				// FIXME: Convert to C.
				triggerTable.append('''this.addInputHandler(null, «functionName».bind(this));''')
			}
			// Define variables for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					// FIXME: Convert to C.
					pr('''var «get» = get("«get»");''')
				}
			}
			// Define variables for each declared output or action.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					if (actions.get(output) !== null) {
						// An action is produced.
						pr('''trigger_t* «output» = trigger_table«reactorCount»[«actionToTriggerTableIndex.get(output)»];''')
					}
				}
			}			

			// Code verbatim from 'reaction'
			prSourceLineNumber(reaction)
			pr(removeCodeDelimiter(reaction.code))
			unindent()
			pr("}")
		}
	}

	/** Scan reaction declarations and print them in the generated code.
	 */
	def scanReactions(EList<Reaction> reactions) {
		val reactionDecls = new StringBuffer()
		
		for (reaction: reactions) {
			val reactionName = "reaction" + reactionCount;
		 	pr("void reaction_function" + reactionCount + "();")
			reactionDecls.append("reaction_t " + reactionName + " = {reaction_function" + reactionCount + ", 0, 0};\n");
			reactionCount++;
			// Iterate over the reaction's triggers
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
                        // FIXME: handle inputs.
                    } else if (getTiming(trigger) !== null) {
                        // The trigger is a timer.
                        // Record this so we can schedule this reaction in initialize
                        // and initialize the trigger table.
                        var list = triggerToReactions.get(trigger)
                        if (list === null) {
                            list = new LinkedList<String>()
                            triggerToReactions.put(trigger, list)
                        }
                        list.add(reactionName)
                    } else if (actions.get(trigger) !== null) {
                        // The trigger is an action.
                        // Record this so we can initialize the trigger table.
                        var list = triggerToReactions.get(trigger)
                        if (list === null) {
                            list = new LinkedList<String>()
                            triggerToReactions.put(trigger, list)
                        }
                        list.add(reactionName)
                    } else {
                        reportError(reaction,
                        		"Trigger '" + trigger + "' is neither an input, a timer, nor an action.")
                    }		
				}	
			}
		}
		pr("\n" + reactionDecls.toString())
	}
	
	/** Generate the trigger table for a reactor.
	 *  A trigger table is an array of trigger_t objects, one for
	 *  each input, clock, and action of the reactor.
	 *  Each trigger_t object is a struct that contains an
	 *  array of function pointers to reactions triggered by
	 *  this trigger, the length of the array, the offset,
	 *  and the period (the latter two are zero if it is not
	 *  a timer).
	 */
	def generateTriggerTable() {
		// timers // map from timer name to timer properties.
		// triggerReactions // map from timer name to a list of function names
		val triggerTable = new StringBuffer()
		val result = new StringBuffer()
		var count = 0
		for (triggerName: triggerToReactions.keySet) {
			val numberOfReactionsTriggered = triggerToReactions.get(triggerName).length
			var names = new StringBuffer();
			for (functionName : triggerToReactions.get(triggerName)) {
				// FIXME: 0, 0 are index and position. Index comes from topological sort.
				// Position is a label to be written by the priority queue as a side effect of inserting.
				var reactionName = 'reaction' + count
				result.append('\n')
				if (names.length != 0) {
					names.append(", ")
				}
				names.append('&' + reactionName)
			}
			result.append('reaction_t* ' + triggerName + triggerCount 
					+ '_reactions[' + numberOfReactionsTriggered + '] = {' + names + '};')
			result.append('\n')
			// Declare a variable with the name of the trigger whose
			// value is a struct.
			result.append('trigger_t ' + triggerName + triggerCount + ' = {')
			result.append('\n')
			var timing = getTiming(triggerName)
			if (timing !== null) {
				result.append(triggerName + triggerCount + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ '0LL, 0LL'
				)
			} else if (actions.get(triggerName) !== null) {
				result.append(triggerName + triggerCount + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ actions.get(triggerName).getDelay()
					+ ', 0' // 0 is ignored since actions don't have a period.
				)
				actionToTriggerTableIndex.put(triggerName, count)
			}
			result.append('\n};\n')
			// Assignment of the offset and period have to occur after creating
			// the struct because the value assigned may not be a compile-time constant.
			if (timing !== null) {
				intializeTriggerTable.append(triggerName + triggerCount + '.offset = ' + timeMacro(timing.offset) + ';\n')
				intializeTriggerTable.append(triggerName + triggerCount + '.period = ' + timeMacro(timing.period) + ';\n')
				
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
		result.append('trigger_t* trigger_table' + reactorCount + '[' + count + '] = {' + triggerTable + '};')
		result.append('\n')
		// This goes directly out to the generated code.
		pr(result.toString())
	}
		
	/** Generate an instantiate statement followed by any required parameter
	 *  assignments. This retrieves the fully-qualified class name from the imports
	 *  table if appropriate.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def instantiate(Instance instance, Hashtable<String,String> importTable) {
		var className = importTable.get(instance.actorClass);
		if (className === null) {
			className = instance.actorClass
		}
		pr('''var «instance.name» = this.instantiate('«instance.name»', '«className»');''')
		if (instance.parameters !== null) {
			for (param: instance.parameters.assignments) {
				pr('''«instance.name».setParameter('«param.name»', «removeCodeDelimiter(param.value)»);''')
			}
		}
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
