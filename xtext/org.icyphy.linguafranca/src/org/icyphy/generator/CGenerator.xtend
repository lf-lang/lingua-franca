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
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Composite
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Timing

/**
 * Generator for C target.
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill
 */
class CGenerator {
	// For each accessor, we collect a set of input and parameter names.
	var inputs = newHashSet()
	var parameters = newLinkedList()
	var reactionCount = 0
	
	// Map from timer name to Timing object.
	var timers = new HashMap<String,Timing>()
	var timerIDs = new HashMap<String,Integer>()
	// Map from timer name to reaction name(s) triggered by the timer.
	var timerReactions = new LinkedHashMap<String,LinkedList<String>>()
	
	// Text of generated code to add input handlers.
	var triggerTable = new StringBuffer()
	
	// All code goes into this string buffer.
	var code = new StringBuilder
	
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
		
		// Handle reactors and composites.
		for (component : resource.allContents.toIterable.filter(Component)) {
			code = new StringBuilder
			generateComponent(component, importTable, false)
			val componentBody = component.componentBody
			fsa.generateFile(componentBody.name + ".c", code)		
		}
	}
	
	////////////////////////////////////////////
	//// Code generators.
	
	/** Generate a reactor or composite definition.
	 *  @param component The parsed component data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 *  @param isComposite True if this is a composite reactor.
	 */	
	def generateComponent(Component component, Hashtable<String,String> importTable, boolean isComposite) {
		inputs.clear()      // Reset set of inputs.
		parameters.clear()  // Reset set of parameters.
		timers.clear()      // Reset map of timer names to timer properties.
		timerIDs.clear()	// Reset map of timer names to timer IDs.
		timerReactions.clear()
		triggerTable = new StringBuffer()
		
		reactionCount = 1   // Start reaction count at 1.
		
		pr(typedefs)

		// Record timers.
		var count = 0;
		for (timer: component.componentBody.timers) {
			timerIDs.put(timer.name, count)
			count++
			var timing = timer.timing
			if (timing === null) {
				timing = LinguaFrancaFactory.eINSTANCE.createTiming()
				timing.setOffset("0") // Same as NOW.
				timing.setPeriod("0") // Same as ONCE.
			} else {
				if (timing.getOffset.equals("NOW")) {
					timing.setOffset("0")
				}
				if (timing.getPeriod.equals("ONCE")) {
					timing.setPeriod("0")
				} else if (timing.getPeriod.equals("STOP")) {
					timing.setPeriod("-1")
				}
			}
			timers.put(timer.name, timing)
		}
		
		/* FIXME
		if (component.componentBody.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(component.componentBody.preamble.code))
			pr("\n// *********** End of preamble.")
		}
		// Reactor setup (inputs, outputs, parameters)
		componentSetup(component, importTable)
		*/
		// Generate reactions
		generateReactions(component.componentBody.reactions)
		// Generate trigger table
		generateTriggerTable()
		pr(initialize)
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
		parameters.add(param.name)
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
	
	/** Generate the initialize function definition for an accessor.
	 *  This adds input handlers and timer reactions.
	 */
	def generateInitialize(
		EList<Timer> triggers, 
		EList<Reaction> reactions
	) {
		pr("exports.initialize = function () {\n")
		indent()
		// Define variables for each parameter.
		for(parameter: parameters) {
			pr('''var «parameter» = this.getParameter("«parameter»");''');
		}
		
		// Add the input handlers.
		pr(triggerTable)

		// Add the timer reactions.
		for (timer: timerReactions.keySet) {
			val timerParams = timers.get(timer)
			for (handler: timerReactions.get(timer)) {
				pr('''schedule("«timer»", «handler».bind(this), «timerParams.offset», «timerParams.period»);''')
			}
		}
		unindent()
		pr("};")
	}			

	/** Generate reaction functions definition for a reactor or a composite.
	 */
	def generateReactions(EList<Reaction> reactions) {
		val functionName = "reaction" + reactionCount++
		for (reaction: reactions) {
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

						// Generate code for the initialize() function here so that input handlers are
						// added in the same order that they are declared.
				   		triggerTable.append('''this.addInputHandler("«trigger»", «functionName».bind(this));''')
					} else if (timers.get(trigger) !== null) {
						// The trigger is a timer.
						// Record this so we can schedule this reaction in initialize
						// and initialize the trigger table.
						var list = timerReactions.get(trigger)
						if (list === null) {
							list = new LinkedList<String>()
							timerReactions.put(trigger, list)
						}
						list.add(functionName)
					} else {
						// This is checked by the validator (See LinguaFrancaValidator.xtend).
						// Nevertheless, in case we are using a command-line tool, we report the line number.
						// Just report the exception. Do not throw an exception so compilation can continue.
						var node = NodeModelUtils.getNode(reaction)
						System.err.println("Line "
							+ node.getStartLine()
							+ ": Trigger '" + trigger + "' is neither an input, a timer, nor an action.")
					}
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare a variable for every input.
				// NOTE: Here we are not using get() because null works in JavaScript.
				for (input: inputs) {
					pr('''var «input» = get("«input»");''')
				}
				triggerTable.append('''this.addInputHandler(null, «functionName».bind(this));''')
			}
			// Define variables for non-triggering inputs.
			if (reaction.gets !== null && reaction.gets.gets !== null) {
				for(get: reaction.gets.gets) {
					pr('''var «get» = get("«get»");''')
				}
			}
			// Define variables for each declared output.
			if (reaction.sets !== null && reaction.sets.sets !== null) {
				for(set: reaction.sets.sets) {
					// Set the output name variable equal to a string.
					// FIXME: String name is too easy to cheat!
					// LF coder could write set('foo', value) to write to
					// output foo without having declared the write.
					pr('''var «set» = "«set»";''');
				}
			}			
			// Define variables for each parameter.
			for(parameter: parameters) {
				pr('''var «parameter» = this.getParameter("«parameter»");''');
			}

			// Code verbatim from 'reaction'
			pr(removeCodeDelimiter(reaction.code))
			unindent()
			pr("}")
		}
	}
	
	/** Generate the trigger table.
	 */
	def generateTriggerTable() {
		// timers // map from timer name to timer properties.
		// timerIDs // map from timer name to timer ID
		// timerReactions // map from timer name to a list of function names
		val triggerTable = new StringBuffer()
		var count = 0
		for (timerName: timerReactions.keySet) {
			val numberOfReactionsTriggered = timerReactions.get(timerName).length
			var names = new StringBuffer();
			for (functionName: timerReactions.get(timerName)) {
				if (names.length != 0) {
					names.append(", ")
				}
				names.append(functionName)
			}
			pr('void* reactionFunctions[' + numberOfReactionsTriggered + '] = {' + names + '};')
			pr('trigger_t ' + timerName + ' = {')
			indent()
			pr('reactionFunctions, '
				+ timers.get(timerName).offset
				+ ', '
				+ timers.get(timerName).period
			)
			unindent()
			pr("};")
			if (triggerTable.length != 0) {
				triggerTable.append(', ')
			}
			triggerTable.append("&")
			triggerTable.append(timerName)
			count++
		}
		pr('trigger_t* triggerTable[] = {' + triggerTable + '};')
		pr('int triggerTableSize = ' + count + ';')
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
	
	var indentation = ""
	
	/** Append the specified text plus a final newline to the current
	 *  code buffer.
	 *  @param text The text to append.
	 */
	private def pr(Object text) {
		// Handle multi-line text.
		var string = text.toString
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
				code.append(indentation)
				// Do not trim the first line
				if (firstLine) {
					code.append(line)
					firstLine = false
				} else {
					code.append(line.substring(offset))
				}
				code.append("\n")
			}
		} else {
			code.append(indentation)
			code.append(text)
			code.append("\n")
		}
	}
	
	private def indent() {
		val buffer = new StringBuffer(indentation)
		for (var i = 0; i < 4; i++) {
			buffer.append(' ');
		}
		indentation = buffer.toString
	}
	
	private def unindent() {
		val end = indentation.length - 4;
		if (end < 0) {
			indentation = ""
		} else {
			indentation = indentation.substring(0, end)
		}
	}

	/** If the argument starts with '{=', then remove it and the last two characters.
	 *  @return The body without the code delimiter or the unmodified argument if it
	 *   is not delimited.
	 */
	private def String removeCodeDelimiter(String code) {
		if (code.startsWith("{=")) {
            code.substring(2, code.length - 2).trim();
        } else {
        	code
        }
	}
	
	val static typedefs = '''
		// ********* Type definitions included for all actors.
		#include <stdio.h>
		// NOTE: Units for time are dealt with at compile time.
		typedef struct {
		  int time;         // a point in time
		  int microstep;    // superdense time index
		} time_t;
		
		// Intervals of time do not involve the microstep.
		typedef int interval_t;
				
		typedef struct {
			void** reactions; // FIXME: more specific type to include argument types.
			interval_t minOffset;
			interval_t minPeriod;
		} trigger_t;
		
		// Event to put in the event queue.
		struct {
		  time_t time;       // time of the event
		  int trigger_id;    // payload is a trigger ID
		} event_t;
		
		// Handles for scheduled triggers.
		typedef int handle_t;
	'''
		
	val static initialize = '''
		time_t currentTime = {0, 0}; // FIXME: This should not be modifiable by reactors.
		void initialize() {
			currentTime.time = 0; // FIXME: Obtain system time.
		}
		handle_t schedule(trigger_t* trigger, interval_t offset, interval_t period) {
			printf("Scheduling %d, %d\n", trigger->minOffset, trigger->minPeriod);
			return 0;
		}
		void startTimers() {
		    for (int i=0; i < triggerTableSize; i++) {
		        schedule(triggerTable[i], 0, 0); 
		    }
		}
		int main(int argc, char* argv[]) {
			initialize();
			printf("Hello World at time %d\n", currentTime.time);
			startTimers();
			return 0;
		}
	'''
}
