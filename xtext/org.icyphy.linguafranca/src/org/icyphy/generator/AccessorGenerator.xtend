/*
 * Generator for Accessors, JavaScript code runnable in Node, CapeCode, and Ptolemy II.
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
import org.icyphy.linguaFranca.Clock
import org.icyphy.linguaFranca.Composite
import org.icyphy.linguaFranca.Constructor
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor

/**
 * Generator for Accessors.
 * @author Edward A. Lee, Chadlia Jerad
 */
class AccessorGenerator {
	// For each accessor, we collect a set of input and parameter names.
	var inputs = newHashSet()
	var parameters = newLinkedList()
	var reactionCount = 0
	
	// Map from clock name to Clock object.
	var clocks = new HashMap<String,Clock>()
	// Map from clock name to reaction name(s) triggered by the clock.
	var clockReactions = new LinkedHashMap<String,LinkedList<String>>()
	
	// Map from input name to list of input handlers.
	var handlers = new LinkedHashMap<String,LinkedList<String>>()
	
	// All code goes into this string buffer.
	var code = new StringBuilder
	
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
		
		// Handle actors.
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			code = new StringBuilder
			generateReactor(reactor, importTable, false);
			fsa.generateFile(reactor.name + ".js", code);		
		}
		
		// Handle composites.
		for (composite : resource.allContents.toIterable.filter(Composite)) {
			code = new StringBuilder
			generateComposite(composite, importTable)
			fsa.generateFile(composite.name + ".js", code);		
		}
	}
	
	////////////////////////////////////////////
	//// Code generators.
	
	/** Generate a reactor definition.
	 *  @param reactor The parsed reactor data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 *  @param isComposite True if this is a composite reactor.
	 */	
	def generateReactor(Reactor reactor, Hashtable<String,String> importTable, boolean isComposite) {
		inputs.clear()      // Reset set of inputs.
		parameters.clear()  // Reset set of parameters.
		clocks.clear()      // Reset map of clock names to clock properties.
		clockReactions.clear()
		handlers.clear()
		
		reactionCount = 1   // Start reaction count at 1.
		
		// Record clocks.
		for (clock: reactor.clocks) {
			clocks.put(clock.name, clock)
		}
		pr(boilerplate())
		if (reactor.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(reactor.preamble.code))
			pr("\n// *********** End of preamble.")
		}
		// Reactor setup (inputs, outputs, parameters)
		/* FIXME: Solve code duplication problem: Have to make composite a subtype of reactor in xtext file.
		if(isComposite) {
			result.append(compositeSetup(reactor, importTable))
		} else {
		*/
			reactorSetup(reactor, importTable)
		//}
		// Generate reactions
		generateReactions(reactor.reactions)
		// Reactor initialize (initialize + triggers scheduling + input handlers)
		generateInitialize(reactor.constructor, reactor.clocks, reactor.reactions)
	}
	
	/** Generate a composite definition.
	 *  @param composite The parsed composite data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def generateComposite(Composite composite, Hashtable<String,String> importTable) {
		// FIXME: Duplicated code with above.
		inputs.clear()      // Reset set of inputs.
		parameters.clear()  // Reset set of parameters.
		clocks.clear()      // Reset map of clock names to clock properties.
		clockReactions.clear()
		handlers.clear()
		
		reactionCount = 1   // Start reaction count at 1.
		
		// Record clocks.
		for (clock: composite.clocks) {
			clocks.put(clock.name, clock)
		}
		pr(boilerplate())
		if (composite.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(composite.preamble.code))
			pr("// *********** End of preamble.")
		}
		compositeSetup(composite, importTable)
		// Generate reactions
		generateReactions(composite.reactions)
		// Reactor initialize (initialize + triggers scheduling + input handlers)
		generateInitialize(composite.constructor, composite.clocks, composite.reactions)
	}
	
	def boilerplate() '''
		// ********* Boilerplate included for all actors.
		// Unbound version of set() function (will be bound below).
		function __setUnbound(port, value) {
		    if (!port) {
		        throw "Illegal reference to undeclared output.";
		    }
		    this.send(port, value);
		}
		// NOTE: bind() returns a new function.
		// It does not alter the original function.
		var set = __setUnbound.bind(this);
		
		// Unbound version of get() function (will be bound below).
		function __getUnbound(port, value) {
			if (!port) {
		    	throw "Illegal reference to undeclared input.";
		    }
			return this.get(port);
		}
		// NOTE: bind() returns a new function.
		// It does not alter the original function.
		var get = __getUnbound.bind(this);
		
		// Variables for schedule function.
		var TRIGGERED = -1;
		var ONCE = -1;
		// Map from schedule names to reaction functions.
		var __SCHEDULE_TABLE = {}
		
		// Function to schedule an action. The arguments are:
		// * action: The name of the action.
		// * offset: The time after the current time for the first action.
		// * period: The period at which to repeat the action, or 0 to not repeat it.
		// FIXME: Make all but the first argument optional.
		function schedule(action, offset, period) {
			if (offset > 0) {
				setTimeout(schedule, offset, action, 0, period);
			} else {
				if (period > 0) {
					// FIXME
				} else {
					// FIXME 
				}
			}
		}
		// Dispatch the specified action.
		function __dispatch(action) {
			var reactions = actionTable[action];
			if (!reactions) {
				throw("No actions named: " + action);
			}
			// for (var reaction in reactions) {
				
			// }
		}
		// A table of actions indexed by the action name.
		// Each property value is an object with function and arguments properties.
		// The arguments property is an array of arguments or an empty array to
		// apply no arguments.
		var actionTable = {};
		// ********** End boilerplate
	'''
	
	/** Generate the setup function definition for a reactor.
	 */
	def reactorSetup(Reactor reactor, Hashtable<String,String> importTable) {
		pr("exports.setup = function () {")
		indent()
		generateIO(reactor)
		unindent()
		pr("}")
	}
		
	/** Generate the setup function definition for a composite.
	 */
	def compositeSetup(Composite composite, Hashtable<String,String> importTable) {
		pr("exports.setup = function () {")
		indent()
		generateCompositeIO(composite)
		// Generated instances
		for (instance: composite.instances) {
			instantiate(instance, importTable)
		}
		// Generated connections
		for (connection: composite.connections) {
			pr('''this.connect(«portSpec(connection.leftPort)», «portSpec(connection.rightPort)»);''')
		}
		unindent()
		pr("}")
	}
		
	/** Generate the inputs, outputs, and parameters for a reactor.
	 */
	def generateIO(Reactor reactor) {
		// Generate Inputs, if any.
		for (input: reactor.inputs) {
			generateInput(input)
		}
		// Generate outputs, if any
		for (output: reactor.outputs) {
			generateOutput(output)
		}
		// Generate parameters, if any
		if (reactor.parameters !== null) {
			for (param : reactor.parameters.params) {
				generateParameter(param)
			}
		}
	}
	
	/** Generate the inputs, outputs, and parameters for a composite.
	 */
	def generateCompositeIO(Composite reactor) {
		// FIXME: Completely redundant with previous method.
		// But Composite and Reactor are completely different classes,
		// so I don't see how to merge these.
		
		// Generate Inputs, if any.
		for (input: reactor.inputs) {
			generateInput(input)
		}
		// Generate outputs, if any
		for (output: reactor.outputs) {
			generateOutput(output)
		}
		// Generate parameters, if any
		if (reactor.parameters !== null) {
			for (param : reactor.parameters.params) {
				generateParameter(param)
			}
		}
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
	 *  This adds input handlers and clock reactions.
	 */
	def generateInitialize(
		Constructor constructor, 
		EList<Clock> triggers, 
		EList<Reaction> reactions
	) {
		pr("exports.initialize = function () {\n")
		indent()
		// Define variables for each parameter.
		for(parameter: parameters) {
			pr('''var «parameter» = this.getParameter("«parameter»");''');
		}
		
		// Add the input handlers.
		for (input: handlers.keySet) {
			for (handler: handlers.get(input)) {
				pr('''this.addInputHandler("«input»", «handler».bind(this));''')
			}
		}
		// Add the clock reactions.
		for (clock: clockReactions.keySet) {
			// FIXME: Handle the variants of clock arguments.
			val clockParams = clocks.get(clock).period
			// FIXME: Above could be null (one-time invocation).
			for (handler: clockReactions.get(clock)) {
				pr('''setInterval(«handler».bind(this), «clockParams.period»);''')
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
			pr('''function «functionName»() {''')
			indent()
			// Add variable declarations for inputs.
			// Meanwhile, record the mapping from triggers to handlers.
			if (reaction.triggers !== null) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
						// The trigger is an input.

						// Declare a variable in the generated code.
						// NOTE: Here we are not using get() because null works
						// in JavaScript.
						pr('''var «trigger» = get("«trigger»");''')

						// Record this input so that we can add an input handler.
						var list = handlers.get(trigger)
						if (list === null) {
							list = new LinkedList<String>()
							handlers.put(trigger, list)
						}
						list.add(functionName)
					} else if (clocks.get(trigger) !== null) {
						// The trigger is a clock.
						// Record this so we can schedule this reaction in initialize.
						var list = clockReactions.get(trigger)
						if (list === null) {
							list = new LinkedList<String>()
							clockReactions.put(trigger, list)
						}
						list.add(functionName)
					} else {
						// This is checked by the validator (See LinguaFrancaValidator.xtend).
						// Nevertheless, in case we are using a command-line tool, we report the line number.
						// Just report the exception. Do not throw an exception so compilation can continue.
						var node = NodeModelUtils.getNode(reaction)
						System.err.println("Line "
							+ node.getStartLine()
							+ ": Trigger '" + trigger + "' is neither an input nor a clock.")
					}
				}
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
}
