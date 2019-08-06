/*
 * Generator for Accessors, JavaScript code runnable in Node, CapeCode, and Ptolemy II.
 */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.Hashtable
import java.util.LinkedHashMap
import java.util.LinkedList
import java.util.StringJoiner
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reactor

/**
 * Generator for Accessors.
 * @author Edward A. Lee, Chadlia Jerad
 */
class AccessorGenerator extends GeneratorBase {
	
	// For each accessor, we collect a set of input and parameter names.
	var inputs = newHashSet()
	var reactionCount = 0
	
	// Map from timer name to reaction name(s) triggered by the timer.
	var timerReactions = new LinkedHashMap<String,LinkedList<String>>()
	
	// Text of generated code to add input handlers.
	var addInputHandlers = new StringBuffer()
		
	override void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
				
		super.doGenerate(resource, fsa, context, importTable)
		
		// Handle reactors and composites.
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			clearCode()
			generateReactor(reactor, importTable)
			var filename = reactor.name
			if (filename.equalsIgnoreCase('main')) {
				filename = _filename
			}
			fsa.generateFile(filename + ".js", code)		
		}
		// Copy the required library files into the target filesystem.
        var runFile = readFileInClasspath("/lib/Accessors/run")
        fsa.generateFile("run", runFile)		
	}
	
	////////////////////////////////////////////
	//// Code generators.
	
	/** Generate a reactor or composite definition.
	 *  @param reactor The parsed reactor data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */	
	override generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
		super.generateReactor(reactor, importTable)

		inputs.clear()      // Reset set of inputs.
		timerReactions.clear()
		addInputHandlers = new StringBuffer()
		
		reactionCount = 1   // Start reaction count at 1.
		
		pr(boilerplate)
		if (reactor.preamble !== null) {
			pr("// *********** From the preamble, verbatim:")
			pr(removeCodeDelimiter(reactor.preamble.code))
			pr("\n// *********** End of preamble.")
		}
		// Reactor setup (inputs, outputs, parameters)
		reactorSetup(reactor, importTable)
		// Generate reactions
		generateReactions(reactor)
		// initialize function (initialize + triggers scheduling + input handlers)
		generateInitialize(reactor)
	}
	
	/** Generate the setup function definition for a reactor or composite.
	 */
	def reactorSetup(Reactor reactor, Hashtable<String,String> importTable) {
		pr("exports.setup = function () {")
		indent()
		// Generate Inputs, if any.
		for (input: reactor.inputs) {
			generateInput(input)
		}
		// Generate outputs, if any
		for (output: reactor.outputs) {
			generateOutput(output)
		}
		// Generate parameters, if any
		for (param : getParameters(reactor)) {
			generateParameter(param)
		}
		// Generated instances
		for (instance: reactor.instances) {
			generateInstantiate(instance, importTable)
		}
		// Generated connections
		for (connection: reactor.connections) {
			pr('''this.connect(«portSpec(connection.leftPort)», «portSpec(connection.rightPort)»);''')
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
	
	/** Generate the initialize function definition for an accessor.
	 *  This adds input handlers and timer reactions.
	 */
	def generateInitialize(Reactor reactor) {
		pr("exports.initialize = function () {\n")
		indent()
		// Define variables for each parameter.
		for(parameter: getParameters(reactor)) {
			pr('''var «parameter.name» = this.getParameter("«parameter.name»");''');
		}
		
		// Add the input handlers.
		pr(addInputHandlers.toString)

		// Add the timer reactions.
		for (timer: timerReactions.keySet) {
			val timerParams = getTiming(reactor, timer)
			for (handler: timerReactions.get(timer)) {
				var offset = unitAdjustment(timerParams.offset, "ms")
				var period = unitAdjustment(timerParams.period, "ms")
				pr('''schedule("«timer»", «handler».bind(this), «offset», «period»);''')
			}
		}
		unindent()
		pr("};")
	}			

	/** Generate reaction functions definition for a reactor or a composite.
	 */
	def generateReactions(Reactor reactor) {
		val reactions = reactor.reactions
		val functionName = "reaction" + reactionCount++
		for (reaction: reactions) {
			pr('''function «functionName»() {''')
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
						pr('''var «trigger» = get("«trigger»");''')

						// Generate code for the initialize() function here so that input handlers are
						// added in the same order that they are declared.
				   		addInputHandlers.append('''this.addInputHandler("«trigger»", «functionName».bind(this));''')
					} else if (getTiming(reactor, trigger) !== null) {
						// The trigger is a timer.
						// Record this so we can schedule this reaction in initialize.
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
				addInputHandlers.append('''this.addInputHandler(null, «functionName».bind(this));''')
			}
			// Define variables for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					pr('''var «get» = get("«get»");''')
				}
			}
			// Define variables for each declared output.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					// Set the output name variable equal to a string.
					// FIXME: String name is too easy to cheat!
					// LF coder could write set('foo', value) to write to
					// output foo without having declared the write.
					pr('''var «output» = "«output»";''');
				}
			}			
			// Define variables for each parameter.
			for(parameter: getParameters(reactor)) {
				pr('''var «parameter.name» = this.getParameter("«parameter.name»");''');
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
	def generateInstantiate(Instance instance, Hashtable<String,String> importTable) {
		var className = importTable.get(instance.reactorClass);
		if (className === null) {
			className = instance.reactorClass
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
	
	val static boilerplate = '''
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
		// Map from trigger names to handles for periodic invocations.
		var __SCHEDULE_TABLE = {}
		
		// Function to schedule an action. The arguments are:
		// * trigger: The name of the timer or action.
		// * handler: Function to invoke.
		// * offset: The time after the current time for the first action.
		// * period: The period at which to repeat the action, 0 for one time, and -1 to cancel
		//   a previously initiated periodic action.
		// FIXME: Make all but the first argument optional.
		function schedule(trigger, handler, offset, period) {
			if (offset >= 0 && period == 0) {
				var handle = setTimeout(handler, offset);
				__SCHEDULE_TABLE[trigger] = handle;
				setTimeout(function() { __SCHEDULE_TABLE[trigger] = null; }, offset);
			} else if (offset == 0 && period > 0) {
				setTimeout(handler, 0); // First invocation.
				var handle = setInterval(handler, period);
				__SCHEDULE_TABLE[trigger] = handle;
			} else if (offset > 0 && period > 0) {
				var handle = setTimeout(function() {
					setTimeout(handler, 0); // First invocation.
					var handle = setInterval(handler, period);
					__SCHEDULE_TABLE[trigger] = handle;
				}, offset);
				__SCHEDULE_TABLE[trigger] = handle;
			} else if (period < 0) {
				if (__SCHEDULE_TABLE[trigger]) {
					clearTimeout(__SCHEDULE_TABLE[trigger]);
					clearInterval(__SCHEDULE_TABLE[trigger]);
					__SCHEDULE_TABLE[trigger] = null;
				}
			} else {
				throw("Illegal schedule arguments: " + offset +", " + period);
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
}
