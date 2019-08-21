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
	
	// The directory name into which generated accessor definitions are put.
	// If there is a main reactor, then this will be the name of the source
	// file, with the extension .lf, appended with a "/".
	var directory = ""
	
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
		
		// If there is a main reactor in the file, then the variable main will be non-null.
		// In this case, create a directory into which to put the reactor definitions
		// because accessors require one file per accessor.
		if (main !== null) {
            // IFileSystemAccess2 uses "/" as file system separator.
            directory = _filename + "/"
		}
		
		// Handle reactors and composites.
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			clearCode()
			generateReactor(reactor, importTable)
			var filename = reactor.name
			if (filename.equalsIgnoreCase('main')) {
				filename = _filename
			}
			fsa.generateFile(directory + filename + ".js", code)		
		}
		// If there is a main accessor, then create a file to run it using node.
		if (main !== null) {
		    var runFile = '''
		    // To run this: node «_filename».js
		    var nodeHost = null;
		    try {
		        nodeHost = require('@terraswarm/accessors');
		    } catch {
		        console.log('ERROR: accessors library is not installed. Install with: npm install @terraswarm/accessors');
		        console.log('NOTE: Do not do this in the lingua-franca source tree because it confuses the build system.');
		    }
		    if (nodeHost !== null) {
		        // Read the command-line arguments after the first two, if there are any.
		        var args = process.argv.slice(2);
		        // Prepend those with the path of the main accessor and process them.
		        args.unshift('«directory»«_filename».js')
		        nodeHost.processCommandLineArguments(args);
		    }
		    '''
            fsa.generateFile(_filename + '.js', runFile)        
		}
		// Copy the required library files into the target filesystem.
		// No longer used.
        // var runFile = readFileInClasspath("/lib/Accessors/run")
        // fsa.generateFile("run", runFile)		
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
				pr('''__scheduleTimer("«timer»", «handler».bind(this), «offset», «period»);''')
			}
		}
		unindent()
		pr("};")
	}			

	/** Generate reaction functions definition for a reactor or a composite.
	 */
	def generateReactions(Reactor reactor) {
		val reactions = reactor.reactions
		for (reaction: reactions) {
            val functionName = "reaction" + (reactionCount++)
            val body = new StringBuilder()
            var args = new LinkedList<String>()
			// Add variable declarations for inputs and actions.
			// Meanwhile, record the mapping from triggers to handlers.
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
						// The trigger is an input.
						// Declare a variable in the generated code.
						// NOTE: Here we are not using get() because null works
						// in JavaScript.
						pr(body, '''var «trigger» = get("«trigger»");''')

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
					} else if (getAction(reactor, trigger) !== null) {
					    // The trigger is an action.
					    args.add(trigger)
					    // Make sure there is an entry for this action in the action table.
					    pr('''
					    if (!actionTable.«trigger») {
					        actionTable.«trigger» = [];
					    }
					    actionTable.«trigger».push(«functionName»);
					    ''')
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
				if (!args.isEmpty()) {
				    // This reaction is triggered by one or more action.
				    // Add to the action table.
				    pr('''reactionArgsTable[«functionName»] = ["«args.join('", "')»"];''')
				}
			} else {
				// No triggers are given, which means react to any input.
				// Declare a variable for every input.
				// NOTE: Here we are not using get() because null works in JavaScript.
				for (input: inputs) {
					pr(body, '''var «input» = get("«input»");''')
				}
				addInputHandlers.append('''this.addInputHandler(null, «functionName».bind(this));''')
			}
			// Define variables for non-triggering inputs.
			if (reaction.uses !== null && reaction.uses.uses !== null) {
				for(get: reaction.uses.uses) {
					pr(body, '''var «get» = get("«get»");''')
				}
			}
			// Define variables for each declared output.
			if (reaction.produces !== null && reaction.produces.produces !== null) {
				for(output: reaction.produces.produces) {
					// Set the output name variable equal to a string.
					// FIXME: String name is too easy to cheat!
					// LF coder could write set('foo', value) to write to
					// output foo without having declared the write.
					pr(body, '''var «output» = "«output»";''');
				}
			}			
			// Define variables for each parameter.
			for(parameter: getParameters(reactor)) {
				pr(body, '''var «parameter.name» = this.getParameter("«parameter.name»");''');
			}

			// Code verbatim from 'reaction'
			pr(body, removeCodeDelimiter(reaction.code))
			
			pr('''function «functionName»(«args.join(',')») {''')
            indent()
			pr(body.toString())
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
		    // This is not an imported accessor.
			className = directory + instance.reactorClass
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
		var __SCHEDULE_TIMER_TABLE = {}
		
		// Function to schedule timer reactions. The arguments are:
		// * trigger: The name of the timer.
		// * handler: Function to invoke.
		// * offset: The time after the current time for the first action.
		// * period: The period at which to repeat the action, 0 for one time, and -1 to cancel
		//   a previously initiated periodic action.
		function __scheduleTimer(trigger, handler, offset, period) {
			if (offset >= 0 && period == 0) {
				var handle = setTimeout(handler, offset);
				__SCHEDULE_TIMER_TABLE[trigger] = handle;
				setTimeout(function() { __SCHEDULE_TIMER_TABLE[trigger] = null; }, offset);
			} else if (offset == 0 && period > 0) {
				setTimeout(handler, 0); // First invocation.
				var handle = setInterval(handler, period);
				__SCHEDULE_TIMER_TABLE[trigger] = handle;
			} else if (offset > 0 && period > 0) {
				var handle = setTimeout(function() {
					setTimeout(handler, 0); // First invocation.
					var handle = setInterval(handler, period);
					__SCHEDULE_TIMER_TABLE[trigger] = handle;
				}, offset);
				__SCHEDULE_TIMER_TABLE[trigger] = handle;
			} else if (period < 0) {
				if (__SCHEDULE_TIMER_TABLE[trigger]) {
					clearTimeout(__SCHEDULE_TIMER_TABLE[trigger]);
					clearInterval(__SCHEDULE_TIMER_TABLE[trigger]);
					__SCHEDULE_TIMER_TABLE[trigger] = null;
				}
			} else {
				throw("Illegal schedule arguments: " + offset +", " + period);
			}
		}
		// A table of actions indexed by the action name.
		// Each property value is an array of function names for reactions triggered by this action.
		var actionTable = {};
		// A table of function arguments for reactions that take arguments.
		var reactionArgsTable = {};
		var self = this;
		
		// Function to schedule reactions to actions. The argument are:
		// * action: The name of the action.
		// * offset: Additional logical time beyond the action delay for reactions to be invoked.
		// * payload: An argument to pass to any reactions that are to be triggered.
		// FIXME: For reactions that are triggered by more than one action, this does not
		// have quite the right semantics. If two of those actions are simultaneous, the reaction
		// will be triggered twice instead of once!
		function schedule(action, offset, payload) {
		    var reactions = actionTable[action];
		    if (reactions) {
		        for (var i = 0; i < reactions.length; i++) {
		            var reaction = reactions[i];
		            var reactionArgNames = reactionArgsTable[reaction];
		            var reactionArgs = [];
		            for (var j = 0; j < reactionArgNames.length; j++) {
		                var argName = reactionArgNames[i];
		                if (argName === action) {
		                    reactionArgs.push(payload);
		                } else {
		                    reactionArgs.push(null);
		                }
		            }
		            // FIXME: Assuming only one argument here.
		            var boundReaction = reaction.bind(self, payload);
		            // FIXME: offset needs to be added to the delay specified by the action.
		            setTimeout(boundReaction, offset);
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

		// ********** End boilerplate
	'''
}
