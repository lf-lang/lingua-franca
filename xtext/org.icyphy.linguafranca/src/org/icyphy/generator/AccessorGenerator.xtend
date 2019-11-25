/* Generator for Accessors, JavaScript code runnable in Node, CapeCode, and Ptolemy II. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/** Generator for Accessors, JavaScript code runnable in Node, CapeCode, and Ptolemy.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Chadlia Jerad <chadlia.jerad@gmail.com>}
 */

package org.icyphy.generator

import java.util.Hashtable
import java.util.LinkedHashMap
import java.util.LinkedList
import java.util.StringJoiner
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef

/**
 * Generator for Accessors.
 * @author Edward A. Lee, Chadlia Jerad
 */
class AccessorGenerator extends GeneratorBase {
	
    // Set of acceptable import targets includes only Accessor.
    val acceptableTargetSet = newHashSet('Accessor')

	// For each accessor, we collect a set of input and parameter names.
	var inputs = newHashSet()
	var reactionCount = 0
	
	// The directory name into which generated accessor definitions are put.
	// If there is a main reactor, then this will be the name of the source
	// file, with the extension .lf, appended with a "/".
	var directory = ""
	
	// Map from timer to reaction name(s) triggered by the timer.
	var timerReactions = new LinkedHashMap<Timer,LinkedList<String>>()
	
	// Text of generated code to add input handlers.
	var addInputHandlers = new StringBuffer()
		
	override void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context) {
				
		super.doGenerate(resource, fsa, context)
		
		// If there is a main reactor in the file, then the variable main will be non-null.
		// In this case, create a directory into which to put the reactor definitions
		// because accessors require one file per accessor.
		if (main !== null) {
            // IFileSystemAccess2 uses "/" as file system separator.
            directory = filename + "/"
		}
		
		// Handle reactors and composites.
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			clearCode()
			generateReactor(reactor)
			var filename = reactor.name
			if (filename.equalsIgnoreCase('main')) {
				filename = filename
			}
			fsa.generateFile(directory + filename + ".js", code)		
		}
		// If there is a main accessor, then create a file to run it using node.
		if (main !== null) {
		    var runFile = '''
		    // To run this: node «filename».js
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
		        args.unshift('«directory»«filename».js')
		        nodeHost.processCommandLineArguments(args);
		    }
		    '''
            fsa.generateFile(filename + '.js', runFile)        
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
	 */	
	override generateReactor(Reactor reactor) {
		super.generateReactor(reactor)

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
		reactorSetup(reactor)
		// Generate reactions
		generateReactions(reactor)
		// initialize function (initialize + triggers scheduling + input handlers)
		generateInitialize(reactor)
	}
	
	/** Generate the setup function definition for a reactor or composite.
	 */
	def reactorSetup(Reactor reactor) {
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
		for (param : reactor.parameters) {
			generateParameter(param)
		}
		// Generated instances
		for (instance: reactor.instantiations) {
			//generateInstantiate(instance, importTable)
			// FIXME: this should be done differently now
		}
		// Generated connections
		for (connection: reactor.connections) {
			pr('''this.connect(«portSpec(connection.leftPort.variable.name)», «portSpec(connection.rightPort.variable.name)»);''')
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
	
	def generateParameter(Parameter param) {
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
		for(parameter: reactor.parameters) {
			pr('''var «parameter.name» = this.getParameter("«parameter.name»");''');
		}
		
		// Add the input handlers.
		pr(addInputHandlers.toString)

		// Add the timer reactions.
		for (timer: timerReactions.keySet) {
			//val timerParams = getTiming(reactor, timer)
			for (handler: timerReactions.get(timer)) {
				var offsetStr = unitAdjustment(timer.offset, TimeUnit.MSEC)
				var periodStr = unitAdjustment(timer.period, TimeUnit.MSEC)
				pr('''__scheduleTimer("«timer»", «handler».bind(this), «offsetStr», «periodStr»);''')
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
				for (TriggerRef trigger: reaction.triggers) {
					if (trigger instanceof VarRef) {
						if (inputs.contains(trigger.variable)) {
						// The trigger is an input.
						// Declare a variable in the generated code.
						// NOTE: Here we are not using get() because null works
						// in JavaScript.
						pr(body, '''var «trigger» = get("«trigger»");''')

						// Generate code for the initialize() function here so that input handlers are
						// added in the same order that they are declared.
				   		addInputHandlers.append('''this.addInputHandler("«trigger»", «functionName».bind(this));''')
					} else if (trigger.variable instanceof Timer) {
						// The trigger is a timer.
						// Record this so we can schedule this reaction in initialize.
						var list = timerReactions.get(trigger)
						if (list === null) {
							list = new LinkedList<String>()
							timerReactions.put(trigger.variable as Timer, list)
						}
						list.add(functionName)
					} else if (trigger.variable instanceof Action) {
					    // The trigger is an action.
					    args.add(trigger.variable.name)
					    // Make sure there is an entry for this action in the action table.
					    pr('''
					    if (!actionTable.«trigger.variable») {
					        actionTable.«trigger.variable» = [];
					    }
					    actionTable.«trigger.variable».push(«functionName»);
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
			for(inp : reaction.sources?:emptyList) {
				pr(body, '''var «inp.variable.name» = get("«inp.variable.name»");''')
			}
			
			// Define variables for each declared output.
			
			for (effect : reaction.effects ?: emptyList) {
				// Set the output name variable equal to a string.
				// FIXME: String name is too easy to cheat!
				// LF coder could write set('foo', value) to write to
				// output foo without having declared the write.
				pr(body, '''var «effect.variable» = "«effect.variable»";''');
			}
						
			// Define variables for each parameter.
			for(parameter: reactor.parameters) {
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
	def generateInstantiate(ReactorInstance instance, Hashtable<String,String> importTable) {
		var className = importTable.get(instance.definition.reactorClass);
		if (className === null) {
		    // This is not an imported accessor.
			className = directory + instance.definition.reactorClass
		}
		pr('''var «instance.definition.name» = this.instantiate('«instance.definition.name»', '«className»');''')
		if (instance.definition.parameters !== null) {
			for (param: instance.definition.parameters) {
				pr('''«instance.definition.name».setParameter('«param.lhs.name»', «removeCodeDelimiter(param.rhs.value)»);''')
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
	
    ////////////////////////////////////////////////
    //// Protected methods
    
    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set is a set of case-insensitive
     *  strings specifying target names.
     */
    override acceptableTargets() {
        acceptableTargetSet
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
		// * value: An argument to pass to any reactions that are to be triggered.
		// FIXME: For reactions that are triggered by more than one action, this does not
		// have quite the right semantics. If two of those actions are simultaneous, the reaction
		// will be triggered twice instead of once!
		function schedule(action, offset, value) {
		    var reactions = actionTable[action];
		    if (reactions) {
		        for (var i = 0; i < reactions.length; i++) {
		            var reaction = reactions[i];
		            var reactionArgNames = reactionArgsTable[reaction];
		            var reactionArgs = [];
		            for (var j = 0; j < reactionArgNames.length; j++) {
		                var argName = reactionArgNames[i];
		                if (argName === action) {
		                    reactionArgs.push(value);
		                } else {
		                    reactionArgs.push(null);
		                }
		            }
		            // FIXME: offset needs to be added to the delay specified by the action.
		            setTimeout(function() {
		                reaction.apply(self, reactionArgs);
		            }, offset);
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
