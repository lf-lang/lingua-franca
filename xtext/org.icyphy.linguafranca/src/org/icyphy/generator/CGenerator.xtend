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
import org.icyphy.linguaFranca.Action
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
	// For each reactor, we collect a set of input and parameter names.
	var inputs = newHashSet()
	var parameters = newLinkedList()
	var reactionCount = 0
	
	// Map from timer name to Timing object.
	var timers = new HashMap<String,Timing>()
	// Map from timer or action name to reaction name(s) triggered by it.
	var triggerToReactions = new LinkedHashMap<String,LinkedList<String>>()
	
	// Map from action name to Action object.
	var actions = new HashMap<String,Action>()
	// Map from action name to index of the trigger in the trigger table.
	var actionToTriggerTableIndex = new HashMap<String,Integer>()
	
	// All header code goes into this string buffer.
	var header = new StringBuilder 
	// FIXME: put all prototypes and constants in a header file
	
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
		triggerToReactions.clear()
		actions.clear()
		actionToTriggerTableIndex.clear()
		//triggerTable = new StringBuffer()
		
		//reactionCount = 1   // Start reaction count at 1
		
		pr(includes)
		pr(defines)
		pr(declarations)

		// Record timers.
		var count = 0;
		
		for (timer: component.componentBody.timers) {
			count++
			var timing = timer.timing
			if (timing === null) {
				timing = LinguaFrancaFactory.eINSTANCE.createTiming()
				timing.setOffset("0") // Same as NOW.
				timing.setPeriod("0") // Same as ONCE.
			} else {
				// FIXME: Do we really want this?
				if (timing.getOffset.equals("NOW")) {
					timing.setOffset("0")
				}
				if (timing.getPeriod.equals("ONCE")) {
					timing.setPeriod("0")
				}
			}
			timers.put(timer.name, timing)
		}
		
		// Record actions.
	    count = 0;
		for (action: component.componentBody.actions) {
			count++
			if (action.getDelay() === null) {
				action.setDelay("0")
			}
			actions.put(action.name, action)
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


		
		// Scan reactions
		scanReactions(component.componentBody.reactions)
		
		// Generate trigger table
		generateTriggerTable()

		// Print boilerplate
		pr(initialize)

		generateStartTimers()

		// Generate reactions
		generateReactions(component.componentBody.reactions)
		
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
//		pr("exports.initialize = function () {\n")
//		indent()
//		// Define variables for each parameter.
//		for(parameter: parameters) {
//			pr('''var «parameter» = this.getParameter("«parameter»");''');
//		}
//				
//		// Add the input and action handlers.
//		pr(triggerTable)
//
//		// Add the timer reactions.
//		for (timer: triggerToReactions.keySet) {
//			val timerParams = timers.get(timer)
//			for (handler: triggerToReactions.get(timer)) {
//				pr('''__schedule("«timer»", «handler».bind(this), «timerParams.offset», «timerParams.period»);''')
//			}
//		}
//		unindent()
//		pr("};")
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
			if (reaction.gets !== null && reaction.gets.gets !== null) {
				for(get: reaction.gets.gets) {
					// FIXME: Convert to C.
					pr('''var «get» = get("«get»");''')
				}
			}
			// Define variables for each declared output or action.
			// FIXME: sets would be better named "produces".
			if (reaction.sets !== null && reaction.sets.sets !== null) {
				for(set: reaction.sets.sets) {
					if (actions.get(set) !== null) {
						// An action is produced.
						pr('''trigger_t* «set» = trigger_table[«actionToTriggerTableIndex.get(set)»];''')
					}
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

	/** Scan reaction declarations and print them in the generated code.
	 */
	def scanReactions(EList<Reaction> reactions) {
		var id = 0
		val reactionDecls = new StringBuffer()
		
		for (reaction: reactions) {
			val reactionName = "reaction" + id;
		 	pr("void reaction_function" + id + "();")
			reactionDecls.append("reaction_t " + reactionName + " = {reaction_function" + id + ", 0, 0};\n");
			id++;
			// Iterate over the reaction's triggers
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
					if (inputs.contains(trigger)) {
                        // FIXME
                    } else if (timers.get(trigger) !== null) {
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
		}
		pr("\n" + reactionDecls.toString())
	}
	
	def generateStartTimers() {
		 pr("void startTimers() {")
		 indent()
		 // __schedule(trigger_table[i], 0); 
		 for (timer : timers.keySet()) {
		 	pr("__schedule(&" + timer + ", 0);")
		 }
		 unindent()
		 pr("}")
	}
	
	/** Generate the trigger table.
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
				//result.append('reaction_t ' + reactionName + ' = {' + functionName + ', 0, 0};')
				result.append('\n')
				if (names.length != 0) {
					names.append(", ")
				}
				names.append('&' + reactionName)
			}
			result.append('reaction_t* ' + triggerName + '_reactions[' + numberOfReactionsTriggered + '] = {' + names + '};')
			result.append('\n')
			result.append('trigger_t ' + triggerName + ' = {')
			result.append('\n')
			if (timers.get(triggerName) !== null) {
				result.append(triggerName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ timers.get(triggerName).offset
					+ ', '
					+ timers.get(triggerName).period
				)
			} else if (actions.get(triggerName) !== null) {
				result.append(triggerName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ actions.get(triggerName).getDelay()
					+ ', 0' // 0 is ignored since actions don't have a period.
				)
				actionToTriggerTableIndex.put(triggerName, count)
			}
			result.append('\n};\n')
			if (triggerTable.length != 0) {
				triggerTable.append(', ')
			}
			triggerTable.append("&")
			triggerTable.append(triggerName)
			count++
		}
		pr('#define TRIGGER_TABLE_SIZE ' + count + '\n')
		result.append('trigger_t* trigger_table[TRIGGER_TABLE_SIZE] = {' + triggerTable + '};')
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
	// FIXME: pqueue.h and pqueue.c need to be copied to target directory.
	val static includes = '''
		#include <stdio.h>
		#include <stdlib.h>
		#include <time.h>
		#include "pqueue.h"
	'''
	
	// FIXME: May want these to application dependent, hence code generated.
	val static defines = '''
		#define INITIAL_TAG_QUEUE_SIZE 10
		#define INITIAL_INDEX_QUEUE_SIZE 10
		#define MILLION 1000000L
		#define BILLION 1000000000L
	'''
	
	// FIXME: The following should probably all go into library files.
	val static declarations = '''
		// ********* Type definitions included for all actors.
		// NOTE: Units for time are dealt with at compile time.
		typedef struct {
		  int time;         // a point in time
		  int microstep;    // superdense time index
		} instant_t;
		
		// Intervals of time do not involve the microstep.
		typedef int interval_t;
		
		// Topological sort index for reactions.
		typedef pqueue_pri_t index_t;
						
		// Handles for scheduled triggers.
		typedef int handle_t;
		
		// Reaction function type
		typedef void(*reaction_function_t)(void);

		// A reaction.
		typedef struct reaction_t {
		  reaction_function_t function;
		  index_t index;
		  // FIXME: add uses, produces, etc.?
		  size_t pos; // Used by priority queue.
		} reaction_t;
		
		typedef struct {
			reaction_t** reactions;
			int number_of_reactions;
			interval_t offset; // For an action, this will be a minimum delay.
			interval_t period;
		} trigger_t;		

		// Event to put in the event queue.
		typedef struct event_t {
		  instant_t tag;      // instant of the event (time, microstep)
		  trigger_t* trigger;
		  size_t pos;         // position in the priority queue 
		} event_t;

		instant_t current_time = {0, 0}; // FIXME: This should not be modifiable by reactors.
		void startTimers();
	'''
		
	val static initialize = '''
		
		// Compare priorities.
		static int cmp_pri(pqueue_pri_t next, pqueue_pri_t curr) {
		  return (next > curr);
		}
		// Get priorities based on tags (time and microstep).
		// Used for sorting event_t structs.
		static pqueue_pri_t get_tag_pri(void *a) {
		  // stick the time and microstep together into an unsigned long long
		  return ((pqueue_pri_t)(((event_t*) a)->tag.time) << 32) | (pqueue_pri_t)(((event_t*) a)->tag.microstep);
		}
		// Get priorities based on indices.
		// Used for sorting reaction_t structs.
		static pqueue_pri_t get_index_pri(void *a) {
		  // stick the time and microstep together into an unsigned long long
		  return ((reaction_t*) a)->index;
		}
		// Set priority.
		static void set_pri(void *a, pqueue_pri_t pri) {
		  // ignore this; priorities are fixed
		}
		// Get position in the queue.
		static size_t get_pos(void *a) {
		  return ((event_t*) a)->pos;
		}
		// Set position.
		static void set_pos(void *a, size_t pos) {
		  ((event_t*) a)->pos = pos;
		}
		// Priority queues.
		pqueue_t* eventQ;     // For sorting by tag (time, microstep)
		pqueue_t* reactionQ;  // For sorting by index (topological sort)
		
		handle_t __handle = 0;
		
		// Schedule the specified trigger at current_time plus the delay.
		handle_t __schedule(trigger_t* trigger, interval_t delay) {
		    event_t* e = malloc(sizeof(struct event_t));
		    e->tag.time = current_time.time + delay;
		    if (delay == 0) {
		    	e->tag.microstep = current_time.microstep + 1;
		    } else {
		    	e->tag.microstep = 0;
		    }
		    e->trigger = trigger;
		    // FIXME: If there already is an event in the queue with the
		    // same tag and trigger, then replace it rather than adding another
		    // one (replacement is not needed until these carry arguments).
		    // This should be fixed in the pqueue_insert() function.
		    pqueue_insert(eventQ, e);
		    // FIXME: make a record of handle and implement unschedule.
		    return __handle++;
		}
		// Schedule the specified trigger at current_time plus the
		// offset declared in the trigger plus the extra_delay.
		handle_t schedule(trigger_t* trigger, interval_t extra_delay) {
			return __schedule(trigger, trigger->offset + extra_delay);
		}
		struct timespec __physicalStartTime;
		
		// Return the time elapsed in nanoseconds from the first to the seconds
		// time or zero if the first is greater than or equal to the second.
		long time_elapsed(struct timespec* first, struct timespec* second) {
		    long result = (second->tv_sec - first->tv_sec) * BILLION
		            + (second->tv_nsec - first->tv_nsec);
		    if (result < 0L) {
		        result = 0L;
		    }
		    return result;
		}
		
		// Wait until physical time matches or exceeds the start time of execution
		// plus the current_time plus the specified logical time.  If this is not
		// interrupted, then advance current_time by the specified logical_delay. 
		// Return 0 if time advanced to the time of the event and -1 if the wait
		// was interrupted.
		int wait_until(event_t* event) {
		    printf("-------- Waiting for logical time %d.\n", event->tag.time);
		    // FIXME: Assuming logical time is in milliseconds.
		    // FIXME: Check this carefully for overflow and efficiency.
		    long logical_time_ns = event->tag.time * MILLION;
		    
		    // Get the current physical time.
		    struct timespec current_physical_time;
		    clock_gettime(CLOCK_REALTIME, &current_physical_time);
		    
		    long elapsed_physical_time_ns = time_elapsed(&__physicalStartTime, &current_physical_time);
		    long ns_to_wait = logical_time_ns - elapsed_physical_time_ns;
		    
		    if (ns_to_wait <= 0) {
		        // Advance current time.
		        current_time.time = event->tag.time;
		        current_time.microstep = event->tag.microstep;
		        return 0;
		    }
		    
		    // FIXME: Use timespec for logical time too!
		    struct timespec wait_time = {ns_to_wait / BILLION, ns_to_wait % BILLION};
		    printf("-------- Waiting %ld seconds, %ld nanoseconds.\n", ns_to_wait / BILLION, ns_to_wait % BILLION);
		    struct timespec remaining_time;
		    // FIXME: If the wait time is less than the time resolution, don't sleep.
		    if (nanosleep(&wait_time, &remaining_time) != 0) {
		        // Sleep was interrupted.
		        // Remaining time is in remaining_time.
		        // FIXME: Set current_time.
		        
		        return -1;
		    }
		    // Advance current time.
		    current_time.time = event->tag.time;
		    current_time.microstep = event->tag.microstep;
		    return 0;
		}
		// Wait until physical time matches or exceeds the time of the least tag
		// on the event queue. If theres is no event in the queue, return 0.
		// After this wait, advance current_time to match
		// this tag. Then pop the next event(s) from the
		// event queue that all have the same tag, and extract from those events
		// the reactions that are to be invoked at this logical time.
		// Sort those reactions by index (determined by a topological sort)
		// and then execute the reactions in order. Each reaction may produce
		// outputs, which places additional reactions into the index-ordered
		// priority queue. All of those will also be executed in order of indices.
		// Finally, return 1.
		int next() {
			event_t* event = pqueue_peek(eventQ);
			if (event == NULL) {
				return 0;
			}
			// Wait until physical time >= event.tag.time
			wait_until(event);
			
		  	// Pop all events from eventQ with timestamp equal to current_time
		  	// stick them into reaction.
		  	do {
		  	 	event = pqueue_pop(eventQ);
		  	 	for (int i = 0; i < event->trigger->number_of_reactions; i++) {
		  	 		// FIXME: As above, don't insert duplicate reactions.
		  	 		// Same fix in pqueue_insert should work.
		  	 		pqueue_insert(reactionQ, event->trigger->reactions[i]);
		  	 	}
		  	 	if (event->trigger->period > 0) {
		  	 		// Reschedule the trigger.
		  	 		__schedule(event->trigger, event->trigger->period);
		  	 	}
		  	 	
				// FIXME: Recycle this event instead of freeing it.
				free(event);
				
				event = pqueue_peek(eventQ);
		  	} while(event != NULL
		  			&& event->tag.time == current_time.time
		  			&& event->tag.microstep == current_time.microstep);
		
			// Handle reactions.
			while(pqueue_size(reactionQ) > 0) {
				reaction_t* reaction = pqueue_pop(reactionQ);
				reaction->function();
			}
			
			return 1;
		}
		
		void initialize() {
			current_time.time = 0; // FIXME: Obtain system time.
			eventQ = pqueue_init(INITIAL_TAG_QUEUE_SIZE, cmp_pri, get_tag_pri, set_pri, get_pos, set_pos);
			reactionQ = pqueue_init(INITIAL_INDEX_QUEUE_SIZE, cmp_pri, get_index_pri, set_pri, get_pos, set_pos);
			clock_gettime(CLOCK_REALTIME, &__physicalStartTime);
			printf("Start execution at time %splus %ld nanoseconds.\n",
					ctime(&__physicalStartTime.tv_sec), __physicalStartTime.tv_nsec);
		}

		int main(int argc, char* argv[]) {
			initialize();
			startTimers();
			// FIXME: Need stopping conditions.
			while (next() != 0);
			return 0;
		}
	'''
}
