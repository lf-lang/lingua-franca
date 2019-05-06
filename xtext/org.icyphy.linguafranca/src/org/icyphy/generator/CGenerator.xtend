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
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Composite
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Timer

/**
 * Generator for C target.
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill, Mehrdad Niknami
 */
class CGenerator extends GeneratorBase {
		
	// For each reactor, we collect a set of input and parameter names.
	var inputs = newHashSet()
	var parameters = newLinkedList()
	var reactionCount = 0
	
	// Map from timer or action name to reaction name(s) triggered by it.
	var triggerToReactions = new LinkedHashMap<String,LinkedList<String>>()
	
	// Map from action name to Action object.
	var actions = new HashMap<String,Action>()
	// Map from action name to index of the trigger in the trigger table.
	var actionToTriggerTableIndex = new HashMap<String,Integer>()
			
	var Resource _resource;
	
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
				
		_resource = resource
		// Handle reactors and composites.
		for (component : resource.allContents.toIterable.filter(Component)) {
			clearCode()
			generateComponent(component, importTable)
			val componentBody = component.componentBody
			fsa.generateFile(componentBody.name + ".c", getCode())		
		}
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
		parameters.clear()  // Reset set of parameters.
		triggerToReactions.clear()
		actions.clear()
		actionToTriggerTableIndex.clear()
		//triggerTable = new StringBuffer()
		
		//reactionCount = 1   // Start reaction count at 1
		
		pr(includes)
		pr(defines)
		pr(windows)		// Windows support.
		pr(declarations)
		pr(initialize_time)
				
		// Record actions.
	    var count = 0;
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
			pr_source_line_number(reaction)
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
	
	def generateStartTimers() {
		 pr("void start_timers() {")
		 indent()
		 for (timer : getTimerNames()) {
		 	var timing = getTiming(timer)
		 	pr("__schedule(&" + timer + ", "
		 			+ unitAdjustment(timing.offset, "ns") + "LL);"
		 	)
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
			var timing = getTiming(triggerName)
			if (timing !== null) {
				result.append(triggerName + '_reactions, '
					+ numberOfReactionsTriggered + ', '
					+ unitAdjustment(timing.offset, "ns")
					+ 'LL, '
					+ unitAdjustment(timing.period, "ns")
					+ 'LL'
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
		
	// Print the #line compiler directive with the line number of
	// the most recently used node.
	private def pr_source_line_number(EObject reaction) {
		var node = NodeModelUtils.getNode(reaction)
		pr("#line " + node.getStartLine() + ' "' + _resource.getURI() + '"')
		
	}
	
	// FIXME: pqueue.h and pqueue.c need to be copied to target directory.
	val static includes = '''
		#include <stdio.h>
		#include <stdlib.h>
		#include <time.h>
		#include <errno.h>
		#include "pqueue.h"
	'''
	
	// FIXME: May want these to application dependent, hence code generated.
	val static defines = '''
		#define INITIAL_TAG_QUEUE_SIZE 10
		#define INITIAL_INDEX_QUEUE_SIZE 10
		#define BILLION 1000000000LL
	'''
	
	// FIXME: The following should probably all go into library files.
	// Windows is not POSIX, so we include here compatibility definitions.
	val static windows = '''
#if _WIN32 || WIN32
#pragma warning(disable: 4204 4255 4459 4710)
#ifdef  _M_X64
typedef long long intptr_t;
#else
typedef int intptr_t;
#endif
intptr_t __cdecl _loaddll(char *);
int __cdecl _unloaddll(intptr_t);
int (__cdecl * __cdecl _getdllprocaddr(intptr_t, char *, intptr_t))(void);
typedef long NTSTATUS;
typedef union _LARGE_INTEGER *PLARGE_INTEGER;
typedef NTSTATUS __stdcall NtDelayExecution_t(unsigned char Alertable, PLARGE_INTEGER Interval); NtDelayExecution_t *NtDelayExecution = NULL;
typedef NTSTATUS __stdcall NtQueryPerformanceCounter_t(PLARGE_INTEGER PerformanceCounter, PLARGE_INTEGER PerformanceFrequency); NtQueryPerformanceCounter_t *NtQueryPerformanceCounter = NULL;
typedef NTSTATUS __stdcall NtQuerySystemTime_t(PLARGE_INTEGER SystemTime); NtQuerySystemTime_t *NtQuerySystemTime = NULL;
typedef enum { CLOCK_REALTIME = 0 } clockid_t;
static int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    int result = -1;
    long long timestamp, counts, counts_per_sec;
    switch (clk_id) {
    case CLOCK_REALTIME:
        NtQuerySystemTime((PLARGE_INTEGER)&timestamp);
        tp->tv_sec = (time_t)(timestamp / (BILLION / 100));
        tp->tv_nsec = (long)((timestamp % (BILLION / 100)) * 100);
        result = 0;
        break;
    default:
        errno = EINVAL;
        result = -1;
        break;
    }
    return result;
}
static int nanosleep(const struct timespec *req, struct timespec *rem)
{
    unsigned char alertable = rem ? 1 : 0;
    long long duration = -(req->tv_sec * (BILLION / 100) + req->tv_nsec / 100);
    NTSTATUS status = (*NtDelayExecution)(alertable, (PLARGE_INTEGER)&duration);
    int result = status == 0 ? 0 : -1;
    if (alertable)
    {
        if (status < 0)
        { errno = EINVAL; }
        else if (status > 0 && clock_gettime(CLOCK_MONOTONIC, rem) == 0)
        { errno = EINTR; }
    }
    return result;
}
#endif
	'''
	
	// FIXME: The following should probably all go into library files.
	val static declarations = '''
		// ********* Type definitions included for all actors.
		// WARNING: If this code is used after about the year 2262,
		// then representing time as a long long will be insufficient.
		typedef long long instant_t;
		
		// Intervals of time.
		typedef long long interval_t;
		
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
		  instant_t time;
		  trigger_t* trigger;
		  size_t pos;         // position in the priority queue 
		} event_t;
	'''
	
	val static initialize_time = '''
		// FIXME: This should not be in scope for reactors.
		instant_t current_time = 0LL;
		// The following should be in scope for reactors:
		long long get_logical_time() {
			return current_time;
		}
		void start_timers();
	'''
		
	val static initialize = '''
		
		// Compare priorities.
		static int cmp_pri(pqueue_pri_t next, pqueue_pri_t curr) {
		  return (next > curr);
		}
		// Compare events.
		static int cmp_evt(void* next, void* curr) {
		  return (((event_t*)next)->trigger == ((event_t*)curr)->trigger);
		}
		// Compare reactions.
		static int cmp_rct(void* next, void* curr) {
		  // each reaction has a unique priority
		  // no need to compare pointers
		  return 1;
		  // (next == curr);
		}
		// Get priorities based on time.
		// Used for sorting event_t structs.
		static pqueue_pri_t get_tag_pri(void *a) {
		  return (pqueue_pri_t)(((event_t*) a)->time);
		}
		// Get priorities based on indices, which reflect topological sort.
		// Used for sorting reaction_t structs.
		static pqueue_pri_t get_index_pri(void *a) {
		  return ((reaction_t*) a)->index;
		}
		// Set priority.
		static void set_pri(void *a, pqueue_pri_t pri) {
		  // ignore this; priorities are fixed
		}
		// Get position in the queue of the specified event.
		static size_t get_pos(void *a) {
		  return ((event_t*) a)->pos;
		}
		// Set position of the specified event.
		static void set_pos(void *a, size_t pos) {
		  ((event_t*) a)->pos = pos;
		}
		// Priority queues.
		pqueue_t* eventQ;     // For sorting by time.
		pqueue_t* reactionQ;  // For sorting by index (topological sort)
		
		handle_t __handle = 0;
		
		// Schedule the specified trigger at current_time plus the delay.
		handle_t __schedule(trigger_t* trigger, interval_t delay) {
		    event_t* e = malloc(sizeof(struct event_t));
		    e->time = current_time + delay;
		    e->trigger = trigger;
		    // NOTE: There is no need for an explicit microstep because
		    // when this is called, all events at the current tag
		    // (time and microstep) have been pulled from the queue,
		    // and any new events added at this tag will go into the reactionQ
		    // rather than the eventQ, so anything put in the eventQ with this
		    // same time will automatically be executed at the next microstep.
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
				
		// Wait until physical time matches or exceeds the start time of execution
		// plus the current_time plus the specified logical time.  If this is not
		// interrupted, then advance current_time by the specified logical_delay. 
		// Return 0 if time advanced to the time of the event and -1 if the wait
		// was interrupted.
		int wait_until(event_t* event) {
		    // printf("-------- Waiting for logical time %lld.\n", event->time);
		    long long logical_time_ns = event->time;
		    
		    // Get the current physical time.
		    struct timespec current_physical_time;
		    clock_gettime(CLOCK_REALTIME, &current_physical_time);
		    
		    long long ns_to_wait = logical_time_ns
		    		- (current_physical_time.tv_sec * BILLION
		    		+ current_physical_time.tv_nsec);
		    
		    if (ns_to_wait <= 0) {
		        // Advance current time.
		        current_time = event->time;
		        return 0;
		    }
		    
		    // timespec is seconds and nanoseconds.
		    struct timespec wait_time = {(time_t)ns_to_wait / BILLION, (long)ns_to_wait % BILLION};
		    // printf("-------- Waiting %lld seconds, %lld nanoseconds.\n", ns_to_wait / BILLION, ns_to_wait % BILLION);
		    struct timespec remaining_time;
		    // FIXME: If the wait time is less than the time resolution, don't sleep.
		    if (nanosleep(&wait_time, &remaining_time) != 0) {
		        // Sleep was interrupted.
		        // May have been an asynchronous call to schedule(), or
		        // it may have been a control-C to stop the process.
		        // Set current time to match physical time, but not less than
		        // current logical time nor more than next time in the event queue.
		    	clock_gettime(CLOCK_REALTIME, &current_physical_time);
		    	long long current_physical_time_ns 
		    			= current_physical_time.tv_sec * BILLION
		    			+ current_physical_time.tv_nsec;
		    	if (current_physical_time_ns > current_time) {
		    		if (current_physical_time_ns < event->time) {
		    			current_time = current_physical_time_ns;
		    			return -1;
		    		}
		    	} else {
		    		// Advance current time.
		    		current_time = event->time;
		    		// FIXME: Make sure that the microstep is dealt with correctly.
		            return -1;
		        }
		    }
		    // Advance current time.
		    current_time = event->time;
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
			// Wait until physical time >= event.time
			if (wait_until(event) < 0) {
				// FIXME: sleep was interrupted. Handle that somehow here!
			}
			
		  	// Pop all events from eventQ with timestamp equal to current_time
		  	// stick them into reaction.
		  	do {
		  	 	event = pqueue_pop(eventQ);
		  	 	for (int i = 0; i < event->trigger->number_of_reactions; i++) {
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
		  			&& event->time == current_time);
		
			// Handle reactions.
			while(pqueue_size(reactionQ) > 0) {
				reaction_t* reaction = pqueue_pop(reactionQ);
				reaction->function();
			}
			
			return 1;
		}
		
		void initialize() {
			#if _WIN32 || WIN32
			    intptr_t ntdll = _loaddll("ntdll.dll");
			    if (ntdll != 0 && ntdll != -1)
			    {
			        NtDelayExecution = (NtDelayExecution_t *)_getdllprocaddr(ntdll, "NtDelayExecution", -1);
			        NtQueryPerformanceCounter = (NtQueryPerformanceCounter_t *)_getdllprocaddr(ntdll, "NtQueryPerformanceCounter", -1);
			        NtQuerySystemTime = (NtQuerySystemTime_t *)_getdllprocaddr(ntdll, "NtQuerySystemTime", -1);
			    }
			#endif
			
			current_time = 0; // FIXME: Obtain system time.
			eventQ = pqueue_init(INITIAL_TAG_QUEUE_SIZE, cmp_pri, get_tag_pri, get_pos, set_pos, cmp_evt);
			reactionQ = pqueue_init(INITIAL_INDEX_QUEUE_SIZE, cmp_pri, get_index_pri, get_pos, set_pos, cmp_rct);

			// Initialize logical time to match physical time.
			clock_gettime(CLOCK_REALTIME, &__physicalStartTime);
			printf("Start execution at time %splus %ld nanoseconds.\n",
					ctime(&__physicalStartTime.tv_sec), __physicalStartTime.tv_nsec);
			current_time = __physicalStartTime.tv_sec * BILLION
					+ __physicalStartTime.tv_nsec;
		}

		int main(int argc, char* argv[]) {
			initialize();
			start_timers();
			// FIXME: Need stopping conditions.
			while (next() != 0);
			return 0;
		}
	'''
}
