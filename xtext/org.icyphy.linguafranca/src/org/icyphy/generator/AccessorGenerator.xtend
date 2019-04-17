/*
 * Generator for Accessors, JavaScript code runnable in Node, CapeCode, and Ptolemy II.
 */
package org.icyphy.generator

import java.util.Hashtable
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Composite
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Clock
import org.icyphy.linguaFranca.Constructor
import org.eclipse.emf.common.util.EList
import org.icyphy.linguaFranca.Reaction

/**
 * Generator for Accessors.
 * @author Edward A. Lee
 */
class AccessorGenerator {
	// For each accessor, we collect a set of input and parameter names.
	var inputs = newHashSet();
	var parameters = newLinkedList();
	
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
		
		// Handle actors.
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			val code = new StringBuffer
			code.append(generateReactor(reactor, importTable));
			fsa.generateFile(reactor.name + ".js", code);		
		}
		
		// Handle composites.
		for (composite : resource.allContents.toIterable.filter(Composite)) {
			val code = new StringBuffer
			code.append(generateComposite(composite, importTable))
			fsa.generateFile(composite.name + ".js", code);		
		}
	}
	
	/** Return an instantiate statement followed by any required parameter
	 *  assignments.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */	
	def generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
		inputs.clear(); // Reset set of inputs.
		parameters.clear(); // Reset set of parameters.
		'''
		«boilerplate()»
		// Clock data structure:
		«FOR clock: reactor.clocks»
			«generateClock(clock)» 
		«ENDFOR»
		«IF reactor.preamble !== null»
		// *********** From the preamble, verbatim:
		«removeCodeDelimiter(reactor.preamble.code)»
		// *********** End of preamble.
		«ENDIF»
		// Reactor setup (inputs, outputs, parameters)
		«reactorSetup(reactor, importTable)»
		// Reactor initialize (initialize + triggers scheduling - missing inputHandlers)
		«generateConstructor(reactor.constructor, reactor.clocks, reactor.reactions)»
		// Generate reactions
		«generateReactions(reactor.reactions)»
		'''
	}
	
	/** Return an instantiate statement followed by any required parameter
	 *  assignments.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def generateComposite(Composite composite, Hashtable<String,String> importTable) {
		inputs.clear(); // Reset set of inputs.
		parameters.clear(); // Reset set of parameters.
		'''
		«boilerplate()»
		// Trigger data structure:
		«FOR clock: composite.clocks»
			«generateClock(clock)» 
		«ENDFOR»			
		«IF composite.preamble !== null»
		// *********** From the preamble, verbatim:
		«removeCodeDelimiter(composite.preamble.code)»
		// *********** End of preamble.
		«ENDIF»
		// Composite setup		
		«compositeSetup(composite, importTable)»
		// Composite initialize (initialize + triggers scheduling - missing inputHandlers)
		«generateConstructor(composite.constructor, composite.clocks, composite.reactions)»
		// Generate reactions
		«generateReactions(composite.reactions)»
		'''
	}
		
	def boilerplate() '''
		////////////////////////
		// Boilerplate included for all actors.
		function schedule(trigger) {
		    if (trigger.periodicity) {
		        return setInterval(trigger.reaction, trigger.period);
		    } else {
		    	return setTimeout(trigger.reaction, trigger.period);
		    }
		}
		// Unbound version of set() function (will be bound below).
		function setUnbound(port, value) {
		    if (!port) {
		        throw "Illegal reference to undeclared output.";
		    }
		    this.send(port, value);
		}
		// NOTE: bind() returns a new function.
		// It does not alter the original function.
		var set = setUnbound.bind(this);
		
		// Unbound version of get() function (will be bound below).
		function getUnbound(port, value) {
			if (!port) {
		    	throw "Illegal reference to undeclared input.";
		    }
			return this.get(port);
		}
		// NOTE: bind() returns a new function.
		// It does not alter the original function.
		var get = getUnbound.bind(this);
		//////////////////////// End boilerplate
		
	'''
	
	/** Return the setup function definition for a reactor.
	 */
	def reactorSetup(Reactor reactor, Hashtable<String,String> importTable) '''
		exports.setup = function () {
			// Generated Inputs, if any
			«FOR input: reactor.inputs»
				«generateInput(input)» 
			«ENDFOR»
			// Generated outputs, if any
			«FOR output: reactor.outputs»
				«generateOutput(output)» 
			«ENDFOR»
			// Generated parameters, if any
			«IF reactor.parameters !== null»
				«FOR param : reactor.parameters.params»
					«generateParameter(param)» 
				«ENDFOR»
			«ENDIF»
		}
	'''
		
	/** Return the setup function definition for a composite.
	 */
	def compositeSetup(Composite composite, Hashtable<String,String> importTable) '''
		exports.setup = function () {
			// Generated Inputs, if any
			«FOR input: composite.inputs»
				«generateInput(input)» 
			«ENDFOR»
			// Generated outputs, if any
			«FOR output: composite.outputs»
				«generateOutput(output)» 
			«ENDFOR»
			// Generated parameters, if any
			«IF composite.parameters !== null»
				«FOR param : composite.parameters.params»
					«generateParameter(param)» 
				«ENDFOR»
			«ENDIF»	
			// Generated instances
			«FOR instance: composite.instances»
				«instantiate(instance, importTable)»
			«ENDFOR»
			// Generated connections
			«FOR connection: composite.connections»
				this.connect(«portSpec(connection.leftPort)», «portSpec(connection.rightPort)»);
			«ENDFOR»
		}
	'''
		
	def generateInput(Input input) {
		inputs.add(input.name);
		'''
		this.input("«input.name»"«IF input.type !== null», { 'type': '«removeCodeDelimiter(input.type)»'}«ENDIF»);
		'''
	}
		
	def generateOutput(Output output)
		'''
		this.output("«output.name»"«IF output.type !== null», { 'type': '«removeCodeDelimiter(output.type)»'}«ENDIF»);
		'''
	
	def generateParameter(Param param) {
		parameters.add(param.name);
		'''
		this.parameter("«param.name»"«IF param.type === null && param.value === null»);«ELSE»,«ENDIF»
			«IF param.type !== null && param.value !== null»{'type': '«removeCodeDelimiter(param.type)»', 'value': «removeCodeDelimiter(param.value)»});«ENDIF»
			«IF param.type !== null && param.value === null»{'type': '«removeCodeDelimiter(param.type)»'});«ENDIF»
			«IF param.type === null && param.value !== null»{'value': «removeCodeDelimiter(param.value)»});«ENDIF»
		'''
	}

	def generateClock(Clock clock)
		'''
		var «clock.name» = {'reactor': this,
		    'period': «IF clock.period !== null»«clock.period.period»«ELSE»0«ENDIF»,
		    'periodicity': «IF clock.period !== null»«IF clock.period.period != "" »1«ELSE»0«ENDIF»«ELSE»0«ENDIF», /* TODO: check this condition, probably wrong, but I changed it so it would compile at all*/ 
		    'reaction': reaction_«clock.name».bind(this)
		};
		'''
	
	/** Return the constructor (constructor) function definition for a reactor or a composite.
	 *  First of all, schedule clocks.
	 *  Second, add input handlers.
	 *  Since a reaction can be triggered by any of the triggers, it is an "OR" condition.
	 *  FIXME: Is the sentence above this correct?
	 *  If yes, several input handlers are added, as described below.
	 */
	def generateConstructor(Constructor constructor, EList<Clock> triggers, EList<Reaction> reactions) '''
		exports.initialize = function () {
			«IF constructor !== null»«removeCodeDelimiter(constructor.code)»«ENDIF»
			«FOR trigger: triggers»
				schedule(«trigger.name»);
			«ENDFOR»
			
			// Adding input handlers of reactions
			«FOR reaction: reactions»
				«FOR trigger:reaction.triggers»
					«IF inputs.contains(trigger)»
						this.addInputHandler("«trigger»", reaction«FOR trig:reaction.triggers»_«trig»«ENDFOR».bind(this));
					«ENDIF»
				«ENDFOR»
			«ENDFOR»
		};
	'''			

	/** Return reaction functions definition for a reactor or a composite.
	 */
	def generateReactions(EList<Reaction> reactions) '''
		«FOR reaction: reactions»
		function reaction«FOR trigger:reaction.triggers»_«trigger»«ENDFOR» () {
			// Define variables for triggers, gets (non-triggering inputs that are read), and sets (outputs).
			«IF reaction.triggers !== null»
				«FOR trigger: reaction.triggers»
					«IF inputs.contains(trigger)»
						var «trigger» = "«trigger»";
					«ENDIF»
				«ENDFOR»
			«ENDIF»
			// Define variables for non-triggering inputs.
			«IF reaction.gets !== null»
				«FOR get: reaction.gets.gets»
					var «get» = "«get»";
				«ENDFOR»
			«ENDIF»
			// Define variables for output ports written to.
			«IF reaction.sets !== null»
				«FOR set: reaction.sets.sets»
					var «set» = "«set»";
				«ENDFOR»
			«ENDIF»
			// Define variables for each parameter.
		«FOR parameter: parameters»
			var «parameter» = this.getParameter("«parameter»");
		«ENDFOR»

			// Code verbatim from 'reaction'
			«removeCodeDelimiter(reaction.code)»
		}
		
		«ENDFOR»
	'''		
		
	/** Return an instantiate statement followed by any required parameter
	 *  assignments.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def instantiate(Instance instance, Hashtable<String,String> importTable) {
		var className = importTable.get(instance.actorClass);
		if (className === null) {
			className = instance.actorClass
		}
		'''
		var «instance.name» = this.instantiate('«instance.name»', '«className»');
		«IF instance.parameters !== null»
			«FOR param: instance.parameters.assignments»
				«instance.name».setParameter('«param.name»', «removeCodeDelimiter(param.value)»);
			«ENDFOR»
		«ENDIF»
		'''
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
	
	
	/** If the argument starts with '{=', then remove it and the last two characters.
	 *  @return The body without the code delimiter or the unmodified argument if it
	 *   is not delimited.
	 */
	def String removeCodeDelimiter(String code) {
		if (code.startsWith("{=")) {
            code.substring(2, code.length - 2).trim();
        } else {
        	code
        }
	}
}
