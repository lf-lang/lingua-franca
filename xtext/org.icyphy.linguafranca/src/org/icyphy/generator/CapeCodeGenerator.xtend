/*
 * Generator for CapeCode.
 */
package org.icyphy.generator

import java.util.Hashtable
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Composite
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Actor
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Trigger
import org.icyphy.linguaFranca.Initialize
import org.eclipse.emf.common.util.EList
import org.icyphy.linguaFranca.Reaction

/**
 * Generator for CapeCode
 * @author Edward A. Lee
 */
class CapeCodeGenerator {
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable<String,String> importTable) {
		
		// Handle actors.
		for (actor : resource.allContents.toIterable.filter(Actor)) {
			val code = new StringBuffer
			code.append(generateActor(actor, importTable));
			fsa.generateFile(actor.name + ".js", code);		
		}
		
		// Handle composites.
		for (composite : resource.allContents.toIterable.filter(Composite)) {
			val code = new StringBuffer
			code.append(generateComposite(composite, importTable))
			// FIXME: More
			fsa.generateFile(composite.name + ".js", code);		
		}
	}
	
	/** Return an instantiate statement followed by any required parameter
	 *  assignments.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */	
	def generateActor(Actor actor, Hashtable<String,String> importTable)
		'''
		«boilerplate()»
		// Trigger data structure:
		«FOR trigger: actor.triggers»
			«generateTrigger(trigger)» 
		«ENDFOR»
		«IF actor.preamble !== null»
		// *********** From the preamble, verbatim:
		«removeCodeDelimiter(actor.preamble.code)»
		// *********** End of preamble.
		«ENDIF»
		// Actor setup (inputs, outputs, parameters)
		«actorSetup(actor, importTable)»
		// Actor initialize (initialize + triggers scheduling - missing inputHandlers)
		«generateInitialize(actor.initialize, actor.triggers, actor.reactions)»
		// Generate reactions
		«generateReactions(actor.reactions)»
		'''
	
	/** Return an instantiate statement followed by any required parameter
	 *  assignments.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def generateComposite(Composite composite, Hashtable<String,String> importTable)
		'''
		«boilerplate()»
		// Trigger data structure:
		«FOR trigger: composite.triggers»
			«generateTrigger(trigger)» 
		«ENDFOR»			
		«IF composite.preamble !== null»
		// *********** From the preamble, verbatim:
		«removeCodeDelimiter(composite.preamble.code)»
		// *********** End of preamble.
		«ENDIF»
		// Composite setup		
		«compositeSetup(composite, importTable)»
		// Composite initialize (initialize + triggers scheduling - missing inputHandlers)
		«generateInitialize(composite.initialize, composite.triggers, composite.reactions)»
		// Generate reactions
		«generateReactions(composite.reactions)»
		'''
		
	def boilerplate() '''
		// Boilerplate included for all actors.
		function schedule(trigger) {
		    if (trigger.periodicity) {
		        return trigger.actor.setInterval(trigger.reaction, trigger.period);
		    } else {
		    	return trigger.actor.setTimeout(trigger.reaction, trigger.period);
		    }
		}
		function setUnbound(port, value) {
		    if (!port) {
		        throw \"Illegal reference to undeclared output.\";
		    }
		    this.send(port, value);
		}
		// NOTE: bind() returns a new function.
		// It does not alter the original function.
		var set = setUnbound.bind(this);
	'''
	
	/** Return the setup function definition for an actor.
	 */
	def actorSetup(Actor actor, Hashtable<String,String> importTable) '''
		exports.setup = function () {
			// Generated Inputs, if any
			«FOR input: actor.inputs»
				«generateInput(input)» 
			«ENDFOR»
			// Generated outputs, if any
			«FOR output: actor.outputs»
				«generateOutput(output)» 
			«ENDFOR»
			// Generated parameters, if any
			«IF actor.parameters !== null»
				«FOR param : actor.parameters.params»
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
		
	def generateInput(Input input) 
		'''
		this.intput("«input.name»"«IF input.type !== null», { 'type': «removeCodeDelimiter(input.type)»}«ENDIF»);
		'''
		
	def generateOutput(Output output)
		'''
		this.output("«output.name»"«IF output.type !== null», { 'type': «removeCodeDelimiter(output.type)»}«ENDIF»);
		'''
	
	def generateParameter(Param param)
		'''
		this.parameter("«param.name»"«IF param.type === null && param.value === null»);«ELSE»,«ENDIF»
			«IF param.type !== null && param.value !== null»{'type': «removeCodeDelimiter(param.type)», 'value': «removeCodeDelimiter(param.value)»});«ENDIF»
			«IF param.type !== null && param.value === null»{'type': «removeCodeDelimiter(param.type)»});«ENDIF»
			«IF param.type === null && param.value !== null»{'value': «removeCodeDelimiter(param.value)»});«ENDIF»
		'''

	def generateTrigger(Trigger trigger)
		'''
		var «trigger.name» = {'actor': this,
		    'period': «IF trigger.period !== null»«trigger.period.period»«ELSE»0«ENDIF»,
		    'periodicity': «IF trigger.period !== null»«IF trigger.period.periodic»1«ELSE»0«ENDIF»«ENDIF»,
		    'reaction': reaction_«trigger.name».bind(this)
		};
		'''
	
	/** Return the initialize function definition for an actor or a composite.
	 *  FIXME: See comment below for adding the input handler of reactions
	 */
	def generateInitialize(Initialize initialize, EList<Trigger> triggers, EList<Reaction> reactions) '''
		exports.initialize = function () {
			«IF initialize !== null»«removeCodeDelimiter(initialize.code)»«ENDIF»
			«FOR trigger: triggers»
				schedule(«trigger.name»);
			«ENDFOR»
			
			// Adding input handlers of reactions
			«FOR reaction: reactions»
			this.addInputHandler(«FOR trigger:reaction.triggers»"«trigger»", «ENDFOR»
				reaction«FOR trigger:reaction.triggers»_«trigger»«ENDFOR»
			);
			
			«ENDFOR»
		};
	'''			

	/** Return reaction functions definition for an actor or a composite.
	 *  FIXME 1: In what sens reaction parameters (triggers) are different from gets?
	 *  FIXME 2: Aren't 'set' instructions part of the code? If yes, Why do
	 *           we need 'Sets' in the header of the 'reaction'? 
	 *  FIXME 3: If the trigger of the reaction is a trigger (not an input port), then
	 *           we use schedule.
	 * 			 We need then to iterate over a reaction parameters to check this condition. 
	 */
	def generateReactions(EList<Reaction> reactions) '''
		«FOR reaction: reactions»
		function reaction«FOR trigger:reaction.triggers»_«trigger»«ENDFOR» () {
			// Reading gets
			«IF reaction.gets !== null»
			«FOR get: reaction.gets.gets»
				var «get» = this.getInput("«get»");
			«ENDFOR»
			«ENDIF»
			
			// Code verbatim from 'reaction'
			«removeCodeDelimiter(reaction.code)»
			
			// Setting outputs
			«IF reaction.sets !== null»
			«FOR set: reaction.sets.sets»
				set("«set»", "Don't know what value to put here?");
			«ENDFOR»
			«ENDIF»
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
		var «instance.name» = instantiate('«instance.name»', «className»);
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
