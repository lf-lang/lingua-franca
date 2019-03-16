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
		«actorSetup(actor, importTable)»
		'''
	
	/** Return an instantiate statement followed by any required parameter
	 *  assignments.
	 *  @param instance The instance declaration.
	 *  @param importTable Substitution table for class names (from import statements).
	 */
	def generateComposite(Composite composite, Hashtable<String,String> importTable)
		'''
		«compositeSetup(composite, importTable)»
		'''
	
		
	def boilerplate() '''
		// Boilerplate included for all actors.
		var PERIODIC = true;
		var ONCE = false;
		function schedule(trigger, time, isPeriodic) {
		    if (isPeriodic) {
		        return trigger.actor.setInterval(trigger.reaction, time);
		    } else {
		        return trigger.actor.setTimeout(trigger.reaction, time);
		    }
		}
		function setUnbound(port, value) {
		    if (!port) {
		        throw \"Illegal reference to undeclared output.\";
		    }
		    this.send(port, value);
		}
		var set = setUnbound.bind(this);
	'''
	
	/** Return the setup function definition for an actor.
	 */
	def actorSetup(Actor actor, Hashtable<String,String> importTable) '''
		exports.setup = function () {
			«FOR input: actor.inputs»
				«generateInput(input)» 
			«ENDFOR»
			«FOR output: actor.outputs»
				«generateOutput(output)» 
			«ENDFOR»
		}
	'''
	
	/** Return the setup function definition for a composite.
	 */
	def compositeSetup(Composite composite, Hashtable<String,String> importTable) '''
		exports.setup = function () {
			«FOR input: composite.inputs»
				«generateInput(input)» 
			«ENDFOR»
			«FOR output: composite.outputs»
				«generateOutput(output)» 
			«ENDFOR»
			«FOR instance: composite.instances»
				«instantiate(instance, importTable)»
			«ENDFOR»
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
