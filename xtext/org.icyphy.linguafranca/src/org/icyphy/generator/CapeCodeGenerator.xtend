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
		// Handle composites.
		for (composite : resource.allContents.toIterable.filter(Composite)) {
			val code = new StringBuffer
			code.append(compositeSetup(composite, importTable))
			// FIXME: More
			fsa.generateFile(composite.name + ".js", code);		
		}
	}
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
	
	/** Return the setup function definition for a composite.
	 */
	def compositeSetup(Composite composite, Hashtable<String,String> importTable) '''
		exports.setup = function () {
			«FOR instance: composite.instances»
				«instantiate(instance, importTable)»
			«ENDFOR»
			«FOR connection: composite.connections»
				this.connect(«portSpec(connection.leftPort)», «portSpec(connection.rightPort)»);
			«ENDFOR»
		}
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
