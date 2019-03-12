/*
 * Generator for CapeCode.
 */
package org.icyphy.generator

import java.util.Hashtable
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Composite

/**
 * Generator for CapeCode
 * @author Edward A. Lee
 */
class CapeCodeGenerator {
	def void doGenerate(
			Resource resource, 
			IFileSystemAccess2 fsa, 
			IGeneratorContext context,
			Hashtable importTable) {
		// Next handle the composites.
		for (composite : resource.allContents.toIterable.filter(Composite)) {
			val code = new StringBuffer
			code.append("exports.setup = function () {\n")
			for (instance: composite.instances) {
				var imported = importTable.get(instance.actorClass.name);
				if (imported === null) {
					imported = instance.actorClass.name
				}
				code.append("    var " 
					+ instance.name 
					+ " = this.instantiate('"
					+ instance.name
					+ "', '"
					+ imported
					+ "');\n"
					)
			}
			code.append("}\n")
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
}
