/*
 * generated by Xtext 2.17.0
 */
package org.icyphy.generator

import java.util.Hashtable
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.AbstractGenerator
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Target

/**
 * Generates code from your model files on save.
 * 
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
class LinguaFrancaGenerator extends AbstractGenerator {
	val importTable = new Hashtable<String,String>

	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
		// First collect all the imports.
		for (import : resource.allContents.toIterable.filter(Import)) {
			val pieces = import.name.split("\\.")
    		val root = pieces.last
    		val filename = pieces.join("/") + ".js"
			importTable.put(root, filename)
		}
		// Determine which target is desired.
		var targetFound = false;
		for (target : resource.allContents.toIterable.filter(Target)) {
			// FIXME: Use reflection here?
			if (target.name.equalsIgnoreCase("Accessor")
					|| target.name.equalsIgnoreCase("Accessors")) {
				val generator = new AccessorGenerator()
				targetFound = true
				generator.doGenerate(resource, fsa, context, importTable)
			} else if (target.name.equalsIgnoreCase("C")) {
				targetFound = true
				val generator = new CGenerator()
				generator.doGenerate(resource, fsa, context, importTable)
			} else if (target.name.equalsIgnoreCase("SCL")) {
				targetFound = true
				val generator = new SCLGenerator()
				generator.doGenerate(resource, fsa, context, importTable)
			}
		}
		if (!targetFound) {
			System.err.println("Warning: No recognized target.")
		}
	}
}
