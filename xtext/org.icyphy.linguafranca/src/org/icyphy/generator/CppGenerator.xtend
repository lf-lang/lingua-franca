/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 * 
 * The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
 * See LICENSE.md file in the top repository directory.
 * 
 * Authors:
 *   Christian Menard
 */
 
 package org.icyphy.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import java.util.Hashtable
import java.io.File
import java.nio.file.Paths
import org.eclipse.core.runtime.FileLocator
import java.net.URL
import java.io.FileOutputStream

class CppGenerator extends GeneratorBase {
 	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context,
		Hashtable<String, String> importTable) {
		
        var srcFile = resource.getURI.toString;
		var mode = Mode.UNDEFINED;
		
		if (srcFile.startsWith("file:")) { // Called from command line
			srcFile = Paths.get(srcFile.substring(5)).normalize.toString
			mode = Mode.STANDALONE;
		} else if (srcFile.startsWith("platform:")) { // Called from Eclipse
			srcFile = FileLocator.toFileURL(new URL(srcFile)).toString
			srcFile = Paths.get(srcFile.substring(5)).normalize.toString
			mode = Mode.INTEGRATED;
		} else {
			System.err.println("ERROR: Source file protocol is not recognized: " + srcFile);
		}
		
		super.doGenerate(resource, fsa, context, importTable)

		pr('''
			#include <iostream>
			
			int main() {
				std::cout << "Hello World!" << std::endl;
				return 0;
			}
		''')
		
		// Create output directory if it does not yet exist
		val srcPath = srcFile.substring(0, srcFile.lastIndexOf(File.separator))
        var srcGenPath = srcPath + File.separator + "src-gen" + File.separator + _filename
		var gendir = new File(srcGenPath)
		if (!gendir.exists()) gendir.mkdirs()
		
		// Write the main source file
		var fOut = new FileOutputStream(new File(srcGenPath + File.separator + "main.cc"));
		fOut.write(getCode().getBytes())
	}
 }