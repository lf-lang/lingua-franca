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

		super.doGenerate(resource, fsa, context, importTable)

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

		val srcPath = srcFile.substring(0, srcFile.lastIndexOf(File.separator))
		var genPath = srcPath + File.separator + "src-gen" + File.separator + _filename
		var binPath = srcPath + File.separator + "bin" + File.separator + _filename

		pr('''
			#include <iostream>
			
			int main() {
				std::cout << "Hello World!" << std::endl;
				return 0;
			}
		''')

		// Create output directories if they do not yet exist
		var genDir = new File(genPath)
		if(!genDir.exists()) genDir.mkdirs()
		var binDir = new File(binPath)
		if(!binDir.exists()) binDir.mkdirs()

		// Write the main source file
		var fOut = new FileOutputStream(new File(genPath + File.separator + "main.cc"));
		fOut.write(getCode().getBytes())

		writeCmake(genPath)

		doCompile(genDir, binDir)
	}

	def void writeCmake(String genPath) {
		var cmake = new StringBuilder()

		pr(cmake, '''
			cmake_minimum_required(VERSION 3.5)
			project(«_filename» VERSION 1.0.0 LANGUAGES CXX)
			
			add_executable(«_filename» main.cc)
		''')

		var fOut = new FileOutputStream(new File(genPath + File.separator + "CMakeLists.txt"));
		fOut.write(cmake.toString().getBytes())
	}

	def void doCompile(File genDir, File binDir) {
		var makeCmd = newArrayList()
		var cmakeCmd = newArrayList()

		makeCmd.addAll("make")
		cmakeCmd.addAll("cmake", genDir.getAbsolutePath())

		println("--- Running cmake:")
		var cmakeBuilder = new ProcessBuilder(cmakeCmd)
		cmakeBuilder.directory(binDir)
		var cmakeProcess = cmakeBuilder.inheritIO().start()
		cmakeProcess.waitFor()

		if (cmakeProcess.exitValue() == 0) {
			println("--- Running make:")
			var makeBuilder = new ProcessBuilder(makeCmd)
			makeBuilder.directory(binDir)
			var makeProcess = makeBuilder.inheritIO().start()
			makeProcess.waitFor()

			if (makeProcess.exitValue() == 0) {
				println("SUCCESS (compiling generated C++ code)")
			} else {
				println("ERRROR (while compiling generated C++ code)")
			}
		} else {
			println("ERRROR (while executing cmake)")
		}
	}
}
