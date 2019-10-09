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

		// Write the main source file
		var fOut = new FileOutputStream(new File(genPath + File.separator + "main.cc"));
		fOut.write(getCode().getBytes())

		writeCmake(genPath)

		doCompile(srcPath, genPath)
	}

	def void writeCmake(String genPath) {
		var cmake = new StringBuilder()

		pr(cmake, '''
			cmake_minimum_required(VERSION 3.5)
			project(«_filename» VERSION 1.0.0 LANGUAGES CXX)

			include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
			include(GNUInstallDirs)

			set(DEFAULT_BUILD_TYPE "Release")
			if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
			  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
			  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
			  # Set the possible values of build type for cmake-gui
			  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
			endif()

			if(NOT DEAR_BUILD_DIR)
			  set(DEAR_BUILD_DIR "" CACHE STRING "Choose the directory to build dear in." FORCE)
			endif()

			ExternalProject_Add(
			  dep-dear
			  PREFIX "${DEAR_BUILD_DIR}"
			  GIT_REPOSITORY "git@github.com:cmnrd/dear.git"
			  CMAKE_ARGS
			    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
			    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
			)

			set(DEAR_LIB_NAME "${CMAKE_SHARED_LIBRARY_PREFIX}dear${CMAKE_SHARED_LIBRARY_SUFFIX}")
			set(DEAR_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

			add_library(dear SHARED IMPORTED)
			add_dependencies(dear dep-dear)
			set_target_properties(dear PROPERTIES IMPORTED_LOCATION "${DEAR_LIB_DIR}/${DEAR_LIB_NAME}")

			set(CMAKE_INSTALL_RPATH "${DEAR_LIB_DIR}")
			set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

			add_executable(«_filename» main.cc)
			target_link_libraries(«_filename» dear)

			install(TARGETS «_filename»)
		''')

		var fOut = new FileOutputStream(new File(genPath + File.separator + "CMakeLists.txt"));
		fOut.write(cmake.toString().getBytes())
	}

	def void doCompile(String srcPath, String genPath) {
		var makeCmd = newArrayList()
		var cmakeCmd = newArrayList()

		var buildPath = srcPath + File.separator + "build" + File.separator + _filename
		var dearPath = srcPath + File.separator + "build" + File.separator + "dear"

		makeCmd.addAll("make", "install")
		cmakeCmd.addAll("cmake", "-DCMAKE_INSTALL_PREFIX=" + srcPath, "-DDEAR_BUILD_DIR=" + dearPath, genPath)

		var buildDir = new File(buildPath)
		if(!buildDir.exists()) buildDir.mkdirs()

		println("--- Running cmake:")
		var cmakeBuilder = new ProcessBuilder(cmakeCmd)
		cmakeBuilder.directory(buildDir)
		var cmakeProcess = cmakeBuilder.inheritIO().start()
		cmakeProcess.waitFor()

		if (cmakeProcess.exitValue() == 0) {
			println("--- Running make:")
			var makeBuilder = new ProcessBuilder(makeCmd)
			makeBuilder.directory(buildDir)
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
