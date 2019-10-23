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
import org.icyphy.linguaFranca.Reactor
import java.text.SimpleDateFormat
import java.util.Date
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.State

class CppGenerator extends GeneratorBase {
	static public var timeUnitsToDearUnits = #{'nsec' -> '_ns', 'usec' -> '_us', 'msec' -> '_ms', 'sec' -> '_s',
		'secs' -> '_s', 'minute' -> '_min', 'minutes' -> '_min', 'hour' -> '_h', 'hours' -> '_h', 'day' -> '_d',
		'days' -> '_d', 'week' -> '_weeks', 'weeks' -> '_weeks'}

	private def void processImports(Resource resource, IFileSystemAccess2 fsa) {
		for (import : resource.allContents.toIterable.filter(Import)) {
			val importResource = openImport(resource, import)
			if (importResource !== null) {
				// Make sure the target of the import is C++.
				var targetOK = false
				for (target : importResource.allContents.toIterable.filter(Target)) {
					if ("Cpp".equalsIgnoreCase(target.name)) {
						targetOK = true
					}
				}
				if (!targetOK) {
					reportError(import, "Import does not have a Cpp target.")
				} else {
					// Process any imports that the import has.
					importResource.processImports(fsa);
					for (r : importResource.allContents.toIterable.filter(Reactor)) {
						fsa.generateFile(_filename + File.separator + r.getName() + ".hh", r.generateReactorHeader)
						fsa.generateFile(_filename + File.separator + r.getName() + ".cc", r.generateReactorSource)
					}
				}
			} else {
				reportError(import, "Unable to open import")
			}
		}
	}

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

		fsa.generateFile(_filename + File.separator + "fwd.hh", fwd_hh)
		fsa.generateFile(_filename + File.separator + "main.cc", main_cc)
		fsa.generateFile(_filename + File.separator + "CMakeLists.txt", cmake)

		for (r : _resource.allContents.toIterable.filter(Reactor)) {
			fsa.generateFile(_filename + File.separator + r.getName() + ".hh", r.generateReactorHeader)
			fsa.generateFile(_filename + File.separator + r.getName() + ".cc", r.generateReactorSource)
		}

		// generate code for all imports
		resource.processImports(fsa)

		doCompile()
	}

	def generate(Time t) '''«t.time»«timeUnitsToDearUnits.get(t.unit)»'''

	def name(Reaction n) {
		var r = n.eContainer as Reactor
		'r' + r.reactions.lastIndexOf(n)
	}

	def priority(Reaction n) {
		var r = n.eContainer as Reactor
		r.reactions.lastIndexOf(n)
	}

	def findMainReactor() {
		var main = null as Reactor
		for (r : _resource.allContents.toIterable.filter(Reactor)) {
			if (r.isMain) {
				if (main !== null) {
					throw new RuntimeException("There is more than one main reactor!")
				}
				main = r
			}
		}
		if (main === null) {
			throw new RuntimeException("No main reactor found!")
		}
		main
	}

	def instantiate(Timer t) {
		if (t.timing !== null) {
			'''dear::Timer «t.name»{"«t.name»", this, «t.timing.period.generate», «t.timing.offset.generate»};'''
		} else {
			'''dear::Timer «t.name»{"«t.name»", this};'''
		}
	}

	def instantiate(Instance i) '''«i.reactorClass.name» «i.name»{"«i.name»", this};'''

	def instantiate(
		Reaction n) '''dear::Reaction «n.name»{"«n.name»", «n.priority», this, [this]() { «n.name»_body(); }};'''

	def instantiate(State s) {
		if (s.type !== null) {
			if (s.value !== null) {
				'''«s.type» «s.name»{«s.value»};'''
			} else {
				'''«s.type» «s.name»{};'''
			}
		} else {
			'''// «reportError(s, "State variable has no type.")»'''
		}
	}

	def instantiateState(Reactor r) '''
		«FOR s : r.states BEFORE '// state variables\n' AFTER '\n'»
			«s.instantiate»
		«ENDFOR»
	'''

	def instantiateInstances(Reactor r) '''
		«FOR i : r.instances BEFORE '// reactor instances\n' AFTER '\n'»
			«i.instantiate»
		«ENDFOR»
	'''

	def instantiateTimers(Reactor r) '''
		«FOR t : r.timers BEFORE '// timers\n' AFTER '\n'»
			«t.instantiate»
		«ENDFOR»
	'''

	def instantiateReactions(Reactor r) '''
		«FOR n : r.reactions BEFORE '// reactions\n' AFTER '\n'»
			«n.instantiate»
		«ENDFOR»
	'''

	def declareReactionBodies(Reactor r) '''
		«FOR n : r.reactions BEFORE '// reactions bodies\n' AFTER '\n'»
			void «n.name»_body();
		«ENDFOR»
	'''

	def implementReactionBodies(Reactor r) '''
		«FOR n : r.reactions SEPARATOR '\n'»
			void «r.name»::«n.name»_body() {
			  «removeCodeDelimiter(n.code)»
			}
		«ENDFOR»
	'''

	def includeInstances(Reactor r) '''
		«FOR i : r.instances AFTER '\n'»
			#include "«i.reactorClass.name».hh"
		«ENDFOR»
	'''

	def generatePreamble(Reactor r) '''
		«IF r.preamble !== null»
			// preamble
			«removeCodeDelimiter(r.preamble.code)»
			
		«ENDIF»
	'''

	def declareTriggers(Reaction n) '''
		«FOR t : n.triggers»
			«IF t.instance !== null»
				«n.name».declare_trigger(&«t.instance.name».«t.variable.name»);
			«ELSE»
				«n.name».declare_trigger(&«t.variable.name»);
			«ENDIF»
		«ENDFOR»
	'''

	def declareDependencies(Reaction n) '''
		«FOR t : n.sources»
			«IF t.instance !== null»
				«n.name».declare_dependency(&«t.instance.name».«t.variable.name»);
			«ELSE»
				«n.name».declare_dependency(&«t.variable.name»);
			«ENDIF»
		«ENDFOR»
	'''

	def declareAntidependencies(Reaction n) '''
		«FOR t : n.effects»
			«IF t.instance !== null»
				«n.name».declare_antidependency(&«t.instance.name».«t.variable.name»);
			«ELSE»
				«n.name».declare_antidependency(&«t.variable.name»);
			«ENDIF»
		«ENDFOR»
	'''

	def assembleReaction(Reactor r, Reaction n) '''
		// «n.name»
		«n.declareTriggers»
		«n.declareDependencies»
		«n.declareAntidependencies»
	'''

	def generateReactorHeader(Reactor r) '''
		«header()»
		
		#pragma once
		
		#include "dear/dear.hh"
		
		«r.includeInstances»
		«r.generatePreamble»
		using namespace dear::literals;
		
		class «r.getName()» : public dear::Reactor {
		 private:
		  «r.instantiateState»
		  «r.instantiateInstances»
		  «r.instantiateTimers»
		  «r.instantiateReactions»
		  «r.declareReactionBodies»
		 public:
		  «IF r.isMain()»
		  	«r.getName()»(const std::string& name, dear::Environment* environment);
		  «ELSE»
		  	«r.getName()»(const std::string& name, dear::Reactor* container);
		  «ENDIF»
		  
		  void assemble() override;
		};
	'''

	def generateReactorSource(Reactor r) '''
		«header()»
		
		#include "«r.getName».hh"
		
		«IF r.isMain()»
			«r.getName()»::«r.getName()»(const std::string& name, dear::Environment* environment)
			  : dear::Reactor(name, environment) {}
		«ELSE»
			«r.getName()»::«r.getName()»(const std::string& name, dear::Reactor* container)
			  : dear::Reactor(name, container) {}
		«ENDIF»
		
		void «r.name»::assemble() {
		  «FOR n : r.reactions SEPARATOR '\n'»
		  	«r.assembleReaction(n)»
		  «ENDFOR»
		}
		
		«r.implementReactionBodies»
	'''

	def header() '''
		/*
		 * This file was autogenerated by the Lingua Franca Compiler
		 *
		 * Source: «_resource.getURI()»
		 * Date: «new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date())»
		 */
	'''

	def fwd_hh() '''
		«header()»
		
		#pragma once
		
		«FOR r : _resource.allContents.toIterable.filter(Reactor)»
			class «r.getName()»;
		«ENDFOR»
	'''

	def main_cc() {
		var main = findMainReactor()
		'''
			«header()»
			
			#include "dear/dear.hh"
			
			#include "«main.name».hh"
			
			int main() {
			  dear::Environment e{4};
			
			  «main.name» main{"main", &e};
			  e.assemble();
			  e.init();
			
			  auto t = e.start();
			  t.join();
			
			  return 0;
			}
		'''
	}

	def cmake() '''
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
		
		add_executable(«_filename»
		  main.cc
		  «FOR r : _resource.allContents.toIterable.filter(Reactor)»
		  	«r.getName()».cc
		  «ENDFOR»
		)
		target_include_directories(«_filename» PUBLIC ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
		target_link_libraries(«_filename» dear)
		
		install(TARGETS «_filename»)
	'''

	def void doCompile() {
		var makeCmd = newArrayList()
		var cmakeCmd = newArrayList()

		var cwd = Paths.get("").toAbsolutePath().toString()
		var srcPath = cwd + File.separator + "src-gen" + File.separator + _filename
		var buildPath = cwd + File.separator + "build" + File.separator + _filename
		var dearPath = cwd + File.separator + "build" + File.separator + "dear"

		makeCmd.addAll("make", "install")
		cmakeCmd.addAll("cmake", "-DCMAKE_INSTALL_PREFIX=" + cwd, "-DDEAR_BUILD_DIR=" + dearPath, srcPath)

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
