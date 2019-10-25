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
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import java.util.List
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Param

class CppGenerator extends GeneratorBase {
	static public var timeUnitsToDearUnits = #{'nsec' -> '_ns', 'usec' -> '_us', 'msec' -> '_ms', 'sec' -> '_s',
		'secs' -> '_s', 'minute' -> '_min', 'minutes' -> '_min', 'hour' -> '_h', 'hours' -> '_h', 'day' -> '_d',
		'days' -> '_d', 'week' -> '_weeks', 'weeks' -> '_weeks'}

	private def validateTarget(Resource resource) {
		var targetOK = false
		for (target : resource.allContents.toIterable.filter(Target)) {
			if ("Cpp".equalsIgnoreCase(target.name)) {
				targetOK = true
			}
		}
		targetOK
	}

	private def collectReactors(Resource resource) {
		val List<Reactor> reactors = newArrayList
		resource.collectReactors(reactors)
	}

	private def List<Reactor> collectReactors(Resource resource, List<Reactor> reactors) {
		reactors.addAll(resource.allContents.toIterable.filter(Reactor))

		for (import : resource.allContents.toIterable.filter(Import)) {
			val importResource = openImport(resource, import)
			if (importResource !== null) {
				var ok = importResource.validateTarget
				if (ok) {
					importResource.collectReactors(reactors)
				} else {
					reportError(import, "Import does not have a Cpp target.")
				}
			} else {
				reportError(import, "Unable to open import")
			}
		}
		reactors
	}

	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context,
		Hashtable<String, String> importTable) {

		super.doGenerate(resource, fsa, context, importTable)

		var reactors = resource.collectReactors
		var mainReactor = resource.findMainReactor

		fsa.generateFile(_filename + File.separator + "fwd.hh", reactors.generateForwardDeclarations)
		fsa.generateFile(_filename + File.separator + "main.cc", mainReactor.generateMain)
		fsa.generateFile(_filename + File.separator + "CMakeLists.txt", reactors.generateCmake)

		for (r : reactors) {
			fsa.generateFile(_filename + File.separator + r.getName() + ".hh", r.generateReactorHeader)
			fsa.generateFile(_filename + File.separator + r.getName() + ".cc", r.generateReactorSource)
		}

		doCompile()
	}

	override removeCodeDelimiter(String code) {
		if (code === null) {
			""
		} else if (code.startsWith("{=")) {
			if (code.split('\n').length > 1) {
				code.substring(2, code.length - 2).trimCodeBlock
			} else {
				code.substring(2, code.length - 2).trim
			}
		} else {
			if (code.split('\n').length > 1) {
				code.trimCodeBlock
			} else {
				code.trim
			}
		}
	}

	def trimCodeBlock(String code) {
		var codeLines = code.split("\n")
		var String prefix = null
		var buffer = new StringBuilder()
		for (line : codeLines) {
			if (prefix === null) {
				// skip any lines that only contain whitespaces
				if (line.trim.length > 0) {
					val characters = line.toCharArray()
					var foundFirstCharacter = false
					var int firstCharacter = 0
					for (var i = 0; i < characters.length(); i++) {
						if (!foundFirstCharacter && !Character.isWhitespace(characters.get(i))) {
							foundFirstCharacter = true
							firstCharacter = i
						}
					}
					prefix = line.substring(0, firstCharacter)
				}
			}

			if (prefix !== null) {
				if (line.startsWith(prefix)) {
					buffer.append(line.substring(prefix.length))
					buffer.append('\n')
				} else {
					buffer.append(line)
					buffer.append('\n')
				}
			}
		}
		buffer.toString
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

	def findMainReactor(Resource resource) {
		var main = null as Reactor
		for (r : resource.allContents.toIterable.filter(Reactor)) {
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

	def instantiate(Instance i) {
		if (i.parameters !== null && i.parameters.assignments.length > 0) {
			var List<String> values = newArrayList
			for (p : i.reactorClass.parameters) {
				var String value = null
				for (a : i.parameters.assignments) {
					if (a.name == p.name) {
						value = '''«a.value»'''
					}
				}
				if (value === null) {
					value = '''«p.value»'''
				}
				values.add(value)
			}
			'''«i.reactorClass.name» «i.name»{"«i.name»", this«FOR v : values», «v»«ENDFOR»};'''
		} else {
			// no assignments
			'''«i.reactorClass.name» «i.name»{"«i.name»", this};'''
		}
	}

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

	def declare(Param p) '''«p.trimmedType» «p.name»;'''

	def instantiate(Input i) {
		if (i.type !== null) {
			'''dear::Input<«i.type»> «i.name»{"«i.name»", this};'''
		} else {
			'''// «reportError(i, "Input port has no type.")»'''
		}
	}

	def instantiate(Output o) {
		if (o.type !== null) {
			'''dear::Output<«o.type»> «o.name»{"«o.name»", this};'''
		} else {
			'''// «reportError(o, "Output port has no type.")»'''
		}
	}

	def instantiateState(Reactor r) '''
		«FOR s : r.states BEFORE '// state variables\n' AFTER '\n'»
			«s.instantiate»
		«ENDFOR»
	'''

	def declareParameters(Reactor r) '''
		«FOR p : r.parameters BEFORE '// parameters\n' AFTER '\n'»
			«p.declare»
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

	def instantiatePorts(Reactor r) '''
		«FOR i : r.inputs BEFORE '// input ports\n' AFTER '\n'»
			«i.instantiate»
		«ENDFOR»
		«FOR o : r.outputs BEFORE '// output ports\n' AFTER '\n'»
			«o.instantiate»
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

	def declareConstructor(Reactor r) '''
		«IF r.isMain()»
			«r.getName()»(const std::string& name, dear::Environment* environment);
		«ELSE»
			«r.getName()»(const std::string& name, dear::Reactor* container);
		«ENDIF»
	'''

	def trimmedType(Param p) {
		val const = p.const ? "const " : ""
		if (p.type !== null) {
			if (p.type == "time") {
				'''«const»dear::time_t'''
			} else {
				'''«const»«p.type.removeCodeDelimiter»'''
			}
		} else {
			'''/* «p.reportError("Parameter has no type")» */'''
		}
	}

	def trimmedValue(Param p) {
		if (p.value !== null) {
			p.value.removeCodeDelimiter
		} else {
			// if its not a value it must be a time
			p.time.generate
		}
	}

	def declareParameterizedConstructor(Reactor r) '''
		«r.getName()»(const std::string& name,
		    «IF r.isMain()»dear::Environment* environment,«ELSE»dear::Reactor* container,«ENDIF»
		    «FOR p : r.parameters SEPARATOR ",\n" AFTER ");"»«p.trimmedType» «p.name»«ENDFOR»
	'''

	def defineConstructor(Reactor r) {
		if (r.parameters.length > 0) {
			'''	
				«r.name»::«r.name»(const std::string& name, «IF r.main»dear::Environment* environment«ELSE»dear::Reactor* container«ENDIF»)
				  : «r.name»::«r.name»(name, «IF r.main»environment«ELSE»container«ENDIF», «FOR p : r.parameters SEPARATOR ", "»«p.trimmedValue»«ENDFOR») {}
			'''
		} else
			'''	
				«r.name»::«r.name»(const std::string& name, «IF r.main»dear::Environment* environment«ELSE»dear::Reactor* container«ENDIF»)
				  : dear::Reactor(name, «IF r.main»environment«ELSE»container«ENDIF») {}
			'''
	}

	def defineParameterizedConstructor(Reactor r) '''
		«r.getName()»::«r.getName()»(const std::string& name,
		    «IF r.isMain()»dear::Environment* environment«ELSE»dear::Reactor* container«ENDIF»,
		    «FOR p : r.parameters SEPARATOR ",\n" AFTER ")"»«p.trimmedType» «p.name»«ENDFOR»
		  : dear::Reactor(name, «IF r.isMain()»environment«ELSE»container«ENDIF»)
		  «FOR p : r.parameters SEPARATOR "\n"», «p.name»(«p.name»)«ENDFOR» {}
	'''

	def assembleReaction(Reactor r, Reaction n) '''
		// «n.name»
		«n.declareTriggers»
		«n.declareDependencies»
		«n.declareAntidependencies»
	'''

	def assembleConnection(Connection c) '''«c.leftPort».bind_to(&«c.rightPort»);'''

	def generateReactorHeader(Reactor r) '''
		«header()»
		
		#pragma once
		
		#include "dear/dear.hh"
		
		«r.includeInstances»
		«r.generatePreamble»
		using namespace dear::literals;
		
		class «r.getName()» : public dear::Reactor {
		 private:
		  «r.declareParameters»
		  «r.instantiateState»
		  «r.instantiateInstances»
		  «r.instantiateTimers»
		  «r.instantiateReactions»
		  «r.declareReactionBodies»
		 public:
		  «r.instantiatePorts»
		  «r.declareConstructor»
		  «IF r.parameters.length > 0»
		  	«r.declareParameterizedConstructor»
		  «ENDIF»
		  
		  void assemble() override;
		};
	'''

	def generateReactorSource(Reactor r) '''
		«header()»
		
		#include "«r.getName».hh"
		
		«r.defineConstructor»
		
		«IF r.parameters.length > 0»
			«r.defineParameterizedConstructor»
			
		«ENDIF»
		void «r.name»::assemble() {
		  «FOR n : r.reactions SEPARATOR '\n' AFTER '\n'»
		  	«r.assembleReaction(n)»
		  «ENDFOR»
		  «FOR c : r.connections BEFORE '// connections\n'»
		  	«c.assembleConnection»
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

	def generateForwardDeclarations(List<Reactor> reactors) '''
		«header()»
		
		#pragma once
		
		«FOR r : reactors»
			class «r.getName()»;
		«ENDFOR»
	'''

	def generateMain(Reactor main) '''
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

	def generateCmake(List<Reactor> reactors) '''
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
		  «FOR r : reactors»
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
