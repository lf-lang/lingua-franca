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

import java.io.File
import java.nio.file.Paths
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Hashtable
import java.util.List
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.State
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.VarRef

class CppGenerator extends GeneratorBase {

	static public var timeUnitsToEnactorUnits = #{
		TimeUnit.NSEC -> '_ns',
		TimeUnit.NSECS -> '_ns',
		TimeUnit.USEC -> '_us',
		TimeUnit.USECS -> '_us',
		TimeUnit.MSEC -> '_ms',
		TimeUnit.MSECS -> '_ms',
		TimeUnit.SEC -> '_s',
		TimeUnit.SECS -> '_s',
		TimeUnit.MIN -> '_min',
		TimeUnit.MINS -> '_min',
		TimeUnit.HOUR -> '_h',
		TimeUnit.HOURS -> '_h',
		TimeUnit.DAY -> '_d',
		TimeUnit.DAYS -> '_d',
		TimeUnit.WEEK -> '_weeks',
		TimeUnit.WEEKS -> '_weeks'
	}

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

		var reactors = resource.collectReactors
		var mainReactor = resource.findMainReactor
		var target = resource.findTarget

		super.doGenerate(resource, fsa, context, importTable)

		fsa.generateFile(filename + File.separator + "fwd.hh", reactors.generateForwardDeclarations)
		fsa.generateFile(filename + File.separator + "main.cc", mainReactor.generateMain(target))
		fsa.generateFile(filename + File.separator + "CMakeLists.txt", target.generateCmake(reactors))

		for (r : reactors) {
			fsa.generateFile(filename + File.separator + r.getName() + ".hh", r.generateReactorHeader)
			fsa.generateFile(filename + File.separator + r.getName() + ".cc", r.generateReactorSource)
		}

		doCompile()
	}

	def extractDir(String path) {
		var result = path
		if (path.startsWith('platform:')) {
			result = result.substring(9)
		}
		if (path.startsWith('file:')) {
			result = result.substring(5)
		}
		var lastSlash = result.lastIndexOf('/')
		if (lastSlash >= 0) {
			result = result.substring(0, lastSlash)
		}
		return result
	}

	static def removeCodeDelimiter(String code) {
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

	static def trimCodeBlock(String code) {
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
		buffer.deleteCharAt(buffer.length - 1) // remove the last newline 
		buffer.toString
	}

	def name(Reaction n) {
		var r = n.eContainer as Reactor
		'r' + r.reactions.lastIndexOf(n)
	}

	def priority(Reaction n) {
		var r = n.eContainer as Reactor
		r.reactions.lastIndexOf(n) + 1
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

	def findTarget(Resource resource) {
		var target = null as Target
		for (t : resource.allContents.toIterable.filter(Target)) {
			if (target !== null) {
				throw new RuntimeException("There is more than one target!")
			}
			target = t
		}
		if (target === null) {
			throw new RuntimeException("No target found!")
		}
		target
	}

	def getProperty(Target t, String s) {
		for (p : t.properties) {
			if (p.name == s) {
				return p.value
			}
		}
		null
	}

	def hasProperty(Target t, String s) {
		for (p : t.properties) {
			if (p.name == s) {
				return true
			}
		}
		false
	}

	def declare(Reaction n) '''
		enactor::Reaction «n.name»{"«n.name»", «n.priority», this, [this]() { «n.name»_body(); }};
	'''

	def declareStateVariables(Reactor r) '''
		«FOR s : r.states BEFORE '// state variables\n' AFTER '\n'»
			«s.trimmedType» «s.name»;
		«ENDFOR»
	'''

	def declareParameters(Reactor r) '''
		«FOR p : r.parameters BEFORE '// parameters\n' AFTER '\n'»
			«p.trimmedType» «p.name»;
		«ENDFOR»
	'''

	def declareInstances(Reactor r) '''
		«FOR i : r.instantiations BEFORE '// reactor instantiations\n' AFTER '\n'»
			«i.reactorClass.name» «i.name»;
		«ENDFOR»
	'''

	def declareTimers(Reactor r) '''
		«FOR t : r.timers BEFORE '// timers\n' AFTER '\n'»
			enactor::Timer «t.name»;
		«ENDFOR»
	'''

	def declareReactions(Reactor r) '''
		«FOR n : r.reactions BEFORE '// reactions\n' AFTER '\n'»
			«n.declare»
		«ENDFOR»
	'''

	def declarePorts(Reactor r) '''
		«FOR i : r.inputs BEFORE '// input ports\n' AFTER '\n'»
			enactor::Input<«i.trimmedType»> «i.name»{"«i.name»", this};
		«ENDFOR»
		«FOR o : r.outputs BEFORE '// output ports\n' AFTER '\n'»
			enactor::Output<«o.trimmedType»> «o.name»{"«o.name»", this};
		«ENDFOR»
	'''

	def declareActions(Reactor r) '''
		«FOR a : r.actions BEFORE '// actions\n' AFTER '\n'»
			«IF a.origin == ActionOrigin.LOGICAL»
				enactor::LogicalAction<«a.trimmedType»> «a.name»{"«a.name»", this};
			«ELSE»
				enactor::PhysicalAction<«a.trimmedType»> «a.name»{"«a.name»", this};
			«ENDIF»
		«ENDFOR»
	'''

	def declareReactionBodies(Reactor r) '''
		«FOR n : r.reactions BEFORE '// reactions bodies\n' AFTER '\n'»
			void «n.name»_body();
		«ENDFOR»
	'''

	def declareDeadlineHandlers(Reactor r) '''
		«FOR n : r.reactions.filter([Reaction x | x.deadline !== null]) BEFORE '// local deadline handlers\n' AFTER '\n'»
			void «n.name»_deadline_handler();
		«ENDFOR»
	'''

	def implementReactionBodies(Reactor r) '''
		«FOR n : r.reactions SEPARATOR '\n'»
			void «r.name»::«n.name»_body() {
			  «n.code.removeCodeDelimiter»
			}
		«ENDFOR»
	'''

	def implementReactionDeadlineHandlers(Reactor r) '''
		«FOR n : r.reactions.filter([Reaction x | x.deadline !== null]) BEFORE '\n' SEPARATOR '\n'»
			void «r.name»::«n.name»_deadline_handler() {
			  «n.deadline.deadlineCode.removeCodeDelimiter»
			}
		«ENDFOR»
	'''

	def includeInstances(Reactor r) '''
		«FOR i : r.instantiations AFTER '\n'»
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
			«n.name».declare_trigger(&«t.fullName»);
		«ENDFOR»
	'''

	def fullName(VarRef v) {
		if (v.container !== null) {
			'''«v.container.name».«v.variable.name»'''
		} else {
			'''«v.variable.name»'''
		}
	}

	def declareDependencies(Reaction n) '''
		«FOR t : n.sources»
			«IF t.container !== null»
				«n.name».declare_dependency(&«t.container.name».«t.variable.name»);
			«ELSE»
				«n.name».declare_dependency(&«t.variable.name»);
			«ENDIF»
		«ENDFOR»
	'''

	def declareAntidependencies(Reaction n) '''
		«FOR t : n.effects»
			«IF t.variable instanceof Action»
				«n.name».declare_scheduable_action(&«t.variable.name»);
			«ELSE»
				«IF t.container !== null»
					«n.name».declare_antidependency(&«t.container.name».«t.variable.name»);
				«ELSE»
					«n.name».declare_antidependency(&«t.variable.name»);
				«ENDIF»
			«ENDIF»
		«ENDFOR»
	'''

	def declareConstructor(Reactor r) {
		if (r.parameters.length > 0) {
			'''
				«r.name»(const std::string& name,
				    «IF r.main»enactor::Environment* environment,«ELSE»enactor::Reactor* container«ENDIF»,
				    «FOR p : r.parameters SEPARATOR ",\n" AFTER ");"»«p.trimmedType» «p.name» = «p.trimmedValue»«ENDFOR»
			'''
		} else {
			if (r.main) {
				'''«r.name»(const std::string& name, enactor::Environment* environment);'''
			} else {
				'''«r.name»(const std::string& name, enactor::Reactor* container);'''
			}
		}
	}

	def trimmedType(Parameter p) {
		val const = "const " // All parameters must be constants
		if (p.ofTimeType) {
			'''«const»enactor::time_t'''
		} else {
			if (p.type !== null) {

				'''«const»«p.type.removeCodeDelimiter»'''
			} else {
				'''/* «p.reportError("Parameter has no type")» */'''
			}
		}
	}

	def trimmedType(State s) {
		if (s.type !== null) {
			s.type.removeCodeDelimiter
		} else {
			'''/* «s.reportError("State variable has no type")» */'''
		}
	}

	def trimmedType(Input i) {
		if (i.type !== null) {
			i.type.removeCodeDelimiter
		} else {
			'''/* «i.reportError("Input port has no type.")» */'''
		}
	}

	def trimmedType(Output o) {
		if (o.type !== null) {
			o.type.removeCodeDelimiter
		} else {
			'''/* «o.reportError("Input port has no type.")» */'''
		}
	}

	def trimmedType(Action a) {
		if (a.type !== null) {
			a.type.removeCodeDelimiter
		} else {
			'''/* «a.reportError("Action has no type.")» */'''
		}
	}

	def trimmedValue(Parameter p) {
		if (p.value !== null) {
			'''«p.value.removeCodeDelimiter»'''
		} else {
			// if its not a value it must be a time
			'''«p.time»'''
		}
	}

	def trimmedValue(TimeOrValue tv) {
		if (tv.unit != TimeUnit.NONE) {
			'''«tv.time»«timeUnitsToEnactorUnits.get(tv.unit)»'''
		} else {
			// time refers to a parameter or is a number without a unit
			'''«tv.time»''' // FIXME: this is incorrect. 
		}
	}

	def trimmedValue(Assignment a) {
		if (a.rhs.unit == TimeUnit.NONE) {
			// assume we have a time
			'''«a.rhs.time»«timeUnitsToEnactorUnits.get(a.rhs.unit)»'''
		} else {
			'''«a.rhs.value.removeCodeDelimiter»'''
		}
	}

	def trimmedValue(State s) { s.value.removeCodeDelimiter }

	def defineConstructor(Reactor r) '''
		«IF r.parameters.length > 0»
			«r.name»::«r.name»(const std::string& name,
			    «IF r.isMain()»enactor::Environment* environment,«ELSE»enactor::Reactor* container«ENDIF»,
			    «FOR p : r.parameters SEPARATOR ",\n" AFTER ")"»«p.trimmedType» «p.name»«ENDFOR»
		«ELSE»
			«IF r.main»
				«r.name»::«r.name»(const std::string& name, enactor::Environment* environment)
			«ELSE»
				«r.name»::«r.name»(const std::string& name, enactor::Reactor* container)
			«ENDIF»
		«ENDIF»
		  : enactor::Reactor(name, «IF r.isMain()»environment«ELSE»container«ENDIF»)
		  «r.initializeParameters»
		  «r.initializeStateVariables»
		  «r.initializeInstances»
		  «r.initializeTimers»
		{}
	'''

	def initializeParameters(Reactor r) '''
		«FOR p : r.parameters BEFORE "// parameters\n"»
			, «p.name»(«p.name»)
		«ENDFOR»
	'''

	def initializeStateVariables(Reactor r) '''
		«FOR s : r.states BEFORE "// state variables\n"»
			, «s.name»(«s.trimmedValue»)
		«ENDFOR»
	'''

	def initializeInstances(Reactor r) '''
		«FOR i : r.instantiations BEFORE "// reactor instantiations \n"»
			, «i.name»{"«i.name»", this«FOR v : i.trimmedValues», «v»«ENDFOR»}
		«ENDFOR»
	'''

	def initializeTimers(Reactor r) '''
		«FOR t : r.timers BEFORE "// timers\n"»
			«t.initialize»
		«ENDFOR»
	'''

	def initialize(Timer t) {
		var String period = "0"
		var String offset = "0"
		if (t.timing !== null) {
			offset = '''«t.timing.offset.trimmedValue»'''
			if (t.timing.period !== null) {
				period = '''«t.timing.period.trimmedValue»'''
			}
		}
		''', «t.name»{"«t.name»", this, «period», «offset»}'''
	}

	def trimmedValues(Instantiation i) {
		var List<String> values = newArrayList
		for (p : i.reactorClass.parameters) {
			var String value = null
			for (a : i.parameters ?: emptyList) {
				if (a.lhs.name == p.name) {
					value = '''«a.trimmedValue»'''
				}
			}
			if (value === null) {
				value = '''«p.trimmedValue»'''
			}
			values.add(value)
		}
		values
	}

	def assembleReaction(Reactor r, Reaction n) '''
		// «n.name»
		«n.declareTriggers»
		«n.declareDependencies»
		«n.declareAntidependencies»
		«IF n.deadline !== null»
			«n.name».set_deadline(«n.deadline.time.trimmedValue», [this]() { «n.name»_deadline_handler(); });
		«ENDIF»
	'''

	def generateReactorHeader(Reactor r) '''
		«header()»
		
		#pragma once
		
		#include "enactor/enactor.hh"
		
		«r.includeInstances»
		«r.generatePreamble»
		using namespace enactor::literals;
		
		class «r.getName()» : public enactor::Reactor {
		 private:
		  «r.declareParameters»
		  «r.declareStateVariables»
		  «r.declareInstances»
		  «r.declareTimers»
		  «r.declareActions»
		  «r.declareReactions»
		  «r.declareReactionBodies»
		  «r.declareDeadlineHandlers»
		 public:
		  «r.declarePorts»
		  «r.declareConstructor»
		  
		  void assemble() override;
		};
	'''

	def generateReactorSource(Reactor r) '''
		«header()»
		
		#include "«r.getName».hh"
		
		«r.defineConstructor»
		
		void «r.name»::assemble() {
		  «FOR n : r.reactions»
		  	«r.assembleReaction(n)»
		  «ENDFOR»
		  «FOR c : r.connections BEFORE "  // connections\n"»
			«'''  «c.leftPort.fullName».bind_to(&«c.rightPort.fullName»);'''»
			«ENDFOR»
		}
		
		«r.implementReactionBodies»
		«r.implementReactionDeadlineHandlers»
	'''

	def header() '''
		/*
		 * This file was autogenerated by the Lingua Franca Compiler
		 *
		 * Source: «resource.getURI()»
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

	def generateMain(Reactor main, Target t) '''
		«header()»
		
		#include <thread>
		#include <chrono>
		
		#include "enactor/enactor.hh"
		
		#include "CLI/CLI11.hpp"
		
		#include "«main.name».hh"
		
		int main(int argc, char **argv) {
		  CLI::App app("«filename» Reactor Program");
		  
		  unsigned threads = «IF t.hasProperty('threads')»«t.getProperty('threads')»«ELSE»4«ENDIF»;
		  app.add_option("-t,--threads", threads, "the number of worker threads used by the scheduler", true);
		  unsigned timeout;
		  auto opt_timeout = app.add_option("--timeout", timeout, "Number of seconds after which the execution is aborted");
		  
		  CLI11_PARSE(app, argc, argv);
		  
		  enactor::Environment e{threads};
		
		  «main.name» main{"main", &e};
		  e.assemble();
		  e.init();
		
		  auto t = e.start();
		  if (opt_timeout->count() > 0) {
		    std::this_thread::sleep_for(std::chrono::seconds(timeout));
		    e.stop();
		  }
		  t.join();
		
		  return 0;
		}
	'''

	def generateCmake(Target target, List<Reactor> reactors) '''
		cmake_minimum_required(VERSION 3.5)
		project(«filename» VERSION 1.0.0 LANGUAGES CXX)
		
		include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
		include(GNUInstallDirs)
		
		set(DEFAULT_BUILD_TYPE "Debug")
		if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
		  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
		  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
		  # Set the possible values of build type for cmake-gui
		  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
		endif()
		
		if(NOT ENACTOR_BUILD_DIR)
		  set(ENACTOR_BUILD_DIR "" CACHE STRING "Choose the directory to build enactor in." FORCE)
		endif()
		
		ExternalProject_Add(
		  dep-enactor
		  PREFIX "${ENACTOR_BUILD_DIR}"
		  GIT_REPOSITORY "https://github.com/tud-ccc/enactor.git"
		  CMAKE_ARGS
		    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
		)
		
		set(CLI11_PATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/CLI/CLI11.hpp")
		file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v1.8.0/CLI11.hpp" "${CLI11_PATH}")
		
		set(ENACTOR_LIB_NAME "${CMAKE_SHARED_LIBRARY_PREFIX}enactor${CMAKE_SHARED_LIBRARY_SUFFIX}")
		set(ENACTOR_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
		
		add_library(enactor SHARED IMPORTED)
		add_dependencies(enactor dep-enactor "${CLI11_PATH}")
		set_target_properties(enactor PROPERTIES IMPORTED_LOCATION "${ENACTOR_LIB_DIR}/${ENACTOR_LIB_NAME}")
		
		set(CMAKE_INSTALL_RPATH "${ENACTOR_LIB_DIR}")
		set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
		
		add_executable(«filename»
		  main.cc
		  «FOR r : reactors»
		  	«r.getName()».cc
		  «ENDFOR»
		)
		target_include_directories(«filename» PUBLIC ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
		target_link_libraries(«filename» enactor)
		
		install(TARGETS «filename» RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
		
		«IF target.hasProperty("cmake_include")»
			include(«resource.URI.toString.extractDir»«File.separator»«target.getProperty("cmake_include").withoutQuotes»)
		«ENDIF»
	'''

	def withoutQuotes(String s) {
		s.replace("\"", "").replace("\'", "")
	}

	def void doCompile() {
		var makeCmd = newArrayList()
		var cmakeCmd = newArrayList()

		var cwd = Paths.get("").toAbsolutePath().toString()
		var srcPath = cwd + File.separator + "src-gen" + File.separator + filename
		var buildPath = cwd + File.separator + "build" + File.separator + filename
		var enactorPath = cwd + File.separator + "build" + File.separator + "enactor"

		makeCmd.addAll("make", "-j" + Runtime.getRuntime().availableProcessors(), "install")
		cmakeCmd.addAll("cmake", "-DCMAKE_INSTALL_PREFIX=" + cwd, "-DENACTOR_BUILD_DIR=" + enactorPath, srcPath)

		var buildDir = new File(buildPath)
		if(!buildDir.exists()) buildDir.mkdirs()

		println("--- Running: " + cmakeCmd.join(' '))
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
