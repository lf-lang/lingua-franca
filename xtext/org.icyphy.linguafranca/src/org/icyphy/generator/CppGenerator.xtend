/* Generator for C target. */

/*************
 * Copyright (c) 2019, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.icyphy.generator

import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.List
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.State
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef

/** Generator for C++ target.
 *
 *  @author{Christian Menard <christian.menard@tu-dresden.de}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class CppGenerator extends GeneratorBase {

    // Set of acceptable import targets includes only Cpp.
    val acceptableTargetSet = newHashSet('Cpp')

    static public var timeUnitsToCppUnits = #{
        TimeUnit.NSEC -> 'ns',
        TimeUnit.NSECS -> 'ns',
        TimeUnit.USEC -> 'us',
        TimeUnit.USECS -> 'us',
        TimeUnit.MSEC -> 'ms',
        TimeUnit.MSECS -> 'ms',
        TimeUnit.SEC -> 's',
        TimeUnit.SECS -> 's',
        TimeUnit.MIN -> 'min',
        TimeUnit.MINS -> 'min',
        TimeUnit.HOUR -> 'h',
        TimeUnit.HOURS -> 'h',
        TimeUnit.DAY -> 'd',
        TimeUnit.DAYS -> 'd',
        TimeUnit.WEEK -> 'd*7',
        TimeUnit.WEEKS -> 'd*7'
    }
    
    static public var logLevelsToInts = #{
    	"ERROR" -> 1,
    	"WARN" -> 2,
    	"INFO" -> 3,
    	"LOG" -> 3,
    	"DEBUG" -> 4
    }

    /** The main Reactor (vs. ReactorInstance, which is in the variable "main"). */
    Reactor mainReactor
   
    def toDir(Resource r) {
        r.toPath.getFilename
    }
    
    def headerFile(Reactor r) {
    	r.eResource.toDir + File.separator + r.name + ".hh"
    }
    
    def sourceFile(Reactor r) {
        r.eResource.toDir + File.separator + r.name + ".cc"
    }
   
        
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {

        var target = resource.findTarget
        
        super.doGenerate(resource, fsa, context)
        mainReactor = this.mainDef?.reactorClass
        
        if (mainReactor === null) {
            // No main reactor. Nothing to do.
            return
        } else {
            generateReactor(mainReactor)
            reactorsByResource.get(resource).add(mainReactor)
        }
        
        fsa.generateFile(filename + File.separator + "main.cc", mainReactor.generateMain(target))
        fsa.generateFile(filename + File.separator + "CMakeLists.txt", target.generateCmake(reactors))

        for (r : reactors) {
            fsa.generateFile(filename + File.separator + r.headerFile, r.generateReactorHeader)
            fsa.generateFile(filename + File.separator + r.sourceFile, r.generateReactorSource)
        }
        
        for(r: reactorsByResource.keySet) {
            fsa.generateFile(filename + File.separator + r.toDir + File.separator + "fwd.hh", r.generateForwardDeclarations)	
        }

        if (!targetNoCompile && !errorsOccurred()) {
            doCompile()
        } else {
            println("Exiting before invoking target compiler.")
        }
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

    def declare(Reaction n) '''
        reactor::Reaction «n.name»{"«n.name»", «n.priority», this, [this]() { «n.name»_body(); }};
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
            reactor::Timer «t.name»;
        «ENDFOR»
    '''

    def declareReactions(Reactor r) '''
        «FOR n : r.reactions BEFORE '// reactions\n' AFTER '\n'»
            «n.declare»
        «ENDFOR»
    '''

    def declarePorts(Reactor r) '''
        «FOR i : r.inputs BEFORE '// input ports\n' AFTER '\n'»
            reactor::Input<«i.trimmedType»> «i.name»{"«i.name»", this};
        «ENDFOR»
        «FOR o : r.outputs BEFORE '// output ports\n' AFTER '\n'»
            reactor::Output<«o.trimmedType»> «o.name»{"«o.name»", this};
        «ENDFOR»
    '''

    def declareActions(Reactor r) '''
        «FOR a : r.actions BEFORE '// actions\n' AFTER '\n'»
            «a.implementationType» «a.name»{"«a.name»", this};
        «ENDFOR»
        // default actions
        reactor::StartupAction startup {"startup", this};
        reactor::ShutdownAction shutdown {"shutdown", this};
        
    '''

    def implementationType(Action a) {
        if (a.origin == ActionOrigin.LOGICAL) {
            '''reactor::LogicalAction<«a.trimmedType»>'''
        } else {
            '''reactor::PhysicalAction<«a.trimmedType»>'''
        }
    }

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
              «n.deadline.code.removeCodeDelimiter»
            }
        «ENDFOR»
    '''

    def includeInstances(Reactor r) '''
        «FOR i : r.instantiations AFTER '\n'»
            #include "«i.reactorClass.headerFile»"
        «ENDFOR»
    '''

    def generatePreamble(Reactor r) '''
        «FOR p : r.preambles ?: emptyList AFTER '\n'»
            // preamble
            «removeCodeDelimiter(p.code)»
        «ENDFOR»
    '''

    def declareTriggers(Reaction n) '''
        «FOR t : n.triggers»
            «n.name».declare_trigger(&«t.triggerName»);
        «ENDFOR»
    '''

    def fullName(VarRef v) {
        if (v.container !== null) {
            '''«v.container.name».«v.variable.name»'''
        } else {
            '''«v.variable.name»'''
        }
    }

    def triggerName(TriggerRef t) {
        if (t instanceof VarRef) {
            fullName(t)
        } else {
            if (t.isShutdown) {
                '''«LinguaFrancaPackage.Literals.TRIGGER_REF__SHUTDOWN.name»'''
            } else if (t.isStartup) {
                '''«LinguaFrancaPackage.Literals.TRIGGER_REF__STARTUP.name»'''
            }
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
                    «IF r == mainReactor»reactor::Environment* environment«ELSE»reactor::Reactor* container«ENDIF»,
                    «FOR p : r.parameters SEPARATOR ",\n" AFTER ");"»«p.trimmedType» «p.name» = «IF p.ofTimeType»«p.trimmedTime»«ELSE»«p.trimmedValue»«ENDIF»«ENDFOR»
            '''
        } else {
            if (r == mainReactor) {
                '''«r.name»(const std::string& name, reactor::Environment* environment);'''
            } else {
                '''«r.name»(const std::string& name, reactor::Reactor* container);'''
            }
        }
    }

    def trimmedType(Parameter p) {
        if (p.ofTimeType) {
            '''const reactor::Duration'''
        } else {
            if (p.type !== null) {
                '''const «p.type.removeCodeDelimiter»'''
            } else {
                '''/* «p.reportError("Parameter has no type")» */'''
            }
        }
    }

    def trimmedType(State s) {
        if (s.ofTimeType) {
            '''reactor::Duration'''
        } else {
            if (s.type !== null) {
                s.type.removeCodeDelimiter
            } else if (s.parameter !== null) {
            	s.parameter.trimmedType
            } else {
                '''/* «s.reportError("State has no type")» */'''
            }
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
        if (p.ofTimeType) {
        	'''/* «p.reportError("Did not expect a parameter of type time!")» */'''
        } else {
            '''«p.value.removeCodeDelimiter»'''
        }
    }

    def trimmedTime(Parameter p) {
        if (p.ofTimeType) {
            if (p.unit === null || p.unit === TimeUnit.NONE) {
            	if (p.time == 0) {
                    '''reactor::Duration::zero()'''
                } else {
                	'''/* «p.reportError("Time values need to be 0 or have a unit!")» */'''
                }
            } else {
                '''«p.time»«timeUnitsToCppUnits.get(p.unit)»'''
            }
        } else {
            '''/* «p.reportError("Expected a parameter of type time!")» */'''
        }
    }
    
    def trimmedValue(State s) {
        if (s.ofTimeType) {
        	'''/* «s.reportError("Did not expect a state of type time!")» */'''
        } else if (s.parameter !== null) {
        	s.parameter.name
        } else {
            s.value.removeCodeDelimiter
        }
    }
    
    def trimmedTime(State s) {
        if (s.ofTimeType) {
            if (s.unit === null || s.unit === TimeUnit.NONE) {
            	if (s.time == 0) {
                    '''reactor::Duration::zero()'''
                } else {
                	'''/* «s.reportError("Time values need to be 0 or have a unit!")» */'''
                }
            } else {
                '''«s.time»«timeUnitsToCppUnits.get(s.unit)»'''
            }
        } else {
            '''/* «s.reportError("Expected a state of type time!")» */'''
        }
    }
    

    def trimmedValue(TimeOrValue tv) {
        if (tv.parameter !== null) {
            if (tv.parameter.ofTimeType) {
                '''/* «tv.reportError("Did not expect a parameter of time type")» */'''
            } else {
                '''«tv.parameter.name»'''
            }
        } else if (tv.value !== null) {
            '''«tv.value.removeCodeDelimiter»'''
        } else {
        	'''/* «tv.reportError("Expected a value or a parameter, not a time")» */'''
        }
    }

    def trimmedTime(TimeOrValue tv) {
    	if (tv.parameter !== null) {
    		if (tv.parameter.ofTimeType) {
    			'''«tv.parameter.name»'''
    		} else {
    			'''/* «tv.reportError("Expected a parameter of time type!")» */'''
    		}
        } else if (tv.value !== null) {
        	if (tv.value == '0') {
         		'''reactor::Duration::zero()'''
        	} else {
            	'''/* «tv.reportError("Time values need to be 0 or have a unit!")» */'''
            }
        } else {
            '''«tv.time»«timeUnitsToCppUnits.get(tv.unit)»'''
        }
    }

    def trimmedValue(Assignment a) '''«a.rhs.trimmedValue»'''

    def trimmedTime(Assignment a) '''«a.rhs.trimmedTime»'''

    def defineConstructor(Reactor r) '''
        «IF r.parameters.length > 0»
            «r.name»::«r.name»(const std::string& name,
                «IF r == mainReactor»reactor::Environment* environment«ELSE»reactor::Reactor* container«ENDIF»,
                «FOR p : r.parameters SEPARATOR ",\n" AFTER ")"»«p.trimmedType» «p.name»«ENDFOR»
        «ELSE»
            «IF r == mainReactor»
                «r.name»::«r.name»(const std::string& name, reactor::Environment* environment)
            «ELSE»
                «r.name»::«r.name»(const std::string& name, reactor::Reactor* container)
            «ENDIF»
        «ENDIF»
          : reactor::Reactor(name, «IF r == mainReactor»environment«ELSE»container«ENDIF»)
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
            , «s.name»(«IF s.ofTimeType»«s.trimmedTime»«ELSE»«s.trimmedValue»«ENDIF»)
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
        var String period = "reactor::Duration::zero()"
        var String offset = "reactor::Duration::zero()"
        if (t.offset !== null) {
          offset = '''«t.offset.trimmedTime»'''
        }
        if (t.period !== null) {
            period = '''«t.period.trimmedTime»'''
        }
        ''', «t.name»{"«t.name»", this, «period», «offset»}'''
    }

    def trimmedValues(Instantiation i) {
        var List<String> values = newArrayList
        for (p : i.reactorClass.parameters) {
            var String value = null
            for (a : i.parameters ?: emptyList) {
                if (a.lhs.name == p.name) {
                	if (p.ofTimeType) {
                        value = '''«a.trimmedTime»'''
                    } else {
                        value = '''«a.trimmedValue»'''
                    }
                }
            }
            if (value === null) {
                if (p.ofTimeType) {
                    value = '''«p.trimmedTime»'''
                } else {
                    value = '''«p.trimmedValue»'''
                }
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
            «n.name».set_deadline(«n.deadline.interval.trimmedTime», [this]() { «n.name»_deadline_handler(); });
        «ENDIF»
    '''

    def generateReactorHeader(Reactor r) '''
        «r.eResource.header»
        
        #pragma once
        
        #include "reactor-cpp/reactor-cpp.hh"
        
        «r.includeInstances»
        «r.generatePreamble»
        using namespace std::chrono_literals;
        using namespace reactor::operators;
        
        class «r.getName()» : public reactor::Reactor {
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
        «r.eResource.header»
        
        #include "«r.headerFile»"
        
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

    def header(Resource r) '''
        /*
         * This file was autogenerated by the Lingua Franca Compiler
         *
         * Source: «r.URI»
         * Date: «new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date())»
         */
    '''

    def generateForwardDeclarations(Resource resource) '''
        «resource.header»
        
        #pragma once
        
        «FOR r : reactorsByResource.get(resource)»
            class «r.name»;
        «ENDFOR»
    '''

    def generateMain(Reactor main, Target t) '''
        «resource.header»

        #include <chrono>        
        #include <thread>
        #include <memory>
        
        #include "reactor-cpp/reactor-cpp.hh"
        
        #include "CLI/CLI11.hpp"
        
        #include "«main.headerFile»"
        
        class Timeout : public reactor::Reactor {
         private:
          reactor::Timer timer;
        
          reactor::Reaction r_timer{"r_timer", 1, this,
                                    [this]() { environment()->sync_shutdown(); }};
        
         public:
          Timeout(const std::string& name, reactor::Environment* env, reactor::Duration timeout)
              : reactor::Reactor(name, env)
              , timer{"timer", this, reactor::Duration::zero(), timeout} {}
        
          void assemble() override { r_timer.declare_trigger(&timer); }
        };
        
        int main(int argc, char **argv) {
          CLI::App app("«filename» Reactor Program");
          
          unsigned threads = «IF targetThreads != 0»«Integer.toString(targetThreads)»«ELSE»std::thread::hardware_concurrency()«ENDIF»;
          app.add_option("-t,--threads", threads, "the number of worker threads used by the scheduler", true);
          unsigned timeout = 0;
          auto opt_timeout = app.add_option("-o,--timeout", timeout, "Number of seconds after which the execution is aborted.");
          bool fast{«targetFast»};
          app.add_flag("-f,--fast", fast, "Allow logical time to run faster than physical time.");
          bool keepalive{«targetKeepalive»};
          app.add_flag("-k,--keepalive", keepalive, "Continue execution even when there are no events to process.");
          
          CLI11_PARSE(app, argc, argv);
          
          reactor::Environment e{threads, keepalive, fast};
        
          // instantiate the main reactor
          «main.name» main{"«main.name»", &e};
          
          // optionally instantiate the timeout reactor
          std::unique_ptr<Timeout> t{nullptr};
          if (opt_timeout->count() > 0) {
            t = std::make_unique<Timeout>("Timeout", &e, std::chrono::seconds(timeout));
          } «IF targetTimeout >= 0»else {
          	t = std::make_unique<Timeout>("Timeout", &e, «targetTimeout»«timeUnitsToCppUnits.get(targetTimeoutUnit)»);
          }«ENDIF»

          // execute the reactor program
          e.assemble();
          auto thread = e.startup();
          thread.join();
        
          return 0;
        }
    '''

    def generateCmake(Target target, List<Reactor> reactors) '''
        cmake_minimum_required(VERSION 3.5)
        project(«filename» VERSION 1.0.0 LANGUAGES CXX)
        
        # require C++ 17
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)
        
        include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
        include(GNUInstallDirs)
        
        set(DEFAULT_BUILD_TYPE «IF targetBuildType === null»"Release"«ELSE»"«targetBuildType»"«ENDIF»)
        if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
          set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
        endif()
        
        if(NOT REACTOR_CPP_BUILD_DIR)
          set(REACTOR_CPP_BUILD_DIR "" CACHE STRING "Choose the directory to build reactor-cpp in." FORCE)
        endif()
        
        ExternalProject_Add(
          dep-reactor-cpp
          PREFIX "${REACTOR_CPP_BUILD_DIR}"
          GIT_REPOSITORY "https://github.com/tud-ccc/reactor-cpp.git"
          GIT_TAG "c8df5e661a6dad30a801e2d6027a2be15acb4cbe"
          CMAKE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DREACTOR_CPP_VALIDATE=«IF targetNoRuntimeValidation»OFF«ELSE»ON«ENDIF»
            «IF targetLoggingLevel !== null»-DREACTOR_CPP_LOG_LEVEL=«logLevelsToInts.get(targetLoggingLevel)»«ENDIF»
        )
        
        set(CLI11_PATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/CLI/CLI11.hpp")
        file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v1.9.0/CLI11.hpp" "${CLI11_PATH}")
        add_custom_target(dep-CLI11 DEPENDS "${CLI11_PATH}")
        
        set(REACTOR_CPP_LIB_NAME "${CMAKE_SHARED_LIBRARY_PREFIX}reactor-cpp${CMAKE_SHARED_LIBRARY_SUFFIX}")
        set(REACTOR_CPP_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
        
        add_library(reactor-cpp SHARED IMPORTED)
        add_dependencies(reactor-cpp dep-reactor-cpp)
        set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "${REACTOR_CPP_LIB_DIR}/${REACTOR_CPP_LIB_NAME}")
        
        if (APPLE)
          set(CMAKE_INSTALL_RPATH "@executable_path/../lib")
        else ()
          set(CMAKE_INSTALL_RPATH "${REACTOR_CPP_LIB_DIR}")
        endif ()
        set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
        
        add_executable(«filename»
          main.cc
          «FOR r : reactors»
              «r.eResource.toDir»«File.separator»«r.getName()».cc
          «ENDFOR»
        )
        target_include_directories(«filename» PUBLIC
            "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}"
            "${PROJECT_SOURCE_DIR}"
        )
        target_link_libraries(«filename» reactor-cpp)
        add_dependencies(«filename» dep-CLI11)

        install(TARGETS «filename» RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        
        «IF targetCmakeInclude !== null»
            include(«directory»«File.separator»«targetCmakeInclude»)
        «ENDIF»
    '''

    def void doCompile() {
        var makeCmd = newArrayList()
        var cmakeCmd = newArrayList()

        var srcPath = directory + File.separator + "src-gen" + File.separator + filename
        var buildPath = directory + File.separator + "build" + File.separator + filename
        var reactorCppPath = directory + File.separator + "build" + File.separator + "reactor-cpp"

        // Make sure cmake is found in the PATH.
        var cmakeTest = newArrayList()
        var cmake = "cmake"
        cmakeTest.addAll("which", cmake)
        var cmakeTestBuilder = new ProcessBuilder(cmakeTest)
        var cmakeTestReturn = cmakeTestBuilder.start().waitFor()
        if (cmakeTestReturn != 0) {
            // Info on MaxOSX PATH variable here: https://scriptingosx.com/2017/05/where-paths-come-from/
            println("WARNING: cmake not found on PATH: " + cmakeTestBuilder.environment.get("PATH"))
            cmake = "/opt/local/bin/cmake"
            println("Trying " + cmake)
            cmakeTest.clear
            cmakeTest.addAll("which", cmake)
            cmakeTestReturn = cmakeTestBuilder.start().waitFor()
            if (cmakeTestReturn != 0) {
                reportError("cmake not found on PATH nor in /opt/local/bin.\n"
                    + "See https://cmake.org/install to install cmake "
                    + "or adjust the global PATH variable on your platform (e.g. /etc/paths).")
                return
            }
        }
        var buildDir = new File(buildPath)
        if(!buildDir.exists()) buildDir.mkdirs()

        makeCmd.addAll("make", "-j" + Runtime.getRuntime().availableProcessors(), "install")
        cmakeCmd.addAll(cmake, "-DCMAKE_INSTALL_PREFIX=" + directory, "-DREACTOR_CPP_BUILD_DIR=" + reactorCppPath, srcPath)

        println("--- In directory: " + buildDir)
        println("--- Running: " + cmakeCmd.join(' '))
        var cmakeBuilder = new ProcessBuilder(cmakeCmd)
        cmakeBuilder.directory(buildDir)
        var cmakeEnv = cmakeBuilder.environment();
        if(targetCompiler !== null) {
        	cmakeEnv.put("CXX", targetCompiler);
        }

        var cmakeProcess = cmakeBuilder.start()
        val returnCode = cmakeProcess.waitFor()

        var stdout = readStream(cmakeProcess.getInputStream())
        var stderr = readStream(cmakeProcess.getErrorStream())
        if (stdout.length() > 0) {
            println("------ Standard output from cmake command:")
            println(stdout)
            println("------ End of standard output from cmake command.")
        }
        if (returnCode != 0) {
            reportError("cmake returns error code " + returnCode)
        }
        if (stderr.length() > 0) {
            reportError("ERROR: cmake reports errors:\n" + stderr.toString)
        }
        // If cmake succeeded, run make.
        if (returnCode == 0) {
            println("--- In directory: " + buildDir)
            println("--- Running: " + makeCmd.join(" "))
            var makeBuilder = new ProcessBuilder(makeCmd)
            makeBuilder.directory(buildDir)
            var makeProcess = makeBuilder.start()
            var makeReturnCode = makeProcess.waitFor()
            stdout = readStream(makeProcess.getInputStream())
            stderr = readStream(makeProcess.getErrorStream())

            if (makeReturnCode == 0) {
                println("SUCCESS (compiling generated C++ code)")
                println("Generated code is in "
                    + directory + File.separator + "bin" + File.separator + filename
                )
            } else {
                reportError("make returns error code " + makeReturnCode)
                if (stderr.length() > 0) {
                	reportError("make reports errors:\n" + stderr.toString)
            	}
            }
        }
    }
    
    ////////////////////////////////////////////////
    //// Protected methods
    
    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set is a set of case-insensitive
     *  strings specifying target names.
     */
    override acceptableTargets() {
        acceptableTargetSet
    }
    
    // FIXME: the following implementations are most certainly incorrect.
    
    override generateDelayBody(Action action, VarRef port) '''
        «IF !action.type.endsWith("*")»
            «action.type»* foo = malloc(sizeof(«action.type»));
            *foo = «generateVarRef(port)»;
        «ELSE»
            «action.type»* foo = &«generateVarRef(port)»;
        «ENDIF»
        schedule(«action.name», 0, foo);
    '''
    

    override generateForwardBody(Action action, VarRef port) '''
        set(«generateVarRef(port)», «action.name»_value);
    '''
    
}
