/* Generator for TypeScript target. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.generator

import java.io.File
import java.io.FileOutputStream
import java.nio.file.Files
import java.nio.file.StandardCopyOption
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.List
import java.util.StringJoiner
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.InferredType
import org.lflang.Target
import org.lflang.TimeValue
import org.lflang.lf.Action
import org.lflang.lf.Delay
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar
import org.lflang.lf.TimeUnit
import org.lflang.lf.Timer
import org.lflang.lf.VarRef
import org.lflang.lf.Variable

import static extension org.lflang.ASTUtils.*

/** Generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de> 
 */
class TypeScriptGenerator extends GeneratorBase {

    /**
     * Path to the TypeScript library files (on the CLASSPATH).
     */
    static val String LIB_PATH = "/lib/TS"
    
    /**
     * Default imports for importing all the core classes and helper classes
     * for CLI argument handling.
     */
    static val DEFAULT_IMPORTS =  '''import commandLineArgs from 'command-line-args'
    import commandLineUsage from 'command-line-usage'
    import {Args as __Args, Present, Parameter as __Parameter, State as __State, Variable as __Variable, Read, Triggers as __Triggers, ReadWrite, Write, Action as __Action, Startup as __Startup, Sched, Timer as __Timer, Reactor as __Reactor, Port as __Port, OutPort as __OutPort, InPort as __InPort, App as __App} from './core/reactor'
    import {Reaction as __Reaction} from './core/reaction'
    import {FederatedApp as __FederatedApp} from './core/federation'
    import {TimeUnit, TimeValue, Tag as __Tag, Origin as __Origin} from './core/time'
    import {Log} from './core/util'
    import {ProcessedCommandLineArgs as __ProcessedCommandLineArgs, CommandLineOptionDefs as __CommandLineOptionDefs, CommandLineUsageDefs as __CommandLineUsageDefs, CommandLineOptionSpec as __CommandLineOptionSpec, unitBasedTimeValueCLAType as __unitBasedTimeValueCLAType, booleanCLAType as __booleanCLAType} from './core/cli'
    
    '''
  
    /**
     * Names of the configuration files to check for and copy to the generated 
     * source package root if they cannot be found in the source directory.
     */
    static val CONFIG_FILES = #["package.json", "tsconfig.json", "babel.config.js"]
    
    /**
     * Files to be copied from the reactor-ts submodule into the generated
     * source directory. 
     */
    static val RUNTIME_FILES = #["cli.ts", "command-line-args.d.ts", "command-line-usage.d.ts",
            "component.ts", "federation.ts", "reaction.ts", "reactor.ts",
            "microtime.d.ts", "nanotimer.d.ts", "time.ts", "ulog.d.ts", "util.ts"]
    
    /**
     * Set of parameters (AST elements) associated with the main reactor.
     */
    var mainParameters = new HashSet<Parameter>()

    new () {
        super()
        // Set defaults for federate compilation.
        targetConfig.compiler = "gcc"
        targetConfig.compilerFlags.add("-O2")
    }


    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
        IGeneratorContext context) {
        
        super.doGenerate(resource, fsa, context)
        
        if (generatorErrorsOccurred) return;
        
        // Generate imports for protocol buffer definitions
        // Note that the preamble is generated as part of
        // super.doGenerate.
        generateProtoPreamble()
        
        // Generate command line argument processing code
        generateCLAProcessing()
           
        // Generate code for each reactor. 
        for (r : reactors) {
           r.toDefinition.generateReactor() // FIXME: put each reactor class in its own file instead.
        }
        
        // Create output directories if they don't yet exist
        // FIXME: move cleanup and initialize code to FileConfig
        // FIXME: I saw some glitches that had to do with the path "existing" but it not being a directory. Add checks for this.
        
        var dir = fileConfig.getSrcGenPkgPath.toFile
        if (!dir.exists()) dir.mkdirs()
        dir = fileConfig.getSrcGenPath.toFile
        if (!dir.exists()) dir.mkdirs()

        // Perform distinct code generation into distinct files for each federate.
        var commonCode = code
        var String federateFilename

        // Every top-level reactor (contained
        // directly by the main reactor) is a federate
        for (federate : federates) {
            
            // Only generate one output if there is no federation.
            if (isFederated) {
                federateFilename = fileConfig.name + '_' + federate.name
                // Clear out previously generated code,
                // but keep the reactor class definitions
                // and the preamble.
                code = new StringBuilder(commonCode)
            } else {
                federateFilename = fileConfig.name
            }
        
            // Build the instantiation tree if a main reactor is present.
            if (this.mainDef !== null) {
                // Generate main instance, if there is one.
                generateReactorFederated(this.mainDef.reactorClass.toDefinition, federate)
                generateReactorInstance(this.mainDef)
                generateRuntimeStart(this.mainDef) 
            }
        
            // Derive target filename from the .lf filename.
            val tsFilename = federateFilename + ".ts";
            val jsFilename = federateFilename + ".js";

            // Delete source previously produced by the LF compiler.
            val generated = fileConfig.getSrcGenPath.resolve(tsFilename).toFile
            if (generated.exists) {
                generated.delete
            }

            // Delete .js previously output by TypeScript compiler
            val compiled = fileConfig.getSrcGenPath.resolve("dist").resolve(jsFilename).toFile
            if (compiled.exists) {
                compiled.delete
            }

            // Write the generated code to the output file.
            var fOut = new FileOutputStream(
                fileConfig.getSrcGenPath.resolve(tsFilename).toFile);
            fOut.write(getCode().getBytes())
            fOut.close()
        }
    
        for (file : TypeScriptGenerator.RUNTIME_FILES) {
            copyFileFromClassPath("/lib/TS/reactor-ts/src/core/" + file, fileConfig.getSrcGenPath.resolve("core").resolve(file).toString)
        }

        // Install default versions of config files into project if
        // they don't exist.       
        this.initializeProjectConfiguration()
        
        // NOTE: (IMPORTANT) at least on my mac, the instance of eclipse running this program did not have
        // the complete PATH variable needed to find the command npm. I had
        // to start my eclipse instance from a terminal so eclipse would have the correct environment
        // variables to run this command. Now, executeCommand makes a second
        // attempt to run the command using a bash shell, so if you have a
        // ~/.bash_profile file that specifies suitable paths, the command should
        // work.
        
            val npmInstall = createCommand("pnpm", #["install"], fileConfig.getSrcGenPkgPath)
            if (npmInstall === null || npmInstall.executeCommand() !== 0) {
                reportError(resource.findTarget, "ERROR: npm install command failed."
                    + "\nFor installation instructions, see: https://www.npmjs.com/get-npm")
                return
            }
        
        refreshProject()
        
        // Invoke the protocol buffers compiler on all .proto files in the project directory
        // Assumes protoc compiler has been installed on this machine
        
        // First test if the project directory contains any .proto files
        if (targetConfig.protoFiles.size != 0) {
            // For more info, see: https://www.npmjs.com/package/ts-protoc-gen
            
            // FIXME: Check whether protoc is installed and provides hints how to install if it cannot be found.
            val List<String> protocArgs = newLinkedList
            val tsOutPath = fileConfig.srcPath.relativize(fileConfig.getSrcGenPath)
            
            protocArgs.addAll(
                "--plugin=protoc-gen-ts=" + fileConfig.getSrcGenPkgPath.resolve("node_modules").resolve(".bin").resolve("protoc-gen-ts"),
                "--js_out=import_style=commonjs,binary:"+tsOutPath,
                "--ts_out=" + tsOutPath)
            protocArgs.addAll(targetConfig.protoFiles.fold(newLinkedList, [list, file | list.add(file); list]))
            val protoc = createCommand("protoc", protocArgs, fileConfig.srcPath)
                
            if (protoc === null) {
                return
            }

            val returnCode = protoc.executeCommand()
            if (returnCode == 0) {
                // FIXME: this code makes no sense. It is removing 6 chars from a file with a 3-char extension
//                val nameSansProto = fileConfig.name.substring(0, fileConfig.name.length - 6)
//               
//                targetConfig.compileAdditionalSources.add(
//                this.fileConfig.getSrcGenPath.resolve(nameSansProto + ".pb-c.c").toString)
//
//                targetConfig.compileLibraries.add('-l')
//                targetConfig.compileLibraries.add('protobuf-c')
            } else {
                reportError("protoc returns error code " + returnCode)    
            }
            // FIXME: report errors from this command.
        } else {
            println("No .proto files have been imported. Skipping protocol buffer compilation.")
        }
        

        // Invoke the compiler on the generated code.
        println("Type Checking")
        val tsc = createCommand("npm", #["run", "check-types"], fileConfig.getSrcGenPkgPath, findCommandEnv("npm"))
        if (tsc !== null) {
            if (tsc.executeCommand() == 0) {
                // Babel will compile TypeScript to JS even if there are type errors
                // so only run compilation if tsc found no problems.
                //val babelPath = codeGenConfig.outPath + File.separator + "node_modules" + File.separator + ".bin" + File.separator + "babel"
                // Working command  $./node_modules/.bin/babel src-gen --out-dir js --extensions '.ts,.tsx'
                println("Compiling")
                val babel = createCommand("npm", #["run", "build"], fileConfig.getSrcGenPkgPath)
                //createCommand(babelPath, #["src", "--out-dir", "dist", "--extensions", ".ts", "--ignore", "**/*.d.ts"], codeGenConfig.outPath)
                
                if (babel !== null) {
                    if (babel.executeCommand() == 0) {
                        println("SUCCESS (compiling generated TypeScript code)")                
                    } else {
                        reportError("Compiler failed.")
                    }   
                }
            } else {
                reportError("Type checking failed.")
            }
        }

        // If this is a federated execution, generate C code for the RTI.
        if (isFederated) {
            createFederateRTI()

            // Copy the required library files into the target file system.
            // This will overwrite previous versions.
            var files = newArrayList("rti.c", "rti.h", "federate.c", "reactor_threaded.c", "reactor.c", "reactor_common.c", "reactor.h", "pqueue.c", "pqueue.h", "util.h", "util.c")

            for (file : files) {
                copyFileFromClassPath(
                    File.separator + "lib" + File.separator + "core" + File.separator + file,
                    fileConfig.getSrcGenPath.toString + File.separator + file
                )
            }
            compileRTI()
        }
    }
    
    override generateDelayBody(Action action, VarRef port) {
        '''actions.«action.name».schedule(0, «generateVarRef(port)» as «getActionType(action)»);'''
    }

    override generateForwardBody(Action action, VarRef port) {
        '''«generateVarRef(port)» = «action.name» as «getActionType(action)»;'''
    }
 
    override String getTargetReference(Parameter param) {
        return '''this.«param.name».get()'''
    }
 
    // //////////////////////////////////////////
    // // Code generators.
    
     def generateReactorFederated(Reactor reactor, FederateInstance federate) {
        var reactorConstructor = new StringBuilder()

        pr("// =============== START reactor class " + reactor.name)

        for (p : reactor.preambles ?: emptyList) {
            pr("// *********** From the preamble, verbatim:")
            pr(p.code.toText)
            pr("\n// *********** End of preamble.")
        }
        
        val reactorName = reactor.name + reactor.typeParms?.join('<', ',', '>', [it.toText])
        // NOTE: type parameters that are referenced in ports or actions must extend
        // Present in order for the program to type check.
        if (reactor.isMain()) {
            pr("class " + reactorName + " extends __App {")
        } else if (reactor.isFederated()) {
            pr("class " + reactorName + " extends __FederatedApp {")
        } else {
            pr("export class " + reactorName + " extends __Reactor {")
        }
        
        indent()
        
        var arguments = new LinkedList()
        if (reactor.isMain() || reactor.isFederated()) {
            arguments.add("timeout: TimeValue | undefined = undefined")
            arguments.add("keepAlive: boolean = false")
            arguments.add("fast: boolean = false")
        } else {
            arguments.add("parent: __Reactor")
        }
        
        // For TS, parameters are arguments of the class constructor.
        for (parameter : reactor.parameters) {
            arguments.add(parameter.initializeParameter)
        }
        
        if (reactor.isMain() || reactor.isFederated()) {
            arguments.add("success?: () => void")
            arguments.add("fail?: () => void")
            pr(reactorConstructor, "constructor (")
            reactorConstructor.indent
            pr(reactorConstructor, arguments.join(', \n'))
            reactorConstructor.unindent
            pr(reactorConstructor, ") {")
            reactorConstructor.indent
            
        } else {
            pr(reactorConstructor, "constructor (")
            reactorConstructor.indent
            pr(reactorConstructor, arguments.join(', \n'))
            reactorConstructor.unindent
            pr(reactorConstructor, ") {")
            reactorConstructor.indent()
        }
        
        var String superCall
        if (reactor.isMain()) {
            superCall = "super(timeout, keepAlive, fast, success, fail);"
        } else if (reactor.isFederated()) {
            var port = federationRTIProperties.get('port')
            // Default of 0 is an indicator to use the default port, 15045.
            if (port === 0) {
                port = 15045
            }
            superCall = '''
            super(«federate.id», «port», «
                »"«federationRTIProperties.get('host')»", «
                »timeout, keepAlive, fast, success, fail);
            '''
        } else {
            superCall = "super(parent);"
        }
        pr(reactorConstructor, superCall)
        
        // Next handle child reactors instantiations.
        // If the app isn't federated, instantiate all
        // the child reactors. If the app is federated
        
        var List<Instantiation> childReactors;
        if (!reactor.isFederated()) {
            childReactors = reactor.instantiations;
        } else {
            childReactors = new LinkedList<Instantiation>();
            childReactors.add(federate.instantiation)
        }
        
        for (childReactor : childReactors ) {
        pr(childReactor.getName() + ": " + childReactor.reactorClass.name + childReactor.typeParms.join('<', ',', '>', [it | it.toText]))
        
        var childReactorArguments = new StringJoiner(", ");
        childReactorArguments.add("this")
    
        // Iterate through parameters in the order they appear in the
        // reactor class, find the matching parameter assignments in
        // the reactor instance, and write the corresponding parameter
        // value as an argument for the TypeScript constructor
        for (parameter : childReactor.reactorClass.toDefinition.parameters) {
            childReactorArguments.add(parameter.getTargetInitializer(childReactor))
        }
        
        pr(reactorConstructor, "this." + childReactor.getName()
            + " = new " + childReactor.reactorClass.name + 
            "(" + childReactorArguments + ")" )
        }
        
        // Next handle timers.
        for (timer : reactor.timers) {   
            var String timerPeriod
            if (timer.period === null) {
                timerPeriod = "0";
            } else {
                timerPeriod = timer.period.targetValue
            }
            
            var String timerOffset
            if (timer.offset === null) {
                timerOffset = "0";
            } else {

                timerOffset = timer.offset.targetValue

            }

            pr(timer.getName() + ": __Timer;")
            pr(reactorConstructor, "this." + timer.getName()
                + " = new __Timer(this, " + timerOffset + ", "+ timerPeriod + ");")
            
        }     

        // Create properties for parameters
        for (param : reactor.parameters) {
            pr(param.name + ": __Parameter<" + param.targetType + ">;")
            pr(reactorConstructor, "this." + param.name +
                " = new __Parameter(" + param.name + ");" )
        }

        // Next handle states.
        for (stateVar : reactor.stateVars) {
            if (stateVar.isInitialized) {
                pr(reactorConstructor, "this." + stateVar.name + ' = ' + 
                    "new __State(" + stateVar.targetInitializer + ');');
            } else {
                pr(reactorConstructor, "this." + stateVar.name + ' = ' + 
                    "new __State(undefined);");
            }
        }
        
        for (stateVar : reactor.stateVars) {            
            pr(stateVar.name + ': ' + "__State<" + stateVar.getTargetType + '>;');            
        }
        // Next handle actions.
        for (action : reactor.actions) {
            // Shutdown actions are handled internally by the
            // TypeScript reactor framework. There would be a
            // duplicate action if we included the one generated
            // by LF.
            if (action.name != "shutdown") {
                pr(action.name + ": __Action<" + getActionType(action) + ">;")

                var actionArgs = "this, __Origin." + action.origin  
                if (action.minDelay !== null) {
                    // Actions in the TypeScript target are constructed
                    // with an optional minDelay argument which defaults to 0.
                    if (action.minDelay.parameter !== null) {
                        actionArgs+= ", " + action.minDelay.parameter.name
                    } else {
                        actionArgs+= ", " + action.minDelay.targetValue    
                    }
                }
                pr(reactorConstructor, "this." + 
                    action.name + " = new __Action<" + getActionType(action) +
                    ">(" + actionArgs  + ");")
            }
        }
        
        // Next handle inputs.
        for (input : reactor.inputs) {
            pr(input.name + ": " + "__InPort<" + getPortType(input) + ">;")
            pr(reactorConstructor, "this." + input.name + " = new __InPort<"
                + getPortType(input) + ">(this);")
        }
        
        // Next handle outputs.
        for (output : reactor.outputs) {
            pr(output.name + ": " + "__OutPort<" + getPortType(output) + ">;")
            pr(reactorConstructor, "this." + output.name + " = new __OutPort<"
                + getPortType(output) + ">(this);")
        }
        
        // Next handle connections
        for (connection : reactor.connections) {
            var leftPortName = ""
            // FIXME: Add support for multiports.
            if (connection.leftPorts.length > 1) {
                reportError(connection, "Multiports are not yet supported in the TypeScript target.")
            } else {
                if (connection.leftPorts.get(0).container !== null) {
                    leftPortName += connection.leftPorts.get(0).container.name + "."
                }
                leftPortName += connection.leftPorts.get(0).variable.name
            }
            var rightPortName = ""
            if (connection.leftPorts.length > 1) {
                reportError(connection, "Multiports are not yet supported in the TypeScript target.")
            } else {
                if (connection.rightPorts.get(0).container !== null) {
                    rightPortName += connection.rightPorts.get(0).container.name + "."
                }
                rightPortName += connection.rightPorts.get(0).variable.name 
            }
            if (leftPortName != "" && rightPortName != "") {
                pr(reactorConstructor, "this._connect(this." + leftPortName + ", this." + rightPortName + ");")
            }
        }
        
        // If the app is federated, register its
        // networkMessageActions with the RTIClient
        if (reactor.isFederated()) {
            // The ID of the receiving port is simply 
            // the position of the action in the networkMessageActions list.
            var fedPortID = 0;
            for (nAction : federate.networkMessageActions) {
                var registration = '''
                this.registerFederatePortAction(«fedPortID», this.«nAction.name»);
                '''
                pr(reactorConstructor, registration)
                fedPortID++
            }
        }
        
        // Next handle reaction instances.
        // If the app is federated, only generate
        // reactions that are contained by that federate
        var List<Reaction> generatedReactions
        if (reactor.isFederated()) {
            generatedReactions = new LinkedList<Reaction>()
            for (reaction : reactor.reactions) {
                if (federate.containsReaction(reactor, reaction)) {
                    generatedReactions.add(reaction)
                }
            }
        } else {
            generatedReactions = reactor.reactions
        }
        
        
        for (reaction : generatedReactions) {
            
            // Determine signature of the react function
            var reactSignature = new StringJoiner(", ")
            reactSignature.add("this")
            
            // The prologue to the react function writes state
            // and parameters to local variables of the same name
            var reactPrologue = new StringBuilder()
            pr(reactPrologue, "const util = this.util;")
            
            // The epilogue to the react function writes local
            // state variables back to the state
            var reactEpilogue = new StringBuilder()
            
            // Assemble react function arguments from sources and effects
            // Arguments are either elements of this reactor, or an object
            // representing a contained reactor with properties corresponding
            // to listed sources and effects.
            
            // If a source or effect is an element of this reactor, add it
            // directly to the reactFunctArgs string. If it isn't, write it 
            // into the containerToArgs map, and add it to the string later.
            var reactFunctArgs = new StringJoiner(", ")
            // Combine triggers and sources into a set
            // so we can iterate over their union
            var triggersUnionSources = new HashSet<VarRef>()
            for (trigger : reaction.triggers) {
                if (!(trigger.startup || trigger.shutdown)) {
                    triggersUnionSources.add(trigger as VarRef)
                }
            }
            for (source : reaction.sources) {
                triggersUnionSources.add(source)
            }
            
            // Create a set of effect names so actions that appear
            // as both triggers/sources and effects can be
            // identified and added to the reaction arguments once.
            // We can't create a set of VarRefs because
            // an effect and a trigger/source with the same name are
            // unequal. 
            // The key of the pair is the effect's container's name,
            // The effect of the pair is the effect's name
            var effectSet = new HashSet<Pair<String, String>>()

            for (effect : reaction.effects) {
                var key = ""; // The container, defaults to an empty string
                var value = effect.variable.name; // The name of the effect
                if (effect.container !== null) {
                    key = effect.container.name
                }
                effectSet.add(new Pair(key, value))
            }

            // Add triggers and sources to the react function
            var containerToArgs = new HashMap<Instantiation, HashSet<Variable>>();           
            for (trigOrSource : triggersUnionSources) {
                // Actions that are both read and scheduled should only
                // appear once as a schedulable effect
                
                var trigOrSourceKey = "" // The default for no container
                var trigOrSourceValue = trigOrSource.variable.name
                if (trigOrSource.container !== null) {
                    trigOrSourceKey = trigOrSource.container.name
                }
                var trigOrSourcePair = new Pair(trigOrSourceKey, trigOrSourceValue)
                 
                if (!effectSet.contains(trigOrSourcePair)) {
                    var reactSignatureElementType = "";
                    
                    if (trigOrSource.variable instanceof Timer) {
                        reactSignatureElementType = "__Tag"
                    } else if (trigOrSource.variable instanceof Action) {
                        reactSignatureElementType = getActionType(trigOrSource.variable as Action)
                    } else if (trigOrSource.variable instanceof Port) {
                        reactSignatureElementType = getPortType(trigOrSource.variable as Port)
                    }
                    
                    reactSignature.add('''«trigOrSource.generateArg»: Read<«reactSignatureElementType»>''')
                    reactFunctArgs.add("this." + trigOrSource.generateVarRef)
                    if (trigOrSource.container === null) {
                        pr(reactPrologue, '''let «trigOrSource.variable.name» = «trigOrSource.generateArg».get();''')
                    } else {
                        var args = containerToArgs.get(trigOrSource.container)
                        if (args === null) {
                           // Create the HashSet for the container
                           // and handle it later.
                           args = new HashSet<Variable>();
                           containerToArgs.put(trigOrSource.container, args)
                        }
                        args.add(trigOrSource.variable)
                    }
                }
            }
            var schedActionSet = new HashSet<Action>();
            for (effect : reaction.effects) {
                var functArg = ""                
                var reactSignatureElement = "" + effect.generateArg
                if (effect.variable instanceof Timer) {
                    reportError("A timer cannot be an effect of a reaction")
                } else if (effect.variable instanceof Action){
                    reactSignatureElement += ": Sched<" + getActionType(effect.variable as Action) + ">"
                    schedActionSet.add(effect.variable as Action)
                } else if (effect.variable instanceof Port){
                    reactSignatureElement += ": ReadWrite<" + getPortType(effect.variable as Port) + ">"
                    if (effect.container === null) {
                        pr(reactEpilogue, "if (" + effect.variable.name + " !== undefined) {")
                        reactEpilogue.indent()
                        pr(reactEpilogue,  "__" + effect.variable.name + ".set(" + effect.variable.name + ");")
                        reactEpilogue.unindent()
                        pr(reactEpilogue, "}")   
                    }
                }

                reactSignature.add(reactSignatureElement)
                
                functArg = "this." + effect.generateVarRef 
                if (effect.variable instanceof Action){
                    reactFunctArgs.add("this.schedulable(" + functArg + ")")
                } else if (effect.variable instanceof Port) {
                    reactFunctArgs.add("this.writable(" + functArg + ")")
                }
                
                if (effect.container === null) {
                    pr(reactPrologue, "let " + effect.variable.name + " = __" + effect.variable.name + ".get();")
                } else {
                    // Hierarchical references are handled later because there
                    // could be references to other members of the same reactor.
                    var args = containerToArgs.get(effect.container)
                    if (args === null) {
                       args = new HashSet<Variable>();
                       containerToArgs.put(effect.container, args)
                    }
                    args.add(effect.variable)
                }
            }
            
            // Iterate through the actions to handle the prologue's
            // "actions" object
            var prologueActionObjectBody = new StringJoiner(", ")
            for (act : schedActionSet) {
                prologueActionObjectBody.add(act.name + ": __" + act.name)
            }
            if (schedActionSet.size > 0) {
                pr(reactPrologue, "let actions = {"
                    + prologueActionObjectBody + "};")
            }
            
            // Add parameters to the react function
            for (param : reactor.parameters) {
                
                // Underscores are added to parameter names to prevent conflict with prologue
                reactSignature.add("__" + param.name + ": __Parameter<"
                    + param.targetType + ">")
                reactFunctArgs.add("this." + param.name)
                
                pr(reactPrologue, "let " + param.name + " = __" + param.name + ".get();")
            }
            
            // Add state to the react function
            for (state : reactor.stateVars) {
                // Underscores are added to state names to prevent conflict with prologue
                reactSignature.add("__" + state.name + ": __State<"
                    + getStateType(state) + ">")
                reactFunctArgs.add("this." + state.name )
                
                pr(reactPrologue, "let " + state.name + " = __" + state.name + ".get();")
                pr(reactEpilogue, "if (" + state.name + " !== undefined) {")
                reactEpilogue.indent()
                pr(reactEpilogue,  "__" + state.name + ".set(" + state.name + ");")
                reactEpilogue.unindent()
                pr(reactEpilogue, "}")
            }
            
            // Initialize objects to enable hierarchical references.
            for (entry : containerToArgs.entrySet) {
                val initializer = new StringJoiner(", ")
                for (variable : entry.value) {
                    initializer.add('''«variable.name»: __«entry.key.name»_«variable.name».get()''')
                    if (variable instanceof Input) {
                        pr(reactEpilogue, 
                        '''if («entry.key.name».«variable.name» !== undefined) {
                            __«entry.key.name»_«variable.name».set(«entry.key.name».«variable.name»)
                        }''')    
                    }
                }
                pr(reactPrologue, '''let «entry.key.name» = {«initializer»}''')
            }
            
            // Assemble reaction triggers          
            var reactionTriggers = new StringJoiner(",\n") 
            for (trigger : reaction.triggers) {
                if (trigger instanceof VarRef) {
                    reactionTriggers.add("this." + trigger.generateVarRef)

                } else if (trigger.startup) {
                    reactionTriggers.add("this.startup")
                } else if (trigger.shutdown) {
                    reactionTriggers.add("this.shutdown")
                }
            }
            
            
            // Write the reaction itself
            pr(reactorConstructor, "this.addReaction(")//new class<T> extends Reaction<T> {")
            reactorConstructor.indent()
            pr(reactorConstructor, "new __Triggers(" + reactionTriggers + "),")
            pr(reactorConstructor, "new __Args(" + reactFunctArgs + "),")
            pr(reactorConstructor, "function (" + reactSignature + ") {")
            reactorConstructor.indent()
            pr(reactorConstructor, "// =============== START react prologue")
            pr(reactorConstructor, reactPrologue)
            pr(reactorConstructor, "// =============== END react prologue")
            pr(reactorConstructor, "try {")
            reactorConstructor.indent()
            pr(reactorConstructor, reaction.code.toText)
            reactorConstructor.unindent()
            pr(reactorConstructor, "} finally {")
            reactorConstructor.indent()
            pr(reactorConstructor, "// =============== START react epilogue")
            pr(reactorConstructor, reactEpilogue)
            pr(reactorConstructor, "// =============== END react epilogue")
            reactorConstructor.unindent()
            pr(reactorConstructor, "}")            
            reactorConstructor.unindent()  
            if (reaction.deadline === null) {
                pr(reactorConstructor, "}")
            } else {
                pr(reactorConstructor, "},")
                var deadlineArgs = ""
                if (reaction.deadline.delay.parameter !== null) {
                    deadlineArgs += "this." + reaction.deadline.delay.parameter.name + ".get()"; 
                } else {
                    deadlineArgs += reaction.deadline.delay.targetValue
                }
                pr(reactorConstructor, deadlineArgs + "," )
                pr(reactorConstructor, "function(" + reactSignature + ") {")
                reactorConstructor.indent()
                pr(reactorConstructor, "// =============== START deadline prologue")
                pr(reactorConstructor, reactPrologue)
                pr(reactorConstructor, "// =============== END deadline prologue")
                pr(reactorConstructor, "try {")
                reactorConstructor.indent()
                pr(reactorConstructor, toText(reaction.deadline.code))
                reactorConstructor.unindent()
                pr(reactorConstructor, "} finally {")
                reactorConstructor.indent()
                pr(reactorConstructor, "// =============== START deadline epilogue")
                pr(reactorConstructor, reactEpilogue)
                pr(reactorConstructor, "// =============== END deadline epilogue")
                reactorConstructor.unindent()
                pr(reactorConstructor, "}")  
                reactorConstructor.unindent()
                pr(reactorConstructor, "}")
            }
            reactorConstructor.unindent()
            pr(reactorConstructor, ");")
        }
        reactorConstructor.unindent()
        pr(reactorConstructor, "}")
        pr(reactorConstructor.toString())
        unindent()
        pr("}")
        pr("// =============== END reactor class " + reactor.name)
        pr("")
        
     }
    
    /** Generate a reactor class definition. This version unconditionally
     *  generates the reactor class definition, regardless of the
     *  federate structure.
     *  @param reactor The parsed reactor data structure.
     */
    def generateReactor(Reactor reactor) {
        if (!reactor.isFederated && !reactor.isMain) { // NOTE: Just to prevent NPE. This code makes no sense to me at all.
            generateReactorFederated(reactor, null)
        }
    }

    def generateArg(VarRef v) {
        if (v.container !== null) {
            '''__«v.container.name»_«v.variable.name»'''
        } else {
            '''__«v.variable.name»'''
        }
    }

    /** Generate the main app instance. This function is only used once
     *  because all other reactors are instantiated as properties of the
     *  main one.
     *  @param instance A reactor instance.
     */
    def void generateReactorInstance(Instantiation defn) {
        var fullName = defn.name

        // Iterate through parameters in the order they appear in the
        // main reactor class. If the parameter is typed such that it can
        // be a custom command line argument, use the parameter's command line
        // assignment variable ("__CL" + the parameter's name). That variable will
        // be undefined if the command line argument wasn't specified. Otherwise
        // use undefined in the constructor.
        var mainReactorParams = new StringJoiner(", ")
        for (parameter : defn.reactorClass.toDefinition.parameters) {
            
            if (mainParameters.contains(parameter)) {
                mainReactorParams.add("__CL" + parameter.name)
            } else {
                mainReactorParams.add("undefined")
            }
        }

        pr('// ************* Instance ' + fullName + ' of class ' +
            defn.reactorClass.name)
        
        pr("let __app;")
        pr("if (!__noStart) {")
        indent()
        pr("__app = new "+ fullName + "(__timeout, __keepAlive, __fast, "
            + mainReactorParams + ");")
        unindent()
        pr("}")
    }
    
    /** Generate code to call the _start function on the main App
     *  instance to start the runtime
     *  @param instance A reactor instance.
     */
    def void generateRuntimeStart(Instantiation defn) {
        pr(
        '''// ************* Starting Runtime for «defn.name» + of class «defn.reactorClass.name».
           if (!__noStart && __app) {
               __app._start();
           }
        ''');
    }


    // //////////////////////////////////////////
    // // Protected methods.
    
    /**
     * Generate code for the body of a reaction that handles input from the network
     * that is handled by the specified action. This base class throws an exception.
     * @param action The action that has been created to handle incoming messages.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type The type.
     * @throws UnsupportedOperationException If the target does not support this operation.
     */
    override String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        int receivingBankIndex,
        int receivingChannelIndex,
        InferredType type
    ) {
        return '''
            // FIXME: For now assume the data is a Buffer, but this is not checked.
            // Replace with ProtoBufs or MessagePack.
            if («action.name» !== undefined) {
                «receivingPort.container.name».«receivingPort.variable.name» = «
                    »«action.name»; // defaults to utf8 encoding
            }
        '''
    }
    
    
    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network. This base class throws an exception.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @throws UnsupportedOperationException If the target does not support this operation.
     */
    override String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        int sendingBankIndex,
        int sendingChannelIndex,
        FederateInstance receivingFed,
        InferredType type,
        boolean isPhysical,
        Delay delay
    ) {
        return '''
            // FIXME: For now assume the data is a Buffer, but this is not checked.
            // Replace with ProtoBufs or MessagePack.
            if («sendingPort.container.name».«sendingPort.variable.name» !== undefined) {
                let buf = Buffer.from(«sendingPort.container.name».«sendingPort.variable.name»)
                this.util.sendRTITimedMessage(buf, «receivingFed.id», «receivingPortID»);
            }
        '''
    } 
       

    /** Generate preamble code that appears in the code generated
     *  file before anything else.
     */
    override generatePreamble() {
        super.generatePreamble
        pr(TypeScriptGenerator.DEFAULT_IMPORTS) 
        pr("") 
    }
    
    /**
     * Generate the code for processing command line arguments 
     */
    private def generateCLAProcessing() {
        // Need to get the main reactor's parameters so they can be made
        // command line arguments
        var Reactor mainReactor
        
        for (reactor : fileConfig.resource.allContents.toIterable.filter(Reactor)) {
            if (reactor.isMain || reactor.isFederated) {
                mainReactor = reactor
            }
        }
        
        // Build the argument spec for commandLineArgs and commandLineUsage
        var customArgs = new StringJoiner(",\n")
        
        // Extend the return type for commandLineArgs
        var clTypeExtension = new StringJoiner(", ")
        
        for (parameter : mainReactor?.parameters ?: emptyList) {
            var String customArgType = null;
            var String customTypeLabel = null;
            var paramType = parameter.targetType
            if (paramType == "string") {
                mainParameters.add(parameter)
                //clTypeExtension.add(parameter.name + " : string")
                customArgType = "String";
            } else if (paramType == "number") {
                mainParameters.add(parameter)
                //clTypeExtension.add(parameter.name + " : number")
                customArgType = "Number";
            } else if (paramType == "boolean") {
                mainParameters.add(parameter)
                //clTypeExtension.add(parameter.name + " : boolean")
                customArgType = "booleanCLAType";
                customTypeLabel = '[true | false]'
            } else if (paramType == "TimeValue") {
                mainParameters.add(parameter)
                //clTypeExtension.add(parameter.name + " : TimeValue | null")
                customArgType = "__unitBasedTimeValueCLAType"
                customTypeLabel = "\'<duration> <units>\'"
            }
            // Otherwise don't add the parameter to customCLArgs
            
            
            if (customArgType !== null) {
                clTypeExtension.add(parameter.name + ": " + paramType)
                if (customTypeLabel !== null) {
                customArgs.add('''
                    { name: '«parameter.name»',
                        type: «customArgType»,
                        typeLabel: "{underline «customTypeLabel»}",
                        description: 'Custom argument. Refer to «fileConfig.srcFile» for documentation.'
                    }
                    ''')
                } else {
                    customArgs.add('''
                        { name: '«parameter.name»',
                            type: «customArgType»,
                            description: 'Custom argument. Refer to «fileConfig.srcFile» for documentation.'
                        }
                    ''')  
                }
            }  
        }
        
        var customArgsList = "[\n" + customArgs + "]"
        var clTypeExtensionDef = "{" + clTypeExtension + "}" 
        val setParameters = '''
            // ************* App Parameters
            let __timeout: TimeValue | undefined = «getTimeoutTimeValue»;
            let __keepAlive: boolean = «targetConfig.keepalive»;
            let __fast: boolean = «targetConfig.fastMode»;
            
            let __noStart = false; // If set to true, don't start the app.
            
            // ************* Custom Command Line Arguments
            let __additionalCommandLineArgs : __CommandLineOptionSpec = «customArgsList»;
            let __customCommandLineArgs = __CommandLineOptionDefs.concat(__additionalCommandLineArgs);
            let __customCommandLineUsageDefs = __CommandLineUsageDefs;
            type __customCLTypeExtension = «clTypeExtensionDef»;
            __customCommandLineUsageDefs[1].optionList = __customCommandLineArgs;
            const __clUsage = commandLineUsage(__customCommandLineUsageDefs);
                         
            // Set App parameters using values from the constructor or command line args.
            // Command line args have precedence over values from the constructor
            let __processedCLArgs: __ProcessedCommandLineArgs & __customCLTypeExtension;
            try {
                __processedCLArgs =  commandLineArgs(__customCommandLineArgs) as __ProcessedCommandLineArgs & __customCLTypeExtension;
            } catch (e){
                Log.global.error(__clUsage);
                throw new Error("Command line argument parsing failed with: " + e);
            }
            
            // Fast Parameter
            if (__processedCLArgs.fast !== undefined) {
                if (__processedCLArgs.fast !== null) {
                    __fast = __processedCLArgs.fast;
                } else {
                    Log.global.error(__clUsage);
                    throw new Error("'fast' command line argument is malformed.");
                }
            }
            
            // KeepAlive parameter
            if (__processedCLArgs.keepalive !== undefined) {
                if (__processedCLArgs.keepalive !== null) {
                    __keepAlive = __processedCLArgs.keepalive;
                } else {
                    Log.global.error(__clUsage);
                    throw new Error("'keepalive' command line argument is malformed.");
                }
            }
            
            // Timeout parameter
            if (__processedCLArgs.timeout !== undefined) {
                if (__processedCLArgs.timeout !== null) {
                    __timeout = __processedCLArgs.timeout;
                } else {
                    Log.global.error(__clUsage);
                    throw new Error("'timeout' command line argument is malformed.");
                }
            }
            
            // Logging parameter (not a constructor parameter, but a command line option)
            if (__processedCLArgs.logging !== undefined) {
                if (__processedCLArgs.logging !== null) {
                    Log.global.level = __processedCLArgs.logging;
                } else {
                    Log.global.error(__clUsage);
                    throw new Error("'logging' command line argument is malformed.");
                }
            } else {
                Log.global.level = Log.levels.«targetConfig.logLevel.name»; // Default from target property.
            }
            
            // Help parameter (not a constructor parameter, but a command line option)
            // NOTE: this arg has to be checked after logging, because the help mode should
            // suppress debug statements from it changes logging
            if (__processedCLArgs.help === true) {
                Log.global.error(__clUsage);
                __noStart = true;
                // Don't execute the app if the help flag is given.
            }
            
            // Now the logging property has been set to its final value,
            // log information about how command line arguments were set,
            // but only if not in help mode.
            
            // Runtime command line arguments 
            if (__processedCLArgs.fast !== undefined && __processedCLArgs.fast !== null
                && !__noStart) {
                Log.global.info("'fast' property overridden by command line argument.");
            }
            if (__processedCLArgs.keepalive !== undefined && __processedCLArgs.keepalive !== null
                && !__noStart) {
                Log.global.info("'keepalive' property overridden by command line argument.");
            }
            if (__processedCLArgs.timeout !== undefined && __processedCLArgs.timeout !== null
                && !__noStart) {
                Log.global.info("'timeout' property overridden by command line argument.");
            }
            if (__processedCLArgs.logging !== undefined && __processedCLArgs.logging !== null
                && !__noStart) {
                 Log.global.info("'logging' property overridden by command line argument.");
            }
            
            // Custom command line arguments
            «logCustomCLArgs()»
            
            // Assign custom command line arguments
            «assignCustomCLArgs()»
        '''
        
        pr(setParameters)
        
    }
    
    /**
     * Assign results of parsing custom command line arguments
     */
    private def assignCustomCLArgs() {
        var code = new StringJoiner("\n")
        for (parameter : mainParameters) {
            code.add('''
                let __CL«parameter.name»: «parameter.targetType» | undefined = undefined;
                if (__processedCLArgs.«parameter.name» !== undefined) {
                    if (__processedCLArgs.«parameter.name» !== null) {
                        __CL«parameter.name» = __processedCLArgs.«parameter.name»;
                    } else {
                        Log.global.error(__clUsage);
                        throw new Error("Custom '«parameter.name»' command line argument is malformed.");
                    }
                }
            ''')
        }
        code.toString()
    }
    
    /**
     * Generate code for extracting custom command line arguments
     * from the object returned from commandLineArgs
     */
    private def logCustomCLArgs() {
        var code = new StringJoiner("\n")
        for (parameter : mainParameters) {
            // We can't allow the programmer's parameter names
            // to cause the generation of variables with a "__" prefix
            // because they could collide with other variables.
            // So prefix variables created here with __CL
            code.add('''
                if (__processedCLArgs.«parameter.name» !== undefined && __processedCLArgs.«parameter.name» !== null
                    && !__noStart) {
                    Log.global.info("'«parameter.name»' property overridden by command line argument.");
                }
            ''')
        }
        code.toString()
    }

    /** Given a representation of time that may possibly include units,
     *  return a string that TypeScript can recognize as a value.
     *  @param time Literal that represents a time value.
     *  @param unit Enum that denotes units
     *  @return A string, as "[ timeLiteral, TimeUnit.unit]" .
     */
    override timeInTargetLanguage(TimeValue value) {
        if (value.unit != TimeUnit.NONE) {
            '''TimeValue.«value.unit»(«value.time»)'''
        } else {
            // The value must be zero.
            "TimeValue.zero()"
        }
    }

    // //////////////////////////////////////////
    // // Private methods.
    
    /**
     * Generate the part of the preamble that is determined
     * by imports to .proto files in the root directory
     */
     private def generateProtoPreamble() {
        pr("// Imports for protocol buffers")
        // Generate imports for .proto files
        for (file : targetConfig.protoFiles) {
            var name = file
            // Remove any extension the file name may have.
            val dot = name.lastIndexOf('.')
            if (dot > 0) {
                name = name.substring(0, dot)
            }
            var protoImportLine = '''
                import * as «name» from "./«name»_pb"
            '''
            pr(protoImportLine)  
        }
        pr("")  
     }
    
    /** 
     * Check whether configuration files are present in the same directory
     * as the source file. For those that are missing, install a default
     * If the given filename is not present in the same directory as the source
     * file, copy a default version of it from /lib/TS/.
     */
    private def initializeProjectConfiguration() {
        
        CONFIG_FILES.forEach [ fName |
            val alt = LIB_PATH + "/" + fName
            val src = new File(fileConfig.srcPath.toFile, fName)
            val dst = new File(fileConfig.getSrcGenPkgPath.toFile, fName)
            if (src.exists) {
                println("Copying '" + fName + "' from " + fileConfig.srcPath)
                Files.copy(src.toPath, dst.toPath, StandardCopyOption.REPLACE_EXISTING);
            } else {
                println("No '" + fName + "' exists in " + fileConfig.srcPath + ". Using default configuration.")
                copyFileFromClassPath(alt, dst.absolutePath)
            }
        ]
    }


    /** Return a TS type for the type of the specified state.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "TimeValue".
     *  If state is a parameter, get the parameter's type.
     *  If the state doesn't have a type, type it as 'unknown'
     *  @param state The state.
     *  @return The TS type.
     */
    private def getStateType(StateVar state) {
        state.getTargetType
    }
    
    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * "Present" which is the base type for Actions.
     * @param action The action
     * @return The TS type.
     */
    private def getActionType(Action action) {
        if (action.type !== null) {
            return action.type.targetType
        } else {
            return "Present"
        }
    }
    
     /**
     * Return a TS type for the specified port.
     * If the type has not been specified, return
     * "Present" which is the base type for ports.
     * @param port The port
     * @return The TS type.
     */
    private def getPortType(Port port) {
        if (port.type !== null) {
            return port.type.targetType
        } else {
            return "Present"
        }
    }


    def String getTargetInitializer(StateVar state) {
        '''«FOR init : state.initializerList SEPARATOR ", "»«init»«ENDFOR»'''
    }

    def private String getTargetInitializerHelper(Parameter param,
        List<String> list) {
        if (list.size == 0) {
            param.reportError("Parameters must have a default value!")
        } else if (list.size == 1) {
            return list.get(0)
        } else {
            '''[«FOR init : list SEPARATOR ", "»«init»«ENDFOR»]'''
        }
    }

    def String getTargetInitializer(Parameter param) {
        return getTargetInitializerHelper(param, param.initializerList)
    }

    def String getTargetInitializer(Parameter param, Instantiation i) {
        return '''«getTargetInitializerHelper(param, param.getInitializerList(i))»'''
    }

	def initializeParameter(Parameter p) 
		'''«p.name»: «p.targetType» = «p.getTargetInitializer()»'''
    
    override getTargetType(StateVar s) {
        val type = super.getTargetType(s)
        if (!s.isInitialized) {
        	type + " | undefined"
        } else {
        	type
        }
    }
    
    private def getTimeoutTimeValue() {
        if (targetConfig.timeout !== null) {
            return timeInTargetLanguage(targetConfig.timeout)
        } else {
            return "undefined"
        }
    }

    override setFileConfig(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
        this.fileConfig = new TypeScriptFileConfig(resource, fsa, context)
    }

    override String getTargetTimeType() {
        "TimeValue"
    }
    
    override getTargetTagType() {
        "TimeValue"
    }
    
    override getTargetTagIntervalType() {
        return getTargetUndefinedType()
    }
    
    override String getTargetUndefinedType() {
        "Present"
    }
    
    override String getTargetFixedSizeListType(String baseType, Integer size) {
        '''Array(«size»)<«baseType»>'''
    }
    
    override String getTargetVariableSizeListType(String baseType) {
        '''Array<«baseType»>'''
    }
    
    override supportsGenerics() {
        true
    }
    
    override String generateDelayGeneric()
        '''T extends Present'''
        
    override getTarget() {
        return Target.TS
    }

}
