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

package org.icyphy.generator

import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.util.HashMap
import java.util.HashSet
import java.util.StringJoiner
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.State
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

// FIXME: This still has a bunch of copied code from CGenerator that should be removed.

/** Generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class TypeScriptGenerator extends GeneratorBase {

    // Set of acceptable import targets includes only TypeScript.
    val acceptableTargetSet = newHashSet('TypeScript')

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override void doGenerate(
        Resource resource,
        IFileSystemAccess2 fsa,
        IGeneratorContext context
    ) {
        analyzeModel(resource, fsa, context)
        
        super.doGenerate(resource, fsa, context)
        
        // Target filename.
        val tsFilename = filename + ".ts"
        val jsFilename = filename + ".js"
        val projectPath = directory + File.separator + filename
        val reactorTSPath = File.separator + "lib" + File.separator +
            "TS" + File.separator + "reactor-ts"
        var srcGenPath = projectPath + File.separator + "src"
        var outPath = projectPath + File.separator + "dist"
        var reactorCorePath = reactorTSPath + File.separator + "src" + File.separator
            + "core" + File.separator
        var tscPath = directory + File.separator + "node_modules" +  File.separator 
            + "typescript" +  File.separator + "bin" +  File.separator + "tsc"
        
        // Generate main instance, if there is one.
        if (this.mainDef !== null) {
            generateReactorInstance(this.mainDef)
            generateRuntimeStart(this.mainDef)
        }
        
        // Create output directories if they don't yet exist
        var dir = new File(projectPath)
        if (!dir.exists()) dir.mkdirs()
        dir = new File(srcGenPath)
        if (!dir.exists()) dir.mkdirs()
        dir = new File(outPath)
        if (!dir.exists()) dir.mkdirs()

        // Delete source previously output the LF compiler
        var file = new File(srcGenPath + File.separator + tsFilename)
        if (file.exists) {
            file.delete
        }

        // Delete .js previously output by TypeScript compiler
        file = new File(outPath + File.separator + jsFilename)
        if (file.exists) {
            file.delete
        }

        // Write the generated code to the src-gen directory.
        var fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + tsFilename));
        fOut.write(getCode().getBytes())
        
        // Copy the required library files into the src directory
        // so they may be compiled as part of the TypeScript project.
        // This requires that the TypeScript submodule has been installed.

        // Use the first file to test whether the submodule has been installed.
        val fileContents = readFileInClasspath(reactorCorePath + "reactor.ts")
        if (fileContents === null) {
            throw new IOException("Required runtime file not found: "
                + reactorCorePath
                + "reactor.ts.\n"
                + "Perhaps the reactor.ts submodule is missing.\n"
                + "See https://github.com/icyphy/lingua-franca/wiki/downloading-and-building#clone-the-lingua-franca-repository."
            )
        }
        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "reactor.ts"));
        fOut.write(fileContents.getBytes())

        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "time.ts"));
        fOut.write(readFileInClasspath(reactorCorePath + "time.ts").getBytes())

        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "util.ts"));
        fOut.write(readFileInClasspath(reactorCorePath + "util.ts").getBytes())

        // Copy default versions of config files into project if
        // they don't exist.
        createDefaultConfigFile(directory, "package.json")
        createDefaultConfigFile(projectPath, "tsconfig.json")
        createDefaultConfigFile(projectPath, "babel.config.js")
        
        // FIXME: (IMPORTANT) at least on my mac, the instance of eclipse running this program did not have
        // the complete PATH variable needed to find the command npm. I had
        // to start my eclipse instance from a terminal so eclipse would have the correct environment
        // variables to run this command.
        
        // Install npm modules.
        println("In directory: " + directory)
        println("Running npm install ...")
        var installCmd = newArrayList();
        installCmd.addAll("npm", "install")
        var installBuilder = new ProcessBuilder(installCmd)
        installBuilder.directory(new File(directory))
        var Process installProcess = installBuilder.start()
        
        // Sleep until the modules have installed
        installProcess.waitFor()
        
        refreshProject()

        // Invoke the compiler on the generated code.
//        val relativeSrcFilename = "src-gen" + File.separator + tsFilename;
//        val relativeTSFilename = "ts" + File.separator + filename + '.js';
        
        // Working example: src-gen/Minimal.ts --outDir bin --module CommonJS --target es2018 --esModuleInterop true --lib esnext,dom --alwaysStrict true --strictBindCallApply true --strictNullChecks true
        // Must compile to ES2015 or later and include the dom library.
        // Working command without a tsconfig.json:
        // compileCommand.addAll(tscPath,  relativeSrcFilename, 
        //      "--outDir", "js", "--module", "CommonJS", "--target", "es2018", "--esModuleInterop", "true",
        //      "--lib", "esnext,dom", "--alwaysStrict", "true", "--strictBindCallApply", "true",
        //      "--strictNullChecks", "true");
        
        // FIXME: Perhaps add a compileCommand option to the target directive, as in C.
        // Here, we just use a generic compile command.
        var typeCheckCommand = newArrayList()

        // If $tsc is run with no arguments, it uses the tsconfig file.
        typeCheckCommand.addAll(tscPath)
        
        println("Type checking with command: " + typeCheckCommand.join(" "))
        var builder = new ProcessBuilder(typeCheckCommand);
        builder.directory(new File(projectPath));
        var process = builder.start()
        var code = process.waitFor()
        var stdout = readStream(process.getInputStream())
        var stderr = readStream(process.getErrorStream())
        if (stdout.length() > 0) {
            println("--- Standard output from TypeScript type checker:")
            println(stdout)
        }
        if (code !== 0) {
            reportError("Type checker returns error code " + code)
        }
        if (stderr.length() > 0) {
            reportError("Type checker reports errors:\n" + stderr.toString)
        }
        
        // Babel will compile TypeScript to JS even if there are type errors
        // so only run compilation if tsc found no problems.
        if (code === 0){
            var babelPath = directory + File.separator + "node_modules" + File.separator + ".bin" + File.separator + "babel"
            // Working command  $./node_modules/.bin/babel src-gen --out-dir js --extensions '.ts,.tsx'
            var compileCommand = newArrayList(babelPath, "src",
                "--out-dir", "dist", "--extensions", ".ts")
            println("Compiling with command: " + compileCommand.join(" "))
            builder = new ProcessBuilder(compileCommand);
            builder.directory(new File(projectPath));
            process = builder.start()
            code = process.waitFor()
            stdout = readStream(process.getInputStream())
            stderr = readStream(process.getErrorStream())
            if (stdout.length() > 0) {
                println("--- Standard output from Babel compiler:")
                println(stdout)
            }
            if (code !== 0) {
                reportError("Compiler returns error code " + code)
            }
            if (stderr.length() > 0) {
                reportError("Compiler reports errors:\n" + stderr.toString)
            } else {
                println("SUCCESS (compiling generated TypeScript code)")
            }
        }
    }
    
    override generateDelayBody(Action action, VarRef port) {
        '''«action.name».schedule(0, «generateVarRef(port)» as «getActionType(action)»)'''
    }

    override generateForwardBody(Action action, VarRef port) {
        '''«generateVarRef(port)».set(«action.name» as «getActionType(action)»)'''
    }
 
    // //////////////////////////////////////////
    // // Code generators.
    /** Generate a reactor class definition.
     *  @param reactor The parsed reactor data structure.
     */
    override generateReactor(Reactor reactor) {
        super.generateReactor(reactor)   
        
        var reactorConstructor = new StringBuilder()

        pr("// =============== START reactor class " + reactor.name)

        for (p : reactor.preambles ?: emptyList) {
            pr("// *********** From the preamble, verbatim:")
            pr(removeCodeDelimiter(p.code))
            pr("\n// *********** End of preamble.")
        }

        if(reactor.isMain()){
            pr("export class " + reactor.name + " extends App {")
        } else {
            pr("export class " + reactor.name + " extends Reactor {")
        }
        
        indent()
        
        var arguments = new StringJoiner(", ")
        if (reactor.isMain()) {
            arguments.add("name: string")
            arguments.add("timeout: TimeInterval | undefined = undefined")
            arguments.add("keepAlive: boolean = false")
            arguments.add("fast: boolean = false")
        } else {
            arguments.add("parent:Reactor")
        }
        
        // For TS, parameters are arguments of the class constructor.
        for (parameter : reactor.parameters) {
            if (getParameterType(parameter).equals("")) {
                reportError(parameter,
                    "Parameter is required to have a type: " + parameter.name)
            } else if (parameter.ofTimeType){
                 arguments.add(parameter.name + ": " + getParameterType(parameter)
                    +" = " + timeInTargetLanguage(parameter.time.toString() , parameter.unit )  )
            } else {
                 arguments.add(parameter.name + ": " + getParameterType(parameter)
                    +" = " + removeCodeDelimiter(parameter.value))
            }
        }
        
        if (reactor.isMain()) {
            
            arguments.add("success?: () => void")
            arguments.add("fail?: () => void")
            pr(reactorConstructor, "constructor (" + arguments + ") {")
            reactorConstructor.indent()
            pr(reactorConstructor, "super(timeout, keepAlive, fast, success, fail);");
        } else {
            pr(reactorConstructor, "constructor (" + arguments + ") {")
            reactorConstructor.indent()
            pr(reactorConstructor, "super(parent);");
        }
        
        // Next handle child reactors instantiations
        for (childReactor : reactor.instantiations ) {
            pr(childReactor.getName() + ": " + childReactor.reactorClass.name )
            
            var childReactorArguments = new StringJoiner(", ");
            childReactorArguments.add("this")
        
            // Iterate through parameters in the order they appear in the
            // reactor class, find the matching parameter assignments in
            // the reactor instance, and write the corresponding parameter
            // value as an argument for the TypeScript constructor
            for (parameter : childReactor.reactorClass.parameters) {
                
                // Attempt to find a non-default parameter value
                for (parameterAssignment : childReactor.parameters) {
                    if (parameterAssignment.lhs.name.equals(parameter.name)) {
                        if (parameterAssignment.rhs instanceof Parameter) {
                            childReactorArguments.add("this."
                            + parameterAssignment.rhs.parameter.name + ".get()")
                        } else if (parameterAssignment.rhs.value !== null) {
                            childReactorArguments.add(removeCodeDelimiter(parameterAssignment.rhs.value))
                        } else {
                            childReactorArguments.add(
                                timeInTargetLanguage(parameterAssignment.rhs.time.toString
                                   , parameterAssignment.rhs.unit))
                        }
                    }
                }
            }
            
            
            pr(reactorConstructor, "this." + childReactor.getName()
                + " = new " + childReactor.reactorClass.name + "("
                + childReactorArguments + ")" )
        }
       
        
        // Next handle timers.
        for (timer : reactor.timers) {
            // FIXME: startup is no longer a timer. The check below can be deleted.
             
            // The startup timer is handled by default within
            // the TypeScript reactor framework. There would be a
            // duplicate startup timer if we also use the one provided
            // by LF. So suppress it.
            if (timer.name != "startup") {
                var String timerPeriod
                if (timer.period === null) {
                    timerPeriod = "0";
                } else {
                    if (timer.period.parameter !== null) {
                        timerPeriod = timer.period.parameter.name
                    } else if (Integer.parseInt(timer.period.time.toString) === 0 ) {
                        timerPeriod = "0"
                    } else {
                        timerPeriod = timeInTargetLanguage(timer.period.time.toString(), timer.period.unit)
                    }
                }
                
                var String timerOffset
                if (timer.offset === null) {
                    timerOffset = "0";
                } else {
                     if (timer.offset.parameter !== null) {
                        timerOffset = timer.offset.parameter.name
                    } else if (Integer.parseInt(timer.offset.time.toString) === 0 ) {
                        timerOffset = "0"
                    } else {
                        timerOffset = timeInTargetLanguage(timer.offset.time.toString(), timer.offset.unit)
                    }
                }
    
                pr(timer.getName() + ": Timer;")
                pr(reactorConstructor, "this." + timer.getName()
                    + " = new Timer(this, " + timerOffset + ", "+ timerPeriod + ");")
            }
        }     

        // Create properties for parameters
        for (param : reactor.parameters) {
            pr(param.name + ": Parameter<" + getParameterType(param) + ">;")
            pr(reactorConstructor, "this." + param.name +
                " = new Parameter(" + param.name + "); // Parameter" )
        }

        // Next handle states.
        for (state : reactor.states) {
            pr(state.name + ': ' +
                    "State<" + getStateType(state) +  '>;');
            if (state.parameter !== null) {
                // State is a parameter
                pr(reactorConstructor, "this." + state.name + " = "
                        + "new State(" + state.parameter.name + ");" )
            } else if (state.ofTimeType) {  
                // State is a time type
                pr(reactorConstructor, "this." + state.name + " = "
                    + "new State(" + timeInTargetLanguage( state.time.toString, state.unit) + ");" )
            } else if (state.value !== null) {
                // State is a literal 
                pr(reactorConstructor, "this." + state.name + " = "
                    + "new State(" +removeCodeDelimiter(state.value) + ");" )
            } else {
                // State has an undefined value
                 pr(reactorConstructor, "this." + state.name + " = "
                    + "new State(undefined);" )
            }
        }
        // Next handle actions.
        for (action : reactor.actions) {
            // Shutdown actions are handled internally by the
            // TypeScript reactor framework. There would be a
            // duplicate action if we included the one generated
            // by LF.
            if (action.name != "shutdown") {
                pr(action.name + ": Action<" + getActionType(action) + ">;")

                var actionArgs = "this, Origin." + action.origin  
                if (action.minTime !== null) {
                    // Actions in the TypeScript target are constructed
                    // with an optional minDelay argument which defaults to 0.
                    actionArgs+= ", " + timeInTargetLanguage(action.minTime.time.toString, action.minTime.unit)
                }
                pr(reactorConstructor, "this." + 
                    action.name + " = new Action<" + getActionType(action) +
                    ">(" + actionArgs  + ");")
            }
        }
        
        // Next handle inputs.
        for (input : reactor.inputs) {
            pr(input.name + ": " + "InPort<" + getPortType(input) + ">;")
            pr(reactorConstructor, "this." + input.name + " = new InPort<"
                + getPortType(input) + ">(this);")
        }
        
        // Next handle outputs.
        for (output : reactor.outputs) {
            pr(output.name + ": " + "OutPort<" + getPortType(output) + ">;")
            pr(reactorConstructor, "this." + output.name + " = new OutPort<"
                + getPortType(output) + ">(this);")
        }
        
        // Next handle connections
        for (connection : reactor.connections) {
            var leftPortName = ""
            if (connection.leftPort.container !== null) {
                leftPortName += connection.leftPort.container.name + "."
            }
            leftPortName += connection.leftPort.variable.name 
            
            var rightPortName = ""
            if (connection.rightPort.container !== null) {
                rightPortName += connection.rightPort.container.name + "."
            }
            rightPortName += connection.rightPort.variable.name 
            
            pr(reactorConstructor, "this._connect(this." + leftPortName + ", this." + rightPortName + ");")
        }
        
        
        // Next handle reaction instances
        for (reaction : reactor.reactions) {
            
            // Determine signature of the react function
            var reactSignature = new StringJoiner(", ")
            reactSignature.add("this")
            
            // The prologue to the react function writes state
            // and parameters to local variables of the same name
            var reactPrologue = new StringBuilder()
            
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
                if (trigger instanceof VarRef) {
                }
                if ( ! (trigger.startup || trigger.shutdown)) {
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
                 
                if (!effectSet.contains(trigOrSourcePair)){
                    if (trigOrSource.container === null) {
                        var reactSignatureElementType = "";
                        
                        if (trigOrSource.variable instanceof Timer) {
                            reactSignatureElementType = "TimeInstant"
                        } else if (trigOrSource.variable instanceof Action) {
                            reactSignatureElementType = getActionType(trigOrSource.variable as Action)
                        } else if (trigOrSource.variable instanceof Port) {
                            reactSignatureElementType = getPortType(trigOrSource.variable as Port)
                        }
                        var reactSignatureElement =  "__" + trigOrSource.variable.name
                            + ": Readable<" + reactSignatureElementType + ">"
                        
                        pr(reactPrologue, "let " + trigOrSource.variable.name + " = __" + trigOrSource.variable.name + ".get();")
                        reactSignature.add(reactSignatureElement)
                        reactFunctArgs.add("this." + trigOrSource.variable.name)
                    } else {
                        var args = containerToArgs.get(trigOrSource.container)
                        if (args === null) {
                           // Create the HashSet for the container
                           args = new HashSet<Variable>();
                           containerToArgs.put(trigOrSource.container, args)
                        }
                        args.add(trigOrSource.variable)
                    }
                }
            }
            for (effect : reaction.effects) {
                var functArg = ""
                if (effect.container === null) {
                    var reactSignatureElement = effect.variable.name
                    if (effect.variable instanceof Timer){
                        reportError("A timer cannot be an effect of a reaction")
                    } else if (effect.variable instanceof Action){
                        reactSignatureElement += ": Schedulable<" + getActionType(effect.variable as Action) + ">"
                    } else if (effect.variable instanceof Port){
                        reactSignatureElement += ": Writable<" + getPortType(effect.variable as Port) + ">"
                    }
                    reactSignature.add(reactSignatureElement)
                    
                    functArg = "this." + effect.variable.name 
                    if( effect.variable instanceof Action ){
                        reactFunctArgs.add("this.getSchedulable(" + functArg + ")")
                    } else if (effect.variable instanceof Port) {
                        reactFunctArgs.add("this.getWritable(" + functArg + ")")
                    }  
                } else {
                    var args = containerToArgs.get(effect.container)
                    if (args === null) {
                       // Create the HashSet for the container
                       args = new HashSet<Variable>();
                       containerToArgs.put(effect.container, args)
                    }
                    args.add(effect.variable)
                }
            }
            
            // Add parameters to the react function
            for (param : reactor.parameters) {
                
                // Underscores are added to parameter names to prevent conflict with prologue
                reactSignature.add("__" + param.name + ": Parameter<"
                    + getParameterType(param) + ">")
                reactFunctArgs.add("this." + param.name)
                
                pr(reactPrologue, "let " + param.name + " = __" + param.name + ".get();")
            }
            
            // Add state to the react function
            for (state : reactor.states) {
                // Underscores are added to state names to prevent conflict with prologue
                reactSignature.add("__" + state.name + ": State<"
                    + getStateType(state) + ">")
                reactFunctArgs.add("this." + state.name )
                
                pr(reactPrologue, "let " + state.name + " = __" + state.name + ".get();")
                pr(reactEpilogue, "__" + state.name + ".set(" + state.name + ");")
            }
            
            // Write an object as an argument for each container
            var containers = containerToArgs.keySet()
            for (container : containers) {
                var containedArgsObject = new StringJoiner(", ")
                var containedSigObject = new StringJoiner(", ") 
                var containedPrologueObject = new StringJoiner(", ")
                var containedVariables = containerToArgs.get(container)
                for (containedVariable : containedVariables) {
                    var functArg = "this." + container.name + "." + containedVariable.name
                    var containedSigElement = ""
                    var containedArgElement = ""
                    var containedPrologueElement = ""
                    if (containedVariable instanceof Input) {
                        containedSigElement +=  containedVariable.name + ": Writable<" + containedVariable.type + ">"
                        containedArgElement += containedVariable.name + ": " + "this.getWritable(" + functArg + ")"
                        containedPrologueElement += containedVariable.name + ": __" + container.name + "." + containedVariable.name
                    } else if(containedVariable instanceof Output) {
                        containedSigElement += containedVariable.name + ": Readable<" + containedVariable.type + ">"
                        containedArgElement += containedVariable.name + ": " + functArg
                        containedPrologueElement += containedVariable.name + ": __" + container.name + "." + containedVariable.name + ".get()"
                    }
                    containedArgsObject.add(containedArgElement)
                    containedSigObject.add(containedSigElement)
                    containedPrologueObject.add(containedPrologueElement) 
                }
//                pr(reactPrologue, "let " + container.name + "." + containedVariable.name + " = " + container.name + "." + "__" + containedVariable.name + ".get();")
                var reactFunctArgsElement = "{ " +  containedArgsObject.toString() + " }"
                var reactSignatureElement = "__" + container.name + ": { " + containedSigObject.toString() + " }"
                reactFunctArgs.add(reactFunctArgsElement) 
                reactSignature.add(reactSignatureElement)
                pr(reactPrologue, "let " + container.name + " = {" + containedPrologueObject + "};")
            }
            
            // Assemble reaction triggers          
            var reactionTriggers = new StringJoiner(", ") 
            for (trigger : reaction.triggers) {
                if (trigger instanceof VarRef) {
                    if (trigger.container === null) {
                        reactionTriggers.add("this." + trigger.variable.name) 
                    } else {
                        reactionTriggers.add("this." + trigger.container.name + "." + trigger.variable.name)
                    }
                } else if (trigger.startup) {
                    reactionTriggers.add("this.startup")
                } else if (trigger.shutdown) {
                    reactionTriggers.add("this.shutdown")
                }
            }
            
            
            // Write the reaction itself
            pr(reactorConstructor, "this.addReaction(")//new class<T> extends Reaction<T> {")
            reactorConstructor.indent()
            pr(reactorConstructor, "new Triggers(" + reactionTriggers + "),")
            pr(reactorConstructor, "new Args(" + reactFunctArgs + "),")
            pr(reactorConstructor, "function (" + reactSignature + ") {")
            reactorConstructor.indent()
            pr(reactorConstructor, "// =============== START react prologue")
            pr(reactorConstructor, reactPrologue)
            pr(reactorConstructor, "// =============== END react prologue")
            pr(reactorConstructor, "try {")
            reactorConstructor.indent()
            pr(reactorConstructor, removeCodeDelimiter(reaction.code))
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
                if (reaction.deadline.time.parameter !== null) {
                    deadlineArgs += "this." + reaction.deadline.time.parameter.name + ".get()"; 
                } else {
                    deadlineArgs += timeInTargetLanguage( reaction.deadline.time.time.toString(), reaction.deadline.time.unit)
                }
                pr(reactorConstructor, deadlineArgs + "," )
                pr(reactorConstructor, "function(" + reactSignature + ") {")
                reactorConstructor.indent()
                pr(reactorConstructor, "// =============== START deadline prologue")
                pr(reactorConstructor, reactPrologue)
                pr(reactorConstructor, "// =============== END deadline prologue")
                pr(reactorConstructor, "try {")
                reactorConstructor.indent()
                pr(reactorConstructor, removeCodeDelimiter(reaction.deadline.deadlineCode))
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

    /** Traverse the runtime hierarchy of reaction instances and generate code.
     *  @param instance A reactor instance.
     */
    def void generateReactorInstance(Instantiation defn) {
        var fullName = defn.name
        pr('// ************* Instance ' + fullName + ' of class ' +
            defn.reactorClass.name)
            
        var arguments = new StringJoiner(", ")
        for (parameter : defn.parameters) {
            arguments.add(removeCodeDelimiter(parameter.rhs.value))
        }

        // Get target properties for the app

        var String timeoutArg
        var isATimeoutArg = false
        
        // Timeout Property
        if (targetTimeout >= 0) {
            isATimeoutArg = true
            timeoutArg = timeInTargetLanguage(Integer.toString(targetTimeout), targetTimeoutUnit)
        }
        
        // KeepAlive Property
        var String keepAliveArg = "false"
        if (targetKeepalive) {
            keepAliveArg = "true"
        }
        
        // Fast Property
        var String fastArg = "false"
        if (targetFast) {
            fastArg = "true"
        }
        
        arguments.add("'" + fullName + "'")
        if (isATimeoutArg) {
            arguments.add(timeoutArg)
        } else {
            arguments.add("undefined")
        }
        arguments.add(keepAliveArg)
        arguments.add(fastArg)
        
        pr("let _app = new "+ fullName + "(" + arguments + ")")
    }
    
    /** Generate code to call the _start function on the main App
     *  instance to start the runtime
     *  @param instance A reactor instance.
     */
    def void generateRuntimeStart(Instantiation defn) {
        pr('// ************* Starting Runtime for ' + defn.name + ' of class ' +
            defn.reactorClass.name)
        pr("_app._start();")
    }


    // //////////////////////////////////////////
    // // Protected methods.

    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set is a set of case-insensitive
     *  strings specifying target names.
     */
    override acceptableTargets() {
        acceptableTargetSet
    }
    

    /** Generate preamble code that appears in the code generated
     *  file before anything else.
     */
    override generatePreamble() {
        super.generatePreamble
        pr(preamble)
        pr("")
        pr("Log.setGlobalLevel(Log.levels." + getLoggingLevel() + ");")
    }

    
     /** Return a string that the target language can recognize as a type
     *  for a time value. In TypeScript this is a TimeInterval.
     *  @return The string "TimeInterval"
     */
    override timeTypeInTargetLanguage() {
        "TimeInterval"
    }

    /** Given a representation of time that may possibly include units,
     *  return a string that TypeScript can recognize as a value.
     *  @param time Literal that represents a time value.
     *  @param unit Enum that denotes units
     *  @return A string, as "[ timeLiteral, TimeUnit.unit]" .
     */
    override timeInTargetLanguage(String timeLiteral, TimeUnit unit) {
        if (unit != TimeUnit.NONE) {
            "new UnitBasedTimeInterval(" + timeLiteral + ", TimeUnit." + unit + ")"
        } else {
            // The default time unit for TypeScript is msec.
            "new UnitBasedTimeInterval(" + timeLiteral + ", TimeUnit.msec)"
        }
    }

    // //////////////////////////////////////////
    // // Private methods.
    
    /** Look for a "logging" target property and return
     *  the appropriate logging level. This level is a
     *  subset of Log.level enum from the ulog module
     *  https://www.npmjs.com/package/ulog. Logged messages
     *  will display if the level of the message <= the logging level.
     *  For now, these log levels are:
     *  Error < Warn < Info < Log < Debug.
     *  The case of the level when expressed as a target property
     *  doesn't matter, but the return value from this function is
     *  in all caps.
     *  @return The logging target property's value in all caps.
     */
    private def getLoggingLevel() {
        if (targetLoggingLevel === null) {
            'ERROR'
        } else {
            targetLoggingLevel
        }
    }
    
    /** If the given filename doesn't already exist in the targetPath
     *  create it by copying over the default from /lib/TS/. Do nothing
     *  if the file already exists because we don't want to overwrite custom
     *  user-specified configurations. 
     *  @param targetPath The path to the where the file will be copied.
     *  @param filename The name of the file for which to create a default in
     *    the root of the project directory
     *  @return true if the file was created, false otherwise.
     */
    private def createDefaultConfigFile(String targetPath, String filename) {
        var File defaultFile = new File(targetPath + File.separator + filename)
        val libFile = File.separator + "lib" + File.separator + "TS" + File.separator + filename
        if(!defaultFile.exists()){
            println(filename + " does not already exist for this project."
                + " Copying over default from " + libFile)
            var fOut = new FileOutputStream(
                new File(targetPath + File.separator + filename));
            fOut.write(readFileInClasspath(libFile).getBytes())      
        } else {
            println("This project already has " + targetPath + File.separator + filename)
        }
    }


    /** Return a TS type for the type of the specified state.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "TimeInterval".
     *  If state is a parameter, get the parameter's type.
     *  If the state doesn't have a type, type it as 'unknown'
     *  @param state The state.
     *  @return The TS type.
     */
    private def getStateType(State state) {
        var String type
        if (state.isOfTimeType) {
            type = timeTypeInTargetLanguage
        } else if (state.type !== null) {
            type = removeCodeDelimiter(state.type)
        } else if (state.parameter !== null) {
            type = getParameterType(state.parameter)
        } else {
            type = 'unknown'
        }
        type
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
            return removeCodeDelimiter(action.type)
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
            return removeCodeDelimiter(port.type)
        } else {
            return "Present"    
        }
    }


    /** Return a TS type for the type of the specified parameter.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "TimeInterval".
     *  @param parameter The parameter.
     *  @return The TS type.
     */
    private def getParameterType(Parameter parameter) {
        var type = removeCodeDelimiter(parameter.type)
        if (parameter.unit != TimeUnit.NONE || parameter.isOfTimeType) {
            type = 'TimeInterval'
        }
        type
    }

    static val reactorLibPath = "." + File.separator + "reactor"
    static val timeLibPath =  "." + File.separator + "time"
    static val utilLibPath =  "." + File.separator + "util"
    val static preamble = '''
import {Args, Present, Parameter, State, Variable, Priority, Mutation, Util, Readable, Schedulable, Triggers, Writable, Named, Reaction, Action, Startup, Scheduler, Timer, Reactor, Port, OutPort, InPort, App } from "''' + reactorLibPath + '''";
import {TimeUnit,TimeInterval, UnitBasedTimeInterval, TimeInstant, Origin, getCurrentPhysicalTime } from "''' + timeLibPath + '''"
import {Log} from "''' + utilLibPath + '''"

    '''
}
