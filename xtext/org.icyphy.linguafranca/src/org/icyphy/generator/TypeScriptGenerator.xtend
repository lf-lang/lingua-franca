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
import java.util.HashMap
import java.util.HashSet
import java.util.regex.Pattern
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Target
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Variable
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Port
import java.util.StringJoiner

// FIXME: This still has a bunch of copied code from CGenerator that should be removed.

/** Generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
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
        super.doGenerate(resource, fsa, context)

        // Generate main instance, if there is one.
        if (this.main !== null) {
            generateReactorInstance(this.main)
            generateRuntimeStart(this.main)
        }
        
        // Target filename.
        val tsFilename = filename + ".ts";
        val jsFilename = filename + ".js";
        val projectPath = directory + File.separator + filename
        var srcGenPath = projectPath + File.separator + "src-gen"
        var outPath = projectPath + File.separator + "js"
        
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

        // Copy the required library files into the src-gen directory
        // so they may be compiled as part of the TypeScript project
        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "reactor.ts"));
        fOut.write(readFileInClasspath("/lib/TS/reactor.ts").getBytes())

        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "time.ts"));
        fOut.write(readFileInClasspath("/lib/TS/time.ts").getBytes())

        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "util.ts"));
        fOut.write(readFileInClasspath("/lib/TS/util.ts").getBytes())

        // Copy default versions of config files into project if
        // they don't exist.
        createDefaultConfigFile(projectPath, "package.json")
        createDefaultConfigFile(projectPath, "tsconfig.json")
        createDefaultConfigFile(projectPath, "babel.config.js")
        
        // FIXME: (IMPORTANT) at least on my mac, the instance of eclipse running this program did not have
        // the complete PATH variable needed to find the command npm. I had
        // to start my eclipse instance from a terminal so eclipse would have the correct environment
        // variables to run this command.
        
        // Install npm modules.
        println("In directory: " + projectPath)
        println("Running npm install ...")
        var installCmd = newArrayList();
        installCmd.addAll("npm", "install")
        var installBuilder = new ProcessBuilder(installCmd)
        installBuilder.directory(new File(projectPath))
        var Process installProcess = installBuilder.start()
        
        // Sleep until the modules have installed
        installProcess.waitFor()
        
        refreshProject()

        // Invoke the compiler on the generated code.
//        val relativeSrcFilename = "src-gen" + File.separator + tsFilename;
//        val relativeTSFilename = "ts" + File.separator + filename + '.js';
        
        // FIXME: Perhaps add a compileCommand option to the target directive, as in C.
        // Here, we just use a generic compile command.
        var typeCheckCommand = newArrayList()
         

        // Working example: src-gen/Minimal.ts --outDir bin --module CommonJS --target es2018 --esModuleInterop true --lib esnext,dom --alwaysStrict true --strictBindCallApply true --strictNullChecks true
        // Must compile to ES2015 or later and include the dom library.
        var tscPath = projectPath + File.separator + "node_modules/typescript/bin/tsc"


//        Working command without a tsconfig.json
//        compileCommand.addAll(tscPath,  relativeSrcFilename, 
//            "--outDir", "js", "--module", "CommonJS", "--target", "es2018", "--esModuleInterop", "true",
//             "--lib", "esnext,dom", "--alwaysStrict", "true", "--strictBindCallApply", "true",
//             "--strictNullChecks", "true");


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
            var babelPath = projectPath + File.separator + "node_modules" + File.separator + ".bin" + File.separator + "babel"
            // Working command  $./node_modules/.bin/babel src-gen --out-dir js --extensions '.ts,.tsx'
            var compileCommand = newArrayList(babelPath, "src-gen",
                "--out-dir", "js", "--extensions", ".ts")
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
       
        
        
        
//        println("Compiling with command: " + compileCommand.join(" "))
//        var builder = new ProcessBuilder(compileCommand);
//        builder.directory(new File(projectPath));
//        var process = builder.start()
//        val code = process.waitFor()
//        var stdout = readStream(process.getInputStream())
//        var stderr = readStream(process.getErrorStream())
//        if (stdout.length() > 0) {
//            println("--- Standard output from TypeScript type checker:")
//            println(stdout)
//        }
//        if (code !== 0) {
//            reportError("Compiler returns error code " + code)
//        }
//        if (stderr.length() > 0) {
//            reportError("Compiler reports errors:\n" + stderr.toString)
//        } else {
//            println("SUCCESS (type checking generated TypeScript code)")
//        }
//        
//        
//        println("SUCCESS (compiling generated TypeScript code to JavaScript)")
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

        if (reactor.preamble !== null) {
            pr("// *********** From the preamble, verbatim:")
            pr(removeCodeDelimiter(reactor.preamble.code))
            pr("\n// *********** End of preamble.")
        }

        if(reactor.isMain()){
            pr("export class " + reactor.name + " extends App {")
        } else {
            pr("export class " + reactor.name + " extends Reactor {")
        }
        
        indent()
        
        // Perhaps leverage TypeScript's mechanism for default
        // parameter values? For now it's simpler to just create
        // the reactor instance with the default argument value.
        var arguments = new StringJoiner(", ")
        if (reactor.isMain()) {
            arguments.add("name: string")
            arguments.add("timeout: TimeInterval | null")
        } else {
            arguments.add("parent:Reactor")
        }
        
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
            
        // For TS, parameters are arguments of the class constructor.
        if (reactor.isMain()) {
            arguments.add("success?: ()=> void")
            arguments.add("fail?: ()=>void")
            pr(reactorConstructor, "constructor(" + arguments + ") {")
            reactorConstructor.indent()
            pr(reactorConstructor, "super(timeout, success, fail);");
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
                var boolean defaultParameter = true
                
                // Attempt to find a non-default parameter value
                for (parameterAssignment : childReactor.parameters) {
                    if (parameterAssignment.lhs.name.equals(parameter.name)) {
                        defaultParameter = false;
                        if (parameterAssignment.rhs instanceof Parameter) {
                            childReactorArguments.add("this." + parameterAssignment.rhs.parameter.name)
                        } else if (parameterAssignment.rhs.value !== null) {
                            childReactorArguments.add(removeCodeDelimiter(parameterAssignment.rhs.value))
                        } else {
                            childReactorArguments.add(
                                timeInTargetLanguage(parameterAssignment.rhs.time.toString
                                   , parameterAssignment.rhs.unit))
                        }
                    }
                }
                // This reactor instance did not have a value for this
                // parameter set, so use the default parameter value.
                if (defaultParameter) {
                    childReactorArguments.add("undefined")
//                    if (parameter.ofTimeType) {
//                        childReactorArguments.add(
//                            timeInTargetLanguage(parameter.time.toString ,parameter.unit))
//                    } else {
//                        childReactorArguments.add(removeCodeDelimiter(parameter.value))
//                    }
                }
            }
            
            
            pr(reactorConstructor, "this." + childReactor.getName()
                + " = new " + childReactor.reactorClass.name + "("
                + childReactorArguments + ")" )
        }
       
        
        // Next handle timers.
        for (timer : reactor.timers) {
            // The startup timer is handled by default within
            // the TypeScript reactor framework. There would be a
            // duplicate startup timer if we also use the one provided
            // by LF.
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
//            var String paramType;
//            if (param.ofTimeType) {
//                paramType = "TimeInterval"
//            } else {
//                paramType = param.type
//            }
            pr(param.name + ": " + getParameterType(param) + "; // Parameter")
            pr(reactorConstructor, "this." + param.name + " = " + param.name + "; // Parameter" )
        }

        // Next handle states.
        for (state : reactor.states) {
            if (state.parameter !== null) {
                // State is a parameter
                pr(state.name + ': ' +
                getParameterType(state.parameter) +  '; // State');
                pr(reactorConstructor, "this." + state.name + " = "
                        + state.parameter.name + "; // State" )
            } else {  
                // State is a literal
                if (state.ofTimeType) {
                    // State is a time type
                    pr(state.name + ': ' +
                        timeTypeInTargetLanguage + '; // State');
                    pr(reactorConstructor, "this." + state.name + " = "
                        + timeInTargetLanguage( state.time.toString, state.unit) + "; // State" )
                } else {
                    // State is a literal 
                    pr(state.name + ': ' +
                        removeCodeDelimiter(state.type) + '; // State');
                    pr(reactorConstructor, "this." + state.name + " = "
                        + removeCodeDelimiter(state.value) + "; // State" )
                }
            }
        }
        // Next handle actions.
        for (action : reactor.actions) {
            // Shutdown actions are handled internally by the
            // TypeScript reactor framework. There would be a
            // duplicate action if we included the one generated
            // by LF.
            if (action.name != "shutdown") {
                pr(action.name + ": Action<" + action.type + ">;")
                var actionArgs = "this, Origin." + action.origin  
                if (action.delay !== null) {
                    // Actions in the TypeScript target are constructed
                    // with an optional minDelay argument which defaults to 0.
                    actionArgs+= ", " + timeInTargetLanguage(action.delay.time.toString, action.delay.unit)
                }
                pr(reactorConstructor, "this." + action.name + " = new Action<" + action.type +">(" + actionArgs  + ");")
            }
        }
        
        // Next handle inputs.
        for (input : reactor.inputs) {
            if (input.type === null) {
                reportError(input,
                    "Input is required to have a type: " + input.name)
            } else {
                pr(input.name + ": " + "InPort<" + input.type + ">;")
                pr(reactorConstructor, "this." + input.name + " = new InPort<"
                    + input.type + ">(this);")
            }
        }
        
        // Next handle outputs.
        for (output : reactor.outputs) {
            if (output.type === null) {
                reportError(output,
                    "Output is required to have a type: " + output.name)
            } else {
                pr(output.name + ": " + "OutPort<" + output.type + ">;")
                pr(reactorConstructor, "this." + output.name + " = new OutPort<"
                    + output.type + ">(this);")
            }
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

//        // Find output ports that receive data from inside reactors
//        // and put them into a HashMap for future use.
//        var outputToContainedOutput = new HashMap<Output, VarRef>();
//        for (connection : reactor.connections) {
//            // If the connection has the form c.x -> y, then it's what we are looking for.
//            if (connection.rightPort.container === null &&
//                connection.leftPort.container !== null) {
//                if (connection.rightPort.variable instanceof Output) {
//                    outputToContainedOutput.put(
//                        connection.rightPort.variable as Output,
//                        connection.leftPort
//                    )
//                } else {
//                    reportError(
//                        connection,
//                        "Expected an output port but got " +
//                            connection.rightPort.variable.name
//                    )
//                }
//            }
//        }
        
        
        // Next handle reaction instances
        for (reaction : reactor.reactions) {
            
            // Determine signature of the react function
            var reactSignature = new StringJoiner(", ")
            
            
            // Assemble react function arguments from sources and effects
            // Arguments are either elements of this reactor, or an object
            // representing a contained reactor with properties corresponding
            // to listed sources and effects.
            
            // If a source or effect is an element of this reactor, add it
            // directly to the reactFunctArgs string. If it isn't, write it 
            // into the containerToArgs map, and add it to the string later.
            var reactFunctArgs = new StringJoiner(", ")
             
            var containerToArgs = new HashMap<Instantiation, HashSet<Variable>>();
            for (source : reaction.sources) {
                if (source.container === null) {
                    var reactSignatureElement = source.variable.name + ": Readable<"
                    
                    if (source.variable instanceof Timer){
                        reactSignatureElement += "TimeInstant" 
                    } else if (source.variable instanceof Action){
                        reactSignatureElement += (source.variable as Action).type 
                    } else if (source.variable instanceof Port){
                        reactSignatureElement += (source.variable as Port).type 
                    }
                    reactSignatureElement += ">"
                    reactSignature.add(reactSignatureElement)
                    
                    reactFunctArgs.add("this." + source.variable.name)
                } else {
                    var args = containerToArgs.get(source.container)
                    if (args === null) {
                       // Create the HashSet for the container
                       args = new HashSet<Variable>();
                       containerToArgs.put(source.container, args)
                    }
                    args.add(source.variable)
//                    reactFunctArgs += "this." + source.container.name + "." + source.variable.name + ", "
                }
            }
            for (effect : reaction.effects) {
                var functArg = ""
                if (effect.container === null) {
                    var reactSignatureElement = effect.variable.name
                    if (effect.variable instanceof Timer){
                        reportError("A timer cannot be an effect of a reaction")
                    } else if (effect.variable instanceof Action){
                        reactSignatureElement += ": Schedulable<" + (effect.variable as Action).type
                    } else if (effect.variable instanceof Port){
                        reactSignatureElement += ": Writable<" + (effect.variable as Port).type
                    }
                    reactSignatureElement += ">"
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
//                    functArg = "this." + effect.container.name + "." + effect.variable.name + ", "
                }
            }
            
            // Write an object as an argument for each container
            var containers = containerToArgs.keySet()
            for (container : containers) {
                var reactFunctArgsElement = "{ "
                var reactSignatureElement = container.name + ": { " 
                var containedVariables = containerToArgs.get(container)
                for (containedVariable : containedVariables) {
                    var functArg = "this." + container.name + "." + containedVariable.name
                     
                    if (containedVariable instanceof Input) {
                        reactSignatureElement += containedVariable.name + ": Writable, "
                        reactFunctArgsElement += containedVariable.name + ": " + "this.getWritable(" + functArg + "), "
                    } else if(containedVariable instanceof Output) {
                        reactSignatureElement += containedVariable.name + ": Readable, "
                        reactFunctArgsElement += containedVariable.name + ": " + functArg + ", "
                    }
                    
                }
                reactFunctArgsElement += " }"
                reactFunctArgs.add(reactFunctArgsElement)
                reactSignatureElement += " }"
                reactSignature.add(reactSignatureElement)
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
                }     
            }
            
            // Combine reaction triggers and react function arguments            
            var reactionArguments = "this, this.check(" + reactionTriggers
                + "), this.check(" + reactFunctArgs + ")";
            
//            var deadlineClassName = reactor.name + '_d' + reactionIndex;
//            if(reaction.deadline !== null){
//                pr(deadlineClass, 'class ' + deadlineClassName + ' extends Deadline {')
//                deadlineClass.indent()
//                pr(deadlineClass, "handler(){")
//                deadlineClass.indent()
//                pr(deadlineClass, removeCodeDelimiter(reaction.deadline.deadlineCode))
//                deadlineClass.unindent()
//                pr(deadlineClass, "}")
//                deadlineClass.unindent()
//                pr(deadlineClass, "}")
//      
//            }
            
            
            // Next, handle the reaction deadline if any
//            if (reaction.deadline !== null) {
//                var deadlineName = reactor.name + '_d' + reactionIndex
//                var deadlineArgs = "this, "
//                if (reaction.deadline.time.parameter !== null) {
//                    deadlineArgs+= "this." + reaction.deadline.time.parameter.name; 
//                } else {
//                    deadlineArgs += timeInTargetLanguage( reaction.deadline.time.time.toString(), reaction.deadline.time.unit)
//                }
//                pr(reactorConstructor, "this." + reactionName + ".setDeadline( new " + deadlineName + "( " + deadlineArgs + "));")
//            }
            
            pr(reactorConstructor, "this.addReaction(new class<T> extends Reaction<T> {")
            reactorConstructor.indent()
            pr(reactorConstructor, "//@ts-ignore")  
            pr(reactorConstructor, "react(" + reactSignature + ") {")
            reactorConstructor.indent()
            pr(reactorConstructor, "var self = this.parent as " + reactor.name + ";")
            pr(reactorConstructor, removeCodeDelimiter(reaction.code))
            reactorConstructor.unindent()
            pr(reactorConstructor, "}")
            if (reaction.deadline !== null) {
                pr(reactorConstructor, "//@ts-ignore") 
                pr(reactorConstructor, "late(" + reactSignature + ") {")
                reactorConstructor.indent()
                pr(reactorConstructor, "var self = this.parent as " + reactor.name + ";")
                pr(reactorConstructor, removeCodeDelimiter(reaction.deadline.deadlineCode))
                reactorConstructor.unindent()
                pr(reactorConstructor, "}")
            }
            reactorConstructor.unindent()
            
            if (reaction.deadline !== null) {
                pr(reactorConstructor, "}(" + reactionArguments
                    + ").setDeadline(" + timeInTargetLanguage(
                    reaction.deadline.time.time.toString(), 
                    reaction.deadline.time.unit)  + "));")
            } else {
                pr(reactorConstructor, "}(" + reactionArguments + "));")                
            }
        }
        reactorConstructor.unindent()
        pr(reactorConstructor, "}")
        pr(reactorConstructor.toString())
        unindent()
        pr("}")
        pr("// =============== END reactor class " + reactor.name)
        pr("")

//      FIXME: IMPLEMENT DEADLINES
//        var reactionArray = "this._reactions = [ "
//        var reactionIndex = 0;
//        for (reaction : reactor.reactions) {
//            val reactionName = 'r' + reactionIndex;
//            val reactionClassName = reactor.name + "_" + reactionName;
//            pr(reactionName + ": " + reactionClassName + ";")
//            
//            var reactionArguments = "this, [ ";
//            for (trigger : reaction.triggers) {
//                if (trigger instanceof VarRef) {
//                    if (trigger.container === null) {
//                        reactionArguments += "this." + trigger.variable.name + ", "    
//                    } else {
//                        reactionArguments += "this." + trigger.container.name + "." + trigger.variable.name + ", ";
//                    }
//                }     
//            }
//            reactionArguments += "], ["
//            
//            for (source : reaction.sources) {
//                if (source.container === null) {
//                    reactionArguments += "this." + source.variable.name + ", "    
//                } else {
//                    reactionArguments += "this." + source.container.name + "." + source.variable.name + ", "
//                }
//                
//            }
//            reactionArguments += "], ["
//            
//            for (effect : reaction.effects) {
//                if (effect.container === null) {
//                    reactionArguments += "this." + effect.variable.name + ", "    
//                } else {
//                    reactionArguments += "this." + effect.container.name + "." + effect.variable.name + ", "
//                }
//                
//            }
//            reactionArguments +="]"
//            
//            pr(reactorConstructor, "this." + reactionName + " =  new "
//                + reactionClassName + "(" + reactionArguments + ");"
//            )
//            reactionArray += "this." + reactionName + ", "
//
//            // Next, handle deadlines for reaction instances.
//            // This must happen after reactions
//            if (reaction.deadline !== null) {
//                var deadlineName = reactor.name + '_d' + reactionIndex
//                var deadlineArgs = "this, "
//                if (reaction.deadline.time.parameter !== null) {
//                    deadlineArgs+= "this." + reaction.deadline.time.parameter.name; 
//                } else {
//                    deadlineArgs += timeInTargetLanguage( reaction.deadline.time.time.toString(), reaction.deadline.time.unit)
//                }
//                pr(reactorConstructor, "this." + reactionName + ".setDeadline( new " + deadlineName + "( " + deadlineArgs + "));")
//            }
//        
//            reactionIndex++
//        }
//        reactionArray += "];"
//        pr(reactorConstructor, reactionArray );
//              
//        reactorConstructor.unindent()
//        pr(reactorConstructor, "}")
//        pr(reactorConstructor.toString())
//        unindent()
//        pr("}")
//        pr("// =============== END reactor class " + reactor.name)
//        pr("")
//
//        // Generate reactions
//        if (!reactor.reactions.empty) {
//            pr("// =============== START reaction classes for " + reactor.name)
//            generateReactions(reactor)
//            pr("// =============== END reaction classes for " + reactor.name)
//            pr("")   
//        }
    }

    /** Generate reaction functions definition for a reactor.
     *  @param reactor The reactor.
     */
//    def generateReactions(Reactor reactor) {
//        var deadlineClass = new StringBuilder();
//        var reactions = reactor.reactions
//        var reactionIndex = 0;
//        for (reaction : reactions) {
//            pr('class ' + reactor.name + '_r' + reactionIndex + ' extends Reaction {')
//            indent()
//            
//            var deadlineClassName = reactor.name + '_d' + reactionIndex;
//            if(reaction.deadline !== null){
//                pr(deadlineClass, 'class ' + deadlineClassName + ' extends Deadline {')
//                deadlineClass.indent()
//                pr(deadlineClass, "handler(){")
//                deadlineClass.indent()
//                pr(deadlineClass, removeCodeDelimiter(reaction.deadline.deadlineCode))
//                deadlineClass.unindent()
//                pr(deadlineClass, "}")
//                deadlineClass.unindent()
//                pr(deadlineClass, "}")
//      
//            }
//            pr('react() {')
//            indent()
//            pr(removeCodeDelimiter(reaction.code))
//            unindent()
//            pr('}')
//            unindent()
//            pr('}')
//            
//            if(reaction.deadline !== null){
//                pr(deadlineClass.toString())
//            }
//            
//            reactionIndex++
//        }
//    }

    /** Traverse the runtime hierarchy of reaction instances and generate code.
     *  @param instance A reactor instance.
     */
    def void generateReactorInstance(ReactorInstance instance) {
        var reactorClass = instance.definition.reactorClass
        var fullName = instance.fullName
        pr('// ************* Instance ' + fullName + ' of class ' +
            reactorClass.name)
            
        var arguments = "";
        for (parameter : instance.parameters) {
            arguments += removeCodeDelimiter(parameter.literalValue) + ", "
        }

        var String timeoutArg = "null";
        
        for (target : resource.allContents.toIterable.filter(Target)) {
            if (target.properties !== null) {
                for (property : target.properties) {
                    if (property.name.equals("timeout")) {
                        // A timeout must be a Time, it can't be a literal
                        // (except 0 which will match as a literal).
                        // Since 0 is the only literal that corresponds to a time
                        // any other literal is an error
                        if (property.literal !== null && property.literal !=0) {
                            reportError("The timeout property only accepts time assignments.")
                        }
                        timeoutArg = timeInTargetLanguage(property.time.toString(), property.unit)
                    }
                }
            }   
        }
        
        arguments += "'" + fullName + "', " + timeoutArg
        pr("let _app" + " = new "+ fullName + "(" + arguments + ")")
    }
    
    /** Generate code to call the _start function on the main App
     *  instance to start the runtime
     *  @param instance A reactor instance.
     */
    def void generateRuntimeStart(ReactorInstance instance) {
        var reactorClass = instance.definition.reactorClass
        var fullName = instance.fullName
        pr('// ************* Starting Runtime for ' + fullName + ' of class ' +
            reactorClass.name)
        pr("_app._start();")
    }

    /** Set the reaction priorities based on dependency analysis.
     *  @param reactor The reactor on which to do this.
     */
    def void setReactionPriorities(ReactorInstance reactor) {
        // Use "reactionToReactionTName" property of reactionInstance
        // to set the levels.
        for (reactionInstance : reactor.reactions) {
            pr(reactionStructName(reactionInstance) + ".index = " +
                    reactionInstance.level + ";")
        }
        for (child : reactor.children) {
            setReactionPriorities(child)
        }
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
    }

    /** Return a unique name for the
     *  specified reaction instance.
     *  @param reaction The reaction instance.
     *  @return A name for the reaction_t struct.
     */
    protected def reactionStructName(ReactionInstance reaction) {
        reaction.uniqueID
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
    
    
    /** If the given filename doesn't already exist in the project root
     *  create it by copying over the default from /lib/TS/. Do nothing
     *  if the file already exists because we don't want to overwrite custom
     *  user-specified configurations. 
     *  @param projectPath The path to the project root.
     *  @param filename The name of the file for which to create a default in
     *    the root of the project directory
     *  @return true if the file was created, false otherwise.
     */
    private def createDefaultConfigFile(String projectPath, String filename) {
        var File defaultFile = new File(projectPath + File.separator + filename)
        val libFile = File.separator + "lib" + File.separator + "TS" + File.separator + filename
        if(!defaultFile.exists()){
            println(filename + " does not already exist for this project."
                + " Copying over default from " + libFile)
            var fOut = new FileOutputStream(
                new File(projectPath + File.separator + filename));
            fOut.write(readFileInClasspath(libFile).getBytes())      
        } else {
            println("This project already has " + projectPath + File.separator + filename)
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
    val static preamble = '''
import {Variable, Priority, VarList, Mutations, Util, Readable, Schedulable, Writable, Named, Reaction, Action, Startup, Scheduler, Timer, Reactor, Port, OutPort, InPort, App } from "''' + reactorLibPath + '''";
import {TimeUnit,TimeInterval, UnitBasedTimeInterval, TimeInstant, Origin, getCurrentPhysicalTime } from "''' + timeLibPath + '''"

    '''
}
