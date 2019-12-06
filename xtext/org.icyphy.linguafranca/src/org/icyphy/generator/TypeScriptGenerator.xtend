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
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef

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

        var srcGenPath = directory + File.separator + "src-gen"
        var outPath = directory + File.separator + "js"

        // Create output directories if they don't yet exist
        var dir = new File(srcGenPath)
        if (!dir.exists()) dir.mkdirs()
        dir = new File(outPath)
        if (!dir.exists()) dir.mkdirs()

        // Delete source previous output the LF compiler
        var file = new File(srcGenPath + File.separator + tsFilename)
        if (file.exists) {
            file.delete
        }

        // Delete .js previously output by TypeScript compiler
        file = new File(outPath + File.separator + filename)
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


        refreshProject()

        // Invoke the compiler on the generated code.
        val relativeSrcFilename = "src-gen" + File.separator + tsFilename;
        val relativeTSFilename = "ts" + File.separator + filename + '.js';
        // FIXME: Perhaps add a compileCommand option to the target directive, as in C.
        // Here, we just use a generic compile command.
        var compileCommand = newArrayList()
         
        // FIXME: The following example command has some problems. First, it creates an
        // entire project structure inside the bin directory. Second, it is probably better
        // to directly expose a tsconfig.json to the programmer so they can choose how their
        // reactors are generated. Third, the user must have Reactor.ts, time.ts, and util.ts in
        // the src-gen directory. Fourth, the user must have already run $npm install microtimer and nanotimer
        // in the src-gen directory. Fifth, the name of generated file's name isn't controlled
        // by this command -- it's just automatically the original tsFilename with a .js extension.
        // Sixth (IMPORTANT) at least on my mac, the instance of eclipse running this program did not have
        // the complete PATH variable needed to find the command tsc from /usr/local/bin/. I had
        // to start my eclipse instance from a terminal so eclipse would have the correct environment
        // variables to run this command.

        // Working example: src-gen/Minimal.ts --outDir bin --module CommonJS --target es2018 --esModuleInterop true --lib esnext,dom --alwaysStrict true --strictBindCallApply true --strictNullChecks true
        // Must compile to ES2015 or later and include the dom library.
        compileCommand.addAll("tsc",  relativeSrcFilename, 
            "--outDir", "js", "--module", "CommonJS", "--target", "es2018", "--esModuleInterop", "true",
             "--lib", "esnext,dom", "--alwaysStrict", "true", "--strictBindCallApply", "true",
             "--strictNullChecks", "true"); //, relativeBinFilename, "--lib DOM")
        
//        val path = System.getenv("PATH");
//        println("path is: " + path); 
//        compileCommand.addAll("tsc","--version");
        println("In directory: " + directory)
        println("Compiling with command: " + compileCommand.join(" "))
        var builder = new ProcessBuilder(compileCommand);
        builder.directory(new File(directory));
        var process = builder.start()
        // FIXME: The following doesn't work. Somehow, the command to
        // run the generated code gets executed before the following is printed!
        val code = process.waitFor()
        var stdout = readStream(process.getInputStream())
        var stderr = readStream(process.getErrorStream())
        if (stdout.length() > 0) {
            println("--- Standard output from TypeScript compiler:")
            println(stdout)
        }
        if (code !== 0) {
            reportError("Compiler returns error code " + code)
        }
        if (stderr.length() > 0) {
            reportError("Compiler reports errors:\n" + stderr.toString)
        } else {
            println("SUCCESS (compiling generated TypeScript code to JavaScript)")
        }
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
            pr("class " + reactor.name + " extends App {")
        } else {
            pr("class " + reactor.name + " extends Reactor {")
        }
        
        indent()
        
        // Perhaps leverage TypeScript's mechanism for default
        // parameter values? For now it's simpler to just create
        // the reactor instance with the default argument value.
        var arguments = "";  
//        for (parameter : reactor.parameters) {
//            if (getParameterType(parameter).equals("")) {
//                reportError(parameter,
//                    "Parameter is required to have a type: " + parameter.name)
//            } else {
//                arguments +=
//                    parameter.name + ": " + getParameterType(parameter) + ", ";
//            }
//        }
        
        
        // For TS, parameters are arguments of the class constructor.
        if(reactor.isMain()){
            pr(reactorConstructor, "constructor(" + arguments 
                + "timeout: TimeInterval | null, name?: string) {"
            )
           
            reactorConstructor.indent()
            pr(reactorConstructor, "super(timeout, name);");
            
        } else {
            pr(reactorConstructor, "constructor(" + arguments 
                + "parent:Reactor, name?: string) {"
            )
           
            reactorConstructor.indent()
            pr(reactorConstructor, "super(parent, name);");
        }
        
        // Next handle child reactors instantiations
        for (childReactor : reactor.instantiations ) {
            pr(childReactor.getName() + ": " + childReactor.reactorClass.name )
            
            var childReactorArguments = new StringBuffer();
        
            // Iterate through parameters in the order they appear in the
            // reactor class, find the matching parameter assignments in
            // the reactor instance, and write the corresponding parameter
            // value as an argument for the TypeScript constructor 
//            for (parameter : childReactor.reactorClass.parameters ){
//                for (parameterAssignment : childReactor.parameters){
//                    if(parameterAssignment.lhs.equals(parameter) ){
//                        childReactorArguments.append(parameterAssignment.rhs)
//                        childReactorArguments.append(", ")
//                    }
//                }
//            }
            
            // These arguments are always the last of a TypeScript reactor constructor
            childReactorArguments.append("this, " +  "'" + reactor.name + "/" + childReactor.name + "'");
            
            pr(reactorConstructor, "this." + childReactor.getName()
                + " = new " + childReactor.reactorClass.name + "("
                + childReactorArguments.toString() + ")" )
        }
       
        
        // Next handle timers.
        for (timer : reactor.timers) {
            var String period;
            var String offset; 
            if(timer.getPeriod() === null){
                period = "0";
            } else {
                var periodUnit = timer.getPeriod().getUnit();
                var periodTime = timer.getPeriod().getTime();
                if(periodUnit === TimeUnit.NONE){
                    // The default time unit for TypeScript is msec.
                    period = "[" + periodTime + ",TimeUnit.msec]"
                }
                else{
                    period = "[" + periodTime + ",TimeUnit." + periodUnit +"]"
                }
            }
            
            if(timer.getOffset() === null){
                offset = "0";
            } else {
                var offsetUnit = timer.getOffset().getUnit();
                var offsetTime = timer.getOffset().getTime();
                if(offsetUnit === TimeUnit.NONE){
                    // The default time unit for TypeScript is msec.
                    offset = "[" + offsetTime + ",TimeUnit.msec]"
                }
                else{
                    offset = "[" + offsetTime + ",TimeUnit." + offsetUnit +"]"
                }
            }

            pr(timer.getName() + ": Timer;")
            pr(reactorConstructor, "this." + timer.getName()
                + " = new Timer(this, " + period + ","+ offset + ");")
        }
        
        // FIXME Handle shutdown triggers
        // The startup trigger is a special timer named _startup
//        pr("_startupTimer: Timer;");
//        pr(reactorConstructor, "this._startupTimer = new Timer(this, 0, 0);");
//        
        
        // FIXME: delete this section. I uncommented parts to keep the rest compiling.
//        // Put parameters into a struct and construct the code to go
//        // into the preamble of any reaction function to extract the
//        // parameters from the struct.
        val argType = reactor.name.toLowerCase + "_self_t"
//        // Construct the typedef for the "self" struct.
        var body = new StringBuilder()
//        // Start with parameters.
//        for (parameter : reactor.parameters) {
//            prSourceLineNumber(parameter)
//            if (getParameterType(parameter).equals("")) {
//                reportError(parameter,
//                    "Parameter is required to have a type: " + parameter.name)
//            } else {
//                pr(body,
//                    getParameterType(parameter) + ' ' + parameter.name + ';');
//            }
//        }

        // Next handle states.
        for (state : reactor.states) {
            if (state.parameter !== null) {
                pr( state.name + ': ' +
                removeCodeDelimiter(state.parameter.type) +  ';');
            } else {
                if (state.ofTimeType) {
                    pr(state.name + ': ' +
                        timeTypeInTargetLanguage + ';');
                    pr(reactorConstructor, "this." + state.name + " = "
                        + timeInTargetLanguage( state.time.toString, state.unit) + ";" )
                } else {
                    pr(state.name + ': ' +
                        removeCodeDelimiter(state.type) + ';');
                    pr(reactorConstructor, "this." + state.name + " = "
                        + state.value + ";" )
                }
            }
        }
        // Next handle actions.
        for (action : reactor.actions) {
            pr(action.name + ": Action<" + action.type + ">;")
            var actionArgs = "this, TimelineClass." + action.origin 
            
            if(action.delay !== null){
                // Actions in the TypeScript target are constructed
                // with an optional minDelay argument which defaults to 0.
                actionArgs+= ", " + action.delay
            }
            pr(reactorConstructor, "this." + action.name + " = new Action<" + action.type +">(" + actionArgs  + ");")
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
            if(connection.leftPort.container !== null ){
                leftPortName += connection.leftPort.container.name + "."
            }
            leftPortName += connection.leftPort.variable.name 
            
            var rightPortName = ""
            if(connection.rightPort.container !== null ){
                rightPortName += connection.rightPort.container.name + "."
            }
            rightPortName += connection.rightPort.variable.name 
            
            pr(reactorConstructor, "this." + leftPortName + ".connect(this." + rightPortName + ");")
        }

        // Find output ports that receive data from inside reactors
        // and put them into a HashMap for future use.
        var outputToContainedOutput = new HashMap<Output, VarRef>();
        for (connection : reactor.connections) {
            // If the connection has the form c.x -> y, then it's what we are looking for.
            if (connection.rightPort.container === null &&
                connection.leftPort.container !== null) {
                if (connection.rightPort.variable instanceof Output) {
                    outputToContainedOutput.put(
                        connection.rightPort.variable as Output,
                        connection.leftPort
                    )
                } else {
                    reportError(
                        connection,
                        "Expected an output port but got " +
                            connection.rightPort.variable.name
                    )
                }
            }
        }

        // Next handle outputs.
        for (output : reactor.outputs) {
//            prSourceLineNumber(output)
            if (output.type === null) {
                reportError(output,
                    "Output is required to have a type: " + output.name)
            } else {
                // NOTE: Slightly obfuscate output name to help prevent accidental use.
                pr(body,
                    removeCodeDelimiter(output.type) + ' __' + output.name +
                        ';')
                pr(body, 'bool __' + output.name + '_is_present;')
                // If there are contained reactors that send data via this output,
                // then create a place to put the pointers to the sources of that data.
                var containedSource = outputToContainedOutput.get(output)
                if (containedSource !== null) {
                    pr(body,
                        removeCodeDelimiter(output.type) + '* __' +
                            output.name + '_inside;')
                    pr(body, 'bool* __' + output.name + '_inside_is_present;')
                }
            }
        }
        
        
        
        // Next handle reaction instances
        var reactionArray = "this._reactions = [ "
        var reactionIndex = 0;
        for (reaction : reactor.reactions) {
            val reactionName = 'r' + reactionIndex;
            val reactionClassName = reactor.name + "_" + reactionName;
            pr(reactionName + ": " + reactionClassName + ";")
            
            var reactionArguments = "this, [ ";
            for(trigger : reaction.triggers ){
                if(trigger instanceof VarRef){
                    var triggerContainerName = ""
                    if(trigger.container === null){
                        triggerContainerName = "this";
                    } else {
                        triggerContainerName = trigger.container.name;
                    }
                    
                    reactionArguments += triggerContainerName + "." + trigger.variable.name + ", "
                } 
//                else {
//                    if( trigger.startup){
//                        reactionArguments += "this._startupTimer, "
//                    }
//                }      
            }
            reactionArguments += "], ["
            
            for(source : reaction.sources ){
                if(source.container === null ){
                    reactionArguments += "this." + source.variable.name + ", "    
                } else {
                    reactionArguments += "this." + source.container.name + "." + source.variable.name + ", "
                }
                
            }
            reactionArguments += "], ["
            
            for(effect : reaction.effects ){
                if(effect.container === null){
                    reactionArguments += "this." + effect.variable.name + ", "    
                } else {
                    reactionArguments += "this." + effect.container.name + "." + effect.variable.name + ", "
                }
                
            }
            reactionArguments +="]"
            
            pr(reactorConstructor, "this." + reactionName + " =  new "
                + reactionClassName + "(" + reactionArguments + ");"
            )
            reactionArray += "this." + reactionName + ", "
            reactionIndex++
        }
        reactionArray += "];"
        pr(reactorConstructor, reactionArray );
        
//        // Next, handle reactions that produce outputs sent to inputs
//        // of contained reactions.
//        for (reaction : reactor.reactions) {
//            if (reaction.effects !== null) {
//                for (effect : reaction.effects) {
//                    if (effect.variable instanceof Input) {
//                        val port = effect.variable as Input
//                        pr(
//                            body,
//                            removeCodeDelimiter(port.type) + ' __' +
//                                effect.container.name + '_' + port.name + ';'
//                        )
//                        pr(
//                            body,
//                            'bool __' + effect.container.name + '_' +
//                                port.name + '_is_present;'
//                        )
//                    }
//                }
//            }
//        }
//        // Finally, handle reactions that are triggered by outputs
//        // of contained reactors. Have to be careful to not duplicate
//        // the struct entries if multiple reactions refer to the same
//        // contained output.
//        var included = new HashSet<Output>
//        for (reaction : reactor.reactions) {
//            for (TriggerRef trigger : reaction.triggers ?: emptyList) {
//                if (trigger instanceof VarRef) {
//                    if (trigger.variable instanceof Output &&
//                        !included.contains(trigger.variable)) {
//                        // Reaction is triggered by an output of a contained reactor.
//                        val port = trigger.variable as Output
//                        included.add(port)
//                        pr(
//                            body,
//                            removeCodeDelimiter(port.type) + '* __' +
//                                trigger.container.name + '_' + port.name + ';'
//                        )
//                        pr(
//                            body,
//                            'bool* __' + trigger.container.name + '_' +
//                                port.name + '_is_present;'
//                        )
//                    }
//                }
//            }
//            // Handle reading (but not triggered by) outputs of contained reactors.
//            for (source : reaction.sources ?: emptyList) {
//                if (source.variable instanceof Output &&
//                    !included.contains(source.variable)) {
//                    // Reaction reads an output of a contained reactor.
//                    val port = source.variable as Output
//                    included.add(port)
//                    pr(
//                        body,
//                        removeCodeDelimiter(port.type) + '* __' +
//                            source.container.name + '_' + port.name + ';'
//                    )
//                    pr(
//                        body,
//                        'bool* __' + source.container.name + '_' + port.name +
//                            '_is_present;'
//                    )
//                }
//            }
//        }

//        if (body.length > 0) {
//            selfStructType(reactor)
//            pr("typedef struct {")
//            indent()
//            pr(body.toString)
//            unindent()
//            pr("} " + argType + ";")
//        }
        
        
        reactorConstructor.unindent()
        pr(reactorConstructor, "}")
        pr(reactorConstructor.toString())
        unindent()
        pr("}")
        pr("// =============== END reactor class " + reactor.name)
        pr("")

        // Generate reactions
        pr("// =============== START reaction classes " + reactor.name)
        generateReactions(reactor)
        pr("// =============== END reaction classes " + reactor.name)
        pr("")

    }

    /** Generate reaction functions definition for a reactor.
     *  These functions have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reactor The reactor.
     */
    def generateReactions(Reactor reactor) {
        var reactions = reactor.reactions
        var reactionIndex = 0;
        for (reaction : reactions) {
            pr('class ' + reactor.name + '_r' + reactionIndex + ' extends Reaction {')
            indent()
            pr('react() {')
            indent()
            pr(removeCodeDelimiter(reaction.code))
            unindent()
            pr('}')
            unindent()
            pr('}')
            reactionIndex++
        }
    }

    /** Return the unique name for the "self" struct of the specified
     *  reactor instance from the instance ID.
     *  @param instance The reactor instance.
     *  @return The name of the self struct.
     */
    static def selfStructName(ReactorInstance instance) {
        return instance.uniqueID + "_self"
    }

    /** Construct a unique type for the "self" struct of the specified
     *  reactor class from the reactor class.
     *  @param instance The reactor instance.
     *  @return The name of the self struct.
     */
    def selfStructType(Reactor reactor) {
        return reactor.name.toLowerCase + "_self_t"
    }

    /** Traverse the runtime hierarchy of reaction instances and generate code.
     *  @param instance A reactor instance.
     */
    def void generateReactorInstance(ReactorInstance instance) {
        var reactorClass = instance.definition.reactorClass
        var fullName = instance.fullName
        pr('// ************* Instance ' + fullName + ' of class ' +
            reactorClass.name)
            
//        var arguments = "";
//        for (parameter : instance.parameters) {
//            arguments += parameter.literalValue + ", "
//        }

        // FIXME: hardcoding a 3 second timeout because
        // I don't know how to get this dynamically right now.
        // Get this from a command line argument?
        var arguments = "[3, TimeUnit.secs], '" + fullName + "'" 
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
        // FIXME: hardcoding success and failure callbacks that do nothing
        // because I haven't yet figured out how to get these yet
        pr("_app._start(() => null, ()=> null);")
    }

    /** Set the reaction priorities based on dependency analysis.
     *  @param reactor The reactor on which to do this.
     */
    def void setReactionPriorities(ReactorInstance reactor) {
        // Use "reactionToReactionTName" property of reactionInstance
        // to set the levels.
        for (reactionInstance : reactor.reactions) {
            pr(
                reactionStructName(reactionInstance) + ".index = " +
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

    /** Return a unique name for the reaction_t struct for the
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
        if (unit != TimeUnit.NONE){
            "[" + timeLiteral + ", " + "TimeUnit." + unit + "]"
        } else {
            "[" + timeLiteral + ", " + "TimeUnit.msec]"
        }
        
    }

    // //////////////////////////////////////////
    // // Private methods.

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

    // Print the #line compiler directive with the line number of
    // the most recently used node.
    private def prSourceLineNumber(EObject eObject) {
        var node = NodeModelUtils.getNode(eObject)
        if (node !== null) {
            pr("#line " + node.getStartLine() + ' "' + resource.getURI() + '"')
        }
    }

    static val reactorLibPath = "." + File.separator + "reactor"
    static val timeLibPath =  "." + File.separator + "time"
    val static preamble = '''
'use strict';

import {Reactor, Trigger, Reaction, Timer, Action, App, InPort, OutPort} from "''' + reactorLibPath + '''";
import {TimeInterval, TimeInstant, TimeUnit, TimelineClass, numericTimeSum, numericTimeDifference } from "''' + timeLibPath + '''"

    '''

}
