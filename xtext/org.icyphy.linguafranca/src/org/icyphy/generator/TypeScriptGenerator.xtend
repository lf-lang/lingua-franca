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
import java.net.URL
import java.nio.file.Paths
import java.util.HashMap
import java.util.HashSet
import java.util.regex.Pattern
import org.eclipse.core.resources.IResource
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.core.runtime.FileLocator
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
        pr(includes)

        println("Generating code for: " + resource.getURI.toString)

        super.doGenerate(resource, fsa, context)

        // Generate main instance, if there is one.
        if (this.main !== null) {
            generateReactorInstance(this.main)
        }

        // FIXME: Perhaps the following should be moved to a base class function that
        // takes an argument the filename extension.
        
        // Determine path to generated code
        val tsFilename = filename + ".ts";
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
            System.err.println(
                "ERROR: Source file protocol is not recognized: " + srcFile);
        }

        val srcPath = srcFile.substring(0, srcFile.lastIndexOf(File.separator))
        var srcGenPath = srcPath + File.separator + "src-gen"
        // FIXME: Perhaps bin is not the best name, but we need a place to put
        // the result of compiling the .ts file to .js.
        var outPath = srcPath + File.separator + "bin"

        // Create output directories if they don't yet exist
        var dir = new File(srcGenPath)
        if (!dir.exists()) dir.mkdirs()
        dir = new File(outPath)
        if (!dir.exists()) dir.mkdirs()

        // Delete source previous output the LF compiler
        var file = new File(srcPath + File.separator + tsFilename)
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

        // Copy the required library files into the target filesystem.
        /* FIXME: This is what it looks like in the CGenerator:
        fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + "reactor_common.c"));
        fOut.write(readFileInClasspath("/lib/C/reactor_common.c").getBytes())
        * 
        */

        // Trigger a refresh so Eclipse also sees the generated files in the package explorer.
        // FIXME: Move to the base class!
        if (mode == Mode.INTEGRATED) {
            // Find name of current project
            val id = "((:?[a-z]|[A-Z]|_\\w)*)";
            val pattern = Pattern.compile(
                "platform:" + File.separator + "resource" + File.separator +
                    id + File.separator);
            val matcher = pattern.matcher(code);
            var projName = ""
            if (matcher.find()) {
                projName = matcher.group(1)
            }
            try {
                val members = ResourcesPlugin.getWorkspace().root.members
                for (member : members) {
                    // Refresh current project, or simply entire workspace if project name was not found
                    if (projName == "" ||
                        projName.equals(
                            member.fullPath.toString.substring(1))) {
                        member.refreshLocal(IResource.DEPTH_INFINITE, null)
                        println("Refreshed " + member.fullPath.toString)
                    }
                }
            } catch (IllegalStateException e) {
                println("Unable to refresh workspace: " + e)
            }
        }

        // Invoke the compiler on the generated code.
        val relativeSrcFilename = "src-gen" + File.separator + tsFilename;
        val relativeBinFilename = "bin" + File.separator + filename + '.js';
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
            "--outDir", "bin", "--module", "CommonJS", "--target", "es2018", "--esModuleInterop", "true",
             "--lib", "esnext,dom", "--alwaysStrict", "true", "--strictBindCallApply", "true",
             "--strictNullChecks", "true"); //, relativeBinFilename, "--lib DOM")
        
//        val path = System.getenv("PATH");
//        println("path is: " + path); 
//        compileCommand.addAll("tsc","--version");
        println("In directory: " + srcPath)
        println("Compiling with command: " + compileCommand.join(" "))
        var builder = new ProcessBuilder(compileCommand);
        builder.directory(new File(srcPath));
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

        pr("// =============== START reactor class " + reactor.name)

        // Scan reactions.
        // Preamble code contains state declarations with static initializers.
        if (reactor.preamble !== null) {
            pr("// *********** From the preamble, verbatim:")
            pr(removeCodeDelimiter(reactor.preamble.code))
            pr("\n// *********** End of preamble.")
        }

        // Put parameters into a struct and construct the code to go
        // into the preamble of any reaction function to extract the
        // parameters from the struct.
        val argType = reactor.name.toLowerCase + "_self_t"
        // Construct the typedef for the "self" struct.
        var body = new StringBuilder()
        // Start with parameters.
        for (parameter : reactor.parameters) {
            prSourceLineNumber(parameter)
            if (getParameterType(parameter).equals("")) {
                reportError(parameter,
                    "Parameter is required to have a type: " + parameter.name)
            } else {
                pr(body,
                    getParameterType(parameter) + ' ' + parameter.name + ';');
            }
        }
        // Next handle states.
        for (state : reactor.states) {
            prSourceLineNumber(state)
            
            if (state.parameter !== null) {
                pr(body,
                removeCodeDelimiter(state.parameter.type) + ' ' + state.name + ';');
            } else {
                if (state.ofTimeType) {
                    pr(body,
                        timeTypeInTargetLanguage + ' ' + state.name + ';');
                } else {
                    pr(body,
                        removeCodeDelimiter(state.type) + ' ' + state.name +
                            ';');
                }

            }
        }
        // Next handle actions.
        for (action : reactor.actions) {
            prSourceLineNumber(action)
            // NOTE: Slightly obfuscate output name to help prevent accidental use.
            pr(body, "trigger_t* __" + action.name + ";")
        }
        // Next handle inputs.
        for (input : reactor.inputs) {
            prSourceLineNumber(input)
            if (input.type === null) {
                reportError(input,
                    "Input is required to have a type: " + input.name)
            } else {
                // NOTE: Slightly obfuscate input name to help prevent accidental use.
                pr(body,
                    removeCodeDelimiter(input.type) + '* __' + input.name +
                        ';');
                pr(body, 'bool* __' + input.name + '_is_present;');
            }
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
            prSourceLineNumber(output)
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
        // Next, handle reactions that produce outputs sent to inputs
        // of contained reactions.
        for (reaction : reactor.reactions) {
            if (reaction.effects !== null) {
                for (effect : reaction.effects) {
                    if (effect.variable instanceof Input) {
                        val port = effect.variable as Input
                        pr(
                            body,
                            removeCodeDelimiter(port.type) + ' __' +
                                effect.container.name + '_' + port.name + ';'
                        )
                        pr(
                            body,
                            'bool __' + effect.container.name + '_' +
                                port.name + '_is_present;'
                        )
                    }
                }
            }
        }
        // Finally, handle reactions that are triggered by outputs
        // of contained reactors. Have to be careful to not duplicate
        // the struct entries if multiple reactions refer to the same
        // contained output.
        var included = new HashSet<Output>
        for (reaction : reactor.reactions) {
            for (TriggerRef trigger : reaction.triggers ?: emptyList) {
                if (trigger instanceof VarRef) {
                    if (trigger.variable instanceof Output &&
                        !included.contains(trigger.variable)) {
                        // Reaction is triggered by an output of a contained reactor.
                        val port = trigger.variable as Output
                        included.add(port)
                        pr(
                            body,
                            removeCodeDelimiter(port.type) + '* __' +
                                trigger.container.name + '_' + port.name + ';'
                        )
                        pr(
                            body,
                            'bool* __' + trigger.container.name + '_' +
                                port.name + '_is_present;'
                        )
                    }
                }
            }
            // Handle reading (but not triggered by) outputs of contained reactors.
            for (source : reaction.sources ?: emptyList) {
                if (source.variable instanceof Output &&
                    !included.contains(source.variable)) {
                    // Reaction reads an output of a contained reactor.
                    val port = source.variable as Output
                    included.add(port)
                    pr(
                        body,
                        removeCodeDelimiter(port.type) + '* __' +
                            source.container.name + '_' + port.name + ';'
                    )
                    pr(
                        body,
                        'bool* __' + source.container.name + '_' + port.name +
                            '_is_present;'
                    )
                }
            }
        }

        if (body.length > 0) {
            selfStructType(reactor)
            pr("typedef struct {")
            indent()
            pr(body.toString)
            unindent()
            pr("} " + argType + ";")
        }

        // Generate reactions
        generateReactions(reactor)
        pr("// =============== END reactor class " + reactor.name)
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
    /** Return a unique name for the reaction_t struct for the
     *  specified reaction instance.
     *  @param reaction The reaction instance.
     *  @return A name for the reaction_t struct.
     */
    protected def reactionStructName(ReactionInstance reaction) {
        reaction.uniqueID
    }

    // //////////////////////////////////////////
    // // Private methods.

    /** Return a C type for the type of the specified parameter.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "interval_t".
     *  @param parameter The parameter.
     *  @return The C type.
     */
    private def getParameterType(Parameter parameter) {
        var type = removeCodeDelimiter(parameter.type)
        if (parameter.unit != TimeUnit.NONE || parameter.isOfTimeType) {
            type = 'interval_t'
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

    val static includes = '''
'use strict';

import {Reactor, Trigger, Reaction, Timer, Action,  App} from '../reactor';
import {TimeInterval, TimeUnit, numericTimeSum } from "../time"

    '''

}
