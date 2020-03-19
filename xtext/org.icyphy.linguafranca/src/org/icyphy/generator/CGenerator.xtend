/* Generator for C target. */

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
import java.math.BigInteger
import java.util.ArrayList
import java.util.Collection
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.regex.Pattern
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.QueuingPolicy
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.State
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

/** Generator for C target.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Mehrdad Niknami <mniknami@berkeley.edu>}
 *  @author{Chris Gill, <cdgill@wustl.edu>}
 */
class CGenerator extends GeneratorBase {
    
    ////////////////////////////////////////////
    //// Private variables
    
    // Set of acceptable import targets includes only C.
    val acceptableTargetSet = newHashSet('C')
    
    // Additional sources to add to the compile command if appropriate.
    var compileAdditionalSources = null as ArrayList<String>

    // Additional libraries to add to the compile command using the "-l" command-line option.
    var compileLibraries = null as ArrayList<String>

    // List of deferred assignments to perform in initialize_trigger_objects.
    var deferredInitialize = new LinkedList<InitializeRemoteTriggersTable>()
    
    // Place to collect code to initialize the trigger objects for all reactors.
    var initializeTriggerObjects = new StringBuilder()

    /** The main (top-level) reactor instance. */
    protected ReactorInstance main
    
    // The command to run the generated code if specified in the target directive.
    var runCommand = new ArrayList<String>()

    // Place to collect shutdown action instances.
    var shutdownActionInstances = new LinkedList<ActionInstance>()

    // Place to collect code to execute at the start of a time step.
    var startTimeStep = new StringBuilder()

    // Place to collect code to initialize timers for all reactors.
    var startTimers = new StringBuilder()

    // For each reactor, we collect a set of input and parameter names.
    var triggerCount = 0

    ////////////////////////////////////////////
    //// Public methods

    /** Generate C code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
        
        // The following generates code needed by all the reactors.
        super.doGenerate(resource, fsa, context)
        
        // Create the output directories if they don't yet exist.
        var srcGenPath = directory + File.separator + "src-gen"
        var outPath = directory + File.separator + "bin"
        var dir = new File(srcGenPath)
        if (!dir.exists()) dir.mkdirs()
        dir = new File(outPath)
        if (!dir.exists()) dir.mkdirs()

        // Copy the required library files into the target filesystem.
        // This will overwrite previous versions.
        var files = newArrayList("reactor_common.c", "reactor.h", "pqueue.c", "pqueue.h")
        if (targetThreads === 0) {
            files.add("reactor.c")
        } else {
            files.add("reactor_threaded.c")
        }
        // If there are federates, copy the required files for that.
        // Also, create the RTI C file.
        if (federates.length > 1) {
            files.addAll("util.c", "rti.c", "rti.h", "federate.c")
            createFederateRTI()
        }
        
        for (file : files) {
            var fOut = new FileOutputStream(
                new File(srcGenPath + File.separator + file));
            fOut.write(readFileInClasspath("/lib/C/" + file).getBytes())
            fOut.close()
        }

        // Perform distinct code generation into distinct files for each federate.
        val baseFilename = filename
        
        var commonCode = code;
        var commonStartTimers = startTimers;
        for (federate : federates) {
            // Only generate one output if there is no federation.
            if (!federate.isSingleton) {
                filename = baseFilename + '_' + federate.name
                // Clear out previously generated code.
                code = new StringBuilder(commonCode)
                initializeTriggerObjects = new StringBuilder()
                startTimeStep = new StringBuilder()
                startTimers = new StringBuilder(commonStartTimers)
                // This should go first in the start_timers function.
                pr(startTimers, 'synchronize_with_other_federates('
                    + federate.id
                    + ', "'
                    + federationRTIHost 
                    + '", ' + federationRTIPort
                    + ");"
                )
            }
        
            // Build the instantiation tree if a main reactor is present.
            if (this.mainDef !== null) {
                generateReactorFederated(this.mainDef.reactorClass, federate)
                if (this.main === null) {
                    // Recursively build instances. This is done once because
                    // it is the same for all federates.
                    this.main = new ReactorInstance(mainDef, null, this) 
                }   
            }
        
            if (main !== null && main.chainIDWidth > 64) {
                throw new Exception("Currently no support for programs with more than 64 branches in the dependency tree. ")
            }
        
            // Derive target filename from the .lf filename.
            val cFilename = filename + ".c";

            // Delete source previously produced by the LF compiler.
            var file = new File(srcGenPath + File.separator + cFilename)
            if (file.exists) {
                file.delete
            }

            // Delete binary previously produced by the C compiler.
            file = new File(outPath + File.separator + filename)
            if (file.exists) {
                file.delete
            }

            // Generate main instance, if there is one.
            // Note that any main reactors in imported files are ignored.        
            if (this.main !== null) {
                generateReactorInstance(this.main, federate)

                // Generate function to set default command-line options.
                // A literal array needs to be given outside any function definition,
                // so start with that.
                if (runCommand.length > 0) {
                    pr('char* __default_argv[] = {"' + runCommand.join('", "') + '"};')
                }
                pr('void __set_default_command_line_options() {\n')
                indent()
                if (runCommand.length > 0) {
                    pr('default_argc = ' + runCommand.length + ';')
                    pr('default_argv = __default_argv;')
                }
                unindent()
                pr('}\n')

                // Generate function to initialize the trigger objects for all reactors.
                pr('void __initialize_trigger_objects() {\n')
                indent()
                pr(initializeTriggerObjects.toString)
                doDeferredInitialize(federate)
                setReactionPriorities(main, federate)
                unindent()
                pr('}\n')

                // Generate function to start timers for all reactors.
                pr("void __start_timers() {")
                indent()
                pr(startTimers.toString)
                unindent()
                pr("}")

                // Generate function to execute at the start of a time step.
                pr('void __start_time_step() {\n')
                indent()
                pr(startTimeStep.toString)
                unindent()
                pr('}\n')

                // Generate function to return a pointer to the action trigger_t
                // that handles incoming network messages destined to the specified
                // port. This will only be used if there are federates.
                pr('trigger_t* __action_for_port(int port_id) {\n')
                indent()
                if (federate.networkMessageActions.size > 0) {
                    // Create a static array of trigger_t pointers.
                    // networkMessageActions is a list of Actions, but we
                    // need a list of trigger struct names for ActionInstances.
                    // There should be exactly one ActionInstance in the
                    // main reactor for each Action.
                    val triggers = new LinkedList<String>()
                    for (action : federate.networkMessageActions) {
                        // Find the corresponding ActionInstance.
                        val actionInstance = main.getActionInstance(action)
                        triggers.add(triggerStructName(actionInstance))
                    }
                    pr('''
                    static trigger_t* action_table[] = {
                        &«triggers.join(', &')»
                    };
                    if (port_id < «federate.networkMessageActions.size») {
                        return action_table[port_id];
                    } else {
                        return NULL;
                    }
                    ''')
                } else {
                    pr('return NULL;')
                }
                unindent()
                pr('}\n')

                // Generate function to schedule shutdown actions if any
                // reactors have reactions to shutdown.
                pr('bool __wrapup() {\n')
                indent()
                for (instance : shutdownActionInstances) {
                    pr('schedule(&' + triggerStructName(instance) + ', 0LL, NULL);')
                }
                if (shutdownActionInstances.length === 0) {
                    pr('return false;')
                } else {
                    pr('return true;')
                }
                unindent()
                pr('}\n')
            }

            // Write the generated code to the output file.
            var fOut = new FileOutputStream(
                new File(srcGenPath + File.separator + cFilename));
            fOut.write(getCode().getBytes())
            fOut.close()
        }
        // Restore the base filename.
        filename = baseFilename
        
        // In case we are in Eclipse, make sure the generated code is visible.
        refreshProject()

        compileCode()
    }
    
    /** Invoke the compiler on the generated code. */
    def compileCode() {
        // If the target directive gives a compile command, use that.
        if (targetCompile !== null) {
            val command = targetCompile.split(' ')
            var compileCommand = newArrayList
            compileCommand.addAll(command)
            executeCommand(compileCommand, directory)
            return
        }
        // If there is more than one federate, compile each one.
        val baseFilename = filename
        for (federate : federates) {
            // Empty string means no federates were defined, so we only
            // compile one file.
            if (!federate.isSingleton) {
                filename = baseFilename + '_' + federate.name
            }
            // Derive target filename from the .lf filename.
            val cFilename = filename + ".c";            
            val relativeSrcFilename = "src-gen" + File.separator + cFilename;
            val relativeBinFilename = "bin" + File.separator + filename;
            
            var compileCommand = newArrayList
            compileCommand.addAll("gcc", "-O2", relativeSrcFilename)
            if (compileAdditionalSources !== null) {
                compileCommand.addAll(compileAdditionalSources)
            }
            if (compileLibraries !== null) {
                compileCommand.addAll(compileLibraries)
            }
            compileCommand.addAll("-o", relativeBinFilename)
            // If threaded computation is requested, add a -pthread option.
            if (targetThreads !== 0) {
                compileCommand.add("-pthread")
            }
            // If there is no main reactor, then use the -c flag to prevent linking from occurring.
            // FIXME: we could add a `-c` flag to `lfc` to make this explicit in stand-alone mode.
            // Then again, I think this only makes sense when we can do linking.
            // In any case, a warning is helpful to draw attention to the fact that no binary was produced.
            if (main === null) {
                compileCommand.add("-c")
                if (mode === Mode.STANDALONE) {
                    reportError(
                        "ERROR: Did not output executable; no main reactor found.")
                }
            }
            executeCommand(compileCommand, directory)
        }
        // Also compile the RTI if there is more than one federate.
        if (federates.length > 1) {
            filename = baseFilename + '_RTI'
            var compileCommand = newArrayList
            compileCommand.addAll("gcc", "-O2", 
                    "src-gen" + File.separator + filename + '.c',
                    "-o", "bin" + File.separator + filename,
                    "-pthread")
            executeCommand(compileCommand, directory)
        }
    }

    // //////////////////////////////////////////
    // // Code generators.
    
    /** If any input is conveyed using the token_t struct,
     *  then add a reaction at the end of the list of reactions
     *  that is triggered by that input that decrements the
     *  reference count and frees allocated memory when the
     *  reference count hits zero.
     */
    def addReferenceCountReaction(Reactor reactor) {
        var triggers = newArrayList
        var body = new StringBuilder
        // This is a bit of a hack, but preface the body with
        // a comment that the preamble should not not be included.
        // Including the preamble could result in multiple writable
        // copies being made if an input is mutable.
        pr(body, CGenerator.DISABLE_REACTION_INITIALIZATION_MARKER)
        for(input : reactor.inputs) {
            if (isTokenType(input.type)) {
                triggers.add(input)
                pr(body, 'if(self->__' + input.name 
                    + '_is_present) {__done_using(self->__' + input.name
                    + ');}'
                )
            }
        }
        if (!triggers.isEmpty) {
            var reaction = LinguaFrancaFactory.eINSTANCE.createReaction
            // Populate the triggers for the reaction.
            for(input: triggers) {
                var variableReference = LinguaFrancaFactory.eINSTANCE.createVarRef()
                variableReference.setVariable(input)
                reaction.triggers.add(variableReference)
            }
            reaction.setCode(body.toString)
            reactor.reactions.add(reaction)
        }
    }
    
    /** Create a file  */
    def createFederateRTI() {
        // Derive target filename from the .lf filename.
        val cFilename = filename + "_RTI.c";
        var srcGenPath = directory + File.separator + "src-gen"
        var outPath = directory + File.separator + "bin"

        // Delete source previously produced by the LF compiler.
        var file = new File(srcGenPath + File.separator + cFilename)
        if (file.exists) {
            file.delete
        }

        // Delete binary previously produced by the C compiler.
        file = new File(outPath + File.separator + filename)
        if (file.exists) {
            file.delete
        }
        
        var fOut = new FileOutputStream(
                new File(srcGenPath + File.separator + cFilename));
        fOut.write('''
#define NUMBER_OF_FEDERATES «federates.length»
#include "rti.c"
int main(int argc, char* argv[]) {
    start_rti(«federationRTIPort»);
}
        '''.toString().getBytes())
        fOut.close()
    }
    
    /** Generate a reactor class definition. This version unconditionally
     *  generates the reactor class definition, regardless of the
     *  federate structure.
     *  @param reactor The parsed reactor data structure.
     */
    override generateReactor(Reactor reactor) {
        generateReactorFederated(reactor, null)
    }
    
    /** Generate a reactor class definition for the specified
     *  federate. If the reactor is the main reactor, then
     *  the generated code may be customized. Specifically,
     *  if the main reactor has reactions, these reactions
     *  will not be generated if are triggered by or send
     *  data to contained reactors that are in the federate.
     *  @param reactor The parsed reactor data structure.
     *  @param federate A federate name, or null
     *   to unconditionally generate.
     */
    def generateReactorFederated(Reactor reactor, FederateInstance federate) {
        super.generateReactor(reactor)

        // Special Timer and Action for startup and shutdown, if they occur.
        // Only one of each of these should be created even if multiple
        // reactions are triggered by them.
        var Timer timer = null
        var Action action = null
        var factory = LinguaFrancaFactory.eINSTANCE
        if (reactor.reactions !== null) {
            for (Reaction reaction : reactor.reactions) {
                // If the reaction triggers include 'startup' or 'shutdown',
                // then create Timer and TimerInstance objects named 'startup'
                // or Action and ActionInstance objects named 'shutdown'.
                // Using a Timer for startup means that the target-specific
                // code generator doesn't have to do anything special to support this.
                // However, for 'shutdown', the target-specific code generator
                // needs to check all reaction instances for a shutdownActionInstance
                // and schedule that action before shutting down the program.
                // These get inserted into both the ECore model and the
                // instance model.
                var TriggerRef startupTrigger = null;
                var TriggerRef shutdownTrigger = null;
                for (trigger : reaction.triggers) {
                    if (trigger.isStartup) {
                        startupTrigger = trigger
                        if (timer === null) {
                            timer = factory.createTimer
                            timer.name = LinguaFrancaPackage.Literals.
                                TRIGGER_REF__STARTUP.name
                            timer.offset = factory.createTimeOrValue
                            timer.offset.time = 0
                            timer.period = factory.createTimeOrValue
                            timer.period.time = 0
                            reactor.timers.add(timer)
                        }
                    } else if (trigger.isShutdown) {
                        shutdownTrigger = trigger
                        if (action === null) {
                            action = factory.createAction
                            action.name = LinguaFrancaPackage.Literals.
                                TRIGGER_REF__SHUTDOWN.name
                            action.origin = ActionOrigin.LOGICAL
                            action.minDelay = factory.createTimeOrValue
                            action.minDelay.time = 0
                            reactor.actions.add(action)
                        }
                    }
                }
                // If appropriate, add a VarRef to the triggers list of this
                // reaction for the startup timer or shutdown action.
                if (startupTrigger !== null) {
                    reaction.triggers.remove(startupTrigger)
                    var variableReference = LinguaFrancaFactory.eINSTANCE.
                        createVarRef()
                    variableReference.setVariable(timer)
                    reaction.triggers.add(variableReference)
                }
                if (shutdownTrigger !== null) {
                    reaction.triggers.remove(shutdownTrigger)
                    var variableReference = LinguaFrancaFactory.eINSTANCE.
                        createVarRef()
                    variableReference.setVariable(action)
                    reaction.triggers.add(variableReference)
                }
            }
        }

        // Add a reaction, if necessary, to decrement the reference
        // count for any input that is conveyed using the token_t struct.
        // This will also free malloc'd memory when the reference count
        // hits zero.
        addReferenceCountReaction(reactor)

        pr("// =============== START reactor class " + reactor.name)

        // Scan reactions.
        // Preamble code contains state declarations with static initializers.
        for (p : reactor.preambles ?: emptyList) {
            pr("// *********** From the preamble, verbatim:")
            pr(removeCodeDelimiter(p.code))
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
            pr(body, getParameterType(parameter) + ' ' + parameter.name + ';');
        }
        // Next handle states.
        for (state : reactor.states) {
            prSourceLineNumber(state)
            pr(body, getStateType(state) + ' ' + state.name + ';');
        }
        // Next handle actions.
        for (a : reactor.actions) {
            prSourceLineNumber(a)
            // NOTE: Slightly obfuscate output name to help prevent accidental use.
            pr(body, "trigger_t* __" + a.name + ";")
        }
        // Next handle inputs.
        for (input : reactor.inputs) {
            prSourceLineNumber(input)
            if (input.type === null) {
                reportError(input,
                    "Input is required to have a type: " + input.name)
            } else {
                val inputType = lfTypeToTokenType(input.type)
                // If the output type has the form type[number], then treat it specially
                // to get a valid C type.
                val matcher = arrayPatternFixed.matcher(inputType)
                if (matcher.find()) {
                    // NOTE: Slightly obfuscate input name to help prevent accidental use.
                    pr(body, matcher.group(1) + '(* __' + input.name + ')' + matcher.group(2) + ';');
                } else {
                    // NOTE: Slightly obfuscate input name to help prevent accidental use.
                    pr(body, inputType + '* __' + input.name + ';');
                }
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
                // If the output type has the form type[] or type*, then change it to token_t.
                val outputType = lfTypeToTokenType(output.type)
                // If there are contained reactors that send data via this output,
                // then create a place to put the pointers to the sources of that data.
                var containedSource = outputToContainedOutput.get(output)
                // If the output type has the form type[number], then treat it specially
                // to get a valid C type.
                val matcher = arrayPatternFixed.matcher(outputType)
                if (matcher.find()) {
                    // Array case.
                    // NOTE: Slightly obfuscate output name to help prevent accidental use.
                    pr(body, matcher.group(1) + ' __' + output.name + matcher.group(2) + ';')
                    if (containedSource !== null) {
                        // This uses the same pattern as an input.
                        pr(body, matcher.group(1) + '(* __' + output.name + '_inside)' + matcher.group(2) + ';')
                    }
                } else {
                    // Normal case.
                    // NOTE: Slightly obfuscate output name to help prevent accidental use.
                    pr(body, outputType + ' __' + output.name +';')
                    // If there are contained reactors that send data via this output,
                    // then create a place to put the pointers to the sources of that data.
                    if (containedSource !== null) {
                        pr(body, outputType + '* __' + output.name + '_inside;')
                    }
                }
                // _is_present variables are the same for both cases.
                pr(body, 'bool __' + output.name + '_is_present;')
                if (containedSource !== null) {
                    pr(body, 'bool* __' + output.name + '_inside_is_present;')
                }
                pr(body, 'int __' + output.name + '_num_destinations;')
            }
        }
        
        // If there are contained reactors that either receive inputs
        // from reactions of this reactor or produce outputs that trigger
        // reactions of this reactor, then we need to create a struct
        // inside the self struct for each contained reactor. That
        // struct has a place to hold the data produced by this reactor's
        // reactions and a place to put pointers to data produced by
        // the contained reactors.
        // The contents of the struct will be collected first so that
        // we avoid duplicate entries and then the struct will be constructed.
        val structs = new HashMap<Instantiation,HashSet<Variable>>
        
        for (reaction : reactor.reactions) {
            // First, handle reactions that produce outputs sent to inputs
            // of contained reactors.
            for (effect : reaction.effects ?: emptyList) {
                if (effect.variable instanceof Input) {
                    var struct = structs.get(effect.container)
                    if (struct === null) {
                        struct = new HashSet<Variable>
                        structs.put(effect.container, struct)
                    }
                    struct.add(effect.variable)
                }
            }            
            // Second, handle reactions that are triggered by outputs
            // of contained reactors.
            for (TriggerRef trigger : reaction.triggers ?: emptyList) {
                if (trigger instanceof VarRef) {
                    if (trigger.variable instanceof Output) {
                        var struct = structs.get(trigger.container)
                        if (struct === null) {
                            struct = new HashSet<Variable>
                            structs.put(trigger.container, struct)
                        }
                        struct.add(trigger.variable)
                    }
                }
            }
            // Third, handle reading (but not triggered by)
            // outputs of contained reactors.
            for (source : reaction.sources ?: emptyList) {
                if (source.variable instanceof Output) {
                    var struct = structs.get(source.container)
                    if (struct === null) {
                        struct = new HashSet<Variable>
                        structs.put(source.container, struct)
                    }
                    struct.add(source.variable)
                }
            }
        }
        for (containedReactor : structs.keySet) {
            pr(body, "struct {")
            indent(body)
            for (variable : structs.get(containedReactor)) {
                if (variable instanceof Input) {
                    val port = variable as Input
                    pr(body, lfTypeToTokenType(port.type) + ' ' + port.name + ';')
                    pr(body, 'bool ' + port.name + '_is_present;')
                } else {
                    // Must be an output entry.
                    val port = variable as Output
                    // Outputs are pointers to the source of data.
                    pr(body, lfTypeToTokenType(port.type) + '* ' + port.name + ';')
                    pr(body, 'bool* ' + port.name + '_is_present;')
                }
            }
            unindent(body)
            pr(body, "} __" + containedReactor.name + ';')
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
        generateReactions(reactor, federate)
        generateTransferOutputs(reactor, outputToContainedOutput)
        pr("// =============== END reactor class " + reactor.name)
        pr("")
    }

    /** Generate reaction functions definition for a reactor.
     *  These functions have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reactor The reactor.
     *  @param federate The federate, or null if this is not
     *   federated or not the main reactor and reactions should be
     *   unconditionally generated.
     */
    def generateReactions(Reactor reactor, FederateInstance federate) {
        var reactions = reactor.reactions
        var reactionIndex = 0;
        for (reaction : reactions) {
            if (federate === null || federate.containsReaction(reactor, reaction)) {
                generateReaction(reaction, reactor, reactionIndex)
            }
            reactionIndex++
        }
    }
    
    /** Generate a reaction function definition for a reactor.
     *  This function has a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reaction The reaction.
     *  @param reactor The reactor.
     *  @param reactionIndex The position of the reaction within the reactor. 
     */
    def generateReaction(Reaction reaction, Reactor reactor, int reactionIndex) {
        // Create a unique function name for each reaction.
        val functionName = reactor.name.toLowerCase + "_rfunc_" + reactionIndex

        // Construct the reactionInitialization code to go into
        // the body of the function before the verbatim code.
        var StringBuilder reactionInitialization = new StringBuilder()

        // Define the "self" struct.
        if (!hasEmptySelfStruct(reactor)) {
            var structType = selfStructType(reactor)
            // A null structType means there are no inputs, state,
            // or anything else. No need to declare it.
            pr(reactionInitialization, structType + "* self = (" + structType + "*)instance_args;")
        }

        // A reaction may send to or receive from multiple ports of
        // a contained reactor. The variables for these ports need to
        // all be declared as fields of the same struct. Hence, we first
        // collect the fields to be defined in the structs and then
        // generate the structs.
        var fieldsForStructsForContainedReactors = new HashMap<Instantiation, StringBuilder>

        // Actions may appear twice, first as a trigger, then with the outputs.
        // But we need to declare it only once. Collect in this data structure
        // the actions that are declared as triggered so that if they appear
        // again with the outputs, they are not defined a second time.
        // That second redefinition would trigger a compile error.  
        var actionsAsTriggers = new HashSet<Action>();

        // Next, add the triggers (input and actions; timers are not needed).
        // This defines a local variable in the reaction function whose
        // name matches that of the trigger. If the trigger is an input
        // (not an action), then it also defines a local variable whose
        // name is the input name with suffix "_is_present", a boolean
        // that indicates whether the input is present.
        // If the trigger is an output, then it is an output of a
        // contained reactor. In this case, a struct with the name
        // of the contained reactor is created with two fields.
        // E.g., if the contained reactor is named 'c' and its output
        // port is named 'out', then c.out and c.out_is_present are
        // defined so that they can be used in the verbatim code.
        for (TriggerRef trigger : reaction.triggers ?: emptyList) {
            if (trigger instanceof VarRef) {
                if (trigger.variable instanceof Port) {
                    generatePortVariablesInReaction(reactionInitialization,
                        fieldsForStructsForContainedReactors, trigger)
                } else if (trigger.variable instanceof Action) {
                    actionsAsTriggers.add(trigger.variable as Action);
                    // If the action has a type, create variables for accessing the value.
                    val type = (trigger.variable as Action).type
                    val valuePointer = 'self->__' + trigger.variable.name + '->value'
                    // Create the _has_value variable.
                    pr(reactionInitialization,
                        'bool ' + trigger.variable.name + '_has_value = (' + valuePointer + ' != NULL);')
                    // Create the _value variable if there is a type.
                    if (type !== null) {
                        // Create the value variable, but initialize it only if the pointer is not null.
                        pr(reactionInitialization, type + ' ' + trigger.variable.name + '_value;')
                        pr(
                            reactionInitialization,
                            'if (' + trigger.variable.name + '_has_value) ' + trigger.variable.name +
                                '_value = (' + type + ')' + valuePointer + ';'
                        );
                    }
                }
            }
        }
        if (reaction.triggers === null || reaction.triggers.size === 0) {
            // No triggers are given, which means react to any input.
            // Declare an argument for every input.
            // NOTE: this does not include contained outputs. 
            for (input : reactor.inputs) {
                generateInputVariablesInReaction(reactionInitialization, input)
            }
        }
        // Define argument for non-triggering inputs.
        for (VarRef src : reaction.sources ?: emptyList) {
            if (src.variable instanceof Port) {
                generatePortVariablesInReaction(reactionInitialization, fieldsForStructsForContainedReactors, src)
            }
        }

        // Define variables for each declared output or action.
        // In the case of outputs, the variable is a pointer to where the
        // output is stored. This gives the reaction code access to any previous
        // value that may have been written to that output in an earlier reaction.
        // In addition, the _is_present variable is a boolean that indicates
        // whether the output has been written.
        if (reaction.effects !== null) {
            for (effect : reaction.effects) {
                // val action = getAction(reactor, output)
                if (effect.variable instanceof Action) {
                    // It is an action, not an output.
                    // If it has already appeared as trigger, do not redefine it.
                    if (!actionsAsTriggers.contains(effect.variable.name)) {
                        pr(reactionInitialization,
                            "trigger_t* " + effect.variable.name + ' = self->__' + effect.variable.name + ';');
                    }
                } else {
                    if (effect.variable instanceof Output) {
                        generateOutputVariablesInReaction(reactionInitialization, effect.variable as Output)
                    } else if (effect.variable instanceof Input) {
                        // It is the input of a contained reactor.
                        generateVariablesForSendingToContainedReactors(
                            reactionInitialization,
                            fieldsForStructsForContainedReactors,
                            effect.container,
                            effect.variable as Input
                        )
                    } else {
                        reportError(
                            reaction,
                            "In generateReactor(): " + effect.variable.name + " is neither an input nor an output."
                        )
                    }
                }
            }
        }
        pr('void ' + functionName + '(void* instance_args) {')
        indent()
        var body = removeCodeDelimiter(reaction.code)

        // Do not generate the initialization code if the body is marked
        // to not generate it.
        if (!body.startsWith(CGenerator.DISABLE_REACTION_INITIALIZATION_MARKER)) {
            // First generate the structs used for communication to and from contained reactors.
            for (containedReactor : fieldsForStructsForContainedReactors.keySet) {
                pr('struct ' + containedReactor.name + '{')
                indent();
                pr(fieldsForStructsForContainedReactors.get(containedReactor).toString)
                unindent();
                pr('} ' + containedReactor.name + ';')
            }
            // Next generate all the collected setup code.
            pr(reactionInitialization.toString)
        } else {
            // Define the "self" struct.
            if (!hasEmptySelfStruct(reactor)) {
                var structType = selfStructType(reactor)
                // A null structType means there are no inputs, state,
                // or anything else. No need to declare it.
                pr(structType + "* self = (" + structType + "*)instance_args;")
            }
        }
        // Code verbatim from 'reaction'
        prSourceLineNumber(reaction)
        pr(body)
        unindent()
        pr("}")

        // Now generate code for the deadline violation function, if there is one.
        if (reaction.deadline !== null) {
            // The following name has to match the choice in generateReactionStructs
            val deadlineFunctionName = reactor.name.toLowerCase + '_deadline_function' + reactionIndex

            pr('void ' + deadlineFunctionName + '(void* instance_args) {')
            indent();
            pr(reactionInitialization.toString)
            // Code verbatim from 'deadline'
            prSourceLineNumber(reaction.deadline.time)
            pr(removeCodeDelimiter(reaction.deadline.deadlineCode))
            unindent()
            pr("}")
        }
    }

    /** Generate reaction_t structs, one for each reaction in the
     *  specified reactor instance. The name of the struct will be
     *  uniqueID of the reaction instance.
     *  @param reactorIntance The reactor instance.
     *  @param federate The federate name or null if no federation.
     */
    def generateReactionStructs(ReactorInstance reactorInstance, FederateInstance federate) {
        val result = new StringBuilder()
        for (reaction : reactorInstance.reactions) {
            if (federate === null || federate.containsReaction(
                    reactorInstance.definition.reactorClass,
                    reaction.definition)) {
                val reactionInstanceName = reaction.uniqueID

                var presentPredicates = new LinkedList<String>()
                var triggeredSizesContents = new LinkedList<String>()
                var triggersContents = new LinkedList<String>()
                var Collection<PortInstance> destinationPorts = null

                // Generate entries for the reaction_t struct that specify how
                // to handle outputs.
                for (port : reaction.dependentPorts) {
                    // Place to collect reactions up the hierarchy triggered by this port.
                    var destinationReactions = new LinkedList<ReactionInstance>()

                    // Collect the destinations for each output port.
                    if (port.definition instanceof Output) {
                        // Reaction sends to an output.
                        // First create the array of pointers to booleans indicating
                        // whether an output is produced.
                        presentPredicates.add(
                            '&' + selfStructName(reactorInstance) + '.__' +
                                port.name + '_is_present')

                        // For each output, obtain the destinations from the parent.
                        var parent = reactorInstance.parent
                        if (parent !== null) {
                            destinationPorts = parent.transitiveClosure(port)
                        } else {
                            // At the top level, where there cannot be any destinations
                            // for an output port.
                            destinationPorts = new LinkedList<PortInstance>()
                        }

                        // The port may also have dependent reactions, which are
                        // reactions in the container of this port's container.
                        for (dependentReactions : port.dependentReactions) {
                            destinationReactions.add(dependentReactions)
                        }
                    } else {
                        // The reaction is sending data to the input of a contained reactor.
                        // First create the array of pointers to booleans indicating whether
                        // an output is produced.
                        presentPredicates.add(
                            '&' + selfStructName(reactorInstance) + '.__' +
                                port.parent.name + '.' + port.name + '_is_present')

                        // Since the port is the input port of a contained reactor,
                        // use that reactor instance to compute the transitive closure.
                        destinationPorts = port.parent.transitiveClosure(port)
                    }

                    val numberOfTriggerTObjects = destinationPorts.size +
                        destinationReactions.size

                    // Next, create an array trigger_t objects, which are
                    // the triggers that fire if this output is produced.
                    // Append to the array that records the sizes of the trigger arrays.
                    triggeredSizesContents.add("" + numberOfTriggerTObjects)

                    // Then, for each destination connected to this output,
                    // create an array of pointers to its trigger_t structs,
                    // and collect pointers to each of these arrays.
                    if (numberOfTriggerTObjects === 0) {
                        triggersContents.add("NULL")
                    } else {
                        // FIXME: This ID may exceed some maximum length.
                        var remoteTriggersArrayName = reactionInstanceName + '_' +
                            presentPredicates.size + '_remote_triggers'
                        var inputCount = 0;
                        for (destination : destinationPorts) {
                            deferredInitialize.add(
                                new InitializeRemoteTriggersTable(
                                    reactorInstance,
                                    remoteTriggersArrayName,
                                    (inputCount++),
                                    destination,
                                    null
                                )
                            )
                        }
                        for (destinationReaction : destinationReactions) {
                            deferredInitialize.add(
                                new InitializeRemoteTriggersTable(
                                    reactorInstance,
                                    remoteTriggersArrayName,
                                    (inputCount++),
                                    port,
                                    destinationReaction
                                )
                            )
                        }
                        pr(
                            result,
                            'trigger_t* ' + remoteTriggersArrayName + '[' +
                                inputCount + '];'
                        )
                        triggersContents.add('&' + remoteTriggersArrayName + '[0]')
                    }
                }
                var outputProducedArray = "NULL"
                var triggeredSizesArray = "NULL"
                var triggersArray = "NULL"
                val outputCount = presentPredicates.size
                if (outputCount > 0) {
                    outputProducedArray = reactionInstanceName +
                        '_outputs_are_present'
                    // Create a array with booleans indicating whether an output has been produced.
                    pr(
                        result,
                        'bool* ' + reactionInstanceName + '_outputs_are_present[]' +
                                ' = {' + presentPredicates.join(", ") + '};'
                    )
                    // Create a array with ints indicating these
                    // numbers and assign it to triggered_reactions_sizes
                    // field of the reaction_t object.
                    triggeredSizesArray = '&' + reactionInstanceName +
                        '_triggered_sizes[0]'
                    pr(
                        result,
                        'int ' + reactionInstanceName + '_triggered_sizes' +
                                '[] = {' + triggeredSizesContents.join(", ") + '};'
                    )
                    // Create an array with pointers to arrays of pointers to trigger_t
                    // structs for each input triggered by an output.
                    triggersArray = '&' + reactionInstanceName + '_triggers[0]'
                    pr(
                        result,
                        'trigger_t** ' + reactionInstanceName + '_triggers' +
                                '[] = {' + triggersContents.join(', ') + '};'
                    )
                }
                // Finally, produce the reaction_t struct.          
                // The argument specifying the self struct may be NULL if there
                // is no self struct.
                var selfStructArgument = ", &" + selfStructName(reactorInstance)
                var reactorClass = reactorInstance.definition.reactorClass
                if (hasEmptySelfStruct(reactorClass)) {
                    selfStructArgument = ", NULL"
                }
                var deadlineFunctionPointer = ", NULL"
                if (reaction.definition.deadline !== null) {
                    // The following has to match the name chosen in generateReactions
                    val deadlineFunctionName = reactorInstance.definition.
                        reactorClass.name.toLowerCase + '_deadline_function' +
                        reaction.reactionIndex

                    deadlineFunctionPointer = ", &" + deadlineFunctionName
                }

                // Use the same function name as in generateReactions.
                // FIXME: Fragile!  Find a better way to get agreement on function name.
                val functionName = reactorClass.name.toLowerCase + "_rfunc_" +
                    reaction.reactionIndex

                // First 0 is an index that specifies priorities based on precedences.
                // It will be set later.
                pr(
                    result,
                    "reaction_t " + reactionInstanceName + " = {&" + functionName +
                        selfStructArgument + ", 0" // index: index from the topological sort.
                        + ", 0" // chain_id: binary encoding of the branches that this reaction has upstream in the dependency graph.
                        + ", 0" // pos: position used by the pqueue implementation for sorting.
                        + ", " + outputCount // num_outputs: number of outputs produced by this reaction.
                        + ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
                        + ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
                        + ", " + triggersArray // triggered: array of pointers to arrays of triggers.
                        + ", false" // Indicator that the reaction is not running.
                        + ", 0LL" // Local deadline.
                        + deadlineFunctionPointer // deadline_violation_handler: Pointer to local handler function.
                        + "};"
                )
            }
        }
        // This goes directly out to the generated code.
        pr(result.toString())
    }

    /** Generate one reaction function definition for each output of
     *  a reactor that relays data from the output of a contained reactor.
     *  This reaction function transfers the data from the output of the
     *  contained reactor (in the self struct of this reactor labeled as
     *  "inside") to the output of this reactor (also in its self struct).
     *  There needs to be one reaction function
     *  for each such output because these reaction functions have to be
     *  individually invoked after each contained reactor produces an
     *  output that must be relayed.
     *  @param reactor The reactor.
     *  @param outputToContainedOutput A map of output ports of this
     *   reactor to output ports of contained reactors that they receive
     *   data from.
     */
    def generateTransferOutputs(Reactor reactor,
        HashMap<Output, VarRef> outputToContainedOutput) {
        for (output : outputToContainedOutput.keySet()) {
            // The following function name will be unique, assuming that
            // reactor class names are unique and within each reactor class,
            // output names are unique.
            // This has to match what's in generateTriggerForTransferOutputs
            val functionName = reactor.name.toLowerCase + "_xfer_outs_" + "_" +
                output.name

            pr('void ' + functionName + '(void* instance_args) {')
            indent()

            // Define the "self" struct. First get its name. Note that this
            // must not be null because there is at least one output.
            var structType = selfStructType(reactor)
            pr(structType + "* self = (" + structType + "*)instance_args;")

            // Transfer the output value from the inside value.
            pr("self->__" + output.name + " = *(self->__" + output.name +
                "_inside);")
            // Transfer the presence flag from the inside value.
            pr(
                "self->__" + output.name + "_is_present = *(self->__" +
                    output.name + "_inside_is_present);")
            unindent()
            pr("}")
        }
    }

    /** Generate trigger_t objects for transferring outputs from inside a composite
     *  to the outside.  Each trigger_t object is a struct that contains an
     *  array of pointers to reaction_t objects representing
     *  the transfer output.
     *  This also creates the reaction_t object for each transfer outputs.
     *  This object has a pointer to the function to invoke for that
     *  reaction.
     *  @param reactorInstance The instance for which we are generating trigger objects.
     */
    def generateTriggerForTransferOutputs(ReactorInstance reactorInstance) {
        var outputCount = 0
        val result = new StringBuilder()
        var triggersContents = new LinkedList<String>()
        var triggeredSizesContents = new LinkedList<String>()
        var nameOfSelfStruct = selfStructName(reactorInstance)

        for (output : reactorInstance.outputs) {
            if (output.dependsOnPort !== null) {
                // The output is connected on the inside.
                // Create the reaction and trigger structs for this port.
                // The function name for the transfer outputs function:
                // This has to match what's in generateTransferOutputs
                val functionName = output.parent.definition.reactorClass.name.
                    toLowerCase + "_xfer_outs_" + "_" + output.name

                pr(
                    result,
                    "// --- Reaction and trigger objects for transfer outputs for " +
                        output.getFullName
                )

                // Figure out how many inputs are connected to the output.
                // This is obtained via the container.
                var parent = reactorInstance.parent
                var Collection<PortInstance> destinations = null
                if (parent !== null) {
                    destinations = parent.transitiveClosure(output)
                } else {
                    // At the top level, where there cannot be any destinations.
                    destinations = new LinkedList<PortInstance>()
                }

                // Place to collect reactions up the hierarchy triggered by this port.
                var destinationReactions = new LinkedList<ReactionInstance>()

                // The port may also have dependent reactions, which are
                // reactions in the container of this port's container.
                for (dependentReactions : output.dependentReactions) {
                    destinationReactions.add(dependentReactions)
                }

                val numberOfTriggerTObjects = destinations.size +
                    destinationReactions.size

                // Append to the array that records the length of each trigger_t array.            
                triggeredSizesContents.add("" + numberOfTriggerTObjects)

                val structName = triggerStructName(output)

                // Then, for each input connected to this output,
                // find its trigger_t struct. Create an array of pointers
                // to these trigger_t structs, and collect pointers to
                // each of these arrays.
                if (destinations.size === 0) {
                    triggersContents.add("NULL")
                } else {
                    var inputCount = 0;
                    for (destination : destinations) {
                        deferredInitialize.add(
                            new InitializeRemoteTriggersTable(
                                parent,
                                output.uniqueID + '_remote_triggers',
                                (inputCount++),
                                destination,
                                null
                            )
                        )
                    }
                    for (destinationReaction : destinationReactions) {
                        deferredInitialize.add(
                            new InitializeRemoteTriggersTable(
                                parent,
                                output.uniqueID + '_remote_triggers',
                                (inputCount++),
                                output,
                                destinationReaction
                            )
                        )
                    }
                    pr(
                        result,
                        'trigger_t* ' + output.uniqueID + '_remote_triggers[' +
                            inputCount + '];'
                    )
                    triggersContents.add('&' + output.uniqueID +
                        '_remote_triggers[0]')
                }
                // Next generate the array of booleans which indicates whether outputs are present.
                var outputProducedArray = output.uniqueID +
                    '_outputs_are_present'
                pr(
                    result,
                    'bool* ' + outputProducedArray + '[]' + ' = {' + '&' +
                        nameOfSelfStruct + '.__' + output.name + '_is_present' +
                        '};'
                )
                // Create a array with ints indicating these
                // numbers and assign it to triggered_reactions_sizes
                // field of the reaction_t object.
                var triggeredSizesArray = '&' + output.uniqueID +
                    '_triggered_sizes[0]'
                pr(
                    result,
                    'int ' + output.uniqueID + '_triggered_sizes' + '[] = {' +
                        triggeredSizesContents.join(', ') + '};'
                )
                // Create an array with pointers to arrays of pointers to trigger_t
                // structs for each input triggered by an output.
                var triggersArray = '&' + output.uniqueID + '_triggers[0]'
                pr(
                    result,
                    'trigger_t** ' + output.uniqueID + '_triggers' + '[] = {' +
                        triggersContents.join(', ') + '};'
                )
                // First 0 is an index that specifies priorities based on precedences.
                // It will be set later.
                var reactionInstanceName = output.uniqueID + "_reaction"
                pr(
                    result,
                    "reaction_t " + reactionInstanceName + " = {&" +
                        functionName + ", &" + nameOfSelfStruct // Function
                        + ", 0" // index: index from the topological sort.
                        + ", 0"
                        + ", 0" // pos: position used by the pqueue implementation for sorting.
                        + ", 1" // num_outputs: number of outputs produced by this reaction. This is just one.
                        + ", " + outputProducedArray // output_produced: array of pointers to booleans indicating whether output is produced.
                        + ", " + triggeredSizesArray // triggered_sizes: array of ints indicating number of triggers per output.
                        + ", " + triggersArray // triggered: array of pointers to arrays of triggers.
                        + ", false" // Indicator that the reaction is not running.
                        + ", 0LL" // Local deadline.
                        + ", NULL" // Pointer to local deadline_violation_handler.
                        + "};"
                )
                pr(result,
                    'reaction_t* ' + structName + '_reactions[1] = {&' +
                        reactionInstanceName + '};')

                pr(result, 'trigger_t ' + structName + ' = {')
                indent(result)
                pr(result, structName + '_reactions, 1, 0LL, 0LL, NULL, false, NEVER, NONE')
                unindent(result)
                pr(result, '};')

                triggerCount++
            }
            outputCount++
        }
        // This goes directly out to the generated code.
        if (result.length > 0) {
            pr("// *********** Transfer outputs structures for " +
                reactorInstance.definition.name)
            pr(result.toString())
        }
    }

    /** Generate trigger_t objects, one for
     *  each input, clock, and action of the reactor instance.
     *  Each trigger_t object is a struct that contains an
     *  array of pointers to reaction_t objects representing
     *  reactions triggered by this trigger. The trigger_t object
     *  also provides the length of the array, and if the trigger
     *  is a timer or an action, the offset,
     *  and the period. (The offset and period are zero if the trigger
     *  is not an action or a timer. The period is zero for an action).
     * 
     *  This also creates the reaction_t object for each reaction.
     *  This object has a pointer to the function to invoke for that
     *  reaction.
     *  @param reactorInstance The instance for which we are generating trigger objects.
     *  @param federate The federate or null if no federation.
     *  @return A map of trigger names to the name of the trigger struct.
     */
    def generateTriggerObjects(ReactorInstance reactorInstance, FederateInstance federate) {
        val result = new StringBuilder()
        var count = 0
        // Iterate over triggers (input ports, actions, and timers that trigger reactions).
        for (triggerInstance : reactorInstance.triggers) {
            var trigger = triggerInstance.definition
            var numberOfReactionsTriggered = triggerInstance.dependentReactions.
                length

            // Collect names of the reaction_t objects that are triggered together.
            var reactionTNames = new LinkedList<String>();

            // Generate reaction_t struct.
            // Along the way, we need to generate its contents, including trigger_t structs.
            for (reactionInstance : triggerInstance.dependentReactions) {
                pr(
                    result,
                    '// --- Reaction and trigger objects for reaction to trigger ' +
                        trigger.name + ' of instance ' +
                        reactorInstance.fullName
                )
                if (federate === null || federate.containsReaction(
                    reactionInstance.parent.definition.reactorClass,
                    reactionInstance.definition
                )) {                    
                    val reactionInstanceName = reactionInstance.uniqueID

                    // Collect the reaction instance names to initialize the
                    // reaction pointer array for the trigger.
                    reactionTNames.add('&' + reactionInstanceName)
                }
            }
            // Trigger could be a Timer, Action, or Input
            var triggerStructName = triggerStructName(triggerInstance)

            pr(result,
                'reaction_t* ' + triggerStructName + '_reactions[' +
                    numberOfReactionsTriggered + '] = {' +
                    reactionTNames.join(", ") + '};')
            // Declare a variable with the name of the trigger whose
            // value is a struct.
            pr(result, 'trigger_t ' + triggerStructName + ' = {')
            indent(result)
            if (trigger instanceof Timer || trigger instanceof Port) {
                pr(
                    result,
                    triggerStructName + '_reactions, ' +
                        numberOfReactionsTriggered + ', ' +
                        '0LL, 0LL, NULL, false, NEVER, NONE'
                )
            } else if (trigger instanceof Action) {
                var isPhysical = "true";
                var minDelay = reactorInstance.resolveTime(trigger.minDelay)
                var minInterArrival = reactorInstance.resolveTime(trigger.minInterArrival)
                
                if (trigger.origin == ActionOrigin.LOGICAL) {
                    isPhysical = "false";
                } else {
                    // For physical actions,
                    // if no minimum interarrival time was specified, then
                    // we do not want zero, which is what resolveTime returns,
                    // but rather some non-zero time.
                    if (trigger.minInterArrival === null) {
                        minInterArrival = DEFAULT_MIN_INTER_ARRIVAL;
                    }
                    if (trigger.policy == QueuingPolicy.NONE) {
                        trigger.policy = QueuingPolicy.DEFER;
                    }
                }
                pr(
                    result,
                    triggerStructName + '_reactions, ' +
                        numberOfReactionsTriggered + ', ' +
                        timeInTargetLanguage(minDelay) + ',' + timeInTargetLanguage(minInterArrival) + 
                        ', NULL, ' + isPhysical + ', NEVER,' + trigger.policy
                )
                // If this is a shutdown action, add it to the list of shutdown actions.
                if ((triggerInstance as ActionInstance).isShutdown) {
                    shutdownActionInstances.add(
                        triggerInstance as ActionInstance)
                }

            } else {
                reportError(trigger,
                    "Internal error: Seems to not be a port, timer, or action: " +
                        trigger.name)
            }
            unindent(result)
            pr(result, '};')
            // Assignment of the offset and period have to occur after creating
            // the struct because the value assigned may not be a compile-time constant.
            if (trigger instanceof Timer) {

                val offset = reactorInstance.resolveTime(trigger.offset)
                val period = reactorInstance.resolveTime(trigger.period)

                pr(initializeTriggerObjects,
                    triggerStructName + '.offset = ' + timeInTargetLanguage(offset) + ';')
                pr(initializeTriggerObjects,
                    triggerStructName + '.period = ' + timeInTargetLanguage(period) + ';')
                // Generate a line to go into the __start_timers() function.
                // Note that the delay, the second argument, is zero because the
                // offset is already in the trigger struct.
                pr(startTimers,
                    "__schedule(&" + triggerStructName + ", 0LL, NULL);")
            }
            count++
            triggerCount++
        }
        // This goes directly out to the generated code.
        pr(result.toString())
    }

    /** Open a non-Lingua Franca import file at the specified URI
     *  in the specified resource set. Throw an exception if the
     *  file import is not supported. This class imports .proto files
     *  and runs, if possible, the protoc protocol buffer code generator
     *  to produce the required .h and .c files.
     *  @param importStatement The original import statement (used for error reporting).
     *  @param resourceSet The resource set in which to find the file.
     *  @param resolvedURI The URI to import.
     */
    override openForeignImport(Import importStatement, ResourceSet resourceSet, URI resolvedURI) {
        // Unfortunately, the resolvedURI appears to be useless for ordinary files
        // (non-xtext files). Use the original importStatement.importURI
        if (importStatement.importURI.endsWith(".proto")) {
            // FIXME: Should we include protoc-c as a submodule? If so, how to invoke it?
            // protoc is commonly installed in /usr/local/bin, which sadly is not by
            // default on the PATH for a Mac.
            // Invoke protoc-c.
            val protocCommand = newArrayList
            protocCommand.addAll("protoc-c", "--c_out=src-gen", importStatement.importURI)
            if (executeCommand(protocCommand, directory) != 0) {
                return reportError(importStatement, "Protocol buffer compiler failed."
                    + "\nFor installation instructions, see: https://github.com/protobuf-c/protobuf-c."
                    + "\nMake sure that your PATH variable includes the directory where protoc-c is installed,"
                    + "\ntypically /usr/local/bin. You can set PATH in ~/.bash_profile on Linux or Mac.")
            }
            if (compileAdditionalSources === null) {
                compileAdditionalSources = newArrayList
            }
            // Strip the ".proto" off the file name.
            // NOTE: This assumes that the filename matches the generated files, which it seems to.
            val rootFilename = importStatement.importURI.substring(0, importStatement.importURI.length - 6)
            compileAdditionalSources.add("src-gen" + File.separator + rootFilename + ".pb-c.c")
            
            // The -l protobuf-c command-line option should be added only once, even if there
            // are multiple protobuf imports.
            if (compileLibraries === null) {
                compileLibraries = newArrayList
                compileLibraries.add('-l')
                compileLibraries.add('protobuf-c')
            }
        } else {
            return reportError(importStatement, "Unsupported imported file type: "
                + importStatement.importURI
            )
        }
        return "OK"
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

    /** Return the unique name for the trigger_t struct of the specified
     *  trigger instance (input port or action).
     *  @param instance The port or action instance.
     *  @return The name of the trigger struct.
     */
    static def triggerStructName(TriggerInstance<Variable> instance) {
        return instance.uniqueID + "_trigger"
    }

    /** Return true of the given reactor has an empty self struct, false otherwise.
     *  @param reactor A reactor class
     */
    def hasEmptySelfStruct(Reactor reactor) {
        if (!reactor.parameters.isEmpty || !reactor.states.isEmpty ||
            !reactor.actions.isEmpty || !reactor.inputs.isEmpty ||
            !reactor.outputs.isEmpty) {
            return false
        }
        for (reaction : reactor.reactions ?: emptyList) {
            for (effect : reaction.effects ?: emptyList) {
                // Sending to input of contained reactor
                if (effect.variable instanceof Input) {
                    return false
                }
            }
            for (TriggerRef trigger : reaction.triggers ?: emptyList) {
                if (trigger instanceof VarRef &&
                    (trigger as VarRef).variable instanceof Output) {
                    // Triggered by the output of a contained reactor.
                    return false
                }
            }
            for (reading : reaction.sources ?: emptyList) {
                if (reading.variable instanceof Output) {
                    // Reading the output of a contained reactor.
                    return false
                }
            }
        }
        return true
    }

    /** Traverse the runtime hierarchy of reaction instances and generate code.
     *  @param instance A reactor instance.
     *  @param federate A federate name to conditionally generate code by
     *   contained reactors or null if there are no federates.
     */
    def void generateReactorInstance(ReactorInstance instance, FederateInstance federate) {
        var reactorClass = instance.definition.reactorClass
        var fullName = instance.fullName
        pr('// ************* Instance ' + fullName + ' of class ' +
            reactorClass.name)

        // Generate the instance struct containing parameters, state variables,
        // and outputs (the "self" struct).
        var nameOfSelfStruct = selfStructName(instance)
        var structType = selfStructType(reactorClass)
        if (!hasEmptySelfStruct(reactorClass)) {
            pr('// --- "self" struct for instance ' + fullName)
            pr(structType + " " + nameOfSelfStruct + ";")
        }

        // Generate code to initialize the "self" struct in the
        // __initialize_trigger_objects function.
        pr(initializeTriggerObjects, "//***** Start initializing " + fullName)

        // Start with parameters.
        for (parameter : instance.parameters) {
            // NOTE: we now use the resolved literal value. For better efficiency, we could
            // store constants in a global array and refer to its elements to avoid duplicate
            // memory allocations.
            
            // Array type parameters have to be handled specially.
            val matcher = arrayPatternVariable.matcher(parameter.type)
            if (matcher.find()) {
                val temporaryVariableName = parameter.uniqueID
                pr(initializeTriggerObjects,
                    "static " + matcher.group(1) + " " +
                    temporaryVariableName + "[] = " + parameter.literalValue + ";"
                )
                pr(initializeTriggerObjects,
                    nameOfSelfStruct + "." + parameter.name + " = " + temporaryVariableName + ";"
                )
            } else {
                pr(initializeTriggerObjects,
                    nameOfSelfStruct + "." + parameter.name + " = " +
                        parameter.literalValue + ";"
                )
            }
        }

        // Next, initialize the "self" struct with state variables.
        // These values may be expressions that refer to the parameter values defined above.
        for (state : reactorClass.states) {
            var time = state.time
            var unit = state.unit
            var value = state.value

            if (state.parameter !== null) {
                time = state.parameter.time
                unit = state.parameter.unit
                value = state.parameter.value
            }
            if (state.ofTimeType) {
                pr(initializeTriggerObjects,
                    nameOfSelfStruct + "." + state.name + " = " +
                        timeInTargetLanguage(new TimeValue(time, unit)) + ";")
            } else {
                // If the state is initialized with a parameter, then do not use
                // a temporary variable. Otherwise, do, because
                // static initializers for arrays and structs have to be handled
                // this way, and there is no way to tell whether the type of the array
                // is a struct.
                if (state.parameter !== null) {
                    pr(initializeTriggerObjects,
                        nameOfSelfStruct + "." + state.name + " = " +
                        removeCodeDelimiter(value) + ";")
                } else {
                    val temporaryVariableName = instance.uniqueID + '_initial_' + state.name
                    var type = removeCodeDelimiter(state.type);
                    val matcher = arrayPatternVariable.matcher(type)
                    if (matcher.find()) {
                        // If the state type ends in [], then we have to move the []
                        // because C is very picky about where this goes. It has to go
                        // after the variable name.
                        pr(initializeTriggerObjects,
                            "static " + matcher.group(1) + " " +
                            temporaryVariableName + "[] = " + removeCodeDelimiter(state.value) + ";"
                        )
                    } else {
                        pr(initializeTriggerObjects,
                            "static " + type + " " +
                            temporaryVariableName + " = " + removeCodeDelimiter(state.value) + ";"
                        )
                    }
                    pr(initializeTriggerObjects,
                        nameOfSelfStruct + "." + state.name + " = " + temporaryVariableName + ";"
                    )                    
                }
            }
        }

        // Generate reaction structs for the instance.
        generateReactionStructs(instance, federate)

        // Generate trigger objects for the instance.
        generateTriggerObjects(instance, federate)

        // Generate trigger objects for transferring outputs of a composite.
        generateTriggerForTransferOutputs(instance)

        // Next, initialize the struct with actions.
        val triggersInUse = instance.triggers
        for (action : instance.actions) {
            var triggerStruct = '&' + triggerStructName(action)
            // If the action doesn't actually trigger anything, then
            // no trigger struct was defined.
            if (!triggersInUse.contains(action)) {
                triggerStruct = 'NULL'
            }
            pr(
                initializeTriggerObjects,
                nameOfSelfStruct + '.__' + action.name + ' = ' + triggerStruct + ';'
            )
        }
        // Next, generate the code to initialize outputs and inputs at the start
        // of a time step to be absent. This will also set the element_size
        // and initial_reference_count fields of any outputs that are carried
        // by a token_t struct.
        for (output : instance.outputs) {
            pr(
                startTimeStep,
                nameOfSelfStruct + '.__' + output.name + '_is_present = false;'
            )
            if (isTokenType((output.definition as Port).type)) {
                pr(initializeTriggerObjects,
                    nameOfSelfStruct + ".__" + output.name + ".element_size = sizeof(" +
                    rootType((output.definition as Port).type) + ");"
                )
                // Set the initial reference count equal to the number of destinations.
                pr(initializeTriggerObjects,
                    nameOfSelfStruct + ".__" + output.name + "_num_destinations = "
                    + output.dependentPorts.size + ";"
                )
            }
        }
        // Handle reaction local deadlines.
        for (reaction : instance.reactions) {
            if (reaction.definition.deadline !== null) {
                var deadline = instance.resolveTime(reaction.definition.deadline.time)
                pr(initializeTriggerObjects,
                    reactionStructName(reaction) + '.local_deadline = ' +
                        timeInTargetLanguage(deadline) + ';')
            }
        }
        for (child : instance.children) {
            if (reactorBelongsToFederate(child, federate)) {
                generateReactorInstance(child, federate)
            }
        }

        pr(initializeTriggerObjects, "//***** End initializing " + fullName)
    }
    
    /** Return true if the specified reactor instance belongs to the specified
     *  federate. This always returns true if the specified federate is
     *  null or a singleton. Otherwise, it returns true only if the
     *  instance is contained by the main reactor and the instance name
     *  was included in the 'reactors' property of the targets 'federates'
     *  specification.
     *  @param instance A reactor instance.
     *  @param federate A federate null if there are no federates.
     */
    def reactorBelongsToFederate(ReactorInstance instance, FederateInstance federate) {
        if (federate === null || federate.isSingleton) {
            return true
        } else {
            if (instance.parent === this.main 
                && !federate.contains(instance.name)
            ) {
                return false
            } else {
                return true
            }
        }
    }

    /** Set the reaction priorities based on dependency analysis.
     *  @param reactor The reactor on which to do this.
     *  @param federate A federate to conditionally generate code for
     *   contained reactors or null if there are no federates.
     */
    def void setReactionPriorities(ReactorInstance reactor, FederateInstance federate) {
        // Use "reactionToReactionTName" property of reactionInstance
        // to set the levels.
        for (reactionInstance : reactor.reactions) {

            if (federate === null || federate.containsReaction(
                reactor.definition.reactorClass,
                reactionInstance.definition
            )) {
                pr(
                    reactionStructName(reactionInstance) + ".index = " +
                        reactionInstance.level + ";")
                pr(reactionStructName(reactionInstance) + ".chain_id = " +
                    reactionInstance.chainID.toString() + ";")

                pr(
                    reactionStructName(reactionInstance) + ".index = " +
                        (reactionInstance.deadline.toNanoSeconds.shiftLeft(16)).
                            or(new BigInteger(reactionInstance.level.toString)) + "LL;")
            // FIXME: add check to validator to make sure no deadline exceeds the maximum
            }
        }
        for (child : reactor.children) {
            if (reactorBelongsToFederate(child, federate)) {
                setReactionPriorities(child, federate)
            }
        }
    }

    // //////////////////////////////////////////
    // // Protected methods.

    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set contains only "C".
     */
    override acceptableTargets() {
        acceptableTargetSet
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action The action to schedule
     * @param port The port to read from
     */
    override generateDelayBody(Action action, VarRef port) { 
        val ref = generateVarRef(port);
        // Note that the action.type set by the base class is actually
        // the port type. The action type needs to be corrected to be a pointer.
        // However, this is deferred to generateForwardBody so that we still
        // have access to the port type there.
        // FIXME: the following check is not detecting pointer types masked by a typedef
        if (isTokenType(action.type)) {
            '''
            // Get a copy of the input that we own (won't be freed).
            if («ref»_is_present) {
                «ref».value = writable_copy(«ref»);
            }
            // FIXME: What to do about the length? Special version of schedule()?
            schedule(«action.name», 0, «ref».value);
            '''
        } else {
            val tmp = action.name + "_to_" + port.variable.name
            '''
            «action.type»* «tmp» = malloc(sizeof(«action.type»));
            *«tmp» = «ref»;
            schedule(«action.name», 0, «tmp»);
            '''
        }
    }
    
    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     * @param action The action that triggers the reaction
     * @param port The port to write to
     */
    override generateForwardBody(Action action, VarRef port) {
        // Note that the action.type set by the base class is actually
        // the port type. The action type needs to be corrected to be a pointer.
        val portType = action.type
        action.type = portType + '*'
        val ref = generateVarRef(port)
        
        if (isTokenType(portType)) {
            // This is similar to set_token macro except for
            // handling of reference counts.
            '''
            self->__«ref».value = «action.name»_value;
            self->__«ref».length = 1;
            self->__«ref».ref_count = 1;
            self->__«ref»_is_present = true;
            '''
        } else {
            // Primitive type. Memory was malloc'd above.
            '''
            set(«ref», *«action.name»_value);
            free(«action.name»_value);
            '''
        }
    }

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param type The type.
     */
    override generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        String type
    ) {
        // Adjust the type of the action.
        // If it is "string", then change it to "char".
        // Pointer types in actions are declared without the "*" (perhaps oddly).
        if (action.type == "string") {
            action.type = "char"
        }
        val sendRef = generateVarRef(sendingPort)
        val receiveRef = generateVarRef(receivingPort)
        '''
        // Receiving from «sendRef» in federate «sendingFed.name» to «receiveRef» in federate «receivingFed.name»
        // NOTE: Docs say that malloc'd char* is freed on conclusion of the time step.
        // So passing it downstream should be OK.
        set(«receiveRef», «action.name»_value);
        '''
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param type The type.
     */
    override generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        String type
    ) { 
        val sendRef = generateVarRef(sendingPort);
        val receiveRef = generateVarRef(receivingPort);
        '''
        // Sending from «sendRef» in federate «sendingFed.name» to «receiveRef» in federate «receivingFed.name»
        // FIXME: Only supporting string type right now.
        int message_length = strlen(«sendRef») + 1;
        send_physical(«receivingPortID», «receivingFed.id», message_length, (unsigned char*) «sendRef»);
        '''
    }

    /** Generate #include of pqueue.c and either reactor.c or reactor_threaded.c
     *  depending on whether threads are specified in target directive.
     *  As a side effect, this populates the runCommand and compileCommand
     *  private variables if such commands are specified in the target directive.
     */
    override generatePreamble() {
        super.generatePreamble()
        
        pr('#include "pqueue.c"')
        pr('#define NUMBER_OF_FEDERATES ' + federates.length);        
                
        // Handle target parameters.
        if (targetThreads > 0) {
            // Set this as the default in the generated code,
            // but only if it has not been overridden on the command line.
            pr(startTimers, "if (number_of_threads == 0) {")
            indent(startTimers)
            pr(startTimers, "number_of_threads = " + targetThreads + ";")
            unindent(startTimers)
            pr(startTimers, "}")
            pr("#include \"reactor_threaded.c\"")
        } else {
            pr("#include \"reactor.c\"")
        }
        if (federates.length > 1) {
            pr("#include \"federate.c\"")
        }
        if (targetFast) {
            // The runCommand has a first entry that is ignored but needed.
            if (runCommand.length === 0) {
                runCommand.add("X")
            }
            runCommand.add("-f")
            runCommand.add("true")
        }
        if (targetKeepalive) {
            // The runCommand has a first entry that is ignored but needed.
            if (runCommand.length === 0) {
                runCommand.add("X")
            }
            runCommand.add("-k")
            runCommand.add("true")
        }
        if (targetTimeout >= 0) {
            // The runCommand has a first entry that is ignored but needed.
            if (runCommand.length === 0) {
                runCommand.add("X")
            }
            runCommand.add("-o")
            runCommand.add(Integer.toString(targetTimeout))
            runCommand.add(targetTimeoutUnit.toString)
        }
        
        // Generate #include statements for each .proto import.
        for (import : resource.allContents.toIterable.filter(Import)) {
            if (import.importURI.endsWith(".proto")) {
                // Strip the ".proto" off the file name.
                // NOTE: This assumes that the filename matches the generated files, which it seems to.
                val rootFilename = import.importURI.substring(0, import.importURI.length - 6)
                // Finally, generate the #include for the generated .h file.
                pr('#include "' + rootFilename + '.pb-c.h"')
            }
        }
    }

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
    
    /** Perform deferred initializations in initialize_trigger_objects.
     *  @param federate The federate for which we are doing this.
     */
    private def doDeferredInitialize(FederateInstance federate) {
        // First, populate the trigger tables for each output.
        // The entries point to the trigger_t structs for the destination inputs.
        pr('// doDeferredInitialize')
        for (init : deferredInitialize) {
            if (init.reaction === null) {
                // Input port being triggered.
                var triggerStructName = triggerStructName(init.input)
                // If the destination of a connection is an input
                // port of a reactor that has no reactions to that input,
                // then this trigger struct will not have been created.
                // In that case, we want NULL.
                // If the destination is an output port, however, then
                // the dependentReactions.size will be zero, but we nevertheless
                // want to set up the trigger.
                if (init.input.dependentReactions.size === 0 &&
                    !init.input.isOutput) {
                    pr(init.remoteTriggersArrayName + '[' + init.arrayIndex +
                        '] = NULL;')
                } else {
                    pr(
                        init.remoteTriggersArrayName + '[' + init.arrayIndex +
                            '] = &' + triggerStructName + ';')
                }
            } else {
                // Reaction in a container being triggered.
                // In this case, the input field is not an input, but the
                // output of a contained reactor. If the contained reactor
                // is not in the federate, then skip this step.
                // Note that in this case, init.input is misnamed.
                // It is an output.
                if (reactorBelongsToFederate(init.input.parent, federate)) {
                    var triggerStructName = triggerStructName(init.input)
                    pr(
                        init.remoteTriggersArrayName + '[' + init.arrayIndex +
                        '] = &' + triggerStructName + ';')
                }
            }
        }
        // Set all inputs _is_present variables to point to False by default.
        setInputsAbsentByDefault(main, federate)

        // Next, for every input port, populate its "self" struct
        // fields with pointers to the output port that send it data.
        connectInputsToOutputs(main, federate)
    }

    /** Generate assignments of pointers in the "self" struct of a destination
     *  port's reactor to the appropriate entries in the "self" struct of the
     *  source reactor.
     *  @param instance The reactor instance.
     *  @param federate The federate for which we are generating code or null
     *   if there is no federation.
     */
    private def void connectInputsToOutputs(ReactorInstance instance, FederateInstance federate) {
        pr('// Connect inputs and outputs.')
        for (source : instance.destinations.keySet) {
            var eventualSource = source
            // If the source is an input port, find the ultimate source,
            // which has to be an output port. Or it may be a dangling
            // connection, in which case the result will still be an
            // input port.
            while (eventualSource.isInput &&
                eventualSource.dependsOnPort !== null) {
                eventualSource = eventualSource.dependsOnPort
            }
            // If the eventual source is still an input, then this is a dangling
            // connection and we don't need to do anything.
            if (eventualSource.isOutput) {
                var sourceStruct = selfStructName(eventualSource.parent)
                // Use the source, not the eventualSource here to find the destinations.
                // If .parent.parent is null, then the source is an input port belonging
                // the top-level reactor, in which case, it cannot receive any inputs
                // and there is nothing to do.
                // NOTE: If the source is an input, we want to use just parent!
                val destinations = if (source.isInput) {
                        source.parent.destinations.get(source)
                    } else if (source.parent.parent !== null) {
                        source.parent.parent.destinations.get(source)
                    }
                if (destinations !== null) {
                    for (destination : destinations) {
                        var destStruct = selfStructName(destination.parent)

                        if (destination.isInput) {
                            pr(
                                destStruct + '.__' + destination.name + ' = &' +
                                    sourceStruct + '.__' +
                                    eventualSource.name + ';'
                            )
                            pr(
                                destStruct + '.__' + destination.name +
                                    '_is_present = &' + sourceStruct + '.__' +
                                    eventualSource.name + '_is_present;'
                            )
                        } else {
                            // Destination is an output.
                            var containerSelfStructName = selfStructName(
                                destination.parent)
                            pr(
                                containerSelfStructName + '.__' +
                                    destination.name + '_inside = &' +
                                    sourceStruct + '.__' + eventualSource.name +
                                    ';'
                            )
                            pr(
                                containerSelfStructName + '.__' +
                                    destination.name +
                                    '_inside_is_present = &' +
                                    sourceStruct + '.__' + eventualSource.name +
                                    '_is_present;'
                            )
                        }
                    }
                }
            }
        }

        for (child : instance.children) {
            // In case this is a composite, recurse.
            connectInputsToOutputs(child, federate)
        }

        var containerSelfStructName = selfStructName(instance)

        // Handle inputs that get sent data from a reaction rather than from
        // another contained reactor and reactions that are triggered by an
        // output of a contained reactor.
        for (reaction : instance.reactions) {
            for (port : reaction.dependentPorts) {
                if (port.definition instanceof Input) {
                    // This reaction is sending to an input. Must be
                    // the input of a contained reactor in the federate.
                    if (reactorBelongsToFederate(port.parent, federate)) {
                        var inputSelfStructName = selfStructName(port.parent)
                        pr(
                            inputSelfStructName + '.__' + port.definition.name +
                                ' = &' + containerSelfStructName + '.__' +
                                port.parent.definition.name + '.' +
                                port.definition.name + ';'
                        )
                        pr(
                            inputSelfStructName + '.__' + port.definition.name +
                                '_is_present = &' + containerSelfStructName +
                                '.__' + port.parent.definition.name + '.' +
                                port.definition.name + '_is_present;'
                        )
                        pr(
                            startTimeStep,
                            containerSelfStructName + '.__' +
                                port.parent.definition.name + '.' +
                                port.definition.name + '_is_present = false;'
                        )
                    }
                }
            }
            for (port : reaction.dependsOnPorts) {
                if (port.definition instanceof Output) {
                    // This reaction is receiving data from an output
                    // of a contained reactor. If the contained reactor is
                    // not in the federate, then we don't do anything here.
                    if (reactorBelongsToFederate(port.parent, federate)) {
                        var outputSelfStructName = selfStructName(port.parent)
                        pr(
                            containerSelfStructName + '.__' +
                                port.parent.definition.name + '.' +
                                port.definition.name + ' = &' +
                                outputSelfStructName + '.__' +
                                port.definition.name + ';'
                        )
                        pr(
                            containerSelfStructName + '.__' +
                                port.parent.definition.name + '.' +
                                port.definition.name + '_is_present' + ' = &' +
                                outputSelfStructName + '.__' +
                                port.definition.name + '_is_present;'
                        )
                    }
                }
            }
        }
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for ports in a reaction function
     *  from the "self" struct. The port may be an input of the
     *  reactor or an output of a contained reactor.
     *  @param builder The string builder.
     *  @param trigger The input statement from the AST.
     */
    private def generateInputVariablesInReaction(
        StringBuilder builder,
        Input input
    ) {
        var present = input.name + '_is_present'
        pr(builder,
            'bool ' + present + ' = *(self->__' + input.name + '_is_present);')
        
        if (isTokenType(input.type)) {
            pr(builder, rootType(input.type) + '* ' + input.name + ';')
            pr(builder, 'if(' + present + ') {')
            indent(builder)
            pr(builder, input.name + ' = ('
                + rootType(input.type)
                + '*)(self->__' + input.name + '->value);')
            // If the input is declared mutable, create a writable copy.
            // Note that this will not copy if the reference count is exactly one.
            if (input.isMutable) {
                pr(builder, input.name + ' = writable_copy(' + input.name + ');')
            }
        } else if (input.type !== null) {
            // Look for array type of form type[number].
            val matcher = arrayPatternFixed.matcher(input.type)
            if (matcher.find()) {
                pr(builder, matcher.group(1) + '* ' + input.name + ';')
            } else {
                pr(builder, input.type + ' ' + input.name + ';')
            }
            pr(builder, 'if(' + present + ') {')
            indent(builder)
            pr(builder, input.name + ' = *(self->__' + input.name + ');')
        }
        unindent(builder)
        pr(builder, '}')
        if (isTokenType(input.type)) {
            pr(builder, 'int ' + input.name + '_length = self->__' + input.name + '->length;')
        }
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for ports in a reaction function
     *  from the "self" struct. The port may be an input of the
     *  reactor or an output of a contained reactor.
     *  @param builder The string builder.
     *  @param trigger The input statement from the AST.
     */
    private def generatePortVariablesInReaction(
        StringBuilder builder,
        HashMap<Instantiation,StringBuilder> structs,
        VarRef port
    ) {
        if (port.variable instanceof Input) {
            generateInputVariablesInReaction(builder, port.variable as Input)
        } else {
            // port is an output of a contained reactor.
            val output = port.variable as Output
            val portName = output.name
            val portType = lfTypeToTokenType(output.type)
            
            var structBuilder = structs.get(port.container)
            if (structBuilder === null) {
                structBuilder = new StringBuilder
                structs.put(port.container, structBuilder)
            }
            val reactorName = port.container.name
            // First define the struct containing the output value, indicator
            // of its presence, and number of destinations.
            pr(structBuilder, portType + ' ' + portName + '; ')
            pr(structBuilder, 'bool ' + portName + '_is_present;')
            pr(structBuilder, 'int ' + portName + '_num_destinations;')

            // Next, initialize the struct with the current values.
            pr(
                builder,
                reactorName + '.' + portName + ' = *(self->__' + reactorName +
                    '.' + portName + ');'
            )
            pr(
                builder,
                reactorName + '.' + portName + '_is_present = *(self->__' +
                    reactorName + '.' + portName + '_is_present);'
            )
        }
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for outputs in a reaction function
     *  from the "self" struct.
     *  @param builder The string builder.
     *  @param output The output statement from the AST.
     */
    private def generateOutputVariablesInReaction(
        StringBuilder builder,
        Output output
    ) {
        if (output.type === null) {
            reportError(output,
                "Output is required to have a type: " + output.name)
        } else {
            val outputType = lfTypeToTokenType(output.type)
            // Define a variable of type 'type*' with name matching the output name.
            // If the output type has the form type[number],
            // then the variable is set equal to the pointer in the self struct
            // to the output value. Otherwise, if the output type has the form
            // type[], or type*, the variable is set to NULL.
            // Otherwise, it is set to the _address_ of the
            // entry in the self struct corresponding to the output.  
            val matcher = arrayPatternFixed.matcher(outputType)
            if (matcher.find()) {
                pr(
                    builder,
                    rootType(output.type) + '* ' + output.name +
                        ' = self->__' + output.name + ';'
                )
            } else if (isTokenType(output.type)) {
                pr(
                    builder,
                    rootType(output.type) + '* ' + output.name + ' = NULL;'
                )
            } else {
                pr(
                    builder,
                    outputType + '* ' + output.name +
                        ' = &(self->__' + output.name + ');'
                )
            }
            // Also define a boolean variable name_is_present with value
            // equal to the current value of the corresponding is_present field
            // in the self struct. This can be used to test whether a previous
            // reaction has already set an output value at the current logical time.
            pr(builder, 'bool ' + output.name + '_is_present = self->__'
                + output.name + '_is_present;'
            )
        }
    }

    /** Generate into the specified string builder the code to
     *  initialize local variables for sending data to an input
     *  of a contained reaction (e.g. for a deadline violation).
     *  The code goes into two builders because some of it has to
     *  collected into a single struct definition.
     *  @param builder The string builder.
     *  @param definition AST node defining the reactor within which this occurs
     *  @param input Input of the contained reactor.
     */
    private def generateVariablesForSendingToContainedReactors(
        StringBuilder builder,
        HashMap<Instantiation,StringBuilder> structs,
        Instantiation definition,
        Input input
    ) {
        var structBuilder = structs.get(definition)
        if (structBuilder === null) {
            structBuilder = new StringBuilder
            structs.put(definition, structBuilder)
        }
        pr(structBuilder, lfTypeToTokenType(input.type) + '* ' + input.name + ';')
        pr(structBuilder, ' bool ' + input.name + '_is_present;')        
        pr(structBuilder, ' int ' + input.name + '_num_destinations;')
        
        pr(builder,
            definition.name + '.' + input.name + ' = &(self->__' +
            definition.name + '.' + input.name + ');'
        )
        pr(builder,
            definition.name + '.' + input.name + '_is_present = self->__' +
            definition.name + '.' + input.name + '_is_present;'
        )
    }

    /** Return a C type for the type of the specified parameter.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "interval_t".
     *  If the type is of the form "type[]", then this is converted
     *  to "type*".
     *  @param parameter The parameter.
     *  @return The C type.
     */
    private def getParameterType(Parameter parameter) {
        if (parameter.ofTimeType) {
            return timeTypeInTargetLanguage
        }
        if (parameter.type === null || parameter.type.equals("")) {
            reportError(parameter,
                "Parameter is required to have a type: " + parameter.name)
            return "(ERROR: NO TYPE)"
        }
        var type = removeCodeDelimiter(parameter.type)
        if (parameter.unit != TimeUnit.NONE || parameter.isOfTimeType) {
            type = 'interval_t'
        } else {
            val matcher = arrayPatternVariable.matcher(type)
            if (matcher.find()) {
                return matcher.group(1) + '*'
            }
        }
        type
    }
    
    /** Return a C type for the type of the specified state variable.
     *  If there are code delimiters around it, those are removed.
     *  If the type is "time", then it is converted to "interval_t".
     *  If the type is of the form "type[]", then this is converted
     *  to "type*".
     *  @param state The state variable.
     *  @return The C type.
     */
    private def getStateType(State state) {
        // A state variable may directly refer to its initializing parameter,
        // in which case, it inherits the type from the parameter.
        if (state.parameter !== null) {
            return removeCodeDelimiter(state.parameter.type)
        }
        if (state.ofTimeType) {
            return timeTypeInTargetLanguage
        }
        if (state.type === null || state.type.equals("")) {
            reportError(state,
                "State is required to have a type: " + state.name)
            return "(ERROR: NO TYPE)"
        }
        var type = removeCodeDelimiter(state.type)
        if (state.unit != TimeUnit.NONE || state.isOfTimeType) {
            type = 'interval_t'
        } else {
            val matcher = arrayPatternVariable.matcher(type)
            if (matcher.find()) {
                return matcher.group(1) + '*'
            }
        }
        type
    }
    
    /** Given a type for an input or output, return true if it should be
     *  carried by a token_t struct rather than the type itself.
     *  It should be carried by such a struct if the type ends with *
     *  (it is a pointer) or [] (it is a array with unspecified length).
     *  @param type The type specification.
     */
    private def isTokenType(String type) {
        if(type !== null && 
                (type.trim.matches("^\\w*\\[\\s*\\]$") || type.trim.endsWith('*'))) {
            true
        } else {
            false
        }
    }
    
    /** If the type specification of the form type[] or
     *  type*, return the type. Otherwise remove the code delimiter,
     *  if there is one, and otherwise just return the argument
     *  unmodified. This should only be called on token types.
     */
    private def rootType(String type) {
        if(type.endsWith(']')) {
            val root = type.indexOf('[')
            type.substring(0, root).trim
        } else if (type.endsWith('*')) {
            type.substring(0, type.length - 1).trim
        } else {
            removeCodeDelimiter(type).trim
        }
    }

    /** Convert a type specification of the form type[], type[num]
     *  or type* to token_t. Otherwise, remove the code delimiter,
     *  if there is one, and otherwise just return the argument
     *  unmodified.
     */
    private def lfTypeToTokenType(String type) {
        var result = removeCodeDelimiter(type)
        if(isTokenType(type)) {
            result = 'token_t'
        }
        result
    }

    // Print the #line compiler directive with the line number of
    // the most recently used node.

    private def prSourceLineNumber(EObject eObject) {
        var node = NodeModelUtils.getNode(eObject)
        if (node !== null) {
            pr("#line " + node.getStartLine() + ' "' + resource.getURI() + '"')
        }

    }

    /** Set inputs _is_present variables to the default to point to False.
     *  @param parent The container reactor.
     *  @param federate The federate, or null if there is no federation.
     */
    private def void setInputsAbsentByDefault(ReactorInstance parent, FederateInstance federate) {
        // For all inputs, set a default where their _is_present variable points to False.
        // This handles dangling input ports that are not connected to anything
        // even if they are connected locally in the hierarchy, but not globally.
        for (containedReactor : parent.children) {
            // Do this only for reactors in the federate.
            if (reactorBelongsToFederate(containedReactor, federate)) {
                for (input : containedReactor.inputs) {
                    var inputSelfStructName = selfStructName(containedReactor)
                    pr(inputSelfStructName + '.__' + input.definition.name +
                        '_is_present = &False;')
                }
                // In case this is a composite, handle its assignments.
                setInputsAbsentByDefault(containedReactor, federate)
            }
        }
    }
    
    // Regular expression pattern for array types with specified length.
    static final Pattern arrayPatternFixed = Pattern.compile("^\\s*+(\\w+)\\s*(\\[[0-9]+\\])\\s*$");
    
    // Regular expression pattern for array types with unspecified length.
    static final Pattern arrayPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\[\\]\\s*$");
    
    static var DISABLE_REACTION_INITIALIZATION_MARKER
        = '// **** Do not include initialization code in this reaction.'
        
    static var DEFAULT_MIN_INTER_ARRIVAL = new TimeValue(1, TimeUnit.NSEC)
}
