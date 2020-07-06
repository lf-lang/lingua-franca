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
import java.util.HashMap
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.VarRef

import static extension org.icyphy.ASTUtils.*

/** 
 * Generator for C target. This class generates C code definining each reactor
 * class given in the input .lf file and imported .lf files. The generated code
 * has the following components:
 * 
 * * A typedef for inputs, outputs, and actions of each reactor class. These
 *   define the types of the variables that reactions use to access inputs and
 *   action values and to set output values.
 * 
 * * A typedef for a "self" struct for each reactor class. One instance of this
 *   struct will be created for each reactor instance. See below for details.
 * 
 * * A function definition for each reaction in each reactor class. These
 *   functions take an instance of the self struct as an argument.
 * 
 * * A constructor function for each reactor class. This is used to create
 *   a new instance of the reactor.
 * 
 * After these, the main generated function is `__initialize_trigger_objects()`.
 * This function creates the instances of reactors (using their constructors)
 * and makes connections between them.
 * 
 * A few other smaller functions are also generated.
 * 
 * ## Self Struct
 * 
 * The "self" struct has fields for each of the following:
 * 
 * * parameter: the field name and type match the parameter.
 * * state: the field name and type match the state.
 * * action: the field name prepends the action name with "__".
 *   A second field for the action is also created to house the trigger_t object.
 *   That second field prepends the action name with "___".
 * * output: the field name prepends the output name with "__".
 * * input:  the field name prepends the output name with "__".
 *   A second field for the input is also created to house the trigger_t object.
 *   That second field prepends the input name with "___".
 *
 * If, in addition, the reactor contains other reactors and reacts to their outputs,
 * then there will be a struct within the self struct for each such contained reactor.
 * The name of that self struct will be the name of the contained reactor prepended with "__".
 * That inside struct will contain pointers the outputs of the contained reactors
 * that are read together with pointers to booleans indicating whether those outputs are present.
 * 
 * If, in addition, the reactor has a reaction to shutdown, then there will be a pointer to
 * trigger_t object (see reactor.h) for the shutdown event and an action struct named
 * __shutdown on the self struct.
 * 
 * ## Reaction Functions
 * 
 * For each reaction in a reactor class, this generator will produce a C function
 * that expects a pointer to an instance of the "self" struct as an argument.
 * This function will contain verbatim the C code specified in the reaction, but
 * before that C code, the generator inserts a few lines of code that extract from the
 * self struct the variables that that code has declared it will use. For example, if
 * the reaction declares that it is triggered by or uses an input named "x" of type
 * int, the function will contain a line like this:
 * ```
 *     e_x_t* x = self->__x;
 * ```
 * where `r` is the full name of the reactor class and the struct type `r_x_t`
 * will be defined like this:
 * ```
 *     typedef struct {
 *         int value;
 *         bool is_present;
 *         int num_destinations;
 *     } r_x_t;
 * ```
 * The above assumes the type of `x` is `int`.
 * If the programmer fails to declare that it uses x, then the absence of the
 * above code will trigger a compile error when the verbatim code attempts to read `x`.
 *
 * ## Constructor
 * 
 * For each reactor class, this generator will create a constructor function named
 * `new_r`, where `r` is the reactor class name. This function will malloc and return
 * a pointer to an instance of the "self" struct.  This struct initially represents
 * an unconnected reactor. To establish connections between reactors, additional
 * information needs to be inserted (see below). The self struct is made visible
 * to the body of a reaction as a variable named "self".  The self struct contains the
 * following:
 * 
 * * Parameters: For each parameter `p` of the reactor, there will be a field `p`
 *   with the type and value of the parameter. So C code in the body of a reaction
 *   can access parameter values as `self->p`.
 * 
 * * State variables: For each state variable `s` of the reactor, there will be a field `s`
 *   with the type and value of the state variable. So C code in the body of a reaction
 *   can access state variables as as `self->s`.
 * 
 * The self struct also contains various fields that the user is not intended to
 * use. The names of these fields begin with at least two underscores. They are:
 * 
 * * Outputs: For each output named `out`, there will be a field `__out` that is
 *   a struct containing a value field whose type matches that of the output.
 *   The output value is stored here. That struct also has a field `is_present`
 *   that is a boolean indicating whether the output has been set.
 *   This field is reset to false at the start of every time
 *   step. There is also a field `num_destinations` whose value matches the
 *   number of downstream reactions that use this variable. This field must be
 *   set when connections are made or changed. It is used to initialize
 *   reference counts for dynamically allocated message payloads.
 * 
 * * Inputs: For each input named `in` of type T, there is a field named `__in`
 *   that is a pointer struct with a value field of type T. The struct pointed
 *   to also has an `is_present` field of type bool that indicates whether the
 *   input is present.
 * 
 * * Outputs of contained reactors: If a reactor reacts to outputs of a
 *   contained reactor `r`, then the self struct will contain a nested struct
 *   named `__r` that has fields pointing to those outputs. For example,
 *   if `r` has an output `out` of type T, then there will be field in `__r`
 *   named `out` that points to a struct containing a value field
 *   of type T and a field named `is_present` of type bool.
 * 
 * * Inputs of contained reactors: If a reactor sends to inputs of a
 *   contained reactor `r`, then the self struct will contain a nested struct
 *   named `__r` that has fields for storing the values provided to those
 *   inputs. For example, if R has an input `in` of type T, then there will
 *   be field in __R named `in` that is a struct with a value field
 *   of type T and a field named `is_present` of type bool.
 * 
 * * Actions: If the reactor has an action a (logical or physical), then there
 *   will be a field in the self struct named `__a` and another named `___a`.
 *   The type of the first is specific to the action and contains a `value`
 *   field with the type and value of the action (if it has a value). That
 *   struct also has a `has_value` field, an `is_present` field, and a
 *   `token` field (which is NULL if the action carries no value).
 *   The `___a` field is of type trigger_t.
 *   That struct contains various things, including an array of reactions
 *   sensitive to this trigger and a token_t struct containing the value of
 *   the action, if it has a value.  See reactor.h in the C library for
 *   details.
 * 
 * * Reactions: Each reaction will have several fields in the self struct.
 *   Each of these has a name that begins with `___reaction_i`, where i is
 *   the number of the reaction, starting with 0. The fields are:
 *   * ___reaction_i: The struct that is put onto the reaction queue to
 *     execute the reaction (see reactor.h in the C library).
 *   * ___reaction_i_outputs_are_present: An array of pointers to the
 *     __out_is_present fields of each output `out` that may be set by
 *     this reaction. This array also includes pointers to the _is_present
 *     fields of inputs of contained reactors to which this reaction writes.
 *     This array is set up by the constructor.
 *   * ___reaction_i_num_outputs: The size of the previous array.
 *   * ___reaction_i_triggers: This is an array of arrays of pointers
 *     to trigger_t structs. The first level array has one entry for
 *     each effect of the reaction that is a port (actions are ignored).
 *     Each such entry is an array containing pointers to trigger structs for
 *     downstream inputs.
 *   * ___reaction_i_triggered_sizes: An array indicating the size of
 *     each array in ___reaction_i_triggers. The size of this array is
 *     the number of ports that are effects of this reaction.
 * 
 *  * Timers: For each timer t, there is are two fields in the self struct:
 *    * ___t_trigger: The trigger_t struct for this timer (see reactor.h).
 *    * ___t_trigger_reactions: An array of reactions (pointers to the
 *      reaction_t structs on this self struct) sensitive to this timer.
 *
 * * Triggers: For each Timer, Action, Input, and Output of a contained
 *   reactor that triggers reactions, there will be a trigger_t struct
 *   on the self struct with name `___t`, where t is the name of the trigger.
 * 
 * ## Connections Between Reactors
 * 
 * Establishing connections between reactors involves two steps.
 * First, each destination (e.g. an input port) must have pointers to
 * the source (the output port). As explained above, for an input named
 * `in`, the field `__in->value` is a pointer to the output data being read.
 * In addition, `__in->is_present` is a pointer to the corresponding
 * `out->is_present` field of the output reactor's self struct.
 *  
 * In addition, the `reaction_i` struct on the self struct has a `triggers`
 * field that records all the trigger_t structs for ports and reactions
 * that are triggered by the i-th reaction. The triggers field is
 * an array of arrays of pointers to trigger_t structs.
 * The length of the outer array is the number of output ports the
 * reaction effects plus the number of input ports of contained
 * reactors that it effects. Each inner array has a length equal to the
 * number final destinations of that output port or input port.
 * The reaction_i struct has an array triggered_sizes that indicates
 * the sizes of these inner arrays. The num_outputs field of the
 * reaction_i struct gives the length of the triggered_sizes and
 * (outer) triggers arrays.
 * 
 * ## Runtime Tables
 * 
 * This generator creates an populates the following tables used at run time.
 * These tables may have to be resized and adjusted when mutations occur.
 * 
 * * __is_present_fields: An array of pointers to booleans indicating whether an
 *   event is present. The __start_time_step() function in reactor_common.c uses
 *   this to mark every event absent at the start of a time step. The size of this
 *   table is contained in the variable __is_present_fields_size.
 * 
 * * __tokens_with_ref_count: An array of pointers to structs that point to token_t
 *   objects, which carry non-primitive data types between reactors. This is used
 *   by the __start_time_step() function to decrement reference counts, if necessary,
 *   at the conclusion of a time step. Then the reference count reaches zero, the
 *   memory allocated for the token_t object will be freed.  The size of this
 *   array is stored in the __tokens_with_ref_count_size variable.
 * 
 * * __shutdown_triggers: An array of pointers to trigger_t structs for shutdown
 *   reactions. The length of this table is in the __shutdown_triggers_size
 *   variable.
 * 
 * * __timer_triggers: An array of pointers to trigger_t structs for timers that
 *   need to be started when the program runs. The length of this table is in the
 *   __timer_triggers_size variable.
 * 
 * * __action_table: For a federated execution, each federate will have this table
 *   that maps port IDs to the corresponding trigger_t struct.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Mehrdad Niknami <mniknami@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 */
class CCppGenerator extends CGenerator {
	
	// Set of acceptable import targets includes only C.
    val acceptableTargetSet = newHashSet('C','CCpp')
	
	new () {
        super()
        // set defaults
        this.targetCompiler = "g++"
        this.targetCompilerFlags = "-O2"// -Wall -Wconversion"
    }
	
	val template_port_type =  "template_input_output_port_struct"
	val template_port_type_with_token = "template_input_output_port_with_token_struct"
    
   ////////////////////////////////////////////
    //// Public methods
    
    
    ////////////////////////////////////////////
    //// Protected methods
    
    /** Return a set of targets that are acceptable to this generator.
     *  Imported files that are Lingua Franca files must specify targets
     *  in this set or an error message will be reported and the import
     *  will be ignored. The returned set contains only "C".
     */
    override acceptableTargets() {
        acceptableTargetSet
    }
    
    
    override includeTargetLanguageHeaders()
    {    	
        pr('#include "ccpptarget.h"')
    }
    
    override getTargetFileName(String fileName)
    {
    	return fileName + ".cc";
    }

    /** Generate C code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
            IGeneratorContext context) {
            	super.doGenerate(resource, fsa, context);
            }
            
            
    override copyTargetFiles()
    {    	
        var srcGenPath = directory + File.separator + "src-gen"
    	// Copy the required target language files into the target file system.
        // This will also overwrite previous versions.
        var targetFiles = newArrayList("ccpptarget.h");
        for (file : targetFiles) {
            copyFileFromClassPath(
                "/" + "lib" + "/" + "CCpp" + "/" + file,
                srcGenPath + File.separator + file
            )
        }
    }
            
    /** Generate into the specified string builder the code to
     *  initialize local variables for ports in a reaction function
     *  from the "self" struct. The port may be an input of the
     *  reactor or an output of a contained reactor. The second
     *  argument provides, for each contained reactor, a place to
     *  write the declaration of the output of that reactor that
     *  is triggering reactions.
     *  @param builder The string builder into which to write the code.
     *  @param structs A map from reactor instantiations to a place to write
     *   struct fields.
     *  @param port The port.
     *  @param reactor The reactor.
     */
    override generatePortVariablesInReaction(
        StringBuilder builder,
        HashMap<Instantiation,StringBuilder> structs,
        VarRef port,
        Reactor reactor
    ) {
        if (port.variable instanceof Input) {
            generateInputVariablesInReaction(builder, port.variable as Input, reactor)
        } else {
            // port is an output of a contained reactor.
            val output = port.variable as Output
            val portName = output.name
            
            val outputType = output.inferredType
            
            var structBuilder = structs.get(port.container)
            if (structBuilder === null) {
                structBuilder = new StringBuilder
                structs.put(port.container, structBuilder)
            }
            val reactorName = port.container.name
            if(!outputType.isTokenType)
            {
                // First define the struct containing the output value and indicator
                // of its presence.
                pr(structBuilder, '''
                   «template_port_type»<«outputType.targetType»>* «portName»;
                ''')

                // Next, initialize the struct with the current values.
                pr(builder, '''
                    «reactorName».«portName» = («template_port_type»<«outputType.targetType»> *) self->__«reactorName».«portName»;
                ''')
            }
            else
            {
            	// First define the struct containing the output value and indicator
                // of its presence.
                pr(structBuilder, '''
                   «template_port_type_with_token»<«outputType.targetType»>* «portName»;
                ''')

                // Next, initialize the struct with the current values.
                pr(builder, '''
                    «reactorName».«portName» = («template_port_type_with_token»<«outputType.targetType»> *) self->__«reactorName».«portName»;
                ''')
            }
        }
    }
    
    
    /** Generate into the specified string builder the code to
     *  initialize local variables for the specified input port
     *  in a reaction function from the "self" struct.
     *  @param builder The string builder.
     *  @param input The input statement from the AST.
     *  @param reactor The reactor.
     */
    override generateInputVariablesInReaction(
        StringBuilder builder,
        Input input,
        Reactor reactor
    ) {
        val structType = variableStructType(input, reactor)
        val inputType = input.inferredType
        // Create the local variable whose name matches the input name.
        // If the input has not been declared mutable, then this is a pointer
        // to the upstream output. Otherwise, it is a copy of the upstream output,
        // which nevertheless points to the same token and value (hence, as done
        // below, we have to use writable_copy()). There are 8 cases,
        // depending on whether the input is mutable, whether it is a multiport,
        // and whether it is a token type.
        // Easy case first.
        if (!input.isMutable && !inputType.isTokenType && input.multiportWidth <= 0) {
            // Non-mutable, non-multiport, primitive type.
            pr(builder, '''
                «template_port_type»<«inputType.targetType»>* «input.name» = («template_port_type»<«inputType.targetType»> *) self->__«input.name»;
            ''')
        } else if (input.isMutable && !inputType.isTokenType && input.multiportWidth <= 0) {
            // Mutable, non-multiport, primitive type.
            pr(builder, '''
                // Mutable input, so copy the input into a temporary variable.
                // The input value on the struct is a copy.
                «structType» __tmp_«input.name» = *(self->__«input.name»);
                «template_port_type»<«inputType.targetType»>* «input.name» = («template_port_type»<«inputType.targetType»> *)&__tmp_«input.name»;
            ''')
        } else if (!input.isMutable && inputType.isTokenType && input.multiportWidth <= 0) {
            // Non-mutable, non-multiport, token type.
            pr(builder, '''
                «template_port_type_with_token»<«inputType.targetType»>* «input.name» = («template_port_type_with_token»<«inputType.targetType»> *) self->__«input.name»;
                if («input.name»->is_present) {
                    «input.name»->length = «input.name»->token->length;
                    «input.name»->value = («inputType.targetType»)«input.name»->token->value;
                } else {
                    «input.name»->length = 0;
                }
            ''')
        } else if (input.isMutable && inputType.isTokenType && input.multiportWidth <= 0) {
            // Mutable, non-multiport, token type.
            pr(builder, '''
                // Mutable input, so copy the input struct into a temporary variable.
                «structType» __tmp_«input.name» = *(self->__«input.name»);
                «template_port_type_with_token»<«inputType.targetType»>* «input.name» = («template_port_type_with_token»<«inputType.targetType»> *) &__tmp_«input.name»;
                if («input.name»->is_present) {
                    «input.name»->length = «input.name»->token->length;
                    token_t* _lf_input_token = «input.name»->token;
                    «input.name»->token = writable_copy(_lf_input_token);
                    if («input.name»->token != _lf_input_token) {
                        // A copy of the input token has been made.
                        // This needs to be reference counted.
                        «input.name»->token->ref_count = 1;
                        // Repurpose the next_free pointer on the token to add to the list.
                        «input.name»->token->next_free = _lf_more_tokens_with_ref_count;
                        _lf_more_tokens_with_ref_count = «input.name»->token;
                    }
                    «input.name»->value = («inputType.targetType»)«input.name»->token->value;
                } else {
                    «input.name»->length = 0;
                }
            ''')            
        } else if (!input.isMutable && !inputType.isTokenType && input.multiportWidth > 0) {
            // Non-mutable, multiport, primitive type.
            pr(builder, '''
                «template_port_type»<«inputType.targetType»>** «input.name» = («template_port_type»<«inputType.targetType»> **) self->__«input.name»;
            ''')
        } else {
            throw new RuntimeException("FIXME: Multiport functionality not yet realized.")
        }
        // Set the _width variable for all cases. This will be -1
        // for a variable-width multiport, which is not currently supported.
        // It will be -2 if it is not multiport.
        pr(builder, '''
            int «input.name»_width = «input.multiportWidth»;
        ''')
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
    override generateVariablesForSendingToContainedReactors(
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
        val inputType = input.inferredType
        val inputStructType = variableStructType(input, definition.reactorClass)
        if(!inputType.isTokenType)
        {
            pr(structBuilder, '''
                «template_port_type»<«inputType.targetType»>* «input.name»;
            ''')
        
            pr(builder, '''
                «definition.name».«input.name» = («template_port_type»<«inputType.targetType»> *) &(self->__«definition.name».«input.name»);
            '''
            )
        }
        else
        {
            pr(structBuilder, '''
                «template_port_type_with_token»<«inputType.targetType»>* «input.name»;
            ''')
        
            pr(builder, '''
                «definition.name».«input.name» = («template_port_type_with_token»<«inputType.targetType»> *) &(self->__«definition.name».«input.name»);
            '''
            )
        }   
    }
    
    
    /** Generate into the specified string builder the code to
     *  initialize local variables for outputs in a reaction function
     *  from the "self" struct.
     *  @param builder The string builder.
     *  @param output The output statement from the AST.
     */
    override generateOutputVariablesInReaction(
        StringBuilder builder,
        Output output,
        Reactor reactor
    ) {
    	
        val outputType = output.inferredType
        if (output.type === null) {
            reportError(output,
                "Output is required to have a type: " + output.name)
        } else {
            val outputStructType = variableStructType(output, reactor)
            // Unfortunately, for the SET macros to work out-of-the-box for
            // multiports, we need an array of *pointers* to the output structs,
            // but what we have on the self struct is an array of output structs.
            // So we have to handle multiports specially here a construct that
            // array of pointers.
            if (!output.isMultiport) {
            	if (!outputType.isTokenType) {
                    pr(builder, '''
                        «template_port_type»<«outputType.targetType»>* «output.name» = («template_port_type»<«outputType.targetType»> *) &self->__«output.name»;
                    ''')
                }
                else 
                {
                    pr(builder, '''
                        «template_port_type_with_token»<«outputType.targetType»>* «output.name» = («template_port_type_with_token»<«outputType.targetType»> *) &self->__«output.name»;
                    ''')                	
                }
            } else {
            	if (!outputType.isTokenType) {
                    pr(builder, '''
                        «template_port_type»<«outputType.targetType»>* «output.name»[«output.multiportWidth»];
                        for(int i=0; i < «output.multiportWidth»; i++) {
                             «output.name»[i] = («template_port_type»<«outputType.targetType»> *) &(self->__«output.name»[i]);
                        }
                    ''')
                }
                else
                {
                    pr(builder, '''
                        «template_port_type_with_token»<«outputType.targetType»>* «output.name»[«output.multiportWidth»];
                        for(int i=0; i < «output.multiportWidth»; i++) {
                             «output.name»[i] = («template_port_type_with_token»<«outputType.targetType»> *) &(self->__«output.name»[i]);
                        }
                    ''')
                }
            }
        }
    }
    
    
    /** Overwrite the generated code after compile with a
     * clean version.
     */
    override writeCleanCode(String baseFilename)
    {
        var srcGenPath = directory + File.separator + "src-gen"
    	//Cleanup the code so that it is more readable
        for (federate : federates) {
                
            // Only clean one file if there is no federation.
            if (!federate.isSingleton) {                
                filename = baseFilename + '_' + federate.name               
            }
            
            // Derive target filename from the .lf filename.
            val cFilename = filename + ".cc";
            
            
            // Write a clean version of the code to the output file
            var fOut = new FileOutputStream(
            new File(srcGenPath + File.separator + cFilename), false);
            fOut.write(getReadableCode().getBytes())
            fOut.close()
            
        }
    	
    }
    
    
    
    /** FIXME: This function is copied from the CGenerator to enable federated
     *  execution. Ideally, the CGenerator.createLauncher() function should be refactored
     *  into a more flexible format that allows for various target source code extensions.
     * 
     *  Create the launcher shell scripts. This will create one or two file
     *  in the output path (bin directory). The first has name equal to
     *  the filename of the source file without the ".lf" extension.
     *  This will be a shell script that launches the
     *  RTI and the federates.  If, in addition, either the RTI or any
     *  federate is mapped to a particular machine (anything other than
     *  the default "localhost" or "0.0.0.0"), then this will generate
     *  a shell script in the bin directory with name filename_distribute.sh
     *  that copies the relevant source files to the remote host and compiles
     *  them so that they are ready to execute using the launcher.
     * 
     *  A precondition for this to work is that the user invoking this
     *  code generator can log into the remote host without supplying
     *  a password. Specifically, you have to have installed your
     *  public key (typically found in ~/.ssh/id_rsa.pub) in
     *  ~/.ssh/authorized_keys on the remote host. In addition, the
     *  remote host must be running an ssh service.
     *  On an Arch Linux system using systemd, for example, this means
     *  running:
     * 
     *      sudo systemctl <start|enable> ssh.service
     * 
     *  Enable means to always start the service at startup, whereas
     *  start means to just start it this once.
     *  On MacOS, open System Preferences from the Apple menu and 
     *  click on the "Sharing" preference panel. Select the checkbox
     *  next to "Remote Login" to enable it.
     */
    override createLauncher() {
        // NOTE: It might be good to use screen when invoking the RTI
        // or federates remotely so you can detach and the process keeps running.
        // However, I was unable to get it working properly.
        // What this means is that the shell that invokes the launcher
        // needs to remain live for the duration of the federation.
        // If that shell is killed, the federation will die.
        // Hence, it is reasonable to launch the federation on a
        // machine that participates in the federation, for example,
        // on the machine that runs the RTI.  The command I tried
        // to get screen to work looks like this:
        // ssh -t «target» cd «path»; screen -S «filename»_«federate.name» -L bin/«filename»_«federate.name» 2>&1
        
        var outPath = directory + File.separator + "bin"

        val shCode = new StringBuilder()
        val distCode = new StringBuilder()
        pr(shCode, '''
            #!/bin/bash
            # Launcher for federated «filename».lf Lingua Franca program.
            # Uncomment to specify to behave as close as possible to the POSIX standard.
            # set -o posix
            # Set a trap to kill all background jobs on error.
            trap 'echo "#### Killing federates."; kill $(jobs -p)' ERR
            # Launch the federates:
        ''')
        val distHeader = '''
            #!/bin/bash
            # Distributor for federated «filename».lf Lingua Franca program.
            # Uncomment to specify to behave as close as possible to the POSIX standard.
            # set -o posix
        '''
        val host = federationRTIProperties.get('host')
        var target = host

        var path = federationRTIProperties.get('dir')
        if(path === null) path = 'LinguaFrancaRemote'

        var user = federationRTIProperties.get('user')
        if (user !== null) {
            target = user + '@' + host
        }
        for (federate : federates) {
            if (federate.host !== null && federate.host != 'localhost' && federate.host != '0.0.0.0') {
                if(distCode.length === 0) pr(distCode, distHeader)
                pr(distCode, '''
                    echo "Making directory «path» and subdirectories src-gen and path on host «federate.host»"
                    ssh «federate.host» mkdir -p «path»/src-gen «path»/bin «path»/log «path»/src-gen/core
                    pushd src-gen/core > /dev/
                    echo "Copying LF core files to host «federate.host»"
                    scp reactor_common.c reactor.h pqueue.c pqueue.h util.h util.c reactor_threaded.c federate.c rti.h «federate.host»:«path»/src-gen/core
                    popd > /dev/null
                    pushd src-gen > /dev/null
                    echo "Copying source files to host «federate.host»"
                    scp «filename»_«federate.name».cc ctarget.h «federate.host»:«path»/src-gen
                    popd > /dev/null
                    echo "Compiling on host «federate.host» using: «this.targetCompiler» -O2 src-gen/«filename»_«federate.name».cc -o bin/«filename»_«federate.name» -pthread"
                    ssh «federate.host» 'cd «path»; «this.targetCompiler» -O2 src-gen/«filename»_«federate.name».cc -o bin/«filename»_«federate.name» -pthread'
                ''')
                pr(shCode, '''
                    echo "#### Launching the federate «federate.name» on host «federate.host»"
                    ssh «federate.host» '\
                        cd «path»; bin/«filename»_«federate.name» >& log/«filename»_«federate.name».out; \
                        echo "****** Output from federate «federate.name» on host «federate.host»:"; \
                        cat log/«filename»_«federate.name».out; \
                        echo "****** End of output from federate «federate.name» on host «federate.host»"' &
                ''')                
            } else {
                pr(shCode, '''
                    echo "#### Launching the federate «federate.name»."
                    «outPath»«File.separator»«filename»_«federate.name» &
                ''')                
            }
        }
        // Launch the RTI in the foreground.
        if (host == 'localhost' || host == '0.0.0.0') {
            pr(shCode, '''
                echo "#### Launching the runtime infrastructure (RTI)."
                «outPath»«File.separator»«filename»_RTI
            ''')
        } else {
            // Copy the source code onto the remote machine and compile it there.
            if (distCode.length === 0) pr(distCode, distHeader)
            // The mkdir -p flag below creates intermediate directories if needed.
            pr(distCode, '''
                cd «path»
                echo "Making directory «path» and subdirectories src-gen and path on host «target»"
                ssh «target» mkdir -p «path»/src-gen «path»/bin «path»/log «path»/src-gen/core
                pushd src-gen/core > /dev/null
                echo "Copying LF core files to host «target»"
                scp rti.c rti.h util.h util.c reactor.h pqueue.h «target»:«path»/src-gen/core
                popd > /dev/null
                pushd src-gen > /dev/null
                echo "Copying source files to host «target»"
                scp «filename»_RTI.cc ctarget.h «target»:«path»/src-gen
                popd > /dev/null
                echo "Compiling on host «target» using: «this.targetCompiler» -O2 «path»/src-gen/«filename»_RTI.cc -o «path»/bin/«filename»_RTI -pthread"
                ssh «target» '«this.targetCompiler» -O2 «path»/src-gen/«filename»_RTI.cc -o «path»/bin/«filename»_RTI -pthread'
            ''')

            // Launch the RTI on the remote machine using ssh and screen.
            // The -t argument to ssh creates a virtual terminal, which is needed by screen.
            // The -S gives the session a name.
            // The -L option turns on logging. Unfortunately, the -Logfile giving the log file name
            // is not standardized in screen. Logs go to screenlog.0 (or screenlog.n).
            // FIXME: Remote errors are not reported back via ssh from screen.
            // How to get them back to the local machine?
            // Perhaps use -c and generate a screen command file to control the logfile name,
            // but screen apparently doesn't write anything to the log file!
            //
            // The cryptic 2>&1 reroutes stderr to stdout so that both are returned.
            // The sleep at the end prevents screen from exiting before outgoing messages from
            // the federate have had time to go out to the RTI through the socket.
            pr(shCode, '''
                echo "#### Launching the runtime infrastructure (RTI) on remote host «host»."
                ssh «target» 'cd «path»; \
                    bin/«filename»_RTI >& log/«filename»_RTI.out; \
                    echo "------ output from «filename»_RTI on host «target»:"; \
                    cat log/«filename»_RTI.out; \
                    echo "------ end of output from «filename»_RTI on host «target»"'
            ''')
        }

        // Write the launcher file.
        // Delete file previously produced, if any.
        var file = new File(outPath + File.separator + filename)
        if (file.exists) {
            file.delete
        }
                
        var fOut = new FileOutputStream(file)
        fOut.write(shCode.toString().getBytes())
        fOut.close()
        if (!file.setExecutable(true, false)) {
            reportWarning(null, "Unable to make launcher script executable.")
        }
        
        // Write the distributor file.
        // Delete the file even if it does not get generated.
        file = new File(outPath + File.separator + filename + '_distribute.sh')
        if (file.exists) {
            file.delete
        }
        if (distCode.length > 0) {
            fOut = new FileOutputStream(file)
            fOut.write(distCode.toString().getBytes())
            fOut.close()
            if (!file.setExecutable(true, false)) {
                reportWarning(null, "Unable to make distributor script executable.")
            }
        }
    }
    
    
}
