/* Utilities for C code generation. */

/*************
Copyright (c) 2019-2021, The University of California at Berkeley.

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

package org.lflang.generator.c;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.TargetConfig.Mode;
import org.lflang.TimeValue;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.generator.ValueGenerator;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.TimeUnit;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthTerm;
import org.lflang.util.LFCommand;

/**
 * A collection of utilities for C code generation.
 * This class codifies the coding conventions for the C target code generator.
 * I.e., it defines how variables are named and referenced.
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
public class CUtil {

    //////////////////////////////////////////////////////
    //// Public fields.

    /**
     * Encapsulation of C-specific functions for representing values
     * in C. Targets that derive from CGenerator must either reuse this
     * ValueGenerator or create their own ValueGenerator instances.
     */
    public static final ValueGenerator VG = new ValueGenerator(
            CUtil::timeInTargetLanguage,    // Time value readable in C.
            CUtil::getTargetReference       // Name of a parameter.  FIXME: Not used. Edit: getTargetReference.apply is called in the implementation of ValueGenerator, so it is used?
    );

    //////////////////////////////////////////////////////
    //// Public methods.

    /**
     * Return a name of a variable to refer to the bank index of a reactor
     * in a bank. This is has the form uniqueID_i where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * If the instance is not a bank, return "0".
     * @param instance A reactor instance.
     */
    static public String bankIndex(ReactorInstance instance) {
        return bankIndex(instance, null);
    }

    /**
     * Return a name of a variable to refer to the bank index of a reactor
     * in a bank. This is has the form uniqueID_suffix where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program and suffix is the
     * specified suffix.  If the suffix is null, then "_i" is used.
     * If the instance is not a bank, return "0".
     * @param instance A reactor instance.
     * @param suffix The suffix or null to use a default suffix.
     */
    static public String bankIndex(ReactorInstance instance, String suffix) {
        if (!instance.isBank()) return "0";
        if (suffix == null) suffix = "_i";
        return instance.uniqueID() + suffix;
    }

    /**
     * Return a name of a variable to refer to the channel index of a port
     * in a bank. This is has the form uniqueID_c where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * If the port is not a multiport, then return the string "0".
     * @param instance A reactor instance.
     */
    static public String channelIndex(PortInstance port) {
        return channelIndex(port, null);
    }

    /**
     * Return a name of a variable to refer to the channel index of a port
     * in a bank. This is has the form uniqueIDsuffix where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * @param suffix The suffix on the name, or null to use the default "_c".
     */
    static public String channelIndex(PortInstance port, String suffix) {
        if (!port.isMultiport()) return "0";
        if (suffix == null) suffix = "_c";
        return port.uniqueID() + suffix;
    }

    /**
     * Return an expression that, when evaluated in a context with
     * bank index variables defined for the specified reactor and
     * any container(s) that are also banks, returns a unique index
     * for a runtime reactor instance. This can be used to maintain an
     * array of runtime instance objects, each of which will have a
     * unique index.
     * 
     * This is rather complicated because
     * this reactor instance and any of its parents may actually
     * represent a bank of runtime reactor instances rather a single
     * runtime instance. This method returns an expression that should
     * be evaluatable in any target language that uses + for addition
     * and * for multiplication and has defined variables in the context
     * in which this will be evaluated that specify which bank member is
     * desired for this reactor instance and any of its parents that is
     * a bank.  The names of these variables need to be values returned
     * by bankIndex().
     * 
     * If this is a top-level reactor, this returns "0".
     * 
     * @see bankIndex(ReactorInstance)
     * @param prefix The prefix used for index variables for bank members.
     */
    static public String indexExpression(ReactorInstance instance) {
        return indexExpression(instance, null);
    }
    
    /**
     * Return an expression that, when evaluated in a context with
     * bank index variables defined for the specified reactor and
     * any container(s) that are also banks, returns a unique index
     * for a runtime reactor instance. This can be used to maintain an
     * array of runtime instance objects, each of which will have a
     * unique index.
     * 
     * This is rather complicated because
     * this reactor instance and any of its parents may actually
     * represent a bank of runtime reactor instances rather a single
     * runtime instance. This method returns an expression that should
     * be evaluatable in any target language that uses + for addition
     * and * for multiplication and has defined variables in the context
     * in which this will be evaluated that specify which bank member is
     * desired for this reactor instance and any of its parents that is
     * a bank.  The names of these variables need to be values returned
     * by {@link bankIndex(ReactorInstance, String)}), where the second
     * argument is the specified suffix.
     * 
     * If this is a top-level reactor, this returns "0".
     * 
     * @param prefix The prefix used for index variables for bank members.
     * @param suffix The suffix to use for bank indices, or null to use the default.
     */
    static public String indexExpression(ReactorInstance instance, String suffix) {
        if (instance.getDepth() == 0) return("0");
        if (instance.isBank()) {
            return(
                    // Position of the bank member relative to the bank.
                    bankIndex(instance, suffix) + " * " + instance.getNumReactorInstances()
                    // Position of the bank within its parent.
                    + " + " + instance.getIndexOffset()
                    // Position of the parent.
                    + " + " + indexExpression(instance.getParent(), suffix)
            );
        } else {
            return(
                    // Position within the parent.
                    instance.getIndexOffset()
                    // Position of the parent.
                    + " + " + indexExpression(instance.getParent(), suffix)
            );
        }
    }
    
    /**
     * Return a reference to the specified port on the self struct of the specified
     * container reactor. The port is required to have the reactor as either its
     * parent or its parent parent or an exception will be thrown.
     * 
     * The returned string will have one of the following forms:
     * 
     * * selfStruct->_lf_portName
     * * selfStruct->_lf_portName[i]
     * * selfStruct->_lf_parent.portName
     * * selfStruct->_lf_parent.portName[i]
     * * selfStruct->_lf_parent[j].portName
     * * selfStruct->_lf_parent[j].portName[i]
     * 
     * where the index j is present if the parent is a bank and is
     * the string returned by {@link bankIndex(ReactorInstance)}, and
     * the index i is present if the port is a multiport and is
     * the string returned by {@link channelIndex(PortInstance)}.
     * 
     * The first two forms are used if isNested is false,
     * and the remaining four are used if isNested is true.
     * Set isNested to true when referencing a port belonging
     * to a contained reactor.
     *
     * @param port The port.
     * @param isNested True to return a reference relative to the parent's parent.
     * @param includeChannelIndex True to include the channel index at the end.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String portRef(
            PortInstance port, boolean isNested, boolean includeChannelIndex, String suffix
    ) {
        String channel = "";
        if (suffix == null) suffix = "";
        if (port.isMultiport() && includeChannelIndex) {
            channel = "[" + channelIndex(port) + "]";
        }
        if (isNested) {
            return reactorRefNested(port.getParent(), suffix) + "." + port.getName() + channel;
        } else {
            String sourceStruct = CUtil.reactorRef(port.getParent(), suffix);
            return sourceStruct + "->_lf_" + port.getName() + channel;
        }
    }
    
    /**
     * Special case of {@link portRef(PortInstance, boolean, boolean)}
     * that provides a reference to the port on the self struct of the
     * port's parent.  This is used when an input port triggers a reaction
     * in the port's parent or when an output port is written to by
     * a reaction in the port's parent.
     * This is equivalent to calling `portRef(port, false, true, null)`.
     * @param port An instance of the port to be referenced.
     */
    static public String portRef(PortInstance port) {                
        return portRef(port, false, true, null);
    }

    /**
     * Special case of {@link portRef(PortInstance, boolean, boolean)}
     * that provides a reference to the port on the self struct of the
     * port's parent.  This is used when an input port triggers a reaction
     * in the port's parent or when an output port is written to by
     * a reaction in the port's parent.
     * This is equivalent to calling `portRef(port, false, true, suffix)`.
     * @param port An instance of the port to be referenced.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String portRef(PortInstance port, String suffix) {                
        return portRef(port, false, true, suffix);
    }

    /**
     * Return the portRef without the channel indexing.
     * This is useful for deriving a reference to the _width variable.
     * @param port An instance of the port to be referenced.
     */
    static public String portRefName(PortInstance port) {                
        return portRef(port, false, false, null);
    }

    /**
     * Return the portRef without the channel indexing.
     * This is useful for deriving a reference to the _width variable.
     * @param port An instance of the port to be referenced.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String portRefName(PortInstance port, String suffix) {                
        return portRef(port, false, false, suffix);
    }
    
    /**
     * Special case of {@link portRef(PortInstance, boolean, boolean)}
     * that provides a reference to the port on the self struct of the
     * parent of the port's parent.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent.
     * This is equivalent to calling `portRef(port, true, true)`.
     *
     * @param port The port.
     */
    static public String portRefNested(PortInstance port) {
        return portRef(port, true, true, null);
    }

    /**
     * Special case of {@link portRef(PortInstance, boolean, boolean)}
     * that provides a reference to the port on the self struct of the
     * parent of the port's parent.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent.
     * This is equivalent to calling `portRef(port, true, true)`.
     *
     * @param port The port.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String portRefNested(PortInstance port, String suffix) {
        return portRef(port, true, true, suffix);
    }

    /**
     * Special case of {@link portRef(PortInstance, boolean, boolean)}
     * that provides a reference to the port on the self struct of the
     * parent of the port's parent, but without the channel indexing,
     * even if it is a multiport.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent.
     * This is equivalent to calling `portRef(port, true, false)`.
     *
     * @param port The port.
     */
    static public String portRefNestedName(PortInstance port) {
        return portRef(port, true, false, null);
    }

    /**
     * Special case of {@link portRef(PortInstance, boolean, boolean)}
     * that provides a reference to the port on the self struct of the
     * parent of the port's parent, but without the channel indexing,
     * even if it is a multiport.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent.
     * This is equivalent to calling `portRef(port, true, false)`.
     *
     * @param port The port.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String portRefNestedName(PortInstance port, String suffix) {
        return portRef(port, true, false, suffix);
    }

    /**
     * Return a reference to the reaction entry on the self struct
     * of the parent of the specified reaction.
     * @param reaction The reaction.
     */
    static public String reactionRef(ReactionInstance reaction) {
        return reactionRef(reaction, null);
    }

    /**
     * Return a reference to the reaction entry on the self struct
     * of the parent of the specified reaction.
     * @param reaction The reaction.
     * @param suffix A suffix to use for the parent reactor or null for the default.
     */
    static public String reactionRef(ReactionInstance reaction, String suffix) {
        return reactorRef(reaction.getParent(), suffix) + "->_lf__reaction_" + reaction.index;
    }

    /** 
     * Return a name for a pointer to the "self" struct of the specified
     * reactor instance.
     * @param instance The reactor instance.
     * @return A name to use for a pointer to the self struct.
     */
    static public String reactorRef(ReactorInstance instance) {
        return reactorRef(instance, null);
    }

    /** 
     * Return a name for a pointer to the "self" struct of the specified
     * reactor instance.
     * @param instance The reactor instance.
     * @return A name to use for a pointer to the self struct.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String reactorRef(ReactorInstance instance, String suffix) {
        if (suffix == null) suffix = "";
        return instance.uniqueID() + "_self" + suffix;
    }
    
    /**
     * For situations where a reaction reacts to or reads from an output
     * of a contained reactor or sends to an input of a contained reactor,
     * then the container's self struct will have a field
     * (or an array of fields if the contained reactor is a bank) that is
     * a struct with fields corresponding to those inputs and outputs.
     * This method returns a reference to that struct or array of structs.
     * Note that the returned reference is not to the self struct of the
     * contained reactor. Use {@link reactorRef(ReactorInstance)} for that.
     * 
     * @param reactor The contained reactor.
     */
    static public String reactorRefNested(ReactorInstance reactor) {
        return reactorRefNested(reactor, null);
    }

    /**
     * For situations where a reaction reacts to or reads from an output
     * of a contained reactor or sends to an input of a contained reactor,
     * then the container's self struct will have a field
     * (or an array of fields if the contained reactor is a bank) that is
     * a struct with fields corresponding to those inputs and outputs.
     * This method returns a reference to that struct or array of structs.
     * Note that the returned reference is not to the self struct of the
     * contained reactor. Use {@link reactorRef(ReactorInstance)} for that.
     * 
     * @param reactor The contained reactor.
     * @param suffix An optional suffix to append to the struct variable name.
     */
    static public String reactorRefNested(ReactorInstance reactor, String suffix) {
        String result = reactorRef(reactor.getParent(), suffix) + "->_lf_" + reactor.getName();
        if (reactor.isBank()) {
            result += "[" + bankIndex(reactor) + "]";
        }
        return result;
    }

    /** 
     * Return a unique type for the "self" struct of the specified
     * reactor class from the reactor class.
     * @param reactor The reactor class.
     * @return The type of a self struct for the specified reactor class.
     */
    static public String selfType(ReactorDecl reactor) {
        return reactor.getName().toLowerCase() + "_self_t";
    }
    
    /** 
     * Construct a unique type for the "self" struct of the specified
     * reactor class from the reactor class.
     * @param reactor The reactor class.
     * @return The name of the self struct.
     */
    static public String selfType(ReactorInstance instance) {
        return selfType(instance.getDefinition().getReactorClass());
    }

    /** Return a reference to the trigger_t struct of the specified
     *  trigger instance (input port or action). This trigger_t struct
     *  is on the self struct.
     *  @param instance The port or action instance.
     *  @return The name of the trigger struct.
     */
    static public String triggerRef(TriggerInstance<? extends Variable> instance) {
        return triggerRef(instance, null);
    }

    /** Return a reference to the trigger_t struct of the specified
     *  trigger instance (input port or action). This trigger_t struct
     *  is on the self struct.
     *  @param instance The port or action instance.
     *  @param suffix The suffix to use for the reactor reference or null for default.
     *  @return The name of the trigger struct.
     */
    static public String triggerRef(TriggerInstance<? extends Variable> instance, String suffix) {
        return reactorRef(instance.getParent(), suffix) 
                + "->_lf__"
                + instance.getName();
    }
    
    /** Return a reference to the trigger_t struct for the specified
     *  port of a contained reactor.
     *  @param port The output port of a contained reactor.
     */
    static public String triggerRefNested(PortInstance port) {
        return triggerRefNested(port, null);
    }

    /** Return a reference to the trigger_t struct for the specified
     *  port of a contained reactor.
     *  @param port The output port of a contained reactor.
     *  @param suffix The suffix to use for the reactor reference or null for default.
     */
    static public String triggerRefNested(PortInstance port, String suffix) {
        return reactorRefNested(port.getParent(), suffix) + "." + port.getName() + "_trigger";
    }

    //////////////////////////////////////////////////////
    //// FIXME: Not clear what the strategy is with the following inner interface.
    // The {@code ReportCommandErrors} interface allows the
    // method runBuildCommand to call a protected
    // method from the CGenerator if that method is passed
    // using a method reference. The method that is passed
    // is then interpreted as a ReportCommandErrors instance.
    // This is a convenient way to factor out part of the
    // internals of the CGenerator while maintaining
    // encapsulation, even though the internals of the CGenerator
    // might seem to be tightly coupled. FIXME: Delete this comment

    /**
     * A {@code ReportCommandErrors} is a way to analyze command
     * output and report any errors that it describes.
     * FIXME: If the VSCode branch passes code review
     *  without significant revision, this mechanism will probably be replaced.
     */
    public interface ReportCommandErrors {
        void report(String errors);
    }

    /**
     * Run the custom build command specified with the "build" parameter.
     * This command is executed in the same directory as the source file.
     *
     * The following environment variables will be available to the command:
     *
     * * LF_CURRENT_WORKING_DIRECTORY: The directory in which the command is invoked.
     * * LF_SOURCE_DIRECTORY: The directory containing the .lf file being compiled.
     * * LF_SOURCE_GEN_DIRECTORY: The directory in which generated files are placed.
     * * LF_BIN_DIRECTORY: The directory into which to put binaries.
     *
     */
    public static void runBuildCommand(
        FileConfig fileConfig,
        TargetConfig targetConfig,
        GeneratorCommandFactory commandFactory,
        ErrorReporter errorReporter,
        ReportCommandErrors reportCommandErrors
    ) {
        List<LFCommand> commands = getCommands(targetConfig.buildCommands, commandFactory, fileConfig.srcPath);
        // If the build command could not be found, abort.
        // An error has already been reported in createCommand.
        if (commands.stream().anyMatch(Objects::isNull)) return;

        for (LFCommand cmd : commands) {
            int returnCode = cmd.run();
            if (returnCode != 0 && fileConfig.getCompilerMode() != Mode.INTEGRATED) {
                errorReporter.reportError(String.format(
                    // FIXME: Why is the content of stderr not provided to the user in this error message?
                    "Build command \"%s\" failed with error code %d.",
                    targetConfig.buildCommands, returnCode
                ));
                return;
            }
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (!cmd.getErrors().toString().isEmpty() && fileConfig.getCompilerMode() == Mode.INTEGRATED) {
                reportCommandErrors.report(cmd.getErrors().toString());
                return; // FIXME: Why do we return here? Even if there are warnings, the build process should proceed.
            }
        }
    }

    //////////////////////////////////////////////////////
    //// Private functions.

    /**
     * If the argument is a multiport, then return a string that
     * gives the width as an expression, and otherwise, return null.
     * The string will be empty if the width is variable (specified
     * as '[]'). Otherwise, if is a single term or a sum of terms
     * (separated by '+'), where each term is either an integer
     * or a parameter reference in the target language.
     */
    public static String multiportWidthExpression(Variable variable) {
        List<String> spec = multiportWidthTerms(variable);
        return spec == null ? null : String.join(" + ", spec);
    }

    //////////////////////////////////////////////////////
    //// Private methods.

    /**
     * Convert the given commands from strings to their LFCommand
     * representation and return a list of LFCommand.
     * @param commands A list of commands as strings.
     * @param factory A command factory.
     * @param dir The directory in which the commands should be executed.
     * @return The LFCommand representations of the given commands,
     *  where {@code null} is a placeholder for commands that cannot be
     *  executed.
     */
    private static List<LFCommand> getCommands(List<String> commands, GeneratorCommandFactory factory, Path dir) {
        return commands.stream()
                       .map(cmd -> List.of(cmd.split("\\s+")))
                       .filter(tokens -> tokens.size() > 0)
                       .map(tokens -> factory.createCommand(tokens.get(0), tokens.subList(1, tokens.size()), dir))
                       .collect(Collectors.toList());
    }

    /**
     * Return target code for a parameter reference, which in
     * this case is just the parameter name.
     *
     * @param param The parameter to generate code for.
     * @return Parameter reference in target code.
     */
    private static String getTargetReference(Parameter param) {
        return param.getName();
    }

    /**
     * If the argument is a multiport, return a list of strings
     * describing the width of the port, and otherwise, return null.
     * If the list is empty, then the width is variable (specified
     * as '[]'). Otherwise, it is a list of integers and/or parameter
     * references.
     * @param variable The port.
     * @return The width specification for a multiport or null if it is
     *  not a multiport.
     */
    private static List<String> multiportWidthTerms(Variable variable) {
        List<String> result = null;
        if (variable instanceof Port) {
            if (((Port) variable).getWidthSpec() != null) {
                result = new ArrayList<>();
                if (!((Port) variable).getWidthSpec().isOfVariableLength()) {
                    for (WidthTerm term : ((Port) variable).getWidthSpec().getTerms()) {
                        if (term.getParameter() != null) {
                            result.add(getTargetReference(term.getParameter()));
                        } else {
                            result.add("" + term.getWidth());
                        }
                    }
                }
            }
        }
        return result;
    }

    /**
     * Given a representation of time that may include units, return
     * a string that the target language can recognize as a value.
     * If units are given, e.g. "msec", then we convert the units to upper
     * case and return an expression of the form "MSEC(value)".
     * @param time A TimeValue that represents a time.
     * @return A string, such as "MSEC(100)" for 100 milliseconds.
     */
    private static String timeInTargetLanguage(TimeValue time) {
        if (time != null) {
            if (time.unit != TimeUnit.NONE) {
                return time.unit.name() + '(' + time.time + ')';
            } else {
                return String.valueOf(time.time);
            }
        }
        return "0"; // FIXME: do this or throw exception?
    }
}
