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
     * in a bank. This is has the form uniqueID_bank_index where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * @param instance A reactor instance.
     */
    static public String bankIndex(ReactorInstance instance) {
        return instance.uniqueID() + "_bank_index";
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
        if (instance.getDepth() == 0) return("0");
        if (instance.isBank()) {
            return(
                    // Position of the bank member relative to the bank.
                    bankIndex(instance) + " * " + instance.getNumReactorInstances()
                    // Position of the bank within its parent.
                    + " + " + instance.getIndexOffset()
                    // Position of the parent.
                    + " + " + indexExpression(instance.getParent())
            );
        } else {
            return(
                    // Position within the parent.
                    instance.getIndexOffset()
                    // Position of the parent.
                    + " + " + indexExpression(instance.getParent())
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
     * * selfStruct->_lf_parent.portName
     * 
     * where the first is returned if the container directly contains the port.
     * The selfStruct points to the port's parent or parent's parent, and if
     * that parent is a bank, then it will have the form selfStruct[bankIndex],
     * where bankIndex is the string returned by {@link bankIndex(ReactorInstance)}.
     * 
     * If the container is port's parent's parent, and the port's parent is a bank,
     * then "_lf_parent" will be replaced by "_lf_parent[bankIndex]", where
     * bank_index is the string returned by bankIndex(port.parent).
     *
     * @param port The port.
     * @param container The container.
     */
    static public String portRef(PortInstance port, ReactorInstance container) {
        if (port.getParent() == container) {
            String sourceStruct = CUtil.reactorRef(port.getParent());
            return sourceStruct + "->_lf_" + port.getName();
        } else if (port.getParent().getParent() == container) {
            String sourceStruct = CUtil.reactorRef(port.getParent().getParent());
            // The form is slightly different depending on whether the port belongs to a bank.
            if (port.getParent().isBank()) {
                return sourceStruct + "->_lf_" + port.getParent().getName()
                        + "[" + bankIndex(port.getParent()) + "]." + port.getName();
            } else {
                return sourceStruct + "->_lf_" + port.getParent().getName() + "." + port.getName();
            }
        } else {
            throw new IllegalArgumentException(
                "Port " + port.getFullName() + " is not visible to " + container.getFullName()
            );
        }
    }
    
    /**
     * This is a special case of {@link portRef(PortInstance, ReactorInstance)
     * where it is know that the reference required for the port is to a
     * sink of data, not a source. This customizes portRef() by figuring out
     * what the appropriate container.
     * 
     * @param port An instance of the port to be referenced.
     */
    static public String portRefDestination(PortInstance port) {                
        if (port.isOutput()) {
            return portRef(port, port.getParent().getParent());
        }
        return portRef(port, port.getParent());
    }

    /**
     * This is a special case of {@link portRef(PortInstance, ReactorInstance)
     * where it is know that the reference required for the port is to a
     * source of data, not a sink. This customizes portRef() by figuring out
     * what the appropriate container.
     * 
     * @param port An instance of the port to be referenced.
     */
    static public String portRefSource(PortInstance port) {                
        if (port.isInput()) {
            return portRef(port, port.getParent().getParent());
        }
        return portRef(port, port.getParent());
    }
    
    /**
     * Return a reference to the reaction entry on the self struct
     * of the parent of the specified reaction.
     * @param reaction The reaction.
     */
    static public String reactionRef(ReactionInstance reaction) {
        return reactorRef(reaction.getParent()) + "->_lf__reaction_" + reaction.index;
    }

    /** 
     * Return the unique reference for the "self" struct of the specified
     * reactor instance. If the instance is a bank of reactors, this returns
     * something of the form name_self[bankIndex], where bankIndex is the 
     * returned by {@link bankIndex(ReactorInstance)}.
     * 
     * @param instance The reactor instance.
     * @return A reference to the self struct.
     */
    static public String reactorRef(ReactorInstance instance) {
        var result = instance.uniqueID() + "_self";
        // If this reactor is a member of a bank of reactors, then change
        // the name of its self struct to append [index].
        if (instance.isBank()) {
            result += "[" + bankIndex(instance) + "]";
        }
        return result;
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
    static public String reactorRefContained(ReactorInstance reactor) {
        String result = reactorRef(reactor.getParent()) + "->_lf_" + reactor.getName();
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
