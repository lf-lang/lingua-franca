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

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.InferredType;
import org.lflang.TargetConfig;

import org.lflang.generator.ActionInstance;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthTerm;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

/**
 * A collection of utilities for C code generation.
 * This class codifies the coding conventions for the C target code generator.
 * I.e., it defines how variables are named and referenced.
 * @author Edward A. Lee
 */
public class CUtil {

    /**
     * Suffix that when appended to the name of a federated reactor yields
     * the name of its corresponding RTI executable.
     */
    public static final String RTI_BIN_SUFFIX = "_RTI";

    /**
     * Suffix that when appended to the name of a federated reactor yields
     * the name of its corresponding distribution script.
     */
    public static final String RTI_DISTRIBUTION_SCRIPT_SUFFIX = "_distribute.sh";

    //////////////////////////////////////////////////////
    //// Public methods.

    /**
     * Return a reference to the action struct of the specified
     * action instance. This action_base_t struct is on the self struct.
     * @param instance The action instance.
     * @param runtimeIndex An optional index variable name to use to address runtime instances.
     */
    public static String actionRef(ActionInstance instance, String runtimeIndex) {
        return reactorRef(instance.getParent(), runtimeIndex)
                + "->_lf_"
                + instance.getName();
    }

    /**
     * Return a default name of a variable to refer to the bank index of a reactor
     * in a bank. This is has the form uniqueID_i where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * If the instance is not a bank, return "0".
     * @param instance A reactor instance.
     */
    public static String bankIndex(ReactorInstance instance) {
        if (!instance.isBank()) return "0";
        return bankIndexName(instance);
    }

    /**
     * Return a default name of a variable to refer to the bank index of a reactor
     * in a bank. This is has the form uniqueID_i where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * @param instance A reactor instance.
     */
    public static String bankIndexName(ReactorInstance instance) {
        return instance.uniqueID() + "_i";
    }

    /**
     * Return a default name of a variable to refer to the channel index of a port
     * in a bank. This has the form uniqueID_c where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     * If the port is not a multiport, then return the string "0".
     */
    public static String channelIndex(PortInstance port) {
        if (!port.isMultiport()) return "0";
        return channelIndexName(port);
    }

    /**
     * Return a default name of a variable to refer to the channel index of a port
     * in a bank. This is has the form uniqueID_c where uniqueID
     * is an identifier for the instance that is guaranteed to be different
     * from the ID of any other instance in the program.
     */
    public static String channelIndexName(PortInstance port) {
        return port.uniqueID() + "_c";
    }

    /**
     * Return the name of the reactor. A '_main` is appended to the name if the
     * reactor is main (to allow for instantiations that have the same name as
     * the main reactor or the .lf file).
     */
    public static String getName(ReactorDecl reactor) {
        if (reactor instanceof Reactor r && r.isMain()) {
            return reactor.getName() + "_main";
        }
        return reactor.getName();
    }

    /**
     * Return a reference to the specified port.
     *
     * The returned string will have one of the following forms:
     *
     * * selfStructs[k]->_lf_portName
     * * selfStructs[k]->_lf_portName
     * * selfStructs[k]->_lf_portName[i]
     * * selfStructs[k]->_lf_parent.portName
     * * selfStructs[k]->_lf_parent.portName[i]
     * * selfStructs[k]->_lf_parent[j].portName
     * * selfStructs[k]->_lf_parent[j].portName[i]
     *
     * where k is the runtime index of either the port's parent
     * or the port's parent's parent, the latter when isNested is true.
     * The index j is present if the parent is a bank, and
     * the index i is present if the port is a multiport.
     *
     * The first two forms are used if isNested is false,
     * and the remaining four are used if isNested is true.
     * Set isNested to true when referencing a port belonging
     * to a contained reactor.
     *
     * @param port The port.
     * @param isNested True to return a reference relative to the parent's parent.
     * @param includeChannelIndex True to include the channel index at the end.
     * @param runtimeIndex A variable name to use to index the runtime instance or
     *  null to use the default, the string returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex A variable name to use to index the bank or null to use the
     *  default, the string returned by {@link CUtil#bankIndex(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil#channelIndex(PortInstance)}.
     */
    public static String portRef(
            PortInstance port,
            boolean isNested,
            boolean includeChannelIndex,
            String runtimeIndex,
            String bankIndex,
            String channelIndex
    ) {
        String channel = "";
        if (channelIndex == null) channelIndex = channelIndex(port);
        if (port.isMultiport() && includeChannelIndex) {
            channel = "[" + channelIndex + "]";
        }
        if (isNested) {
            return reactorRefNested(port.getParent(), runtimeIndex, bankIndex) + "." + port.getName() + channel;
        } else {
            String sourceStruct = CUtil.reactorRef(port.getParent(), runtimeIndex);
            return sourceStruct + "->_lf_" + port.getName() + channel;
        }
    }

    /**
     * Return a reference to the port on the self struct of the
     * port's parent.  This is used when an input port triggers a reaction
     * in the port's parent or when an output port is written to by
     * a reaction in the port's parent.
     * This is equivalent to calling `portRef(port, false, true, null, null)`.
     * @param port An instance of the port to be referenced.
     */
    public static String portRef(PortInstance port) {
        return portRef(port, false, true, null, null, null);
    }

    /**
     * Return a reference to the port on the self struct of the
     * port's parent using the specified index variables.
     * This is used when an input port triggers a reaction
     * in the port's parent or when an output port is written to by
     * a reaction in the port's parent.
     * This is equivalent to calling `portRef(port, false, true, bankIndex, channelIndex)`.
     * @param port An instance of the port to be referenced.
     * @param runtimeIndex A variable name to use to index the runtime instance or
     *  null to use the default, the string returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex A variable name to use to index the bank or null to use the
     *  default, the string returned by {@link CUtil#bankIndex(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil#channelIndex(PortInstance)}.
     */
    public static String portRef(
            PortInstance port, String runtimeIndex, String bankIndex, String channelIndex
    ) {
        return portRef(port, false, true, runtimeIndex, bankIndex, channelIndex);
    }

    /**
     * Return a reference to a port without any channel indexing.
     * This is useful for deriving a reference to the _width variable.
     * @param port An instance of the port to be referenced.
     */
    public static String portRefName(PortInstance port) {
        return portRef(port, false, false, null, null, null);
    }

    /**
     * Return the portRef without the channel indexing.
     * This is useful for deriving a reference to the _width variable.
     *
     * @param port An instance of the port to be referenced.
     * @param runtimeIndex A variable name to use to index the runtime instance or
     *  null to use the default, the string returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex A variable name to use to index the bank or null to use the
     *  default, the string returned by {@link CUtil#bankIndex(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil#channelIndex(PortInstance)}.
     */
    public static String portRefName(
            PortInstance port, String runtimeIndex, String bankIndex, String channelIndex
    ) {
        return portRef(port, false, false, runtimeIndex, bankIndex, channelIndex);
    }

    /**
     * Return a port reference to a port on the self struct of the
     * parent of the port's parent.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent. This is equivalent to calling
     * `portRef(port, true, true, null, null, null)`.
     *
     * @param port The port.
     */
    public static String portRefNested(PortInstance port) {
        return portRef(port, true, true, null, null, null);
    }

    /**
     * Return a reference to the port on the self struct of the
     * parent of the port's parent.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent. This is equivalent to calling
     * `portRef(port, true, true, runtimeIndex, bankIndex, channelIndex)`.
     *
     * @param port The port.
     * @param runtimeIndex A variable name to use to index the runtime instance or
     *  null to use the default, the string returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex A variable name to use to index the bank or null to use the
     *  default, the string returned by {@link CUtil#bankIndex(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil#channelIndex(PortInstance)}.
     */
    public static String portRefNested(
            PortInstance port, String runtimeIndex, String bankIndex, String channelIndex
    ) {
        return portRef(port, true, true, runtimeIndex, bankIndex, channelIndex);
    }

    /**
     * Return a reference to the port on the self struct of the
     * parent of the port's parent, but without the channel indexing,
     * even if it is a multiport.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent.
     * This is equivalent to calling `portRef(port, true, false, null, null, null)`.
     *
     * @param port The port.
     */
    public static String portRefNestedName(PortInstance port) {
        return portRef(port, true, false, null, null, null);
    }

    /**
     * Return a reference to the port on the self struct of the
     * parent of the port's parent, but without the channel indexing,
     * even if it is a multiport.  This is used when an input port
     * is written to by a reaction in the parent of the port's parent,
     * or when an output port triggers a reaction in the parent of the
     * port's parent. This is equivalent to calling
     * `portRefNested(port, true, false, runtimeIndex, bankIndex, channelIndex)`.
     *
     * @param port The port.
     * @param runtimeIndex A variable name to use to index the runtime instance or
     *  null to use the default, the string returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex A variable name to use to index the bank or null to use the
     *  default, the string returned by {@link CUtil#bankIndex(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil#channelIndex(PortInstance)}.
     */
    public static String portRefNestedName(
            PortInstance port, String runtimeIndex, String bankIndex, String channelIndex
    ) {
        return portRef(port, true, false, runtimeIndex, bankIndex, channelIndex);
    }

    /**
     * Return code for referencing a port within a reaction body possibly indexed by
     * a bank index and/or a multiport index. If the provided reference is
     * not a port, then this returns the string "ERROR: not a port."
     * @param reference The reference to the port.
     * @param bankIndex A bank index or null or negative if not in a bank.
     * @param multiportIndex A multiport index or null or negative if not in a multiport.
     */
    public static String portRefInReaction(VarRef reference, Integer bankIndex, Integer multiportIndex) {
        if (!(reference.getVariable() instanceof Port)) {
            return "ERROR: not a port."; // FIXME: This is not the fail-fast approach, and it seems arbitrary.
        }
        var prefix = "";
        if (reference.getContainer() != null) {
            var bank = "";
            if (reference.getContainer().getWidthSpec() != null && bankIndex != null && bankIndex >= 0) {
                bank = "[" + bankIndex + "]";
            }
            prefix = reference.getContainer().getName() + bank + ".";
        }
        var multiport = "";
        if (((Port) reference.getVariable()).getWidthSpec() != null && multiportIndex != null && multiportIndex >= 0) {
            multiport = "[" + multiportIndex + "]";
        }
        return prefix + reference.getVariable().getName() + multiport;
    }

    /**
     * Return a reference to the reaction entry on the self struct
     * of the parent of the specified reaction.
     * @param reaction The reaction.
     */
    public static String reactionRef(ReactionInstance reaction) {
        return reactionRef(reaction, null);
    }

    /**
     * Return a reference to the reaction entry on the self struct
     * of the parent of the specified reaction.
     * @param reaction The reaction.
     * @param runtimeIndex An index into the array of self structs for the parent.
     */
    public static String reactionRef(ReactionInstance reaction, String runtimeIndex) {
        return reactorRef(reaction.getParent(), runtimeIndex)
                + "->_lf__reaction_" + reaction.index;
    }

    /**
     * Return a reference to the "self" struct of the specified
     * reactor instance. The returned string has the form
     * self[j], where self is the name of the array of self structs
     * for this reactor instance and j is the expression returned
     * by {@link #runtimeIndex(ReactorInstance)} or 0 if there are no banks.
     * @param instance The reactor instance.
     */
    public static String reactorRef(ReactorInstance instance) {
        return reactorRef(instance, null);
    }

    /**
     * Return the name of the array of "self" structs of the specified
     * reactor instance.  This is similar to {@link #reactorRef(ReactorInstance)}
     * except that it does not index into the array.
     * @param instance The reactor instance.
     */
    public static String reactorRefName(ReactorInstance instance) {
        return instance.uniqueID() + "_self";
    }

    /**
     * Return a reference to the "self" struct of the specified
     * reactor instance. The returned string has the form
     * self[runtimeIndex], where self is the name of the array of self structs
     * for this reactor instance. If runtimeIndex is null, then it is replaced by
     * the expression returned
     * by {@link #runtimeIndex(ReactorInstance)} or 0 if there are no banks.
     * @param instance The reactor instance.
     * @param runtimeIndex An optional expression to use to address bank members.
     *  If this is null, the expression used will be that returned  by
     *  {@link #runtimeIndex(ReactorInstance)}.
     */
    public static String reactorRef(ReactorInstance instance, String runtimeIndex) {
        if (runtimeIndex == null) runtimeIndex = runtimeIndex(instance);
        return reactorRefName(instance) + "[" + runtimeIndex + "]";
    }

    /**
     * For situations where a reaction reacts to or reads from an output
     * of a contained reactor or sends to an input of a contained reactor,
     * then the container's self struct will have a field
     * (or an array of fields if the contained reactor is a bank) that is
     * a struct with fields corresponding to those inputs and outputs.
     * This method returns a reference to that struct or array of structs.
     * Note that the returned reference is not to the self struct of the
     * contained reactor. Use {@link #reactorRef(ReactorInstance)} for that.
     *
     * @param reactor The contained reactor.
     */
    public static String reactorRefNested(ReactorInstance reactor) {
        return reactorRefNested(reactor, null, null);
    }

    /**
     * For situations where a reaction reacts to or reads from an output
     * of a contained reactor or sends to an input of a contained reactor,
     * then the container's self struct will have a field
     * (or an array of fields if the contained reactor is a bank) that is
     * a struct with fields corresponding to those inputs and outputs.
     * This method returns a reference to that struct or array of structs.
     * Note that the returned reference is not to the self struct of the
     * contained reactor. Use {@link CUtil#reactorRef(ReactorInstance)} for that.
     *
     * @param reactor The contained reactor.
     * @param runtimeIndex A variable name to use to index the runtime instance or
     *  null to use the default, the string returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex A variable name to use to index the bank or null to use the
     *  default, the string returned by {@link CUtil#bankIndex(ReactorInstance)}.
     */
    public static String reactorRefNested(ReactorInstance reactor, String runtimeIndex, String bankIndex) {
        String result = reactorRef(reactor.getParent(), runtimeIndex) + "->_lf_" + reactor.getName();
        if (reactor.isBank()) {
            // Need the bank index not the runtimeIndex.
            if (bankIndex == null) bankIndex = bankIndex(reactor);
            result += "[" + bankIndex + "]";
        }
        return result;
    }

    /**
     * Return an expression that, when evaluated, gives the index of
     * a runtime instance of the specified ReactorInstance. If the reactor
     * is not within any banks, then this will return "0". Otherwise, it
     * will return an expression that evaluates a mixed-radix number
     * d0%w0, d1%w1, ... , dn%wn, where n is the depth minus one of the
     * reactor. The radixes, w0 to wn, are the widths of this reactor,
     * its parent reactor, on up to the top-level reactor. Since the top-level
     * reactor is never a bank, dn = 0 and wn = 1. The digits, di, are
     * either 0 (of the parent is not a bank) or the variable name returned
     * by {@link #bankIndexName(ReactorInstance)} if the parent is a bank.
     * The returned expression, when evaluated, will yield the following value:
     *
     *     d0 + w0 * (d1 + w1 * ( ... (dn-1 + wn-1 * dn) ... )
     *
     * @param reactor The reactor.
     */
    public static String runtimeIndex(ReactorInstance reactor) {
        StringBuilder result = new StringBuilder();
        int width = 0;
        int parens = 0;
        while (reactor != null) {
            if (reactor.isBank() && reactor.getWidth() > 1) {
                if (width > 0) {
                    result.append(" + " + width + " * (");
                    parens++;
                }
                result.append(bankIndexName(reactor));
                width = reactor.getWidth();
            }
            reactor = reactor.getParent();
        }
        while (parens-- > 0) {
            result.append(")");
        }
        if (result.length() == 0) return "0";
        return result.toString();
    }

    /**
     * Return a unique type for the "self" struct of the specified
     * reactor class from the reactor class.
     * @param reactor The reactor class.
     * @return The type of a self struct for the specified reactor class.
     */
    public static String selfType(ReactorDecl reactor) {
        if (reactor instanceof Reactor r && r.isMain()) {
            return reactor.getName().toLowerCase() + "_main_self_t";
        }
        return reactor.getName().toLowerCase() + "_self_t";
    }

    /** Construct a unique type for the "self" struct of the class of the given reactor. */
    public static String selfType(ReactorInstance instance) {
        return selfType(instance.getDefinition().getReactorClass());
    }

    /**
     * Return a reference to the trigger_t struct of the specified
     * trigger instance (input port or action). This trigger_t struct
     * is on the self struct.
     * @param instance The port or action instance.
     */
    public static String triggerRef(TriggerInstance<? extends Variable> instance) {
        return triggerRef(instance, null);
    }

    /**
     * Return a reference to the trigger_t struct of the specified
     * trigger instance (input port or action). This trigger_t struct
     * is on the self struct.
     * @param instance The port or action instance.
     * @param runtimeIndex An optional index variable name to use to address runtime instances.
     */
    public static String triggerRef(TriggerInstance<? extends Variable> instance, String runtimeIndex) {
        return reactorRef(instance.getParent(), runtimeIndex)
                + "->_lf__"
                + instance.getName();
    }

    /**
     * Return a reference to the trigger_t struct for the specified
     * port of a contained reactor.
     * @param port The output port of a contained reactor.
     */
    public static String triggerRefNested(PortInstance port) {
        return triggerRefNested(port, null, null);
    }

    /**
     * Return a reference to the trigger_t struct for the specified
     * port of a contained reactor.
     * @param port The output port of a contained reactor.
     * @param runtimeIndex An optional index variable name to use to index
     *  the runtime instance of the port's parent's parent, or null to get the
     *  default returned by {@link CUtil#runtimeIndex(ReactorInstance)}.
     * @param bankIndex An optional index variable name to use to index
     *  the the bank of the port's parent, or null to get the default returned by
     *  {@link CUtil#bankIndex(ReactorInstance)}.
     */
    public static String triggerRefNested(PortInstance port, String runtimeIndex, String bankIndex) {
        return reactorRefNested(port.getParent(), runtimeIndex, bankIndex) + "." + port.getName() + "_trigger";
    }

    /**
     * Copy the 'fileName' (which also could be a directory name) from the
     * 'srcDirectory' to the 'destinationDirectory'. This function has a
     * fallback search mechanism, where if `fileName` is not found in the
     * `srcDirectory`, it will try to find `fileName` via the following
     * procedure:
     *     1- Search in LF_CLASSPATH. @see findFile()
     *     2- Search in CLASSPATH. @see findFile()
     *     3- Search for 'fileName' as a resource. That means the `fileName`
     *        can be '/path/to/class/resource'. @see java.lang.Class.getResourceAsStream()
     *
     * @param fileName Name of the file or directory.
     * @param srcDir   Where the file or directory is currently located.
     * @param dstDir   Where the file or directory should be placed.
     * @return The name of the file or directory in destinationDirectory or an empty string on failure.
     */
    public static String copyFileOrResource(String fileName, Path srcDir,
            Path dstDir) {
        // Try to copy the file or directory from the file system.
        Path file = findFileOrDirectory(fileName, srcDir);
        if (file != null) {
            Path target = dstDir.resolve(file.getFileName());
            try {
                if (Files.isDirectory(file)) {
                    FileUtil.copyDirectory(file, target);
                } else if (Files.isRegularFile(file)) {
                    Files.copy(file, target,
                            StandardCopyOption.REPLACE_EXISTING);
                }
                return file.getFileName().toString();
            } catch (IOException e) {
                // Failed to copy the file or directory, most likely
                // because it doesn't exist. Will try to find it as a
                // resource before giving up.
            }
        }

        String filenameWithoutPath = fileName;
        int lastSeparator = fileName.lastIndexOf(File.separator);
        if (lastSeparator > 0) {
            // FIXME: Brittle. What if the file is in a subdirectory?
            filenameWithoutPath = fileName.substring(lastSeparator + 1);
        }
        // Try to copy the file or directory as a resource.
        try {
            FileUtil.copyFileFromClassPath(fileName,
                    dstDir.resolve(filenameWithoutPath));
            return filenameWithoutPath;
        } catch (IOException ex) {
            // Will try one more time as a directory
        }

        try {
            FileUtil.copyDirectoryFromClassPath(fileName,
                    dstDir.resolve(filenameWithoutPath), false);
            return filenameWithoutPath;
        } catch (IOException ex) {
            System.err.println(
                    "WARNING: Failed to find file or directory " + fileName);
        }

        return "";
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
     * Search for a given file or directory name in the given directory.
     * If not found, search in directories in LF_CLASSPATH.
     * If there is no LF_CLASSPATH environment variable, use CLASSPATH,
     * if it is defined. The first file or directory that is found will
     * be returned. Otherwise, null is returned.
     *
     * @param fileName The file or directory name or relative path + name
     * as a String.
     * @param directory String representation of the directory to search in.
     * @return A Java Path or null if not found.
     */
    public static Path findFileOrDirectory(String fileName, Path directory) {
        Path foundFile;

        // Check in local directory
        foundFile = directory.resolve(fileName);
        if (Files.exists(foundFile)) {
            return foundFile;
        }

        // Check in LF_CLASSPATH
        // Load all the resources in LF_CLASSPATH if it is set.
        String classpathLF = System.getenv("LF_CLASSPATH");
        if (classpathLF == null) {
            classpathLF = System.getenv("CLASSPATH");
        }
        if (classpathLF != null) {
            String[] paths = classpathLF.split(System.getProperty("path.separator"));
            for (String path : paths) {
                foundFile = Paths.get(path).resolve(fileName);
                if (Files.exists(foundFile)) {
                    return foundFile;
                }
            }
        }
        // Not found.
        return null;
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
        ReportCommandErrors reportCommandErrors,
        LFGeneratorContext.Mode mode
    ) {
        List<LFCommand> commands = getCommands(targetConfig.buildCommands, commandFactory, fileConfig.srcPath);
        // If the build command could not be found, abort.
        // An error has already been reported in createCommand.
        if (commands.stream().anyMatch(Objects::isNull)) return;

        for (LFCommand cmd : commands) {
            int returnCode = cmd.run();
            if (returnCode != 0 && mode != LFGeneratorContext.Mode.EPOCH) {
                errorReporter.reportError(String.format(
                    // FIXME: Why is the content of stderr not provided to the user in this error message?
                    "Build command \"%s\" failed with error code %d.",
                    targetConfig.buildCommands, returnCode
                ));
                return;
            }
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (!cmd.getErrors().toString().isEmpty() && mode == LFGeneratorContext.Mode.EPOCH) {
                reportCommandErrors.report(cmd.getErrors().toString());
                return; // FIXME: Why do we return here? Even if there are warnings, the build process should proceed.
            }
        }
    }


    /**
     * Remove files in the bin directory that may have been created.
     * Call this if a compilation occurs so that files from a previous
     * version do not accidentally get executed.
     * @param fileConfig
     */
    public static void deleteBinFiles(FileConfig fileConfig) {
        String name = FileUtil.nameWithoutExtension(fileConfig.srcFile);
        String[] files = fileConfig.binPath.toFile().list();
        List<String> federateNames = new LinkedList<>(); // FIXME: put this in ASTUtils?
        fileConfig.resource.getAllContents().forEachRemaining(node -> {
            if (node instanceof Reactor r) {
                if (r.isFederated()) {
                    r.getInstantiations().forEach(inst -> federateNames
                        .add(inst.getName()));
                }
            }
        });
        for (String f : files) {
            // Delete executable file or launcher script, if any.
            // Delete distribution file, if any.
            // Delete RTI file, if any.
            if (f.equals(name) || f.equals(name + RTI_BIN_SUFFIX)
                || f.equals(name + RTI_DISTRIBUTION_SCRIPT_SUFFIX)) {
                //noinspection ResultOfMethodCallIgnored
                fileConfig.binPath.resolve(f).toFile().delete();
            }
            // Delete federate executable files, if any.
            for (String federateName : federateNames) {
                if (f.equals(name + "_" + federateName)) {
                    //noinspection ResultOfMethodCallIgnored
                    fileConfig.binPath.resolve(f).toFile().delete();
                }
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
     * Given a type for an input or output, return true if it should be
     * carried by a lf_token_t struct rather than the type itself.
     * It should be carried by such a struct if the type ends with *
     * (it is a pointer) or [] (it is a array with unspecified length).
     * @param type The type specification.
     */
    public static boolean isTokenType(InferredType type, CTypes types) {
        if (type.isUndefined()) return false;
        // This is a hacky way to do this. It is now considered to be a bug (#657)
        String targetType = types.getVariableDeclaration(type, "", false);
        return type.isVariableSizeList || targetType.trim().endsWith("*");
    }

    public static String generateWidthVariable(String var) {
        return var + "_width";
    }

    /** If the type specification of the form {@code type[]},
     *  {@code type*}, or {@code type}, return the type.
     *  @param type A string describing the type.
     */
    public static String rootType(String type) {
        if (type.endsWith("]")) {
            return type.substring(0, type.indexOf("[")).trim();
        } else if (type.endsWith("*")) {
            return type.substring(0, type.length() - 1).trim();
        } else {
            return type.trim();
        }
    }

    /**
     * Return the full name of the specified instance without
     * the leading name of the top-level reactor, unless this
     * is the top-level reactor, in which case return its name.
     * @param instance The instance.
     * @return A shortened instance name.
     */
    public static String getShortenedName(ReactorInstance instance) {
        var description = instance.getFullName();
        // If not at the top level, strip off the name of the top level.
        var period = description.indexOf(".");
        if (period > 0) {
            description = description.substring(period + 1);
        }
        return description;
    }
}
