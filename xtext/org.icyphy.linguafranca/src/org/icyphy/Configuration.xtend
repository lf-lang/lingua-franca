/* Class for storing information derived from target properties. */
/** 
 * Copyright (c) 2021, The University of California at Berkeley.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.icyphy

import java.util.List
import org.icyphy.Target.ClockSyncMode
import org.icyphy.Target.LogLevel
import org.icyphy.Target.BuildType
import org.icyphy.Target.CoordinationType

/** 
 * A class for keeping the current target configuration.
 * 
 * Class members of type String are initialized as empty strings, 
 * unless otherwise stated.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class Configuration {

    /**
     * The custom build command, which replaces the default build process of
     * directly invoking a designated compiler. A common usage of this target
     * property is to set the command to build on the basis of a Makefile.
     */
    public List<String> buildCommands = newLinkedList

    /**
     * The mode of clock synchronization to be used in federated programs.
     * The default is 'initial'.
     */
    public ClockSyncMode clockSync = ClockSyncMode.INITIAL

    /**
     * Parameter passed to cmake. The default is 'Release'.
     */
    public BuildType cmakeBuildType = BuildType.Release

    /**
     * An optional additional .cmake file to include.
     */
    public String cmakeInclude = ""

    /**
     * The compiler to invoke, unless a build command has been specified.
     */
    public String compiler = ""

    /**
     * Additional sources to add to the compile command if appropriate.
     */
    public List<String> compileAdditionalSources = newArrayList

    /**
     * Additional libraries to add to the compile command using the "-l" command-line option.
     */
    public List<String> compileLibraries = newArrayList

    /**
     * Flags to pass to the compiler, unless a build command has been specified.
     */
    public String compilerFlags = ""

    /**
     * The type of coordination used during the execution of a federated program.
     * The default is 'centralized'.
     */
    public CoordinationType coordination = CoordinationType.CENTRALIZED

    /**
     * If true, configure the execution environment such that it does not
     * wait for physical time to match logical time. The default is false.
     */
    public boolean fastMode = false

    /**
     * List of files to be copied to src-gen.
     */
    public List<String> fileNames = newLinkedList;

    /**
     * List of file names from the files target property with no path info.
     * Useful for copying them to remote machines. This is needed because
     * target files can be resources with resource paths.
     */
    public List<String> filesNamesWithoutPath = newLinkedList;

    /**
     * If true, configure the execution environment to keep executing if there
     * are no more events on the event queue. The default is false.
     */
    public boolean keepalive = false

    /**
     * The level of logging during execution. The default is INFO.
     */
    public LogLevel logLevel = LogLevel.INFO

    /**
     * Flags to pass to the linker, unless a build command has been specified.
     */
    public String linkerFlags = ""

    /**
     * If true, do not invoke the target compiler or build command.
     * The default is false.
     */
    public boolean noCompile = false

    /**
     * If true, do not perform runtime validation. The default is false.
     */
    public boolean noRuntimeValidation = false

    /**
     * The number of worker threads to deploy. The default is zero (i.e.,
     * all work is done in the main thread).
     */
    public int threads = 0

    /**
     * The timeout to be observed during execution of the program.
     */
    public TimeValue timeout

    /**
     * If true, configure the runtime environment to perform tracing.
     * The default is false.
     */
    public boolean tracing = false

}
