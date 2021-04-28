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
package org.lflang

import java.util.List
import org.lflang.TargetProperty.BuildType
import org.lflang.TargetProperty.ClockSyncMode
import org.lflang.TargetProperty.CoordinationType
import org.lflang.TargetProperty.LogLevel
import org.lflang.lf.TimeUnit

/** 
 * A class for keeping the current target configuration.
 * 
 * Class members of type String are initialized as empty strings, 
 * unless otherwise stated.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class TargetConfig {

    /**
     * A list of custom build commands that replace the default build process of
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
     * Clock sync options.
     */
    public ClockSyncOptions clockSyncOptions = new ClockSyncOptions();

    /**
     * Parameter passed to cmake. The default is 'Release'.
     */
    public BuildType cmakeBuildType = BuildType.RELEASE

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
    public List<String> compilerFlags = newArrayList

    /**
     * The type of coordination used during the execution of a federated program.
     * The default is 'centralized'.
     */
    public CoordinationType coordination = CoordinationType.CENTRALIZED

    /**
     * Docker options.
     */
    public DockerOptions dockerOptions = null;
    
    /**
     * Coordination options.
     */
    public CoordinationOptions coordinationOptions = new CoordinationOptions();

    /**
     * Link to an external runtime library instead of the default one. 
     */
    public String externalRuntimePath = null;

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
     * List of proto files to be processed by the code generator.
     */
    public List<String> protoFiles = newLinkedList

    /**
     * The version of the runtime library to be used in the generated target. 
     */
    public String runtimeVersion = null

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
     * If non-null, configure the runtime environment to perform tracing.
     * The default is null.
     */
    public TracingOptions tracing = null
    
}

/**
 * Settings related to clock synchronization.
 */
class ClockSyncOptions {
    
    /**
     * FIXME
     * The default is 10.
     */
    public int attenuation = 10

    /**
     * Whether or not to collect statistics while performing clock synchronization.
     * This setting is only considered when clock synchronization has been activated.
     * The default is true.
     */
    public boolean collectStats = true

    /**
     * FIXME
     */
    public boolean localFederatesOn

    
    /**
     * FIXME
     * The default is 5 milliseconds.
     */
    public TimeValue period = new TimeValue(5, TimeUnit.MSEC)
    
    /**
     * FIXME
     * The default is 10.
     */
    public int trials = 10
    
    /**
     * Used to create an artificial clock synchronization error for the purpose of testing.
     * The default is null.
     */
    public TimeValue testOffset;
}

/**
 * Settings related to coordination of federated execution.
 */
class CoordinationOptions {
    
    /**
     * For centralized coordination, if a federate has a physical action that can trigger
     * an output, directly or indirectly, then it will send NET (next event tag) messages
     * to the RTI periodically as its physical clock advances. This option sets the amount
     * of time to wait between sending such messages. Increasing this value results in
     * downstream federates that lag further behind physical time (if the "after" delays
     * are insufficient).
     * The default is null, which means it is up the implementation to choose an interval.
     */
    public TimeValue advance_message_interval = null;
}

/**
 * Settings related to Docker options.
 */
class DockerOptions {
    /**
     * The base image and tag from which to build the Docker image. The default is "alpine:latest".
     */
    public String from = "alpine:latest"
}

enum Mode {
    STANDALONE,
    INTEGRATED,
    UNDEFINED
}

/**
 * Settings related to tracing options.
 */
class TracingOptions {
    /**
     * The name to use as the root of the trace file produced.
     * This defaults to the name of the .lf file.
     */
    public String traceFileName = null
}
