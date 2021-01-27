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
import org.icyphy.TargetSupport.CoordinationTypes
import org.icyphy.TargetSupport.LogLevel
import org.icyphy.TargetSupport.BuildTypes

/** 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class TargetConfig {

    /**
     * The build-type target parameter. The default is 'Release'.
     */
    public BuildTypes buildType = BuildTypes.Release

    /**
     * The cmake-include target parameter. The default is null.
     */
    public String cmakeInclude

    /**
     * The compiler target parameter. The default is null.
     */
    public String compiler

    /**
     * The compiler flags target parameter, or null if there is none.
     */
    public String compilerFlags

    /**
     * The linker flags target parameter, or null if there is none.
     */
    public String linkerFlags

    /**
     * The compiler target no-compile parameter, or false if there is none.
     */
    public boolean targetNoCompile = false

    /**
     * The compiler target no-runtime-validation parameter, or false if there is none.
     */
    public boolean targetNoRuntimeValidation = false

    /**
     * The fast target parameter, or false if there is none.
     */
    public boolean targetFast = false

    /**
     * The coordination target parameter. Default is
     * centralized.
     */
    public CoordinationTypes targetCoordination = CoordinationTypes.CENTRALIZED

    /**
     * List of files to be copied to src-gen.
     */
    public List<String> targetFiles = newLinkedList;

    /**
     * List of file names from the files target property with no path info.
     * Useful for copying them to remote machines. This is needed because
     * target files can be resources with resource paths.
     */
    public List<String> targetFilesNamesWithoutPath = newLinkedList;

    /**
     * The value of the keepalive target parameter, or false if there is none.
     */
    public boolean targetKeepalive

    /**
     * The level of logging or null if not given. The default is INFO.
     */
    public LogLevel targetLoggingLevel = LogLevel.INFO

    /**
     * The threads target parameter, or the default 0 if there is none.
     */
    public int targetThreads = 0

    /**
     * The timeout parameter, or null if there is none.
     */
    public TimeValue targetTimeout

    /**
     * The tracing target parameter, or false if there is none.
     */
    public boolean targetTracing = false

}
