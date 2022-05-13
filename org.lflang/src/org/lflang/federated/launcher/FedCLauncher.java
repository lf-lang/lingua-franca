/*************
 * Copyright (c) 2019-2021, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.federated.launcher;

import java.io.File;
import java.io.IOException;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.federated.FedFileConfig;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.c.CCompiler;

/**
 * Utility class that can be used to create a launcher for federated LF programs
 * that are written in C.
 *
 * @author Soroush Bateni <soroush@utdallas.edu>
 */
public class FedCLauncher extends FedLauncher {

    /**
     * Create an instance of FedCLauncher.
     *
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter A error reporter for reporting any errors or warnings during the code generation
     */
    public FedCLauncher(
            TargetConfig targetConfig,
            FileConfig fileConfig,
            ErrorReporter errorReporter
    ) {
        super(targetConfig, fileConfig, errorReporter);
    }

    /**
     * Return the compile command for a federate.
     *
     * @param federate The federate to compile.
     * @throws IOException
     */
    @Override
    protected
    String compileCommandForFederate(FederateInstance federate) {
        FedFileConfig fedFileConfig = null;
        TargetConfig localTargetConfig = targetConfig;
        try {
            fedFileConfig = new FedFileConfig(true, fileConfig, federate.name);
        } catch (IOException e) {
            errorReporter.reportError("Failed to create file config for federate "+federate.name);
            return "";
        }

        String commandToReturn = "";
        // FIXME: Hack to add platform support only for linux systems.
        // We need to fix the CMake build command for remote federates.
        String linuxPlatformSupport = "core" + File.separator + "platform" + File.separator + "lf_linux_support.c";
        if (!localTargetConfig.compileAdditionalSources.contains(linuxPlatformSupport)) {
            localTargetConfig.compileAdditionalSources.add(linuxPlatformSupport);
        }
        CCompiler cCompiler= new CCompiler(localTargetConfig, fedFileConfig, errorReporter);
        commandToReturn = String.join(" ",
                cCompiler.compileCCommand(
                        fileConfig.name+"_"+federate.name,
                        false
                    ).toString());
        return commandToReturn;
    }

    /**
     * Return the command that will execute a remote federate, assuming that the current
     * directory is the top-level project folder. This is used to create a launcher script
     * for federates.
     *
     * @param federate The federate to execute.
     */
    @Override
    protected
    String executeCommandForRemoteFederate(FederateInstance federate) {
        return "bin/"+fileConfig.name+"_"+federate.name+" -i '$FEDERATION_ID'";
    }

    /**
     * Return the command that will execute a local federate, assuming that the current
     * directory is the top-level project folder. This is used to create a launcher script
     * for federates.
     *
     * @param federate The federate to execute.
     */
    @Override
    protected
    String executeCommandForLocalFederate(FileConfig fileConfig, FederateInstance federate) {
        return fileConfig.binPath.resolve(fileConfig.name)+"_"+federate.name+" -i $FEDERATION_ID";
    }
}
