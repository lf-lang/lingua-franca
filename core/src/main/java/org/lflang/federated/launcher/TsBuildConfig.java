/*************
 * Copyright (c) 2019-2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
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

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

/**
 * Utility class that can be used to create a launcher for federated LF programs that are written in
 * TypeScript.
 *
 * @author Soroush Bateni
 * @author Hokeun Kim
 * @author Marten Lohstroh
 */
public class TsBuildConfig extends BuildConfig {

  public TsBuildConfig(
      FederateInstance federate, FedFileConfig fileConfig, MessageReporter messageReporter) {
    super(federate, fileConfig, messageReporter);
  }

  @Override
  public String compileCommand() {
    return null;
  }

  @Override
  public String localExecuteCommand() {
    String jsFilename = federate.name + ".js";
    return "node "
        + fileConfig.getSrcGenPath().resolve(federate.name).resolve("dist").resolve(jsFilename)
        + " -i $FEDERATION_ID";
  }
}
