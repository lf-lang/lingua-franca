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

package org.lflang.federated;

import java.io.IOException;
import java.nio.file.Path;
import org.lflang.FileConfig;

/**
 * A child class of @see FileConfig that extends the base functionality to add support
 * for federated execution. The code generator should create one instance of this class
 * for each federate.
 *
 * @author Soroush Bateni
 *
 */
public class FedFileConfig extends FileConfig {

    /** Name of the federate for this FedFileConfig */
    protected final String federateName;
    private final boolean isFederated;

    /**
     * Create an instance of FedFileConfig for federate 'federateName' from an existing
     * 'fileConfig' instance (an instance of 'FileConfig').
     *
     * @param fileConfig The existing instance of the 'FileConfig' class.
     * @param federateName The name of the federate.
     * @throws IOException
     */
    public FedFileConfig(boolean isFederated, FileConfig fileConfig, String federateName) throws IOException {
        super(fileConfig.resource, fileConfig.getSrcGenBasePath(), fileConfig.useHierarchicalBin);
        this.federateName = federateName;
        this.isFederated = isFederated;
    }

    @Override
    public Path getSrcGenPath() {
        return isFederated ?
               srcGenPath.resolve(federateName) :
               srcGenPath;
    }
}
