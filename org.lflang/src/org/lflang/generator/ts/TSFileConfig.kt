/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.

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

package org.lflang.generator.ts

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.FileConfig
import org.lflang.util.FileUtil
import java.io.IOException
import java.nio.file.Path

/**
 * Generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de>}
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */
class TSFileConfig(
    resource: Resource, srcGenBasePath: Path, useHierarchicalBin: Boolean
) : FileConfig(resource, srcGenBasePath, useHierarchicalBin) {

    /**
     * Clean any artifacts produced by the TypeScript code generator.
     */
    @Throws(IOException::class)
    override fun doClean() {
        super.doClean()
        FileUtil.deleteDirectory(srcGenPath)
    }

    /**
     * Path to TypeScript source code.
     */
    fun tsSrcGenPath(): Path = srcGenPath.resolve("src")

    /**
     * Path to TypeScript core source code.
     */
    fun reactorTsPath(): Path = srcGenPath.resolve("reactor-ts")

    /**
     * Path to the generated docker file
     */
    fun tsDockerFilePath(tsFileName: String): Path {
        return srcGenPath.resolve("$tsFileName.Dockerfile")
    }

    /**
     * Path to the generated docker compose file
     */
    fun tsDockerComposeFilePath(): Path {
        return srcGenPath.resolve("docker-compose.yml")
    }
}
