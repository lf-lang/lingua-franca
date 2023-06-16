
/**
 * @author Erling R. Jellum (erling.r.jellum@ntnu.no)
 *
 * Copyright (c) 2023, The Norwegian University of Science and Technology.
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
package org.lflang.generator.chisel

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.FileConfig
import org.lflang.lf.Reactor
import org.lflang.name
import org.lflang.util.FileUtil
import java.io.IOException
import java.nio.file.Path

class ChiselFileConfig(resource: Resource, srcGenBasePath: Path, useHierarchicalBin: Boolean) :
    FileConfig(resource, srcGenBasePath, useHierarchicalBin) {

    /**
     * Clean any artifacts produced by the C++ code generator.
     */
    @Throws(IOException::class)
    override fun doClean() {
        super.doClean()
        this.chiselBuildDirectories.forEach { FileUtil.deleteDirectory(it) }
    }

    val chiselBuildDirectories = listOf<Path>(
    )

    /** Relative path to the directory where all source files for this resource should be generated in. */
    private fun getGenDir(r: Resource): Path = this.getDirectory(r).resolve(r.name)

    /** Path to the source file corresponding to this reactor (needed for non generic reactors)  */
    fun getReactorSourcePath(r: Reactor): Path = getGenDir(r.eResource()).resolve("${r.name}.scala")

    /** Path to the build directory containing CMake-generated files */
    val buildPath: Path get() = this.outPath.resolve("build")
}