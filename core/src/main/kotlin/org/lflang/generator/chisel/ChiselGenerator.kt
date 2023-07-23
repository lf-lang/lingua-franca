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
import org.lflang.Target
import org.lflang.generator.CodeMap

import org.lflang.generator.GeneratorBase
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.TargetTypes
import org.lflang.lf.Reactor
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path

class ChiselGenerator (val context: LFGeneratorContext,
        private val scopeProvider: LFGlobalScopeProvider
    ) : GeneratorBase(context) {

    val fileConfig: ChiselFileConfig = context.fileConfig as ChiselFileConfig

    val chiselSources = mutableListOf<Path>()
    val codeMaps = mutableMapOf<Path, CodeMap>()
    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)
        val scalaSrcGenPath = fileConfig.srcGenPath.resolve("src/main/scala/")
        var mainReactor: Reactor = reactors.get(0)
        for (r in reactors) {
            val generator = ChiselReactorGenerator(r, fileConfig, errorReporter)
            val sourceFile = fileConfig.getReactorSourcePath(r)
            val reactorCodeMap = CodeMap.fromGeneratedCode(generator.generateSource())
            codeMaps[scalaSrcGenPath.resolve(sourceFile)] = reactorCodeMap
            FileUtil.writeToFile(reactorCodeMap.generatedCode, scalaSrcGenPath.resolve(sourceFile), true)

            if (r.isMain) {
                mainReactor = r
            }
        }

        // Generate the Main.scala file
        val mainFileGenerator = ChiselMainFileGenerator(mainReactor, fileConfig, context.targetConfig, errorReporter)
        FileUtil.writeToFile(mainFileGenerator.generateSource(), scalaSrcGenPath.resolve("Main.scala"), true)

        // Generate the build.sbt file
        val sbtGenerator = ChiselSbtGenerator(mainReactor, fileConfig, errorReporter)
        FileUtil.writeToFile(sbtGenerator.generateSource(), fileConfig.srcGenPath.resolve("build.sbt"), true)

        // Copy reactor-chisel
        copyReactorChisel()

        // Create symlink to reactor-chisel in directory above
        Files.deleteIfExists(fileConfig.srcGenPath.resolve("reactor-chisel"));
        Files.createSymbolicLink(fileConfig.srcGenPath.resolve("reactor-chisel"), fileConfig.srcGenBasePath.resolve("reactor-chisel"));

        // compile
        // FIXME: check no-compile
        compile()
    }

    private fun compile() {
        val cmd = commandFactory.createCommand("sbt", listOf("run"), fileConfig.srcGenPath)
        val returnCode = cmd.run()
        if (returnCode != 0) {
            errorReporter.reportError("`sbt run` failed due to either bug in reaction bodies or reactor-chisel.")
        }
    }

    private fun copyReactorChisel() {
        FileUtil.copyFromClassPath(
            "/lib/chisel/reactor-chisel",
            fileConfig.srcGenBasePath.resolve("reactor-chisel"),
            true,
            true
        )
    }
    override fun getTarget() = Target.Chisel
    override fun getTargetTypes(): TargetTypes = ChiselTypes
}