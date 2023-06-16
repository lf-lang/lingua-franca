package org.lflang.generator.chisel

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.Target
import org.lflang.generator.CodeMap

import org.lflang.generator.GeneratorBase
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.TargetTypes
import org.lflang.generator.cpp.CppFileConfig
import org.lflang.generator.cpp.CppReactorGenerator
import org.lflang.generator.cpp.CppTypes
import org.lflang.isGeneric
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.util.FileUtil
import java.nio.file.Path

class ChiselGenerator (val context: LFGeneratorContext,
        private val scopeProvider: LFGlobalScopeProvider
    ) : GeneratorBase(context) {

    val fileConfig: ChiselFileConfig = context.fileConfig as ChiselFileConfig

    val chiselSources = mutableListOf<Path>()
    val codeMaps = mutableMapOf<Path, CodeMap>()
    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)
        val srcGenPath = fileConfig.srcGenBasePath

        for (r in reactors) {
            val generator = ChiselReactorGenerator(r, fileConfig, errorReporter)
            val sourceFile = fileConfig.getReactorSourcePath(r)
            val reactorCodeMap = CodeMap.fromGeneratedCode(generator.generateSource())
            codeMaps[srcGenPath.resolve(sourceFile)] = reactorCodeMap
            FileUtil.writeToFile(reactorCodeMap.generatedCode, srcGenPath.resolve(sourceFile), true)
        }
    }
    private fun fetchReactorChisel() {}
    override fun getTarget() = Target.Chisel
    override fun getTargetTypes(): TargetTypes = ChiselTypes
}