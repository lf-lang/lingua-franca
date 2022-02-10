package org.lflang.generator.cpp

import org.lflang.generator.JavaGeneratorUtils
import org.lflang.generator.LFGeneratorContext

class CppRos2Generator(generator: CppGenerator) : CppPlatformGenerator(generator) {

    override fun generatePlatformFiles() {
        val packageXml = CppRos2PackageGenerator(generator).generatePackageXml()
        JavaGeneratorUtils.writeToFile(relSrcGenPath.resolve("package.xml").toString(), packageXml)

        val cmake = CppRos2CmakeGenerator(generator).generateCode(generator.cppSources)
        JavaGeneratorUtils.writeToFile(relSrcGenPath.resolve("CMakeLists.txt").toString(), cmake)
    }

    override fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean): Boolean {
        TODO("Not yet implemented")
    }
}