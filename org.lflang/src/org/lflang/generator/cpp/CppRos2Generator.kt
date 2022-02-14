package org.lflang.generator.cpp

import org.lflang.generator.JavaGeneratorUtils
import org.lflang.generator.LFGeneratorContext

class CppRos2Generator(generator: CppGenerator) : CppPlatformGenerator(generator) {

    override fun generatePlatformFiles() {
        val packageXml = CppRos2PackageGenerator(generator).generatePackageXml()
        JavaGeneratorUtils.writeToFile(packageXml, relSrcGenPath.resolve("package.xml"))

        val cmake = CppRos2CmakeGenerator(generator).generateCode(generator.cppSources)
        JavaGeneratorUtils.writeToFile(cmake, relSrcGenPath.resolve("CMakeLists.txt"))
    }

    override fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean): Boolean {
        TODO("Not yet implemented")
    }
}