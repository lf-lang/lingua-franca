package org.lflang.generator.cpp

import org.eclipse.xtext.generator.IFileSystemAccess2
import org.lflang.generator.LFGeneratorContext

class CppRos2Generator(generator: CppGenerator) : CppPlatformGenerator(generator) {

    override fun generatePlatformFiles(fsa: IFileSystemAccess2) {
        val packageXml = CppRos2PackageGenerator(generator).generatePackageXml()
        fsa.generateFile(relSrcGenPath.resolve("package.xml").toString(), packageXml)

        val cmake = CppRos2CmakeGenerator(generator).generateCode(generator.cppSources)
        fsa.generateFile(relSrcGenPath.resolve("CMakeLists.txt").toString(), cmake)
    }

    override fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean): Boolean {
        TODO("Not yet implemented")
    }
}