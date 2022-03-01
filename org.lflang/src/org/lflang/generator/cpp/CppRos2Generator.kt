package org.lflang.generator.cpp

import org.lflang.generator.LFGeneratorContext
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path

class CppRos2Generator(generator: CppGenerator) : CppPlatformGenerator(generator) {

    override val srcGenPath: Path = generator.cppFileConfig.srcGenPath.resolve("src")
    private val packagePath: Path = generator.cppFileConfig.srcGenPath

    override fun generatePlatformFiles() {

        val nodeGenerator = CppRos2NodeGenerator(mainReactor, targetConfig, fileConfig);
        FileUtil.writeToFile(
            nodeGenerator.generateHeader(),
            packagePath.resolve("include").resolve("${nodeGenerator.nodeName}.hh"),
            true
        )
        FileUtil.writeToFile(
            nodeGenerator.generateSource(),
            packagePath.resolve("src").resolve("${nodeGenerator.nodeName}.cc"),
            true
        )

        val packageGenerator = CppRos2PackageGenerator(generator, nodeGenerator.nodeName)
        FileUtil.writeToFile(packageGenerator.generatePackageXml(), packagePath.resolve("package.xml"), true)
        FileUtil.writeToFile(
            packageGenerator.generatePackageCmake(generator.cppSources),
            packagePath.resolve("CMakeLists.txt"),
            true
        )
        val scriptPath = fileConfig.binPath.resolve(fileConfig.name);
        FileUtil.writeToFile(packageGenerator.generateBinScript(), scriptPath)
        scriptPath.toFile().setExecutable(true);
    }

    override fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean): Boolean {
        TODO("Not yet implemented")
    }
}