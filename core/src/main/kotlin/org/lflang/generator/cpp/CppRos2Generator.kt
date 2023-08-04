package org.lflang.generator.cpp

import org.lflang.AttributeUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.reactor
import org.lflang.util.FileUtil
import java.nio.file.Path

/** C++ platform generator for the ROS2 platform.*/
class CppRos2Generator(generator: CppGenerator) : CppPlatformGenerator(generator) {

    override val srcGenPath: Path = generator.fileConfig.srcGenPath.resolve("src")
    private val packagePath: Path = generator.fileConfig.srcGenPath
    private val nodeGenerators : MutableList<CppRos2NodeGenerator> = mutableListOf(CppRos2NodeGenerator(mainReactor, targetConfig, fileConfig))
    private val packageGenerator = CppRos2PackageGenerator(generator)
    private val lfMsgsRosPackageName = "lf_msgs_ros"

    override fun generatePlatformFiles() {
        val reactorsToSearch : MutableList<org.lflang.lf.Reactor> = mutableListOf(mainReactor)
        /** Recursively searching for federates */
        while (reactorsToSearch.isNotEmpty()) {
            reactorsToSearch[0].instantiations.forEach {
                reactorsToSearch.add(it.reactor)
                if (AttributeUtils.isFederate(it)) {
                    nodeGenerators.add(
                        CppRos2NodeGenerator(it.reactor, targetConfig, fileConfig))
                }
            }
            reactorsToSearch.removeFirst()
        }

        packageGenerator.nodeGenerators =  nodeGenerators

        // tag message package
        val lfMsgsRosDir = "/lib/cpp/$lfMsgsRosPackageName"
        FileUtil.copyFromClassPath(lfMsgsRosDir, fileConfig.srcGenBasePath, true, false)

        val messageTypesToWrap : MutableSet<String> = mutableSetOf()
        for (nodeGen in nodeGenerators) {
            messageTypesToWrap.addAll(nodeGen.getMessageTypes())
        }
        val msgWrapGen = CppRos2MessageWrapperGenerator(messageTypesToWrap)
        for ((messageFileName, messageFileContent) in msgWrapGen.generateMessageFiles()) {
            FileUtil.writeToFile(messageFileContent, fileConfig.srcGenBasePath.resolve("lf_wrapped_msgs").resolve("msg").resolve("$messageFileName.msg"))
        }
        FileUtil.writeToFile(msgWrapGen.generatePackageCmake(), fileConfig.srcGenBasePath.resolve("lf_wrapped_msgs").resolve("CMakeLists.txt"))
        FileUtil.writeToFile(msgWrapGen.generatePackageXml(), fileConfig.srcGenBasePath.resolve("lf_wrapped_msgs").resolve("package.xml"))

        for (nodeGen in nodeGenerators) {
            FileUtil.writeToFile(
                nodeGen.generateHeader(),
                packagePath.resolve("include").resolve("${nodeGen.nodeName}.hh"),
                true
            )
            FileUtil.writeToFile(
                nodeGen.generateSource(),
                packagePath.resolve("src").resolve("${nodeGen.nodeName}.cc"),
                true
            )
        }

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
        val ros2Version = System.getenv("ROS_DISTRO")

        if (ros2Version.isNullOrBlank()) {
            messageReporter.nowhere(
            ).error(
                "Could not find a ROS2 installation! Please install ROS2 and source the setup script. " +
                        "Also see https://docs.ros.org/en/galactic/Installation.html"
            )
            return false
        }

        val colconCommand = commandFactory.createCommand(
            "colcon", listOf(
                "build",
                "--packages-select",
                lfMsgsRosPackageName,
                "lf_wrapped_msgs",
                fileConfig.name,
                packageGenerator.reactorCppName,
                "--cmake-args",
                "-DLF_REACTOR_CPP_SUFFIX=${packageGenerator.reactorCppSuffix}",
                "-DLF_SRC_PKG_PATH=${fileConfig.srcPkgPath}"
            ) + cmakeArgs,
            fileConfig.srcGenBasePath
        )


        val returnCode = colconCommand?.run(context.cancelIndicator);
        if (returnCode != 0 && !messageReporter.errorsOccurred) {
            // If errors occurred but none were reported, then the following message is the best we can do.
            messageReporter.nowhere().error("colcon failed with error code $returnCode")
        }

        return !messageReporter.errorsOccurred
    }
}
