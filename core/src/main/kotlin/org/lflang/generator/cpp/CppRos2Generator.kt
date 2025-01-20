package org.lflang.generator.cpp

import org.apache.commons.text.StringEscapeUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.docker.DockerGenerator
import org.lflang.target.property.DockerProperty
import org.lflang.util.FileUtil
import java.nio.file.Path

/** C++ platform generator for the ROS2 platform.*/
class CppRos2Generator(generator: CppGenerator) : CppPlatformGenerator(generator) {

    override val srcGenPath: Path = generator.fileConfig.srcGenPath.resolve("src")
    private val packagePath: Path = generator.fileConfig.srcGenPath
    private val nodeGenerator = CppRos2NodeGenerator(mainReactor, targetConfig, fileConfig);
    private val packageGenerator = CppRos2PackageGenerator(generator, nodeGenerator.nodeName)

    companion object {
        const val DEFAULT_BASE_IMAGE: String = "ros:humble-ros-base"
    }

    override fun generatePlatformFiles() {
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

        FileUtil.writeToFile(packageGenerator.generatePackageXml(), packagePath.resolve("package.xml"), true)
        FileUtil.writeToFile(
            packageGenerator.generatePackageCmake(generator.cppSources),
            packagePath.resolve("CMakeLists.txt"),
            true
        )
        val scriptPath =
            if (targetConfig.get(DockerProperty.INSTANCE).enabled)
                fileConfig.srcGenPath.resolve(relativeBinDir).resolve(fileConfig.name)
            else
                fileConfig.binPath.resolve(fileConfig.name)
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
            "colcon", colconArgs(), fileConfig.outPath
        )
        val returnCode = colconCommand?.run(context.cancelIndicator)
        if (returnCode != 0 && !messageReporter.errorsOccurred) {
            // If errors occurred but none were reported, then the following message is the best we can do.
            messageReporter.nowhere().error("colcon failed with error code $returnCode")
        }

        return !messageReporter.errorsOccurred
    }

    private fun colconArgs(): List<String> {
        return listOf(
            "build",
            "--packages-select",
            fileConfig.name,
            packageGenerator.reactorCppName,
            "--cmake-args",
            "-DLF_REACTOR_CPP_SUFFIX=${packageGenerator.reactorCppSuffix}",
        ) + cmakeArgs
    }

    inner class CppDockerGenerator(context: LFGeneratorContext?) : DockerGenerator(context) {
        override fun generateCopyForSources() =
            """
                COPY src src
                COPY src-gen src-gen
                COPY bin bin
            """.trimIndent()

        override fun defaultImage(): String = DEFAULT_BASE_IMAGE

        override fun generateRunForInstallingDeps(): String = ""

        override fun defaultEntryPoint(): List<String> =
            listOf("$relativeBinDir/${fileConfig.name}")

        override fun generateCopyOfExecutable(): String {
            val name = fileConfig.name
            return """
                COPY --from=builder /lingua-franca/$name/$relativeBinDir/$name ./$relativeBinDir/$name
                COPY --from=builder lingua-franca/${fileConfig.name}/install install
            """.trimIndent()
        }

        override fun defaultBuildCommands(): List<String> {
            val commands = listOf(
                listOf("mkdir", "-p", "build"),
                listOf("colcon") + colconArgs(),
            )
            return commands.map { argListToCommand(it) }
        }

        override fun getPreBuildCommand(): MutableList<String> {
            val script = context.targetConfig.get(DockerProperty.INSTANCE).preBuildScript
            if (script.isNotEmpty()) {
                return mutableListOf(". src/" + StringEscapeUtils.escapeXSI(script))
            }
            return mutableListOf(". /opt/ros/humble/setup.sh")
        }

        override fun getPostBuildCommand(): MutableList<String> {
            val script = context.targetConfig.get(DockerProperty.INSTANCE).postBuildScript
            if (script.isNotEmpty()) {
                return mutableListOf(". src/" + StringEscapeUtils.escapeXSI(script))
            }
            return mutableListOf()
        }
    }

    override fun getDockerGenerator(context: LFGeneratorContext?): DockerGenerator = CppDockerGenerator(context)

}
