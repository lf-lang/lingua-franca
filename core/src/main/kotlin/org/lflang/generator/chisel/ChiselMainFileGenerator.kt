package org.lflang.generator.chisel

import org.lflang.ErrorReporter
import org.lflang.TargetConfig
import org.lflang.lf.Reactor

class ChiselMainFileGenerator(private val mainReactor: Reactor, val fileConfig: ChiselFileConfig, val targetConfig: TargetConfig, val errorReporter: ErrorReporter) {

    fun generateSource(): String {
        var timeOut = "Time.NEVER"

        if (targetConfig.timeout != null) {
            timeOut = "Time.nsec(${targetConfig.timeout.toNanoSeconds()})"
        }
        return """
            |package reactor
            |import scala.sys.process.Process
            |import java.nio.file.{Files, Paths}
            |object LfMain {
            |  def main(args: Array[String]): Unit = {
            |    val targetDir = if (args.length == 1) args(0) else s"build/${mainReactor.name}"
            |    val binDir = "${fileConfig.binPath}"
            |    val mainReactorFunc = () => new lf.${mainReactor.name}.${mainReactor.name}()
            |    println("------------------------------------------------------------------------------------------------------")
            |    println("Running Chisel compiler to generate verilator project")
            |    println("------------------------------------------------------------------------------------------------------")
            |    val chiselArgs = Array("--target-dir", s"${"$"}targetDir/tmp")
            |    implicit val globalConfig = GlobalReactorConfig(timeout = ${timeOut})
            |    val verilog = (new chisel3.stage.ChiselStage).emitVerilog(new StandaloneMainReactor((mainReactorFunc)),chiselArgs)
            |    val saveLocation = targetDir + "/ReactorChisel.v"
            |    Settings.writeVerilogToFile(verilog, saveLocation)
            |    println(s"Wrote the generated verilog to `${"$"}saveLocation`")
            |    // Copy main cpp file for emulation
            |    fpgatidbits.TidbitsMakeUtils.fileCopy("reactor-chisel/src/main/resources/simulator/standalone/main.cpp", targetDir + "/main.cpp")
            |    fpgatidbits.TidbitsMakeUtils.fileCopy("reactor-chisel/src/main/resources/simulator/standalone/Makefile", targetDir + "/Makefile")
            |    println("------------------------------------------------------------------------------------------------------")
            |    println("Building verilator project")
            |    println("------------------------------------------------------------------------------------------------------")
            |    val result = Process("make", new java.io.File(targetDir)).!
            |    println("------------------------------------------------------------------------------------------------------")
            |    println("Copying executable into bin dir. ")
            |    println("------------------------------------------------------------------------------------------------------")
            |    if (!Files.exists(Paths.get(binDir))) {
            |      Files.createDirectories(Paths.get(binDir))
            |    }
            |    fpgatidbits.TidbitsMakeUtils.fileCopy(targetDir + "/emu", binDir + "/${mainReactor.name}")   
            |    println("------------------------------------------------------------------------------------------------------")
            |    println("To execute program run:`" + binDir + "/${mainReactor.name}`")
            |    println("------------------------------------------------------------------------------------------------------")
            |  }
            |}
            | 
        """.trimMargin()
    }
}