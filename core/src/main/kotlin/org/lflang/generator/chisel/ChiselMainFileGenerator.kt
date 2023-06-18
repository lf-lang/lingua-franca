package org.lflang.generator.chisel

import org.lflang.ErrorReporter
import org.lflang.lf.Reactor

class ChiselMainFileGenerator(private val mainReactor: Reactor, val fileConfig: ChiselFileConfig, val errorReporter: ErrorReporter) {

    fun generateSource() =
        """
            |package lf
            |import reactor.{StandaloneMainReactor, Settings}
            |import scala.sys.process.Process
            |import java.nio.file.{Files, Paths}
            |object LfMain {
            |  def main(args: Array[String]): Unit = {
            |    val targetDir = if (args.length == 1) args(0) else s"build/${mainReactor.name}"
            |    val binDir = "${fileConfig.binPath}"
            |    val mainReactorFunc = () => new lf.Timer.Timer()
            |    println("------------------------------------------------------------------------------------------------------")
            |    println("Running Chisel compiler to generate verilator project")
            |    println("------------------------------------------------------------------------------------------------------")
            |    val chiselArgs = Array("--target-dir", s"${"$"}targetDir/tmp")
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
            |    println("To execute program run:`./" + binDir + "/${mainReactor.name}")
            |    println("------------------------------------------------------------------------------------------------------")
            |  }
            |}
            | 
        """.trimMargin()
}