package org.icyphy.lf

import java.io.PrintStream

class EmitCode(ps: PrintStream) extends LinguaFrancaBaseListener {

  def println(s: String): Unit = {
    ps.println(s)
  }

  override def enterSys(ctx: LinguaFrancaParser.SysContext): Unit = {
    println("// Boilerplate included for all actors.")
    println("var PERIODIC = true;")
  }

}
