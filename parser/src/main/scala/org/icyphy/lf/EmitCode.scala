package org.icyphy.lf

import java.io.PrintStream

import scala.collection.mutable

class EmitCode(ps: PrintStream) extends LinguaFrancaBaseListener {

  def println(s: String): Unit = {
    ps.println(s)
  }

  val actors = new mutable.HashMap[String, BaseActor]()

  override def enterHead(ctx: LinguaFrancaParser.HeadContext): Unit = {
    val name = ctx.ID().getText
    actors += (name -> new Actor(name))
  }

  override def enterSys(ctx: LinguaFrancaParser.SysContext): Unit = {
    println("// Boilerplate included for all actors.")
    println("var PERIODIC = true;")
  }

}
