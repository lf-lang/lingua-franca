package org.icyphy.lf

import java.io.PrintStream
import java.util

class EmitJSJ2S(ps: PrintStream) extends LinguaFrancaBaseListener {

  private[lf] class TriggerData {
    private[lf] var param: String = null
    private[lf] var `type`: String = null
  }

  private[lf] val param: util.Map[String, String] = new util.HashMap[String, String]
  private[lf] val paramType: util.Map[String, String] = new util.HashMap[String, String]

  private[lf] val outType: util.Map[String, String] = new util.HashMap[String, String]

  private[lf] val trigger: util.Map[String, TriggerData] = new util.HashMap[String, TriggerData]

  private[lf] def println(s: String): Unit = {
    ps.println(s)
  }

  override def enterSys(ctx: LinguaFrancaParser.SysContext): Unit = {
    println("// Boilerplate included for all actors.")
    println("var PERIODIC = true;")
    println("var ONCE = false;")
    println("function schedule(trigger, time, isPeriodic) {")
    println("    if (isPeriodic) {")
    println("        return trigger.actor.setInterval(trigger.reaction, time);")
    println("    } else {")
    println("        return trigger.actor.setTimeout(trigger.reaction, time);")
    println("    }")
    println("}")
    println("function setUnbound(port, value) {")
    println("    if (!port) {")
    println("        throw \"Illegal reference to undeclared output.\";")
    println("    }")
    println("    this.send(port, value);")
    println("}")
    println("var set = setUnbound.bind(this);")
  }

  override def enterParam(ctx: LinguaFrancaParser.ParamContext): Unit = {
    param.put(ctx.ID.getText, ctx.`def`.INTVAL.getText)
    paramType.put(ctx.ID.getText, ctx.`type`.getText)
  }

  override def enterOutp(ctx: LinguaFrancaParser.OutpContext): Unit = {
    outType.put(ctx.ID.getText, ctx.`type`.getText)
  }

  override def enterCode(ctx: LinguaFrancaParser.CodeContext): Unit = {
    var s: String = ctx.CODE.getText
    s = s.substring(2, s.length - 2)
    println(s)
  }

  override def enterTrig(ctx: LinguaFrancaParser.TrigContext): Unit = {
    val td: TriggerData = new TriggerData
    td.param = ctx.trigparam.ID.getText
    td.`type` = ctx.trigtype.getText
    trigger.put(ctx.ID.getText, td)
  }

  override def enterPre(ctx: LinguaFrancaParser.PreContext): Unit = {
    println("// Code generated for this particular actor.")
    println("// Trigger data structure:")
    import scala.collection.JavaConversions._
    for (key <- trigger.keySet) {
      val s: String = "var " + key + " = {'actor':this, 'triggerName':'" + key + "', 'reaction':reaction_" + key + ".bind(this)};"
      println(s)
    }
  }

  override def exitPre(ctx: LinguaFrancaParser.PreContext): Unit = {
    println("// Generated setup function:")
    println("exports.setup = function() {")
    import scala.collection.JavaConversions._
    for (key <- param.keySet) {
      val `def`: String = param.get(key)
      val typ: String = paramType.get(key)
      val s: String = "    this.parameter('" + key + "', {'type':'" + typ + "', 'value':" + `def` + "});"
      println(s)
    }
    import scala.collection.JavaConversions._
    for (key <- outType.keySet) {
      val typ: String = outType.get(key)
      val s: String = "    this.output('" + key + "', {'type':'" + typ + "'});"
      println(s)
    }
    println("}")
  }

  private[lf] def printParam(): Unit = {
    import scala.collection.JavaConversions._
    for (key <- param.keySet) {
      val s: String = "    var " + key + " = this.getParameter('" + key + "');"
      println(s)
    }
  }

  override def enterInit(ctx: LinguaFrancaParser.InitContext): Unit = {
    println("exports.initialize = function() {")
    printParam()
    import scala.collection.JavaConversions._
    for (key <- trigger.keySet) {
      val td: TriggerData = trigger.get(key)
      val s: String = "    schedule(" + key + ", " + td.param + ", " + td.`type` + ");"
      println(s)
    }
  }

  override def exitInit(ctx: LinguaFrancaParser.InitContext): Unit = {
    println("}")
  }

  override def enterReact(ctx: LinguaFrancaParser.ReactContext): Unit = {
    println("// Reaction ")
    var s: String = "function reaction_" + ctx.ID.getText + "() {"
    println(s)
    printParam()
    // sets is really a list
    s = ctx.sets(0).ID.getText
    s = "    var " + s + " = '" + s + "'; // FIXME in original .js code"
    println(s)
  }

  override def exitReact(ctx: LinguaFrancaParser.ReactContext): Unit = {
    println("}")
  }

}
