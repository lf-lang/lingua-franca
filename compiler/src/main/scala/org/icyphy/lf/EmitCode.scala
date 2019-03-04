package org.icyphy.lf

import java.io.PrintStream

import scala.collection.mutable

class EmitCode(ps: PrintStream) extends LinguaFrancaBaseListener {

  def println(s: String): Unit = {
    ps.println(s)
  }

  var current: Actor = null

  var codeState = "unknown" // TODO: should be an enum
  var currentReact: Reaction = null

  override def enterHead(ctx: LinguaFrancaParser.HeadContext): Unit = {
    val name = ctx.ID().getText
    current = new Actor(name)
    System.actors += (name -> current)
  }

  override def enterParam(ctx: LinguaFrancaParser.ParamContext): Unit = {
    current.parameter += (ctx.ID.getText -> new Parameter(ctx.ID.getText, ctx.`type`.getText, ctx.`def`.INTVAL.getText))
  }

  override def enterOutp(ctx: LinguaFrancaParser.OutpContext): Unit = {
    // TODO: mybe not use the word `type` and `def` in the grammar
    current.outPorts += (ctx.ID.getText -> new Port(ctx.ID.getText, ctx.`type`.getText))
  }

  override def enterPre(ctx: LinguaFrancaParser.PreContext): Unit = {
    codeState = "pre"
  }

  override def enterInit(ctx: LinguaFrancaParser.InitContext): Unit = {
    codeState = "init"
  }

  override def enterReact(ctx: LinguaFrancaParser.ReactContext): Unit = {
    codeState = "react"
    val t = current.triggers(ctx.ID.getText)
    // sets is really a list
    val s = ctx.sets(0).ID.getText
    val r = new Reaction(t, s, "")
    currentReact = r
    current.reactions += r
  }

  override def enterCode(ctx: LinguaFrancaParser.CodeContext): Unit = {
    var s: String = ctx.CODE.getText
    s = s.substring(2, s.length - 2)
    codeState match {
      case "pre" => current.preCode = s
      case "init" => current.initCode = s
      case "react" => currentReact.code = s
      case _ => "???"
    }
  }

  override def enterTrig(ctx: LinguaFrancaParser.TrigContext): Unit = {
    val t = new Trigger(ctx.ID.getText, ctx.trigparam.ID.getText, ctx.trigtype.getText)
    current.triggers += (ctx.ID.getText -> t)
  }

  // TODO: shall be in a JS/C/Rust printer class
  def printStuff(): Unit = {

    val actor = current

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

    // trigger
    println("// Code generated for this particular actor.")
    println("// Trigger data structure:")
    for (t <- actor.triggers) {
      val key = t._1
      val s = "var " + key + " = {'actor':this, 'triggerName':'" + key + "', 'reaction':reaction_" + key + ".bind(this)};"
      println(s)
    }

    // preamble code
    println(current.preCode)
    // xxx
    println("// Generated setup function:")
    println("exports.setup = function() {")

    actor.parameter.foreach {
      case (k, v) => {
        // TODO: why do we need getter functions? visibility?
        val s = "    this.parameter('" + v.gid() + "', {'type':'" + v.gtyp() + "', 'value':" + v.gdefault() + "});"
        println(s)
      }

    }
    actor.outPorts.foreach {
      case (k, v) => {
        val s: String = "    this.output('" + v.gid + "', {'type':'" + v.gtyp + "'});"
        println(s)
      }
    }
    println("}")

    // initialize function
    def printParam(): Unit = {
      actor.parameter.foreach {
        case (k, v) => {
          val s: String = "    var " + v.gid + " = this.getParameter('" + v.gid + "');"
          println(s)
        }
      }
    }

    println("exports.initialize = function() {")
    printParam()
    actor.triggers.foreach {
      case (k, v) => {
        val s: String = "    schedule(" + v.gid + ", " + v.gparam + ", " + v.gtyp + ");"
        println(s)

      }
    }
    println(actor.initCode)
    println("}")

    // reaction functions
    println("// Reaction ")
    actor.reactions.foreach {
      case (r) => {
        var s: String = "function reaction_" + r.gt().gid() + "() {"
        println(s)
        printParam()
        s = r.goutput
        s = "    var " + s + " = '" + s + "'; // FIXME in original .js code"
        println(s)
        println(r.gcoude())
      }
        println("}")
    }

  }
}
