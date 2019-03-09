package org.icyphy.lf

import java.io.PrintStream

import scala.collection.mutable

class EmitCode(ps: PrintStream) extends LinguaFrancaBaseListener {

  def pr(s: String): Unit = {
    ps.println(s)
  }

  var current: Actor = null
  var composite: Composite = null

  var codeState = "unknown" // TODO: should be an enum
  var currentReact: Reaction = null
  var currentInstance: Instance = null
  
  // Assignment statements to set parameters within instance constructors.
  override def enterAssignment(ctx: LinguaFrancaParser.AssignmentContext): Unit = {
    if (currentInstance != null) {
        val parameterName = ctx.ID().getText
        var parameterValue = ctx.`value`.getText
        // FIXME: The following should be handled by the parser somehow!
        if (parameterValue.startsWith("{-")) {
            parameterValue = parameterValue.substring(2, parameterValue.length - 2);
        }
        currentInstance.instanceParameters += (parameterName -> parameterValue)
    }
  }

  override def enterCompositeHead(ctx: LinguaFrancaParser.CompositeHeadContext): Unit = {
    val name = ctx.ID().getText
    composite = new Composite(name)
    current = composite
    System.actors += (name -> current)
  }
  
  // Handle connections such as a.out -> b.in.
  override def enterConnection(ctx: LinguaFrancaParser.ConnectionContext): Unit = {
    val leftPort = ctx.`lport`.getText
    val rightPort = ctx.`rport`.getText
    // FIXME: Check for composite.
    composite.connections += new Connection(leftPort, rightPort);
  }

  override def enterHead(ctx: LinguaFrancaParser.HeadContext): Unit = {
    val name = ctx.ID().getText
    current = new Actor(name)
    System.actors += (name -> current)
  }
  
  // import statements are assumed to specify where to find accessors to be
  // instantiated in a composite.
  override def enterImp(ctx: LinguaFrancaParser.ImpContext): Unit = {
    val path = ctx.path().getText
    val pieces = path.split('.')
    val root = pieces.last
    val filename = pieces.mkString("/") + ".js"
    System.imports += (root -> filename)
  }

  // instance statement
  override def enterInstance(ctx: LinguaFrancaParser.InstanceContext): Unit = {
    // FIXME: Generate a friendly error if composite is null (actor is not a Composite).
    val instanceName = ctx.ID.getText;
    currentInstance = new Instance(instanceName, ctx.`actorClass`.getText)
    composite.instances += (instanceName -> currentInstance)
  }

  override def enterParam(ctx: LinguaFrancaParser.ParamContext): Unit = {
    current.parameter += (ctx.ID.getText -> new Parameter(ctx.ID.getText, ctx.`type`.getText, ctx.`value`.getText))
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

    pr("// Boilerplate included for all actors.")
    pr("var PERIODIC = true;")
    pr("var ONCE = false;")
    pr("function schedule(trigger, time, isPeriodic) {")
    pr("    if (isPeriodic) {")
    pr("        return trigger.actor.setInterval(trigger.reaction, time);")
    pr("    } else {")
    pr("        return trigger.actor.setTimeout(trigger.reaction, time);")
    pr("    }")
    pr("}")
    pr("function setUnbound(port, value) {")
    pr("    if (!port) {")
    pr("        throw \"Illegal reference to undeclared output.\";")
    pr("    }")
    pr("    this.send(port, value);")
    pr("}")
    pr("var set = setUnbound.bind(this);")

    // trigger
    pr("// Code generated for this particular actor.")
    pr("// Trigger data structure:")
    for (t <- actor.triggers) {
      val key = t._1
      val s = "var " + key + " = {'actor':this, 'triggerName':'" + key + "', 'reaction':reaction_" + key + ".bind(this)};"
      pr(s)
    }

    // preamble code
    pr(current.preCode)
    
    /////////////////////////////////////////////////
    // setup
    pr("exports.setup = function() {")

    actor.parameter.foreach {
      case (k, v) => {
        // TODO: why do we need getter functions? visibility?
        val s = "    this.parameter('" + v.gid() + "', {'type':'" + v.gtyp() + "', 'value':" + v.gdefault() + "});"
        pr(s)
      }

    }
    actor.outPorts.foreach {
      case (k, v) => {
        val s: String = "    this.output('" + v.gid + "', {'type':'" + v.gtyp + "'});"
        pr(s)
      }
    }
    if (composite != null) {
        composite.instances.foreach {
            case (k, v) => {
                // If the actorClass was defined in an import, use that.
                var actorClass = v.getActorClass()
                var actorPath = System.imports(actorClass)
                if (actorPath != null) {
                    actorClass = actorPath;
                }
                val s: String = "    var " + k + " = this.instantiate('" + v.getInstanceName() + "', '" + actorClass + "');"
                pr(s)
                v.instanceParameters.foreach {
                    case (name, value) => {
                        val s: String = "    " + k + ".setParameter('" + name + "', " + value + ");"
                        pr(s)
                    }
                }
            }
        }
        composite.connections.foreach {
            case (connection) => {
                val s: String = "    this.connect(" + connection.getLPort() + ", " + connection.getRPort() + ");"
                pr(s)
            }
        }
    }
    pr("}")

    /////////////////////////////////////////////////
    // initialize function
    def printParam(): Unit = {
      actor.parameter.foreach {
        case (k, v) => {
          val s: String = "    var " + v.gid + " = this.getParameter('" + v.gid + "');"
          pr(s)
        }
      }
    }

    pr("exports.initialize = function() {")
    printParam()
    actor.triggers.foreach {
      case (k, v) => {
        val s: String = "    schedule(" + v.gid + ", " + v.gparam + ", " + v.gtyp + ");"
        pr(s)

      }
    }
    pr(actor.initCode)
    pr("}")

    // reaction functions
    pr("// Reaction ")
    actor.reactions.foreach {
      case (r) => {
        var s: String = "function reaction_" + r.gt().gid() + "() {"
        pr(s)
        printParam()
        s = r.goutput
        s = "    var " + s + " = '" + s + "'; // FIXME in original .js code"
        pr(s)
        pr(r.gcoude())
      }
        pr("}")
    }

  }
}
