package org.icyphy.lf

import java.io.PrintStream

class EmitJS(ps: PrintStream) {

  def pr(s: String): Unit = {
    ps.println(s)
  }


  def printStuff(current: Actor, composite: Composite): Unit = {

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
