package org.icyphy.lf

import scala.collection.mutable.{HashMap, ListBuffer}

class Connection(lport: String, rport: String) {
    // Return the specification for the port as a string.
    // If the input is "x.y", then return "x, 'y'"
    // If the input is "x", then return "'x'"
    // FIXME: this is JS code generation specific and should be moved to EmitJS.scala
    def getPortSpec(port: String): String = {
        val a = port.split('.');
        if (a.length == 1) {
            "'" + a(0) + "'"
        } else if (a.length > 1) {
            a(0) + ", '" + a(1) + "'"
        } else {
            "INVALID_PORT_SPEC:" + port
        }
    }
    def getLPort() = getPortSpec(lport);
    def getRPort() = getPortSpec(rport);
}

case class Port(id: String, typ: String)

case class Instance(instanceName: String, actorClass: String) {
  val instanceParameters = new HashMap[String, String]()
}

case class Parameter(id: String, typ: String, default: String)

case class Trigger(id: String, param: String, typ: String)

case class Reaction(t: Trigger, output: String, var code: String)

class Actor(name: String) {
  val parameter = new HashMap[String, Parameter]()
  val inPorts = new HashMap[String, Port]()
  val outPorts = new HashMap[String, Port]()

  val triggers = new HashMap[String, Trigger]()
  val reactions = new ListBuffer[Reaction]()
  
  var preCode = ""
  var initCode = ""
}

class Composite(name: String) extends Actor(name) {
  val instances = new HashMap[String, Instance]()
  val connections = new ListBuffer[Connection]()
}

object System {
  val actors = new HashMap[String, Actor]() // FIXME: maybe this should be a list, as we have an order
  val imports = new HashMap[String, String]()
}
