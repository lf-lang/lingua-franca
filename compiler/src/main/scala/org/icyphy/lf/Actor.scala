package org.icyphy.lf

import scala.collection.mutable.{HashMap, ListBuffer}

class Connection(lport: String, rport: String) {
    // Return the specification for the port as a string.
    // If the input is "x.y", then return "x, 'y'"
    // If the input is "x", then return "'x'"
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

class Port(id: String, typ: String) {
  def gid() = id
  def gtyp = typ
}

class Instance(instanceName: String, actorClass: String) {
  def getActorClass() = actorClass;
  def getInstanceName() = instanceName;
  val instanceParameters = new HashMap[String, String]()
}

class Parameter(id: String, typ: String, default: String) {
  // TODO: getter functions?????
  def gid() = id
  def gtyp() = typ
  def gdefault() = default
}

class Trigger(id: String, param: String, typ: String) {
  def gid() = id
  def gparam() = param
  def gtyp() = typ
}

class Reaction(t: Trigger, output: String, var code: String) {
  def gt() = t
  def goutput() = output
  def gcoude() = code
}

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
