package org.icyphy.lf

import scala.collection.mutable.{HashMap, ListBuffer}

class Port(id: String, typ: String) {
  def gid() = id
  def gtyp = typ
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

object System {
  val actors = new HashMap[String, Actor]() // FIXME: maybe this should be a list, as we have an order

}
