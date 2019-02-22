package org.icyphy.lf

import scala.collection.mutable

class Port(id: String, t: String)

class BaseActor(name: String) {

  val inPorts = new mutable.HashMap[String, Port]()
  val outPorts = new mutable.HashMap[String, Port]()

  println("Actor " + name + " found")

}

class Actor(name: String) extends BaseActor(name) {

}

class Composition(name: String) extends BaseActor(name) {

}
