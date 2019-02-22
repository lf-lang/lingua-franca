package org.icyphy.lf

import org.antlr.v4.runtime._
import org.antlr.v4.runtime.tree._
import java.io._

object Compiler extends App {

  val is = new FileInputStream(args(0))
  val ps = new PrintStream(args(1))
  val input = new ANTLRInputStream(is)

  val lexer = new LinguaFrancaLexer(input)
  val tokens = new CommonTokenStream(lexer)
  val parser = new LinguaFrancaParser(tokens)
  val tree = parser.sys()

  val walker = new ParseTreeWalker()
  walker.walk(new EmitJSJ2S(ps), tree)
  ps.println()
}
