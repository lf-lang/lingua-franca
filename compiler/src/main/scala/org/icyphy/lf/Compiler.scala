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
  val lang = new GetLanguage()
  walker.walk(lang, tree)
  val lf = new WalkLF()
  walker.walk(lf, tree)

  val printer = lang.language match {
    case "JavaScript" => new EmitJS(ps)
    // FIXME: The following throughs a very unfriendly null pointer exception.
    // Should instead have a generic code generator, e.g. one that generates
    // documentation.
    case _ => null
  }

  // TODO: each actor into it's own file to support more than one actor in a file
  printer.printStuff(lf.current, lf.composite)
  ps.println()
}
