package org.icyphy.lf

class GetLang() extends LinguaFrancaBaseListener {

  var language = "UNDEFINED"

  override def exitLang(ctx: LinguaFrancaParser.LangContext) {
    language = ctx.ID().getText
  }
}
