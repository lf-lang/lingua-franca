package org.icyphy.lf

class GetLanguage() extends LinguaFrancaBaseListener {

  var language = "UNDEFINED"

  override def exitLanguage(ctx: LinguaFrancaParser.LanguageContext) {
    language = ctx.ID().getText
  }
}
