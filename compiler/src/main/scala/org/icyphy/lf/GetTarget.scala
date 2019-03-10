package org.icyphy.lf

class GetTarget() extends LinguaFrancaBaseListener {

  var target = "UNDEFINED"

  override def exitTarget(ctx: LinguaFrancaParser.TargetContext) {
    target = ctx.ID().getText
  }
}
