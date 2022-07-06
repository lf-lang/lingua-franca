package org.lflang.generator.uclid;

import org.lflang.dsl.MTLParser;
import org.lflang.dsl.MTLParserBaseListener;

public class MTLTranspiler extends MTLParserBaseListener {

    @Override
    public void enterMtl(MTLParser.MtlContext ctx) {
        System.out.println("Formula: " + ctx.getText());
    }

    @Override
    public void enterEquivalence(MTLParser.EquivalenceContext ctx) {
        System.out.println("Equivalence: " + ctx.getText());
    }
    
    @Override
    public void enterImplication(MTLParser.ImplicationContext ctx) {
        System.out.println("Implication: " + ctx.getText());
    }

    @Override
    public void enterDisjunction(MTLParser.DisjunctionContext ctx) {
        System.out.println("Disjunction: " + ctx.getText());
    }

    @Override
    public void enterConjunction(MTLParser.ConjunctionContext ctx) {
        System.out.println("Conjunction: " + ctx.getText());
    }

    @Override
    public void enterBinaryOp(MTLParser.BinaryOpContext ctx) {
        System.out.println("BinaryOp: " + ctx.getText());
    }

    @Override
    public void enterNoUnaryOp(MTLParser.NoUnaryOpContext ctx) {
        System.out.println("NoUnaryOp: " + ctx.getText());
        System.out.println("formula within NoUnaryOp: " + ctx.formula.getText());
    }

    @Override
    public void enterNegation(MTLParser.NegationContext ctx) {
        System.out.println("Negation: " + ctx.getText());
    }

    @Override
    public void enterNext(MTLParser.NextContext ctx) {
        System.out.println("Next: " + ctx.getText());
    }

    @Override
    public void enterGlobally(MTLParser.GloballyContext ctx) {
        System.out.println("Globally: " + ctx.getText());
    }

    @Override
    public void enterFinally(MTLParser.FinallyContext ctx) {
        System.out.println("Finally: " + ctx.getText());
    }

    @Override
    public void enterPrimary(MTLParser.PrimaryContext ctx) {
        System.out.println("Primary: " + ctx.getText());
    }

    @Override
    public void enterAtomicProp(MTLParser.AtomicPropContext ctx) {
        System.out.println("AtomicProp: " + ctx.getText());
    }

    @Override
    public void enterRange(MTLParser.RangeContext ctx) {
        System.out.println("Range: " + ctx.getText());
    }
}