/*************
Copyright (c) 2021, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/** 
 * Transpiler from an MTL specification to a Uclid axiom.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.generator.uclid;

import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.dsl.MTLParser;
import org.lflang.dsl.MTLParserBaseVisitor;
import org.lflang.generator.CodeBuilder;

public class MTLVisitor extends MTLParserBaseVisitor<String> {

    ////////////////////////////////////////////
    //// Protected fields

    /** The main place to put generated code. */
    protected CodeBuilder code  = new CodeBuilder();

    /** Tactic to be used to prove the property. */
    protected String tactic;

    // Constructor
    public MTLVisitor(String tactic) {
        this.tactic = tactic;
    }

    ////////////////////////////////////////////
    //// Public methods
    public String visitMtl(MTLParser.MtlContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        return visitEquivalence(ctx.equivalence(),
            prefixIdx, QFIdx, prevPrefixIdx, horizon);
    }

    public String visitEquivalence(MTLParser.EquivalenceContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        if (ctx.right == null) {
            return visitImplication(ctx.left,
            prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }
        return "(" + visitImplication(ctx.left,
            prefixIdx, QFIdx, prevPrefixIdx, horizon) 
            + ")"
            + " <==> " 
            + "(" + visitImplication(ctx.right,
            prefixIdx, QFIdx, prevPrefixIdx, horizon)
            + ")";
    }

    public String visitImplication(MTLParser.ImplicationContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        if (ctx.right == null) {
            return visitDisjunction(ctx.left,
            prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }
        return "(" + visitDisjunction(ctx.left,
            prefixIdx, QFIdx, prevPrefixIdx, horizon) 
            + ")"
            + " ==> " 
            + "(" + visitDisjunction(ctx.right,
            prefixIdx, QFIdx, prevPrefixIdx, horizon)
            + ")";
    }

    public String visitDisjunction(MTLParser.DisjunctionContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitConjunction(ctx.terms.get(i),
                prefixIdx, QFIdx, prevPrefixIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "||");
        }
        return str;
    }

    public String visitConjunction(MTLParser.ConjunctionContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitUntil((MTLParser.UntilContext)ctx.terms.get(i),
                    prefixIdx, QFIdx, prevPrefixIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "&&");
        }
        return str;
    }

    // A custom dispatch function
    public String _visitUnaryOp(MTLParser.UnaryOpContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        // FIXME: Is there a more "antlr" way to do dispatch here?
        if (ctx instanceof MTLParser.NoUnaryOpContext) {
            return visitNoUnaryOp((MTLParser.NoUnaryOpContext)ctx, 
                prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }
        if (ctx instanceof MTLParser.NegationContext) {
            return visitNegation((MTLParser.NegationContext)ctx,
                prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }
        if (ctx instanceof MTLParser.NextContext) {
            return visitNext((MTLParser.NextContext)ctx,
                prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }
        if (ctx instanceof MTLParser.GloballyContext) {
            return visitGlobally((MTLParser.GloballyContext)ctx,
                prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }
        if (ctx instanceof MTLParser.FinallyContext) {
            return visitFinally((MTLParser.FinallyContext)ctx,
                prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }

        // FIXME: Throw an exception.
        return "";
    }

    public String visitUntil(MTLParser.UntilContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        // If right is null, continue recursion.
        if (ctx.right == null) {
            return _visitUnaryOp(ctx.left,
                prefixIdx, QFIdx, prevPrefixIdx, horizon);
        }

        String end;
        if (this.tactic.equals("induction")) {
            end = "(" + prefixIdx + " + CT)";
        } else {
            end = "END";
        }

        // Otherwise, create the Until formula.
        // Check if the time interval is a range or a singleton.
        long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        long currentHorizon = horizon + upperBoundNanoSec;
        String timePredicate = generateTimePredicate(ctx.timeInterval, lowerBoundNanoSec,
                                                    upperBoundNanoSec, "j" + QFIdx, prefixIdx);

        return "finite_exists " + "(" + "j" + QFIdx + " : integer) in indices :: "
                + "j" + QFIdx + " >= " + prefixIdx + " && " + "j" + QFIdx + " <= " + end
                + " && " + "rxn" + "(" + "j" + QFIdx + ")" + " != " + "NULL"
                + " && " + "(" + _visitUnaryOp(ctx.right, ("j"+QFIdx), QFIdx+1, prefixIdx, currentHorizon) + ")"
                + " && " + "(" + "\n" 
                + "// Time Predicate\n"
                + timePredicate + "\n"
                + ")" + " && " + "(" + "finite_forall " + "(" + "i" + QFIdx + " : integer) in indices :: "
                + "(" + "i" + QFIdx + " >= " + prefixIdx + " && " + "i" + QFIdx + " < " + "j" + QFIdx + ")"
                + " ==> " + "(" + _visitUnaryOp(ctx.left, ("i"+QFIdx), QFIdx+1, ("j"+QFIdx), currentHorizon) + ")" + ")";
    }

    public String visitNoUnaryOp(MTLParser.NoUnaryOpContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        return visitPrimary(ctx.formula, prefixIdx, QFIdx, prevPrefixIdx, horizon);
    }

    public String visitNegation(MTLParser.NegationContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        return "!(" + visitPrimary(ctx.formula, prefixIdx, QFIdx, prevPrefixIdx, horizon) + ")";
    }

    public String visitNext(MTLParser.NextContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        return visitPrimary(ctx.formula, ("(" + prefixIdx + "+1)"), QFIdx, prevPrefixIdx, horizon);
    }

    public String visitGlobally(MTLParser.GloballyContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        String end;
        if (this.tactic.equals("induction")) {
            end = "(" + prefixIdx + " + CT)";
        } else {
            end = "END";
        }
        
        long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        long currentHorizon = horizon + upperBoundNanoSec;
        String timePredicate = generateTimePredicate(ctx.timeInterval, lowerBoundNanoSec,
                                                    upperBoundNanoSec, "j" + QFIdx, prefixIdx);        
        return "!(" + "finite_exists " + "(" + "j" + QFIdx + " : integer) in indices :: "
                + "j" + QFIdx + " >= " + prefixIdx + " && " + "j" + QFIdx + " <= " + end
                + " && " + "rxn" + "(" + "j" + QFIdx + ")" + " != " + "NULL"
                + " && " + "!" + "(" + visitPrimary(ctx.formula, ("j"+QFIdx), QFIdx+1, prefixIdx, currentHorizon) + ")"
                + " && " + "(" + "\n" 
                + "// Time Predicate\n"
                + timePredicate + "\n"
                + "))";
    }

    public String visitFinally(MTLParser.FinallyContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        String end;
        if (this.tactic.equals("induction")) {
            end = "(" + prefixIdx + " + CT)";
        } else {
            end = "END";
        }

        long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        long currentHorizon = horizon + upperBoundNanoSec;
        String timePredicate = generateTimePredicate(ctx.timeInterval, lowerBoundNanoSec,
                                                    upperBoundNanoSec, "j" + QFIdx, prefixIdx);        
        return "finite_exists " + "(" + "j" + QFIdx + " : integer) in indices :: "
                + "j" + QFIdx + " >= " + prefixIdx + " && " + "j" + QFIdx + " <= " + end
                + " && " + "rxn" + "(" + "j" + QFIdx + ")" + " != " + "NULL"
                + " && " + "(" + visitPrimary(ctx.formula, ("j"+QFIdx), QFIdx+1, prefixIdx, currentHorizon) + ")"
                + " && " + "(" + "\n" 
                + "// Time Predicate\n"
                + timePredicate + "\n"
                + ")";
    }

    public String visitPrimary(MTLParser.PrimaryContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        if (ctx.atom != null)
            return visitAtomicProp(ctx.atom, prefixIdx, QFIdx, prevPrefixIdx, horizon);
        else if (ctx.id != null) {
            // Check if the ID is a reaction.
            // FIXME: Not robust.
            if (ctx.id.getText().contains("_reaction_")) {
                return "rxn(" + prefixIdx + ") == " + ctx.id.getText();
            } else if (ctx.id.getText().contains("_is_present")) { 
                return ctx.id.getText() + "(" + "t(" + prefixIdx + ")" + ")";
            } else {
                return ctx.id.getText() + "(" + "s(" + prefixIdx + ")" + ")";
            }
        }
        else
            return visitMtl(ctx.formula, prefixIdx, QFIdx, prevPrefixIdx, horizon);
    }

    public String visitAtomicProp(MTLParser.AtomicPropContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        if (ctx.primitive != null)
            return ctx.primitive.getText();
        else
            return visitExpr(ctx.left, prefixIdx, QFIdx, prevPrefixIdx, horizon)
                    + " " + ctx.op.getText() + " "
                    + visitExpr(ctx.right, prefixIdx, QFIdx, prevPrefixIdx, horizon);
    }

    public String visitExpr(MTLParser.ExprContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {

        if (ctx.ID() != null)
            return ctx.ID().getText() + "(" + "s(" + prefixIdx + ")" + ")";
        else if (ctx.INTEGER() != null)
            return ctx.INTEGER().getText();
        else
            return visitSum(ctx.sum(), prefixIdx, QFIdx, prevPrefixIdx, horizon);
    }

    public String visitSum(MTLParser.SumContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitDifference(ctx.terms.get(i),
                prefixIdx, QFIdx, prevPrefixIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "+");
        }
        return str;
    }

    public String visitDifference(MTLParser.DifferenceContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitProduct(ctx.terms.get(i),
                prefixIdx, QFIdx, prevPrefixIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "-");
        }
        return str;
    }

    public String visitProduct(MTLParser.ProductContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitQuotient(ctx.terms.get(i),
                prefixIdx, QFIdx, prevPrefixIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "*");
        }
        return str;
    }

    public String visitQuotient(MTLParser.QuotientContext ctx,
        String prefixIdx, int QFIdx, String prevPrefixIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitExpr(ctx.terms.get(i),
                prefixIdx, QFIdx, prevPrefixIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "/");
        }
        return str;
    }

    ///////////////////////////////////////
    //// Private methods

    /**
     * Return a time value in nanoseconds from an IntervalContext.
     * 
     * @param ctx
     * @param getUpper
     * @return
     */
    private long getNanoSecFromIntervalContext(MTLParser.IntervalContext ctx, boolean getUpper) {
        // If we have a singleton, the return value is the same regardless of getUpper.
        if (ctx instanceof MTLParser.SingletonContext) {
            MTLParser.SingletonContext singletonCtx = (MTLParser.SingletonContext)ctx;
            String timeInstantValue = singletonCtx.instant.value.getText();
            String timeInstantUnit = "";
            long timeInstantNanoSec = 0;
            if (!timeInstantValue.equals("0")) {
                timeInstantUnit = singletonCtx.instant.unit.getText();
                TimeValue timeValue = new TimeValue(
                    Integer.valueOf(timeInstantValue), 
                    TimeUnit.fromName(timeInstantUnit));
                timeInstantNanoSec = timeValue.toNanoSeconds();
            }
            return timeInstantNanoSec;
        }
        
        MTLParser.RangeContext rangeCtx = (MTLParser.RangeContext)ctx;
        if (!getUpper) {
            String lowerBoundTimeValue = rangeCtx.lowerbound.value.getText();
            String lowerBoundTimeUnit = "";
            long lowerBoundNanoSec = 0;
            if (!lowerBoundTimeValue.equals("0")) {
                lowerBoundTimeUnit = rangeCtx.lowerbound.unit.getText();
                TimeValue lowerTimeValue = new TimeValue(
                    Integer.valueOf(lowerBoundTimeValue), 
                    TimeUnit.fromName(lowerBoundTimeUnit));
                lowerBoundNanoSec = lowerTimeValue.toNanoSeconds();
            }
            return lowerBoundNanoSec;
        } else {
            String upperBoundTimeValue = rangeCtx.upperbound.value.getText();
            String upperBoundTimeUnit = "";
            long upperBoundNanoSec = 0;
            if (!upperBoundTimeValue.equals("0")) {
                upperBoundTimeUnit = rangeCtx.upperbound.unit.getText();
                TimeValue upperTimeValue = new TimeValue(
                    Integer.valueOf(upperBoundTimeValue), 
                    TimeUnit.fromName(upperBoundTimeUnit));
                upperBoundNanoSec = upperTimeValue.toNanoSeconds();
            }
            return upperBoundNanoSec;
        }
    }

    /**
     * Generate a time predicate from a range.
     * 
     * @param ctx
     * @param lowerBoundNanoSec
     * @param upperBoundNanoSec
     * @return
     */
    private String generateTimePredicate(MTLParser.IntervalContext ctx,
        long lowerBoundNanoSec, long upperBoundNanoSec,
        String prefixIdx, String prevPrefixIdx) {
        String timePredicate = "";

        if (ctx instanceof MTLParser.SingletonContext) {
            MTLParser.SingletonContext singletonCtx = (MTLParser.SingletonContext)ctx;
            timePredicate += "tag_same(g(" + prefixIdx + "), " + "tag_schedule(g(" 
                                + prevPrefixIdx + "), nsec(" + upperBoundNanoSec + ")))";
        } else {
            MTLParser.RangeContext rangeCtx = (MTLParser.RangeContext)ctx;
            timePredicate += "(";
            if (rangeCtx.LBRACKET() != null) {
                // FIXME: Check if this can be replaced by a !tag_earlier.
                timePredicate += "tag_later(g(" + prefixIdx + "), "
                    + "tag_schedule(g(" + prevPrefixIdx + "), nsec(" + lowerBoundNanoSec + ")))"
                    + " || " + "tag_same(g(" + prefixIdx + "), "
                    + "tag_schedule(g(" + prevPrefixIdx + "), nsec(" + lowerBoundNanoSec + ")))";
            } else {
                timePredicate += "tag_later(g(" + prefixIdx + "), "
                    + "tag_schedule(g(" + prevPrefixIdx + "), nsec(" + lowerBoundNanoSec + ")))";
            }
            timePredicate += ") && (";
            if (rangeCtx.RBRACKET() != null) {
                timePredicate += "tag_earlier(g(" + prefixIdx + "), "
                    + "tag_schedule(g(" + prevPrefixIdx + "), nsec(" + upperBoundNanoSec + ")))"
                    + " || " + "tag_same(g(" + prefixIdx + "), "
                    + "tag_schedule(g(" + prevPrefixIdx + "), nsec(" + upperBoundNanoSec + ")))";
            } else {
                timePredicate += "tag_earlier(g(" + prefixIdx + "), "
                    + "tag_schedule(g(" + prevPrefixIdx + "), nsec(" + upperBoundNanoSec + ")))";
            }
            timePredicate += ")";
        }

        return timePredicate;
    }
}
