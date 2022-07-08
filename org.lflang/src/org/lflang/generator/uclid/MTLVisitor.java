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
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        return visitEquivalence(ctx.equivalence(),
            QFPrefix, QFIdx, prevQFIdx, horizon);
    }

    public String visitEquivalence(MTLParser.EquivalenceContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        if (ctx.right == null) {
            return visitImplication(ctx.left,
            QFPrefix, QFIdx, prevQFIdx, horizon);
        }
        return "(" + visitImplication(ctx.left,
            QFPrefix, QFIdx, prevQFIdx, horizon) 
            + ")"
            + " <==> " 
            + "(" + visitImplication(ctx.right,
            QFPrefix, QFIdx, prevQFIdx, horizon)
            + ")";
    }

    public String visitImplication(MTLParser.ImplicationContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {
        
        if (ctx.right == null) {
            return visitDisjunction(ctx.left,
            QFPrefix, QFIdx, prevQFIdx, horizon);
        }
        return "(" + visitDisjunction(ctx.left,
            QFPrefix, QFIdx, prevQFIdx, horizon) 
            + ")"
            + " ==> " 
            + "(" + visitDisjunction(ctx.right,
            QFPrefix, QFIdx, prevQFIdx, horizon)
            + ")";
    }

    public String visitDisjunction(MTLParser.DisjunctionContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitConjunction(ctx.terms.get(i),
                QFPrefix, QFIdx, prevQFIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "||");
        }
        return str;
    }

    public String visitConjunction(MTLParser.ConjunctionContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {
        
        String str = "";
        for (int i = 0; i < ctx.terms.size(); i++) {
            str += "("
                + visitUntil((MTLParser.UntilContext)ctx.terms.get(i),
                    QFPrefix, QFIdx, prevQFIdx, horizon)
                + ")"
                + (i == ctx.terms.size()-1 ? "" : "&&");
        }
        return str;
    }

    // A custom dispatch function
    public String _visitUnaryOp(MTLParser.UnaryOpContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        // FIXME: Is there a more "antlr" way to do dispatch here?
        if (ctx instanceof MTLParser.NoUnaryOpContext) {
            return visitNoUnaryOp((MTLParser.NoUnaryOpContext)ctx, 
                QFPrefix, QFIdx, prevQFIdx, horizon);
        }
        if (ctx instanceof MTLParser.NegationContext) {
            return visitNegation((MTLParser.NegationContext)ctx,
                QFPrefix, QFIdx, prevQFIdx, horizon);
        }
        if (ctx instanceof MTLParser.NextContext) {
            return visitNext((MTLParser.NextContext)ctx,
                QFPrefix, QFIdx, prevQFIdx, horizon);
        }
        if (ctx instanceof MTLParser.GloballyContext) {
            return visitGlobally((MTLParser.GloballyContext)ctx,
                QFPrefix, QFIdx, prevQFIdx, horizon);
        }
        if (ctx instanceof MTLParser.FinallyContext) {
            return visitFinally((MTLParser.FinallyContext)ctx,
                QFPrefix, QFIdx, prevQFIdx, horizon);
        }

        // FIXME: Throw an exception.
        return "";
    }

    public String visitUntil(MTLParser.UntilContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        // If right is null, continue recursion.
        if (ctx.right == null) {
            return _visitUnaryOp(ctx.left,
                QFPrefix, QFIdx, prevQFIdx, horizon);
        }

        String end;
        if (this.tactic.equals("induction")) {
            end = "(" + QFPrefix + " + N)";
        } else {
            end = "END";
        }

        // Otherwise, create the Until formula.
        // Check if the time interval is a range or a singleton.
        long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        long currentHorizon = horizon + upperBoundNanoSec;
        String timePredicate = generateTimePredicate(ctx.timeInterval, lowerBoundNanoSec,
                                                    upperBoundNanoSec, QFPrefix, prevQFIdx);
        // if (ctx.timeInterval instanceof MTLParser.SingletonContext) {
        //     long timeInstantNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        //     currentHorizon = horizon + timeInstantNanoSec;
        //     timePredicate = generateTimePredicate(timeInstantNanoSec, QFPrefix, prevQFIdx);
        // } 
        // else {
        //     long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        //     long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        //     currentHorizon = horizon + upperBoundNanoSec;
        //     timePredicate = generateTimePredicate((MTLParser.RangeContext)ctx.timeInterval,
        //         lowerBoundNanoSec, upperBoundNanoSec, QFPrefix, prevQFIdx);           
        // }
        return "finite_exists " + "(" + "j" + QFIdx + " : integer) in indices :: "
                + "j" + QFIdx + " >= " + QFPrefix + " && " + "j" + QFIdx + " <= " + end
                + " && " + "(" + _visitUnaryOp(ctx.right, ("j"+QFIdx), QFIdx+1, QFPrefix, currentHorizon) + ")"
                + " && " + "(" + "\n" 
                + "// Time Predicate\n"
                + timePredicate + "\n"
                + ")" + " && " + "(" + "finite_forall " + "(" + "i" + QFIdx + " : integer) in indices :: "
                + "(" + "i" + QFIdx + " >= " + QFPrefix + " && " + "i" + QFIdx + " < " + "j" + QFIdx + ")"
                + " ==> " + "(" + _visitUnaryOp(ctx.left, ("i"+QFIdx), QFIdx+1, ("j"+QFIdx), currentHorizon) + ")" + ")";
    }

    public String visitNoUnaryOp(MTLParser.NoUnaryOpContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        return visitPrimary(ctx.formula, QFPrefix, QFIdx, prevQFIdx, horizon);
    }

    public String visitNegation(MTLParser.NegationContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        return "!(" + visitPrimary(ctx.formula, QFPrefix, QFIdx, prevQFIdx, horizon) + ")";
    }

    public String visitNext(MTLParser.NextContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        return visitPrimary(ctx.formula, ("(" + QFPrefix + "+1)"), QFIdx, prevQFIdx, horizon);
    }

    public String visitGlobally(MTLParser.GloballyContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        String end;
        if (this.tactic.equals("induction")) {
            end = "(" + QFPrefix + " + N)";
        } else {
            end = "END";
        }
        
        long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        long currentHorizon = horizon + upperBoundNanoSec;
        String timePredicate = generateTimePredicate(ctx.timeInterval, lowerBoundNanoSec,
                                                    upperBoundNanoSec, QFPrefix, prevQFIdx);        
        return "!(" + "finite_exists " + "(" + "j" + QFIdx + " : integer) in indices :: "
                + "j" + QFIdx + " >= " + QFPrefix + " && " + "j" + QFIdx + " <= " + end
                + " && " + "!" + "(" + visitPrimary(ctx.formula, ("j"+QFIdx), QFIdx+1, QFPrefix, currentHorizon) + ")"
                + " && " + "(" + "\n" 
                + "// Time Predicate\n"
                + timePredicate + "\n"
                + "))";
    }

    public String visitFinally(MTLParser.FinallyContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        String end;
        if (this.tactic.equals("induction")) {
            end = "(" + QFPrefix + " + N)";
        } else {
            end = "END";
        }

        long lowerBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, false);
        long upperBoundNanoSec = getNanoSecFromIntervalContext(ctx.timeInterval, true);
        long currentHorizon = horizon + upperBoundNanoSec;
        String timePredicate = generateTimePredicate(ctx.timeInterval, lowerBoundNanoSec,
                                                    upperBoundNanoSec, QFPrefix, prevQFIdx);        
        return "finite_exists " + "(" + "j" + QFIdx + " : integer) in indices :: "
                + "j" + QFIdx + " >= " + QFPrefix + " && " + "j" + QFIdx + " <= " + end
                + " && " + "(" + visitPrimary(ctx.formula, ("j"+QFIdx), QFIdx+1, QFPrefix, currentHorizon) + ")"
                + " && " + "(" + "\n" 
                + "// Time Predicate\n"
                + timePredicate + "\n"
                + ")";
    }

    public String visitPrimary(MTLParser.PrimaryContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        if (ctx.atom != null)
            return visitAtomicProp(ctx.atom, QFPrefix, QFIdx, prevQFIdx, horizon);
        else if (ctx.id != null)
            return ctx.id.getText();
        else
            return visitMtl(ctx.formula, QFPrefix, QFIdx, prevQFIdx, horizon);
    }

    public String visitAtomicProp(MTLParser.AtomicPropContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        if (ctx.primitive != null)
            return ctx.primitive.getText();
        else
            return visitExpr(ctx.left, QFPrefix, QFIdx, prevQFIdx, horizon)
                    + " " + ctx.op.getText() + " "
                    + visitExpr(ctx.right, QFPrefix, QFIdx, prevQFIdx, horizon);
    }

    public String visitExpr(MTLParser.ExprContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {

        if (ctx.ID() != null)
            return ctx.ID().getText();
        else if (ctx.INTEGER() != null)
            return ctx.INTEGER().getText();
        else
            return visitSum(ctx.sum(), QFPrefix, QFIdx, prevQFIdx, horizon);
    }

    public String visitSum(MTLParser.SumContext ctx,
        String QFPrefix, int QFIdx, String prevQFIdx, long horizon) {
        return "";
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
        String QFPrefix, String prevQFIdx) {
        String timePredicate = "";

        if (ctx instanceof MTLParser.SingletonContext) {
            MTLParser.SingletonContext singletonCtx = (MTLParser.SingletonContext)ctx;
            timePredicate += "tag_same(g(" + QFPrefix + "), " + "tag_schedule(g(" 
                                + prevQFIdx + "), nsec(" + upperBoundNanoSec + ")))";
        } else {
            MTLParser.RangeContext rangeCtx = (MTLParser.RangeContext)ctx;
            timePredicate += "(";
            if (rangeCtx.LBRACKET() != null) {
                // FIXME: Check if this can be replaced by a !tag_earlier.
                timePredicate += "tag_later(g(" + QFPrefix + "), "
                    + "tag_schedule(g(" + prevQFIdx + "), nsec(" + lowerBoundNanoSec + ")))"
                    + " || " + "tag_same(g(" + QFPrefix + "), "
                    + "tag_schedule(g(" + prevQFIdx + "), nsec(" + lowerBoundNanoSec + ")))";
            } else {
                timePredicate += "tag_later(g(" + QFPrefix + "), "
                    + "tag_schedule(g(" + prevQFIdx + "), nsec(" + lowerBoundNanoSec + ")))";
            }
            timePredicate += ") && (";
            if (rangeCtx.RBRACKET() != null) {
                timePredicate += "tag_earlier(g(" + QFPrefix + "), "
                    + "tag_schedule(g(" + prevQFIdx + "), nsec(" + upperBoundNanoSec + ")))"
                    + " || " + "tag_same(g(" + QFPrefix + "), "
                    + "tag_schedule(g(" + prevQFIdx + "), nsec(" + upperBoundNanoSec + ")))";
            } else {
                timePredicate += "tag_earlier(g(" + QFPrefix + "), "
                    + "tag_schedule(g(" + prevQFIdx + "), nsec(" + upperBoundNanoSec + ")))";
            }
            timePredicate += ")";
        }

        return timePredicate;
    }
}
