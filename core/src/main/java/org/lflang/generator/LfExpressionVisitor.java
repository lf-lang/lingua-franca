/*
 * Copyright (c) 2023, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator;

import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BracketListExpression;
import org.lflang.lf.Code;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Expression;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Literal;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Time;

/**
 * A visitor for expressions in LF.
 *
 * @author Clément Fournier &lt;clement.fournier@tu-dresden.de&gt;
 */
public interface LfExpressionVisitor<P, R> {

  R visitLiteral(Literal expr, P param);

  R visitBracedListExpr(BracedListExpression expr, P param);

  R visitBracketListExpr(BracketListExpression expr, P param);

  R visitTimeLiteral(Time expr, P param);

  R visitCodeExpr(CodeExpr expr, P param);

  R visitParameterRef(ParameterReference expr, P param);

  /**
   * Dispatch the visitor on the given expression type.
   *
   * @param e An expression that will be visited
   * @param arg Argument for the visitor
   * @param visitor Visitor
   * @param <P> Type of parameter expected by the visitor
   * @param <R> Return type of the visitor
   * @return The return value of the visitor
   */
  static <P, R> R dispatch(
      Expression e, P arg, LfExpressionVisitor<? super P, ? extends R> visitor) {
    if (e instanceof Literal) {
      return visitor.visitLiteral((Literal) e, arg);
    } else if (e instanceof BracedListExpression) {
      return visitor.visitBracedListExpr((BracedListExpression) e, arg);
    } else if (e instanceof BracketListExpression) {
      return visitor.visitBracketListExpr((BracketListExpression) e, arg);
    } else if (e instanceof Time) {
      return visitor.visitTimeLiteral((Time) e, arg);
    } else if (e instanceof CodeExpr) {
      return visitor.visitCodeExpr((CodeExpr) e, arg);
    } else if (e instanceof ParameterReference) {
      return visitor.visitParameterRef((ParameterReference) e, arg);
    }

    throw new IllegalArgumentException("Expression of type " + e.getClass() + " not handled");
  }

  /** Base visitor class where methods are defaulted to a common one. */
  abstract class DefaultLfVisitor<P, R> implements LfExpressionVisitor<P, R> {

    abstract R visitExpression(Expression expr, P param);

    @Override
    public R visitLiteral(Literal expr, P param) {
      return visitExpression(expr, param);
    }

    @Override
    public R visitBracedListExpr(BracedListExpression expr, P param) {
      return visitExpression(expr, param);
    }

    @Override
    public R visitTimeLiteral(Time expr, P param) {
      return visitExpression(expr, param);
    }

    @Override
    public R visitCodeExpr(CodeExpr expr, P param) {
      return visitExpression(expr, param);
    }

    @Override
    public R visitParameterRef(ParameterReference expr, P param) {
      return visitExpression(expr, param);
    }

    @Override
    public R visitBracketListExpr(BracketListExpression expr, P param) {
      return visitExpression(expr, param);
    }
  }

  /**
   * A visitor that deep copies the expression. Can be extended to replace certain expressions
   * during the copy.
   *
   * @param <P> Parameter type
   */
  class LfExpressionDeepCopyVisitor<P> implements LfExpressionVisitor<P, Expression> {

    @Override
    public Expression visitLiteral(Literal expr, P param) {
      Literal clone = LfFactory.eINSTANCE.createLiteral();
      clone.setLiteral(expr.getLiteral());
      return clone;
    }

    @Override
    public Expression visitBracedListExpr(BracedListExpression expr, P param) {
      BracedListExpression clone = LfFactory.eINSTANCE.createBracedListExpression();
      for (Expression item : expr.getItems()) {
        clone.getItems().add(dispatch(item, param, this));
      }
      return clone;
    }

    @Override
    public Expression visitTimeLiteral(Time expr, P param) {
      Time clone = LfFactory.eINSTANCE.createTime();
      clone.setUnit(expr.getUnit());
      clone.setInterval(expr.getInterval());
      return clone;
    }

    @Override
    public Expression visitParameterRef(ParameterReference expr, P param) {
      ParameterReference clone = LfFactory.eINSTANCE.createParameterReference();
      clone.setParameter(expr.getParameter());
      return clone;
    }

    @Override
    public Expression visitBracketListExpr(BracketListExpression expr, P param) {
      BracketListExpression clone = LfFactory.eINSTANCE.createBracketListExpression();
      for (Expression item : expr.getItems()) {
        clone.getItems().add(dispatch(item, param, this));
      }
      return clone;
    }

    @Override
    public Expression visitCodeExpr(CodeExpr expr, P param) {
      CodeExpr codeExpr = LfFactory.eINSTANCE.createCodeExpr();
      Code code = LfFactory.eINSTANCE.createCode();
      code.setBody(expr.getCode().getBody());
      codeExpr.setCode(code);
      return codeExpr;
    }
  }
}
