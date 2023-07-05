package org.lflang.analyses.c;

import java.util.List;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.misc.Interval;

public class AstUtils {

  public static CAst.AstNode takeConjunction(List<CAst.AstNode> conditions) {
    if (conditions.size() == 0) {
      return new CAst.LiteralNode("true");
    } else if (conditions.size() == 1) {
      return conditions.get(0);
    } else {
      // Take the conjunction of all the conditions.
      CAst.LogicalAndNode top = new CAst.LogicalAndNode();
      CAst.LogicalAndNode cur = top;
      for (int i = 0; i < conditions.size() - 1; i++) {
        cur.left = conditions.get(i);
        if (i == conditions.size() - 2) {
          cur.right = conditions.get(i + 1);
        } else {
          cur.right = new CAst.LogicalAndNode();
          cur = (CAst.LogicalAndNode) cur.right;
        }
      }
      return top;
    }
  }

  public static CAst.AstNode takeDisjunction(List<CAst.AstNode> conditions) {
    if (conditions.size() == 0) {
      return new CAst.LiteralNode("true");
    } else if (conditions.size() == 1) {
      return conditions.get(0);
    } else {
      // Take the disjunction of all the conditions.
      CAst.LogicalOrNode top = new CAst.LogicalOrNode();
      CAst.LogicalOrNode cur = top;
      for (int i = 0; i < conditions.size() - 1; i++) {
        cur.left = conditions.get(i);
        if (i == conditions.size() - 2) {
          cur.right = conditions.get(i + 1);
        } else {
          cur.right = new CAst.LogicalOrNode();
          cur = (CAst.LogicalOrNode) cur.right;
        }
      }
      return top;
    }
  }

  /**
   * A handy function for debugging ASTs. It prints the stack trace of the visitor functions and
   * shows the text matched by the ANTLR rules.
   */
  public static void printStackTraceAndMatchedText(ParserRuleContext ctx) {
    System.out.println("========== AST DEBUG ==========");

    // Print matched text
    int a = ctx.start.getStartIndex();
    int b = ctx.stop.getStopIndex();
    Interval interval = new Interval(a, b);
    String matchedText = ctx.start.getInputStream().getText(interval);
    System.out.println("Matched text: " + matchedText);

    // Print stack trace
    StackTraceElement[] cause = Thread.currentThread().getStackTrace();
    System.out.print("Stack trace: ");
    for (int i = 0; i < cause.length; i++) {
      System.out.print(cause[i].getMethodName());
      if (i != cause.length - 1) System.out.print(", ");
    }
    System.out.println(".");

    System.out.println("===============================");
  }
}
