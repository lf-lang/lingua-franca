package org.lflang.generator.uclid.ast;

import java.util.List;

public class CAstUtils {
    
    public static CAst.AstNode takeConjunction(List<CAst.AstNode> conditions) {
        if (conditions.size() == 0) {
            return new CAst.LiteralNode("true");
        } else if (conditions.size() == 1) {
            return conditions.get(0);
        } else {
            // Take the conjunction of all the conditions.
            CAst.LogicalAndNode top = new CAst.LogicalAndNode();
            CAst.LogicalAndNode cur = top;
            for (int i = 0; i < conditions.size()-1; i++) {
                cur.left = conditions.get(i);
                if (i == conditions.size()-2) {
                    cur.right = conditions.get(i+1);
                } else {
                    cur.right = new CAst.LogicalAndNode();
                    cur =(CAst.LogicalAndNode)cur.right;
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
            // Take the conjunction of all the conditions.
            CAst.LogicalOrNode top = new CAst.LogicalOrNode();
            CAst.LogicalOrNode cur = top;
            for (int i = 0; i < conditions.size()-1; i++) {
                cur.left = conditions.get(i);
                if (i == conditions.size()-2) {
                    cur.right = conditions.get(i+1);
                } else {
                    cur.right = new CAst.LogicalOrNode();
                    cur =(CAst.LogicalOrNode)cur.right;
                }
            }
            return top;
        }
    }
}