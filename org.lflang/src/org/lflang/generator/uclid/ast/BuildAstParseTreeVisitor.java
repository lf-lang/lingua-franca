package org.lflang.generator.uclid.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.lflang.dsl.CBaseVisitor;
import org.lflang.dsl.CParser.*;

public class BuildAstParseTreeVisitor extends CBaseVisitor<CAst.AstNode> {

    @Override
    public CAst.AstNode visitBlockItemList(BlockItemListContext ctx) {
        CAst.StatementSequenceNode stmtSeq = new CAst.StatementSequenceNode();
        
        // Populate the children.
        for (BlockItemContext blockItem : ctx.blockItem()) {
            // System.out.println(blockItem);
            stmtSeq.children.add(visit(blockItem));
        }

        System.out.println(stmtSeq.children);

        return stmtSeq;
    }

    @Override
    public CAst.AstNode visitBlockItem(BlockItemContext ctx) {
        if (ctx.statement() != null)
            return visit(ctx.statement());
        else
            return visit(ctx.declaration());
    }

    @Override
    public CAst.AstNode visitDeclaration(DeclarationContext ctx) {
        if (ctx.declarationSpecifiers() != null 
            && ctx.initDeclaratorList() != null) {
            //// Extract type from declarationSpecifiers.
            List<DeclarationSpecifierContext> declSpecList 
                = ctx.declarationSpecifiers().declarationSpecifier();
            
            // Cannot handle more than 1 specifiers, e.g. static const int.
            // We can augment the analytical capability later.
            if (declSpecList.size() > 1) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "the analyzer cannot handle more than 1 specifiers,",
                    "e.g. static const int.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }
            
            // Check if the declaration specifier is a type specifier: e.g. int or long.
            DeclarationSpecifierContext declSpec = declSpecList.get(0);
            if (declSpec.typeSpecifier() == null) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "only type specifiers are supported.",
                    "e.g. \"static const int\" is not analyzable.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }
            
            // Check if the type specifier is what we currently support.
            // Right now we only support int, long, & double.
            CAst.VariableNode.Type type;
            ArrayList<String> supportedTypes = new ArrayList<String>(
                Arrays.asList(
                    "int",
                    "long",
                    "double",
                    "_Bool"
                )
            );
            if (declSpec.typeSpecifier().Int() != null
                || declSpec.typeSpecifier().Long() != null
                || declSpec.typeSpecifier().Double() != null) 
                type = CAst.VariableNode.Type.INT;
            else if (declSpec.typeSpecifier().Bool() != null)
                type = CAst.VariableNode.Type.BOOLEAN;
            // Mark the declaration unanalyzable if the type is unsupported.
            else {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "unsupported type detected at " + declSpec.typeSpecifier(),
                    "Only " + supportedTypes + " are supported.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }

            //// Extract variable name and value from initDeclaratorList.
            List<InitDeclaratorContext> initDeclList
                = ctx.initDeclaratorList().initDeclarator();
            
            // Cannot handle more than 1 initDeclarator: e.g. x = 1, y = 2;
            // We can augment the analytical capability later.
            if (initDeclList.size() > 1) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "more than 1 declarators are detected on a single line,",
                    "e.g. \"int x = 1, y = 2;\" is not yet analyzable.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }

            // Get the variable name from the declarator.
            DeclaratorContext decl = initDeclList.get(0).declarator();
            if (decl.pointer() != null) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "pointers are currently not supported,",
                    "e.g. \"int *x;\" is not yet analyzable.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }
            if (decl.gccDeclaratorExtension().size() > 0) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "GCC declarator extensions are currently not supported,",
                    "e.g. \"__asm\" and \"__attribute__\" are not yet analyzable.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }
            DirectDeclaratorContext directDecl = decl.directDeclarator();
            if (directDecl.Identifier() == null) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "the variable identifier is missing.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }

            // Extract the name of the variable.
            String name = directDecl.Identifier().getText();             
            // Create a variable Ast node.
            CAst.VariableNode variable = new CAst.VariableNode(type, name);


            //// Convert the initializer to a value.

            // Make sure that there is an initializer.
            InitDeclaratorContext initDecl = initDeclList.get(0);
            if (initDecl.initializer() == null) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "the initializer is missing,",
                    "e.g. \"int x;\" is not yet analyzable.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();

                // FIXME: Use UninitCAst.VariableNode to perform special encoding.
                // return new UninitCAst.VariableNode(type, name);
            }

            // Extract the primaryExpression from the initializer.
            if (initDecl.initializer().assignmentExpression() == null
                || initDecl.initializer().assignmentExpression()
                    .conditionalExpression() == null) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "assignmentExpression or conditionalExpression is missing.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }

            // Finally return the assignment node.
            CAst.AssignmentNode assignmentNode = new CAst.AssignmentNode();
            CAst.AstNode initNode = visitAssignmentExpression(initDecl.initializer().assignmentExpression());
            assignmentNode.left = variable;
            assignmentNode.right = initNode;
            return assignmentNode;
        }
        // Return OpaqueNode as default.
        return new CAst.OpaqueNode();
    }

    /**
     * This visit function builds StatementSequenceNode, AssignmentNode,
     * OpaqueNode, IfBlockNode,
     * AdditionNode, SubtractionNode, MultiplicationNode, DivisionNode,
     * EqualNode, NotEqualNode, LessThanNode, GreaterThanNode, LessEqualNode, GreaterEqualNode,
     * SetPortNode, ScheduleNode.
     * 
     * @param ctx
     * @return
     */
    @Override
    public CAst.AstNode visitStatement(StatementContext ctx) {
        if (ctx.compoundStatement() != null) {
            BlockItemListContext bilCtx = ctx.compoundStatement().blockItemList();
            if (bilCtx != null) {
                return visitBlockItemList(bilCtx);
            }
        } else if (ctx.expressionStatement() != null) {
            ExpressionContext exprCtx = ctx.expressionStatement().expression();
            if (exprCtx != null) {
                return visitExpression(exprCtx);
            }
        } else if (ctx.selectionStatement() != null) {
            return visitSelectionStatement(ctx.selectionStatement());
        }
        return new CAst.OpaqueNode();
    }

    @Override
    public CAst.AstNode visitExpression(ExpressionContext ctx) {
        if (ctx.assignmentExpression().size() == 1) {
            return visitAssignmentExpression(ctx.assignmentExpression().get(0));
        }
        System.out.println(String.join(" ", 
            "Warning (line " + ctx.getStart().getLine() + "):",
            "only one assignmentExpression in an expression is currently supported.",
            "Marking the statement as opaque."
        ));
        return new CAst.OpaqueNode();
    }

    @Override
    public CAst.AstNode visitPrimaryExpression(PrimaryExpressionContext ctx) {
        if (ctx.Identifier() != null) {
            return new CAst.VariableNode(ctx.Identifier().getText());
        } else if (ctx.Constant() != null) {
            return new CAst.LiteralNode(ctx.Constant().getText());
        }  else if (ctx.expression() != null) {
            return visitExpression(ctx.expression());
        }
        System.out.println(String.join(" ", 
            "Warning (line " + ctx.getStart().getLine() + "):",
            "only identifier, constant, and expressions are supported in a primary expression.",
            "Marking the declaration as opaque."
        ));
        return new CAst.OpaqueNode();
    }

    // FIXME: More checks needed. This implementation currently silently omit
    // certain cases, such as arr[1].
    @Override
    public CAst.AstNode visitPostfixExpression(PostfixExpressionContext ctx) {
        if (ctx.PlusPlus().size() > 0
            || ctx.MinusMinus().size() > 0
            || ctx.Dot().size() > 0
            || (ctx.LeftBracket().size() > 0 && ctx.RightBracket().size() > 0)) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Postfix '++', '--', '.', '[]' are currently not supported.",
                "Marking the statement as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        // State variables on the self struct, ports and actions.
        if (ctx.primaryExpression() != null
            && ctx.Identifier().size() == 1
            && ctx.Arrow().size() == 1) {
            CAst.AstNode primaryExprNode = visitPrimaryExpression(ctx.primaryExpression());
            if (primaryExprNode instanceof CAst.LiteralNode) {
                // Unreachable.
                System.out.println("Unreachable!");
                return new CAst.OpaqueNode(); // FIXME: Throw an exception instead.
            }
            CAst.VariableNode varNode = (CAst.VariableNode) primaryExprNode;
            if (varNode.name.equals("self")) {
                // return a state variable node.
                return new CAst.StateVarNode(ctx.Identifier().get(0).getText());
            } else if (ctx.Identifier().get(0).getText().equals("is_present")) {
                // return a trigger present node.
                return new CAst.TriggerValueNode(varNode.name);
            } else if (ctx.Identifier().get(0).getText().equals("value")) {
                // return a trigger value node.
                return new CAst.TriggerIsPresentNode(varNode.name);
            } else {
                // Generic pointer dereference, unanalyzable.
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "Generic pointer dereference is not supported in a postfix expression.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }
        }
        // LF built-in function calls (set or schedule)
        if (ctx.primaryExpression() != null
            && ctx.argumentExpressionList().size() == 1
            && ctx.LeftParen() != null && ctx.RightParen() != null) {
            CAst.AstNode primaryExprNode = visitPrimaryExpression(ctx.primaryExpression());
            List<AssignmentExpressionContext> params = ctx.argumentExpressionList().get(0).assignmentExpression();
            if (primaryExprNode instanceof CAst.LiteralNode) {
                // Unreachable.
                System.out.println("Unreachable!");
                return new CAst.OpaqueNode(); // FIXME: Throw an exception instead.
            }
            CAst.VariableNode varNode = (CAst.VariableNode) primaryExprNode;
            if (varNode.name.equals("lf_set")) {
                // return a set port node.
                if (params.size() != 2) {
                    System.out.println(String.join(" ", 
                        "Warning (line " + ctx.getStart().getLine() + "):",
                        "lf_set must have 2 arguments. Detected " + ctx.argumentExpressionList().size(),
                        "Marking the function call as opaque."
                    ));
                    return new CAst.OpaqueNode();
                }
                CAst.SetPortNode node = new CAst.SetPortNode();
                node.left = visitAssignmentExpression(params.get(0));
                node.right = visitAssignmentExpression(params.get(1));
                return node;
            } else if (varNode.name.equals("lf_schedule")) {
                // return a set port node.
                if (params.size() != 2
                    && params.size() != 3) {
                    System.out.println(String.join(" ", 
                        "Warning (line " + ctx.getStart().getLine() + "):",
                        "lf_schedule must have 2 or 3 arguments. Detected " + ctx.argumentExpressionList().size(),
                        "Marking the function call as opaque."
                    ));
                    return new CAst.OpaqueNode();
                }
                CAst.ScheduleActionNode node = new CAst.ScheduleActionNode();
                for (AssignmentExpressionContext param : params) {
                    node.children.add(visitAssignmentExpression(param));
                }
                return node;
            } else {
                // Generic pointer dereference, unanalyzable.
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "Generic pointer dereference is not supported in a postfix expression.",
                    "Marking the declaration as opaque."
                ));
                return new CAst.OpaqueNode();
            }
        }
        // Variable or literal
        if (ctx.primaryExpression() != null) {
            return visitPrimaryExpression(ctx.primaryExpression());
        }
        System.out.println(String.join(" ", 
            "Warning (line " + ctx.getStart().getLine() + "):",
            "only an identifier, constant, state variable, port, and action are supported in a primary expression.",
            "Marking the declaration as opaque."
        ));
        return new CAst.OpaqueNode();
    }

    @Override
    public CAst.AstNode visitUnaryExpression(UnaryExpressionContext ctx) {
        if (ctx.PlusPlus().size() > 0
            || ctx.MinusMinus().size() > 0
            || ctx.Sizeof().size() > 0) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Prefix '++', '--', and 'sizeof' are currently not supported.",
                "Marking the statement as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        if (ctx.postfixExpression() != null) {
            return visitPostfixExpression(ctx.postfixExpression());
        }
        if (ctx.unaryOperator() != null
            && ctx.unaryOperator().Not() != null
            && ctx.castExpression() != null) {
            CAst.LogicalNotNode node = new CAst.LogicalNotNode();
            node.child = visitCastExpression(ctx.castExpression());
            return node;
        }
        System.out.println(String.join(" ", 
            "Warning (line " + ctx.getStart().getLine() + "):",
            "only postfixExpression and '!' in a unaryExpression is currently supported.",
            "Marking the statement as opaque."
        ));
        return new CAst.OpaqueNode();
    }

    @Override
    public CAst.AstNode visitCastExpression(CastExpressionContext ctx) {
        if (ctx.unaryExpression() != null) {
            return visitUnaryExpression(ctx.unaryExpression());
        }
        System.out.println(String.join(" ", 
            "Warning (line " + ctx.getStart().getLine() + "):",
            "only unaryExpression in a castExpression is currently supported.",
            "Marking the statement as opaque."
        ));
        return new CAst.OpaqueNode();
    }

    @Override
    public CAst.AstNode visitMultiplicativeExpression(MultiplicativeExpressionContext ctx) {
        if (ctx.castExpression().size() > 1) {
            CAst.AstNodeBinary node;
            if (ctx.Star() != null) {
                node = new CAst.MultiplicationNode();
            } else if (ctx.Div() != null) {
                node = new CAst.DivisionNode();
            } else if (ctx.Mod() != null) {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "Mod expression '%' is currently unsupported.",
                    "Marking the expression as opaque."
                ));
                return new CAst.OpaqueNode();
            } else {
                node = new CAst.AstNodeBinary();
            }
            node.left = visitCastExpression(ctx.castExpression().get(0));
            node.right = visitCastExpression(ctx.castExpression().get(1));
            return node;
        }
        return visitCastExpression(ctx.castExpression().get(0));
    }

    @Override
    public CAst.AstNode visitAdditiveExpression(AdditiveExpressionContext ctx) {
        if (ctx.multiplicativeExpression().size() > 1) {
            CAst.AstNodeBinary node;
            if (ctx.Plus() != null) {
                node = new CAst.AdditionNode();
            } else if (ctx.Minus() != null) {
                node = new CAst.SubtractionNode();
            } else {
                node = new CAst.AstNodeBinary();
            }
            node.left = visitMultiplicativeExpression(ctx.multiplicativeExpression().get(0));
            node.right = visitMultiplicativeExpression(ctx.multiplicativeExpression().get(1));
            return node;
        }
        return visitMultiplicativeExpression(ctx.multiplicativeExpression().get(0));
    }

    @Override
    public CAst.AstNode visitShiftExpression(ShiftExpressionContext ctx) {
        if (ctx.additiveExpression().size() > 1) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Shift expression '<<' or '>>' is currently unsupported.",
                "Marking the expression as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        return visitAdditiveExpression(ctx.additiveExpression().get(0));
    }

    @Override
    public CAst.AstNode visitRelationalExpression(RelationalExpressionContext ctx) {
        if (ctx.shiftExpression().size() > 1) {
            CAst.AstNodeBinary node;
            if (ctx.Less() != null) {
                node = new CAst.LessThanNode();
            } else if (ctx.LessEqual() != null) {
                node = new CAst.LessEqualNode();
            } else if (ctx.Greater() != null) { 
                node = new CAst.GreaterThanNode();
            } else if (ctx.GreaterEqual() != null) {
                node = new CAst.GreaterEqualNode();
            } else {
                node = new CAst.AstNodeBinary();
            }
            node.left = visitShiftExpression(ctx.shiftExpression().get(0));
            node.right = visitShiftExpression(ctx.shiftExpression().get(1));
            return node;
        }
        return visitShiftExpression(ctx.shiftExpression().get(0));
    }

    @Override
    public CAst.AstNode visitEqualityExpression(EqualityExpressionContext ctx) {
        if (ctx.relationalExpression().size() > 1) {
            CAst.AstNodeBinary node;
            if (ctx.Equal().size() > 0) {
                node = new CAst.EqualNode();
            }
            else if (ctx.NotEqual().size() > 0) {
                node = new CAst.NotEqualNode();
            } else {
                node = new CAst.AstNodeBinary();
            }
            node.left = visitRelationalExpression(ctx.relationalExpression().get(0));
            node.right = visitRelationalExpression(ctx.relationalExpression().get(1));
            return node;
        }
        return visitRelationalExpression(ctx.relationalExpression().get(0));
    }

    @Override
    public CAst.AstNode visitAndExpression(AndExpressionContext ctx) {
        if (ctx.equalityExpression().size() > 1) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "And expression '&' is currently unsupported.",
                "Marking the expression as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        return visitEqualityExpression(ctx.equalityExpression().get(0));
    }

    @Override
    public CAst.AstNode visitExclusiveOrExpression(ExclusiveOrExpressionContext ctx) {
        if (ctx.andExpression().size() > 1) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Exclusive Or '^' is currently unsupported.",
                "Marking the expression as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        return visitAndExpression(ctx.andExpression().get(0));
    }

    @Override
    public CAst.AstNode visitInclusiveOrExpression(InclusiveOrExpressionContext ctx) {
        if (ctx.exclusiveOrExpression().size() > 1) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Inclusive Or '|' is currently unsupported.",
                "Marking the expression as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        return visitExclusiveOrExpression(ctx.exclusiveOrExpression().get(0));
    }

    @Override
    public CAst.AstNode visitLogicalAndExpression(LogicalAndExpressionContext ctx) {
        if (ctx.inclusiveOrExpression().size() > 1) {
            CAst.LogicalAndNode node = new CAst.LogicalAndNode();
            node.left = visitInclusiveOrExpression(ctx.inclusiveOrExpression().get(0));
            node.right = visitInclusiveOrExpression(ctx.inclusiveOrExpression().get(1));
            return node;
        }
        return visitInclusiveOrExpression(ctx.inclusiveOrExpression().get(0));
    }

    @Override
    public CAst.AstNode visitLogicalOrExpression(LogicalOrExpressionContext ctx) {
        if (ctx.logicalAndExpression().size() > 1) {
            CAst.LogicalOrNode node = new CAst.LogicalOrNode();
            node.left = visitLogicalAndExpression(ctx.logicalAndExpression().get(0));
            node.right = visitLogicalAndExpression(ctx.logicalAndExpression().get(1));
            return node;
        }
        return visitLogicalAndExpression(ctx.logicalAndExpression().get(0));
    }

    @Override
    public CAst.AstNode visitConditionalExpression(ConditionalExpressionContext ctx) {
        if (ctx.expression() != null) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Currently do not support inline conditional expression.",
                "Marking the expression as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        return visitLogicalOrExpression(ctx.logicalOrExpression());
    }

    @Override
    public CAst.AstNode visitAssignmentExpression(AssignmentExpressionContext ctx) {
        if (ctx.conditionalExpression() != null) {
            return visitConditionalExpression(ctx.conditionalExpression());
        }
        if (ctx.unaryExpression() != null
            && ctx.assignmentExpression() != null) {
            CAst.AstNodeBinary assignmentNode = new CAst.AssignmentNode();
            assignmentNode.left = visitUnaryExpression(ctx.unaryExpression());
            if (ctx.assignmentOperator().getText().equals("=")) {
                assignmentNode.right = visitAssignmentExpression(ctx.assignmentExpression());
            }
            else if (ctx.assignmentOperator().getText().equals("+=")) {
                CAst.AdditionNode subnode = new CAst.AdditionNode();
                subnode.left = visitUnaryExpression(ctx.unaryExpression());
                subnode.right = visitAssignmentExpression(ctx.assignmentExpression());
                assignmentNode.right = subnode;
            }
            else if (ctx.assignmentOperator().getText().equals("-=")) {
                CAst.SubtractionNode subnode = new CAst.SubtractionNode();
                subnode.left = visitUnaryExpression(ctx.unaryExpression());
                subnode.right = visitAssignmentExpression(ctx.assignmentExpression());
                assignmentNode.right = subnode;
            }
            else if (ctx.assignmentOperator().getText().equals("*=")) {
                CAst.MultiplicationNode subnode = new CAst.MultiplicationNode();
                subnode.left = visitUnaryExpression(ctx.unaryExpression());
                subnode.right = visitAssignmentExpression(ctx.assignmentExpression());
                assignmentNode.right = subnode;
            }
            else if (ctx.assignmentOperator().getText().equals("/=")) {
                CAst.DivisionNode subnode = new CAst.DivisionNode();
                subnode.left = visitUnaryExpression(ctx.unaryExpression());
                subnode.right = visitAssignmentExpression(ctx.assignmentExpression());
                assignmentNode.right = subnode;
            }
            else {
                System.out.println(String.join(" ", 
                    "Warning (line " + ctx.getStart().getLine() + "):",
                    "Only '=', '+=', '-=', '*=', '/=' assignment operators are supported.",
                    "Marking the expression as opaque."
                ));
                return new CAst.OpaqueNode();
            }
            return assignmentNode;
        }
        System.out.println(String.join(" ", 
            "Warning (line " + ctx.getStart().getLine() + "):",
            "DigitSequence in an assignmentExpression is currently not supported.",
            "Marking the expression as opaque."
        ));
        return new CAst.OpaqueNode();
    }

    @Override
    public CAst.AstNode visitSelectionStatement(SelectionStatementContext ctx) {
        if (ctx.Switch() != null) {
            System.out.println(String.join(" ", 
                "Warning (line " + ctx.getStart().getLine() + "):",
                "Switch case statement is currently not supported.",
                "Marking the expression as opaque."
            ));
            return new CAst.OpaqueNode();
        }
        CAst.IfBlockNode ifBlockNode =  new CAst.IfBlockNode();
        CAst.IfBodyNode ifBodyNode =  new CAst.IfBodyNode();
        ifBlockNode.left = visitExpression(ctx.expression());
        ifBlockNode.right = ifBodyNode;
        ifBodyNode.left = visitStatement(ctx.statement().get(0));
        if (ctx.statement().size() > 1) {
            ifBodyNode.right = visitStatement(ctx.statement().get(1));
        }
        return ifBlockNode;
    }
}
