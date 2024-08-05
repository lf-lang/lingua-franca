package org.lflang.ast;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.ILeafNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BracketListExpression;
import org.lflang.lf.CStyleArraySpec;
import org.lflang.lf.Code;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Host;
import org.lflang.lf.Literal;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.VarRef;
import org.lflang.lf.util.LfSwitch;
import org.lflang.util.StringUtil;

/**
 * Switch class for converting AST nodes to some textual representation that seems likely to be
 * useful for as many code generators as possible.
 */
public class ToText extends LfSwitch<String> {

  /// public instance initialized when loading the class
  public static final ToText instance = new ToText();

  // private constructor
  private ToText() {
    super();
  }

  @Override
  public String caseCStyleArraySpec(CStyleArraySpec spec) {
    return new ToLf().doSwitch(spec).toString();
  }

  @Override
  public String caseCodeExpr(CodeExpr object) {
    return caseCode(object.getCode());
  }

  @Override
  public String caseCode(Code code) {
    ICompositeNode node = NodeModelUtils.getNode(code);
    if (node != null) {
      StringBuilder builder = new StringBuilder(Math.max(node.getTotalLength(), 1));
      boolean started = false;
      for (ILeafNode leaf : node.getLeafNodes()) {
        if (!leaf.getText().equals("{=") && !leaf.getText().equals("=}")) {
          var nothing = leaf.getText().isBlank() || ASTUtils.isSingleLineComment(leaf);
          if (!nothing
              || started
              || leaf.getText().startsWith("\n")
              || leaf.getText().startsWith("\r")) builder.append(leaf.getText());
          if ((leaf.getText().contains("\n") || (!nothing))) {
            started = true;
          }
        }
      }
      String str = builder.toString();
      if (str.contains("\n")) {
        // multi line code
        return StringUtil.trimCodeBlock(str, 0);
      } else {
        // single line code
        return str.trim();
      }
    } else if (code.getBody() != null) {
      // Code must have been added as a simple string.
      return code.getBody();
    }
    return "";
  }

  @Override
  public String caseBracedListExpression(BracedListExpression object) {
    return new ToLf().caseBracedListExpression(object).toString();
  }

  @Override
  public String caseBracketListExpression(BracketListExpression object) {
    return new ToLf().caseBracketListExpression(object).toString();
  }

  @Override
  public String caseHost(Host host) {
    return new ToLf().caseHost(host).toString();
  }

  @Override
  public String caseLiteral(Literal l) {
    return new ToLf().caseLiteral(l).toString();
  }

  @Override
  public String caseParameterReference(ParameterReference p) {
    return new ToLf().caseParameterReference(p).toString();
  }

  @Override
  public String caseTime(Time t) {
    return new ToLf().caseTime(t).toString();
  }

  @Override
  public String caseType(Type type) {
    if (type.getCode() != null) {
      return caseCode(type.getCode());
    }
    return new ToLf().caseType(type).toString();
  }

  @Override
  public String caseTypeParm(TypeParm t) {
    if (t.getCode() != null) return doSwitch(t.getCode());
    return new ToLf().caseTypeParm(t).toString();
  }

  @Override
  public String caseVarRef(VarRef v) {
    if (v.getContainer() != null) {
      return String.format("%s.%s", v.getContainer().getName(), v.getVariable().getName());
    } else {
      return v.getVariable().getName();
    }
  }

  @Override
  public String defaultCase(EObject object) {
    throw new UnsupportedOperationException(
        "ToText has no case for " + object.getClass().getName());
  }
}
