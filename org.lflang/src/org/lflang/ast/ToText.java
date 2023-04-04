package org.lflang.ast;

import java.util.regex.Pattern;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.ILeafNode;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;

import org.lflang.ASTUtils;
import org.lflang.lf.ArraySpec;
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
 * Switch class for converting AST nodes to some textual representation that seems likely
 * to be useful for as many code generators as possible.
 */
public class ToText extends LfSwitch<String> {

    /// public instance initialized when loading the class
    public static final ToText instance = new ToText();

    // private constructor
    private ToText() { super(); }

    @Override
    public String caseArraySpec(ArraySpec spec) {
        return ToLf.instance.doSwitch(spec).toString();
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
            for (ILeafNode leaf : node.getLeafNodes()) {
                builder.append(leaf.getText());
            }
            String str = builder.toString().trim();
            // Remove the code delimiters (and any surrounding comments).
            // This assumes any comment before {= does not include {=.
            int start = str.indexOf("{=");
            int end = str.lastIndexOf("=}");
            if (start == -1 || end == -1) {
                // Silent failure is needed here because toText is needed to create the intermediate representation,
                // which the validator uses.
                return str;
            }
            str = str.substring(start + 2, end);
            if (str.split("\n").length > 1) {
                // multi line code
                return StringUtil.trimCodeBlock(str, 1);
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
    public String caseHost(Host host) {
        return ToLf.instance.caseHost(host).toString();
    }

    @Override
    public String caseLiteral(Literal l) {
        return ToLf.instance.caseLiteral(l).toString();
    }

    @Override
    public String caseParameterReference(ParameterReference p) {
        return ToLf.instance.caseParameterReference(p).toString();
    }

    @Override
    public String caseTime(Time t) {
        return ToLf.instance.caseTime(t).toString();
    }

    @Override
    public String caseType(Type type) {
        String base = ASTUtils.baseType(type);
        String arr = (type.getArraySpec() != null) ? doSwitch(type.getArraySpec()) : "";
        return base + arr;
    }

    @Override
    public String caseTypeParm(TypeParm t) {
        if (t.getCode() != null) return doSwitch(t.getCode());
        return ToLf.instance.caseTypeParm(t).toString();
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
        throw new UnsupportedOperationException("ToText has no case for " + object.getClass().getName());
    }
}
