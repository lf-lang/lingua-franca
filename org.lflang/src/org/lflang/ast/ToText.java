package org.lflang.ast;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.ILeafNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.xbase.lib.StringExtensions;

import org.lflang.ASTUtils;
import org.lflang.lf.Action;
import org.lflang.lf.Array;
import org.lflang.lf.ArraySpec;
import org.lflang.lf.Assignment;
import org.lflang.lf.Code;
import org.lflang.lf.Connection;
import org.lflang.lf.Deadline;
import org.lflang.lf.Element;
import org.lflang.lf.Expression;
import org.lflang.lf.Host;
import org.lflang.lf.IPV4Host;
import org.lflang.lf.IPV6Host;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.Literal;
import org.lflang.lf.Method;
import org.lflang.lf.MethodArgument;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Mutation;
import org.lflang.lf.NamedHost;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Port;
import org.lflang.lf.Preamble;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.STP;
import org.lflang.lf.Serializer;
import org.lflang.lf.StateVar;
import org.lflang.lf.TargetDecl;
import org.lflang.lf.Time;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.Type;
import org.lflang.lf.TypeParm;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.lf.util.LfSwitch;
import org.lflang.util.StringUtil;


/**
 * Switch class for converting AST nodes to their textual representation as
 * it would appear in LF code.
 */
public class ToText extends LfSwitch<String> {

    /// public instance initialized when loading the class
    public static final ToText instance = new ToText();

    // private constructor
    private ToText() { super(); }

    @Override
    public String caseArraySpec(ArraySpec spec) {
        return (spec.isOfVariableLength()) ? "[]" : "[" + spec.getLength() + "]";
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
            int end = str.indexOf("=}", start);
            if (start == -1 || end == -1) {
                // Silent failure is needed here because toText is needed to create the intermediate representation,
                // which the validator uses.
                return str;
            }
            str = str.substring(start + 2, end);
            if (str.split("\n").length > 1) {
                // multi line code
                return StringUtil.trimCodeBlock(str);
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
        StringBuilder sb = new StringBuilder();
        if (!StringExtensions.isNullOrEmpty(host.getUser())) {
            sb.append(host.getUser()).append("@");
        }
        if (!StringExtensions.isNullOrEmpty(host.getAddr())) {
            sb.append(host.getAddr());
        }
        if (host.getPort() != 0) {
            sb.append(":").append(host.getPort());
        }
        return sb.toString();
    }

    @Override
    public String caseLiteral(Literal l) {
        return l.getLiteral();
    }

    @Override
    public String caseParameterReference(ParameterReference p) {
        return p.getParameter().getName();
    }

    @Override
    public String caseTime(Time t) {
        return ASTUtils.toTimeValue(t).toString();
    }

    @Override
    public String caseType(Type type) {
        String base = ASTUtils.baseType(type);
        String arr = (type.getArraySpec() != null) ? doSwitch(type.getArraySpec()) : "";
        return base + arr;
    }

    @Override
    public String caseTypeParm(TypeParm t) {
        return !StringExtensions.isNullOrEmpty(t.getLiteral()) ? t.getLiteral() : doSwitch(t.getCode());
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
    public String caseModel(Model object) {
        StringBuilder sb = new StringBuilder();

        sb.append(caseTargetDecl(object.getTarget()));
        object.getImports().forEach(i -> sb.append(caseImport(i)));
        object.getPreambles().forEach(p -> sb.append(casePreamble(p)));
        object.getReactors().forEach(r -> sb.append(caseReactor(r)));

        return sb.toString();

    }

    @Override
    public String caseImport(Import object) {
        return super.caseImport(object);
    }

    @Override
    public String caseReactorDecl(ReactorDecl object) {
        return super.caseReactorDecl(object);
    }

    @Override
    public String caseImportedReactor(ImportedReactor object) {
        return super.caseImportedReactor(object);
    }

    @Override
    public String caseReactor(Reactor object) {
        return super.caseReactor(object);
    }

    @Override
    public String caseTargetDecl(TargetDecl object) {
            return String.format("target %s", object);
    }

    @Override
    public String caseStateVar(StateVar object) {
        return super.caseStateVar(object);
    }

    @Override
    public String caseMethod(Method object) {
        return super.caseMethod(object);
    }

    @Override
    public String caseMethodArgument(MethodArgument object) {
        return super.caseMethodArgument(object);
    }

    @Override
    public String caseInput(Input object) {
        return super.caseInput(object);
    }

    @Override
    public String caseOutput(Output object) {
        return super.caseOutput(object);
    }

    @Override
    public String caseTimer(Timer object) {
        return super.caseTimer(object);
    }

    @Override
    public String caseMode(Mode object) {
        return super.caseMode(object);
    }

    @Override
    public String caseAction(Action object) {
        return super.caseAction(object);
    }

    @Override
    public String caseReaction(Reaction object) {
        return super.caseReaction(object);
    }

    @Override
    public String caseTriggerRef(TriggerRef object) {
        return super.caseTriggerRef(object);
    }

    @Override
    public String caseDeadline(Deadline object) {
        return super.caseDeadline(object);
    }

    @Override
    public String caseSTP(STP object) {
        return super.caseSTP(object);
    }

    @Override
    public String caseMutation(Mutation object) {
        return super.caseMutation(object);
    }

    @Override
    public String casePreamble(Preamble object) {
        return super.casePreamble(object);
    }

    @Override
    public String caseInstantiation(Instantiation object) {
        return super.caseInstantiation(object);
    }

    @Override
    public String caseConnection(Connection object) {
        return super.caseConnection(object);
    }

    @Override
    public String caseSerializer(Serializer object) {
        return super.caseSerializer(object);
    }

    @Override
    public String caseKeyValuePairs(KeyValuePairs object) {
        return super.caseKeyValuePairs(object);
    }

    @Override
    public String caseKeyValuePair(KeyValuePair object) {
        return super.caseKeyValuePair(object);
    }

    @Override
    public String caseArray(Array object) {
        return super.caseArray(object);
    }

    @Override
    public String caseElement(Element object) {
        return super.caseElement(object);
    }

    @Override
    public String caseTypedVariable(TypedVariable object) {
        return super.caseTypedVariable(object);
    }

    @Override
    public String caseVariable(Variable object) {
        return super.caseVariable(object);
    }

    @Override
    public String caseAssignment(Assignment object) {
        return super.caseAssignment(object);
    }

    @Override
    public String caseParameter(Parameter object) {
        return super.caseParameter(object);
    }

    @Override
    public String caseExpression(Expression object) {
        return super.caseExpression(object);
    }

    @Override
    public String casePort(Port object) {
        return super.casePort(object);
    }

    @Override
    public String caseWidthSpec(WidthSpec object) {
        return super.caseWidthSpec(object);
    }

    @Override
    public String caseWidthTerm(WidthTerm object) {
        return super.caseWidthTerm(object);
    }

    @Override
    public String caseIPV4Host(IPV4Host object) {
        return super.caseIPV4Host(object);
    }

    @Override
    public String caseIPV6Host(IPV6Host object) {
        return super.caseIPV6Host(object);
    }

    @Override
    public String caseNamedHost(NamedHost object) {
        return super.caseNamedHost(object);
    }

    @Override
    public String defaultCase(EObject object) {
        throw new UnsupportedOperationException("ToText has no case for " + object.getClass().getName());
    }
}
