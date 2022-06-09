package org.lflang.ast;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import org.eclipse.emf.common.util.EList;
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

    // FIXME: This class needs to respect comments, which lost at the EObject level of abstraction and must be obtained
    //  using NodeModelUtils like this: https://github.com/lf-lang/lingua-franca/blob/c4dfbd9cebdb9aaf249508360e0bac1ce545458b/org.lflang/src/org/lflang/ASTUtils.java#L1692

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
            // FIXME: Exclusion of {= =} violates the spec given by the docstring of this class.
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
        // 'import' reactorClasses+=ImportedReactor (',' reactorClasses+=ImportedReactor)* 'from' importURI=STRING ';'?
        return String.format(
            "import %s from %s\n",
            list(object.getReactorClasses(), ", ", "", "", false),
            object.getImportURI()
        );
    }

    @Override
    public String caseReactorDecl(ReactorDecl object) {
        // Reactor | ImportedReactor
        return defaultCase(object);
    }

    @Override
    public String caseImportedReactor(ImportedReactor object) {
        // reactorClass=[Reactor] ('as' name=ID)?
        return String.format("%s as %s", object.getReactorClass(), object.getName());
    }

    @Override
    public String caseReactor(Reactor object) {
        // {Reactor} ((federated?='federated' | main?='main')? & realtime?='realtime'?) 'reactor' (name=ID)?
        // ('<' typeParms+=TypeParm (',' typeParms+=TypeParm)* '>')?
        // ('(' parameters+=Parameter (',' parameters+=Parameter)* ')')?
        // ('at' host=Host)?
        // ('extends' (superClasses+=[ReactorDecl] (',' superClasses+=[ReactorDecl])*))?
        // '{'
        // (     (preambles+=Preamble)
        //     | (stateVars+=StateVar)
        //     | (methods+=Method)
        //     | (inputs+=Input)
        //     | (outputs+=Output)
        //     | (timers+=Timer)
        //     | (actions+=Action)
        //     | (instantiations+=Instantiation)
        //     | (connections+=Connection)
        //     | (reactions+=Reaction)
        //     | (modes+=Mode)
        //     | (mutations+=Mutation)
        // )* '}'
        StringBuilder sb = new StringBuilder();
        if (object.isFederated()) sb.append("federated ");
        if (object.isMain()) sb.append("main ");
        if (object.isRealtime()) sb.append("realtime ");
        sb.append("reactor ");
        if (object.getName() != null) sb.append(object.getName());
        sb.append(list(object.getTypeParms(), ", ", "<", ">", true));
        sb.append(list(object.getParameters()));
        if (object.getHost() != null) sb.append(" at ").append(doSwitch(object.getHost()));
        if (object.getSuperClasses() != null) {
            sb.append(" extends ").append(list(object.getSuperClasses(), ", ", "", "", true));
        }
        sb.append(" {\n");
        sb.append(indentedStatements(List.of(
            object.getPreambles(),
            object.getStateVars(),
            object.getMethods(),
            object.getInputs(),
            object.getOutputs(),
            object.getTimers(),
            object.getActions(),
            object.getInstantiations(),
            object.getConnections(),
            object.getReactions(),
            object.getModes(),
            object.getMutations()
        )));
        sb.append("}\n");
        return sb.toString();
    }

    @Override
    public String caseTargetDecl(TargetDecl object) {
        return String.format("target %s", object);
    }

    @Override
    public String caseStateVar(StateVar object) {
        // 'state' name=ID (
        //     (':' (type=Type))?
        //     ((parens+='(' (init+=Expression (','  init+=Expression)*)? parens+=')')
        //         | (braces+='{' (init+=Expression (','  init+=Expression)*)? braces+='}')
        //     )?
        // ) ';'?
        return "state " + object.getName() + typeAnnotationFor(object.getType());
    }

    @Override
    public String caseMethod(Method object) {
        // const?='const'? 'method' name=ID
        // '(' (arguments+=MethodArgument (',' arguments+=MethodArgument)*)? ')'
        // (':' return=Type)?
        // code=Code
        // ';'?
        StringBuilder sb = new StringBuilder();
        if (object.isConst()) sb.append("const ");
        sb.append("method");
        return sb.toString();
    }

    @Override
    public String caseMethodArgument(MethodArgument object) {
        return object.getName() + typeAnnotationFor(object.getType());
    }

    @Override
    public String caseInput(Input object) {
        // mutable?='mutable'? 'input' (widthSpec=WidthSpec)? name=ID (':' type=Type)? ';'?
        StringBuilder sb = new StringBuilder();
        if (object.isMutable()) sb.append("mutable ");
        sb.append("input");
        if (object.getWidthSpec() != null) sb.append(doSwitch(object.getWidthSpec()));
        sb.append(" ").append(object.getName()).append(typeAnnotationFor(object.getType()));
        return sb.toString();
    }

    @Override
    public String caseOutput(Output object) {
        // 'output' (widthSpec=WidthSpec)? name=ID (':' type=Type)? ';'?
        StringBuilder sb = new StringBuilder();
        sb.append("output");
        if (object.getWidthSpec() != null) sb.append(doSwitch(object.getWidthSpec()));
        sb.append(" ").append(object.getName());
        if (object.getType() != null) sb.append(doSwitch(object.getType()));
        return sb.toString();
    }

    @Override
    public String caseTimer(Timer object) {
        // 'timer' name=ID ('(' offset=Expression (',' period=Expression)? ')')? ';'?
        StringBuilder sb = new StringBuilder();
        sb.append("timer ").append(object.getName());
        if (object.getOffset() != null) {
            sb.append("(");
            sb.append(doSwitch(object.getOffset()));
            if (object.getPeriod() != null) sb.append(", ").append(doSwitch(object.getPeriod()));
            sb.append(")");
        }
        return sb.toString();
    }

    @Override
    public String caseMode(Mode object) {
        // {Mode} (initial?='initial')? 'mode' (name=ID)?
        // '{' (
        //     (stateVars+=StateVar) |
        //     (timers+=Timer) |
        //     (actions+=Action) |
        //     (instantiations+=Instantiation) |
        //     (connections+=Connection) |
        //     (reactions+=Reaction)
        // )* '}'
        StringBuilder sb = new StringBuilder();
        if (object.isInitial()) sb.append("initial ");
        sb.append("mode ");
        if (object.getName() != null) sb.append(object.getName()).append(" ");
        sb.append("{\n");
        sb.append(indentedStatements(List.of(
            object.getStateVars(),
            object.getTimers(),
            object.getActions(),
            object.getInstantiations(),
            object.getConnections(),
            object.getReactions()
        )));
        sb.append("}\n");
        return sb.toString();
    }

    @Override
    public String caseAction(Action object) {
        // (origin=ActionOrigin)? 'action' name=ID
        // ('(' minDelay=Expression (',' minSpacing=Expression (',' policy=STRING)? )? ')')?
        // (':' type=Type)? ';'?
        StringBuilder sb = new StringBuilder();
        if (object.getOrigin() == null) sb.append(object.getOrigin().getLiteral()).append(" ");
        sb.append("action");
        if (object.getMinDelay() != null) {
            sb.append(doSwitch(object.getMinDelay()));
            if (object.getMinSpacing() != null) sb.append(", ").append(doSwitch(object.getMinSpacing()));
            if (object.getPolicy() != null) sb.append(", ").append(object.getPolicy());
        }
        sb.append(typeAnnotationFor(object.getType()));
        return sb.toString();
    }

    @Override
    public String caseReaction(Reaction object) {
        // ('reaction')
        // ('(' (triggers+=TriggerRef (',' triggers+=TriggerRef)*)? ')')?
        // (sources+=VarRef (',' sources+=VarRef)*)?
        // ('->' effects+=VarRefOrModeTransition (',' effects+=VarRefOrModeTransition)*)?
        // code=Code
        // (stp=STP)?
        // (deadline=Deadline)?
        StringBuilder sb = new StringBuilder();
        sb.append("reaction");
        sb.append(list(object.getTriggers()));
        sb.append(list(object.getSources(), ", ", " ", "", true));
        sb.append(" ->").append(list(object.getEffects(), ", ", " ", "", true));
        sb.append(doSwitch(object.getCode()));
        if (object.getStp() != null) sb.append(" ").append(doSwitch(object.getStp()));
        if (object.getDeadline() != null) sb.append(" ").append(doSwitch(object.getDeadline()));
        return sb.toString();
    }

    @Override
    public String caseTriggerRef(TriggerRef object) {
        if (object.isStartup()) return "startup";
        if (object.isShutdown()) return "shutdown";
        throw new IllegalArgumentException("The given TriggerRef object appears to be a VarRef.");
    }

    @Override
    public String caseDeadline(Deadline object) {
        // 'deadline' '(' delay=Expression ')' code=Code
        return String.format("deadline(%s) %s", doSwitch(object.getDelay()), doSwitch(object.getCode()));
    }

    @Override
    public String caseSTP(STP object) {
        // 'STP' '(' value=Expression ')' code=Code
        return String.format("STP(%s) %s", doSwitch(object.getValue()), doSwitch(object.getCode()));
    }

    @Override
    public String caseMutation(Mutation object) {
        // ('mutation')
        // ('(' (triggers+=TriggerRef (',' triggers+=TriggerRef)*)? ')')?
        // (sources+=VarRef (',' sources+=VarRef)*)?
        // ('->' effects+=[VarRef] (',' effects+=[VarRef])*)?
        // code=Code
        StringBuilder sb = new StringBuilder();
        sb.append("mutation");
        if (!object.getTriggers().isEmpty()) {
            sb.append(object.getTriggers().stream().map(this::doSwitch).collect(
                Collectors.joining(", ", "(", ")"))
            );
        }
        return sb.toString();
    }

    @Override
    public String casePreamble(Preamble object) {
        // (visibility=Visibility)? 'preamble' code=Code
        return String.format(
            "%s preamble %s",
            object.getVisibility() != null ? object.getVisibility().getLiteral() : "",
            doSwitch(object.getCode())
        );
    }

    @Override
    public String caseInstantiation(Instantiation object) {
        // name=ID '=' 'new' (widthSpec=WidthSpec)?
        // reactorClass=[ReactorDecl] ('<' typeParms+=TypeParm (',' typeParms+=TypeParm)* '>')? '('
        // (parameters+=Assignment (',' parameters+=Assignment)*)?
        // ')' ('at' host=Host)? ';'?;
        StringBuilder sb = new StringBuilder();
        sb.append(object.getName()).append(" = new");
        if (object.getWidthSpec() != null) sb.append(doSwitch(object.getWidthSpec()));
        sb.append(doSwitch(object.getReactorClass()));
        if (!object.getTypeParms().isEmpty()) {
            sb.append(object.getTypeParms().stream().map(this::doSwitch).collect(
                Collectors.joining(", ", "<", ">"))
            );
        }
        sb.append(object.getParameters().stream().map(this::doSwitch).collect(
            Collectors.joining(", ", "(", ")"))
        );
        // TODO: Delete the following case when the corresponding feature is removed
        if (object.getHost() != null) sb.append(" at ").append(doSwitch(object.getHost()));
        return sb.toString();
    }

    @Override
    public String caseConnection(Connection object) {
        // ((leftPorts += VarRef (',' leftPorts += VarRef)*)
        //     | ( '(' leftPorts += VarRef (',' leftPorts += VarRef)* ')' iterated ?= '+'?))
        // ('->' | physical?='~>')
        // rightPorts += VarRef (',' rightPorts += VarRef)*
        //     ('after' delay=Expression)?
        // (serializer=Serializer)?
        // ';'?
        StringBuilder sb = new StringBuilder();
        if (object.isIterated()) sb.append("(");
        sb.append(object.getLeftPorts().stream().map(this::doSwitch).collect(Collectors.joining(", ")));
        if (object.isIterated()) sb.append(")+");
        sb.append(object.isPhysical() ? " ~> " : " -> ");
        sb.append(object.getRightPorts().stream().map(this::doSwitch).collect(Collectors.joining(", ")));
        if (object.getDelay() != null) sb.append(" after ").append(doSwitch(object.getDelay()));
        if (object.getSerializer() != null) sb.append(" ").append(doSwitch(object.getSerializer()));
        return sb.toString();
    }

    @Override
    public String caseSerializer(Serializer object) {
        // 'serializer' type=STRING
        return "serializer " + object.getType();
    }

    @Override
    public String caseKeyValuePairs(KeyValuePairs object) {
        return object.getPairs().stream().map(this::doSwitch).collect(
            Collectors.joining(",\n    ", "{\n    ", "\n}")
        );
    }

    @Override
    public String caseKeyValuePair(KeyValuePair object) {
        return object.getName() + ": " + object.getValue();
    }

    @Override
    public String caseArray(Array object) {
        // '[' elements+=Element (',' (elements+=Element))* ','? ']'
        return object.getElements().stream().map(this::doSwitch).collect(
            Collectors.joining(", ", "[", "]")
        );
    }

    @Override
    public String caseElement(Element object) {
        // keyvalue=KeyValuePairs
        // | array=Array
        // | literal=Literal
        // | (time=INT unit=TimeUnit)
        // | id=Path
        return defaultCase(object);
    }

    @Override
    public String caseTypedVariable(TypedVariable object) {
        // Port | Action
        return defaultCase(object);
    }

    @Override
    public String caseVariable(Variable object) {
        // TypedVariable | Timer | Mode
        return defaultCase(object);
    }

    @Override
    public String caseAssignment(Assignment object) {
        // (lhs=[Parameter] (
        //     (equals='=' rhs+=Expression)
        //     | ((equals='=')? (
        //         parens+='(' (rhs+=Expression (',' rhs+=Expression)*)? parens+=')'
        //         | braces+='{' (rhs+=Expression (',' rhs+=Expression)*)? braces+='}'))
        // ));
        StringBuilder sb = new StringBuilder();
        sb.append(doSwitch(object.getLhs()));
        if (object.getEquals() != null) sb.append(" = ");
        if (!object.getParens().isEmpty()) sb.append("(");
        if (!object.getBraces().isEmpty()) sb.append("{");
        sb.append(object.getRhs().stream().map(this::doSwitch).collect(Collectors.joining(", ", "", "")));
        if (!object.getParens().isEmpty()) sb.append(")");
        if (!object.getBraces().isEmpty()) sb.append("}");
        return sb.toString();
    }

    @Override
    public String caseParameter(Parameter object) {
        // name=ID (':' (type=Type))?
        // ((parens+='(' (init+=Expression (','  init+=Expression)*)? parens+=')')
        // | (braces+='{' (init+=Expression (','  init+=Expression)*)? braces+='}')
        // )?
        return object.getName() + typeAnnotationFor(object.getType()) + list(
            object.getInit(),
            ", ",
            object.getBraces().isEmpty() ? "(" : "{",
            object.getBraces().isEmpty() ? ")" : "}",
            true
        );
    }

    @Override
    public String caseExpression(Expression object) {
        // {Literal} literal = Literal
        // | Time
        // | ParameterReference
        // | Code
        return defaultCase(object);
    }

    @Override
    public String casePort(Port object) {
        // Input | Output
        return defaultCase(object);
    }

    @Override
    public String caseWidthSpec(WidthSpec object) {
        // ofVariableLength?='[]' | '[' (terms+=WidthTerm) ('+' terms+=WidthTerm)* ']';
        if (object.isOfVariableLength()) return "[]";
        return list(object.getTerms(), " + ", "[", "]", false);
    }

    @Override
    public String caseWidthTerm(WidthTerm object) {
        // width=INT
        // | parameter=[Parameter]
        // | 'widthof(' port=VarRef ')'
        // | code=Code;
        if (object.getWidth() != 0) {
            return Objects.toString(object.getWidth());
        } else if (object.getParameter() != null) {
            return doSwitch(object.getParameter());
        } else if (object.getPort() != null) {
            return String.format("widthof(%s)", object.getPort());
        } else if (object.getCode() != null) {
            return doSwitch(object.getCode());
        }
        throw new IllegalArgumentException();
    }

    @Override
    public String caseIPV4Host(IPV4Host object) {
        // (user=Kebab '@')? addr=IPV4Addr (':' port=INT)?
        return caseHost(object);
    }

    @Override
    public String caseIPV6Host(IPV6Host object) {
        // ('[' (user=Kebab '@')? addr=IPV6Addr ']' (':' port=INT)?)
        return caseHost(object);
    }

    @Override
    public String caseNamedHost(NamedHost object) {
        // (user=Kebab '@')? addr=HostName (':' port=INT)?
        return caseHost(object);
    }

    @Override
    public String defaultCase(EObject object) {
        throw new UnsupportedOperationException("ToText has no case for " + object.getClass().getName());
    }

    /**
     * Represent the given EList as a string.
     * @param delimiter The delimiter separating elements of the list.
     * @param prefix The token marking the start of the list.
     * @param suffix The token marking the end of the list.
     * @param nothingIfEmpty Whether the result should be simplified to the
     * empty string as opposed to just the prefix and suffix.
     */
    private <E extends EObject> String list(List<E> items, String delimiter, String prefix, String suffix, boolean nothingIfEmpty) {
        if (nothingIfEmpty && items.isEmpty()) return "";
        return items.stream().map(this::doSwitch).collect(Collectors.joining(delimiter, prefix, suffix));
    }

    private <E extends EObject> String list(EList<E> items) {
        return list(items, ", ", "(", ")", true);
    }

    private String typeAnnotationFor(Type type) {
        if (type == null) return "";
        return String.format(": %s", doSwitch(type));
    }

    private String indentedStatements(List<EList<? extends EObject>> statementListList) {
        StringBuilder sb = new StringBuilder();
        for (EList<? extends EObject> statementList : statementListList) {
            sb.append(list(statementList, "\n    ", "    ", "\n\n", true));
        }
        return sb.toString();
    }
}
