package org.lflang.ast;

import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
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
import org.lflang.lf.Visibility;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.lf.util.LfSwitch;

/**
 * Switch class for converting AST nodes to their textual representation as
 * it would appear in LF code.
 */
public class ToLf extends LfSwitch<String> {

    /** The number of spaces to prepend to a line per indentation level. */
    private static final int INDENTATION = 4;

    // FIXME: This class needs to respect comments, which are lost at the EObject level of abstraction and must be
    //  obtained using NodeModelUtils like this: https://github.com/lf-lang/lingua-franca/blob/c4dfbd9cebdb9aaf249508360e0bac1ce545458b/org.lflang/src/org/lflang/ASTUtils.java#L1692

    /// public instance initialized when loading the class
    public static final ToLf instance = new ToLf();

    // private constructor
    private ToLf() { super(); }

    @Override
    public String caseArraySpec(ArraySpec spec) {
        return (spec.isOfVariableLength()) ? "[]" : "[" + spec.getLength() + "]";
    }

    @Override
    public String caseCode(Code code) {
        String content = ToText.instance.doSwitch(code);
        if (content.lines().count() > 1 || content.contains("#") || content.contains("//")) {
            return String.format("{=%n%s=}", content.strip().indent(INDENTATION));
        }
        return String.format("{= %s =}", content.strip());
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
        // time?='time' (arraySpec=ArraySpec)?
        // | id=DottedName ('<' typeParms+=Type (',' typeParms+=Type)* '>')? (stars+='*')* (arraySpec=ArraySpec)?
        // | code=Code
        if (type.getCode() != null) return doSwitch(type.getCode());
        StringBuilder sb = new StringBuilder();
        if (type.isTime()) {
            sb.append("time");
        } else if (type.getId() != null) {
            sb.append(type.getId());
            if (type.getTypeParms() != null) {
                sb.append(list(type.getTypeParms(), ", ", "<", ">", true));
            }
            sb.append("*".repeat(type.getStars().size()));
        }
        if (type.getArraySpec() != null) sb.append(doSwitch(type.getArraySpec()));
        return sb.toString();
    }

    @Override
    public String caseTypeParm(TypeParm t) {
        // literal=TypeExpr | code=Code
        return !StringExtensions.isNullOrEmpty(t.getLiteral()) ? t.getLiteral() : doSwitch(t.getCode());
    }

    @Override
    public String caseVarRef(VarRef v) {
        // variable=[Variable] | container=[Instantiation] '.' variable=[Variable]
        // | interleaved?='interleaved' '(' (variable=[Variable] | container=[Instantiation] '.' variable=[Variable]) ')'
        if (!v.isInterleaved()) return ToText.instance.doSwitch(v);
        return String.format("interleaved (%s)", ToText.instance.doSwitch(v));
    }

    @Override
    public String caseModel(Model object) {
        // target=TargetDecl
        // (imports+=Import)*
        // (preambles+=Preamble)*
        // (reactors+=Reactor)+
        StringBuilder sb = new StringBuilder();
        sb.append(caseTargetDecl(object.getTarget())).append(System.lineSeparator().repeat(2));
        object.getImports().forEach(i -> sb.append(caseImport(i)).append(System.lineSeparator()));
        if (!object.getImports().isEmpty()) sb.append(System.lineSeparator());
        object.getPreambles().forEach(
            p -> sb.append(casePreamble(p)).append(System.lineSeparator().repeat(2))
        );
        if (!object.getPreambles().isEmpty()) sb.append(System.lineSeparator());
        sb.append(
             object.getReactors().stream().map(this::doSwitch)
                   .collect(Collectors.joining(System.lineSeparator().repeat(2)))
        ).append(System.lineSeparator());
        return sb.toString();
    }

    @Override
    public String caseImport(Import object) {
        // 'import' reactorClasses+=ImportedReactor (',' reactorClasses+=ImportedReactor)* 'from' importURI=STRING ';'?
        return String.format(
            "import %s from \"%s\"",
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
        if (object.getName() != null) {
            return String.format("%s as %s", object.getReactorClass().getName(), object.getName());
        }
        return object.getReactorClass().getName();
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
        sb.append(reactorHeader(object));
        String smallFeatures = indentedStatements(
            List.of(
                object.getPreambles(),
                object.getInputs(),
                object.getOutputs(),
                object.getTimers(),
                object.getActions(),
                object.getInstantiations(),
                object.getConnections(),
                object.getStateVars()
            ),
            0
        );
        String bigFeatures = indentedStatements(
            List.of(
                object.getReactions(),
                object.getMethods(),
                object.getMutations(),
                object.getModes()
            ),
            1
        );
        sb.append(smallFeatures);
        if (!smallFeatures.isBlank() && !bigFeatures.isBlank()) sb.append(System.lineSeparator());
        sb.append(bigFeatures);
        sb.append("}");
        return sb.toString();
    }

    /** Return the signature of the given reactor. */
    private String reactorHeader(Reactor object) {
        StringBuilder sb = new StringBuilder();
        if (object.isFederated()) sb.append("federated ");
        if (object.isMain()) sb.append("main ");
        if (object.isRealtime()) sb.append("realtime ");
        sb.append("reactor");
        if (object.getName() != null) sb.append(" ").append(object.getName());
        sb.append(list(object.getTypeParms(), ", ", "<", ">", true));
        sb.append(list(object.getParameters()));
        if (object.getHost() != null) sb.append(" at ").append(doSwitch(object.getHost()));
        if (object.getSuperClasses() != null && !object.getSuperClasses().isEmpty()) {
            sb.append(" extends ").append(
                object.getSuperClasses().stream().map(ReactorDecl::getName).collect(Collectors.joining(", "))
            );
        }
        sb.append(String.format(" {%n"));
        return sb.toString();
    }

    @Override
    public String caseTargetDecl(TargetDecl object) {
        // 'target' name=ID (config=KeyValuePairs)? ';'?
        StringBuilder sb = new StringBuilder();
        sb.append("target ").append(object.getName());
        if (object.getConfig() != null) sb.append(" ").append(doSwitch(object.getConfig()));
        return sb.toString();
    }

    @Override
    public String caseStateVar(StateVar object) {
        // 'state' name=ID (
        //     (':' (type=Type))?
        //     ((parens+='(' (init+=Expression (','  init+=Expression)*)? parens+=')')
        //         | (braces+='{' (init+=Expression (','  init+=Expression)*)? braces+='}')
        //     )?
        // ) ';'?
        StringBuilder sb = new StringBuilder();
        sb.append("state ").append(object.getName());
        sb.append(typeAnnotationFor(object.getType()));
        if (!object.getParens().isEmpty()) sb.append(list(object.getInit()));
        if (!object.getBraces().isEmpty()) sb.append(list(object.getInit(), ", ", "{", "}", true));
        return sb.toString();
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
        sb.append("method ").append(object.getName());
        sb.append(list(object.getArguments(), ", ", "(", ")", false));
        sb.append(typeAnnotationFor(object.getReturn())).append(" ").append(doSwitch(object.getCode()));
        return sb.toString();
    }

    @Override
    public String caseMethodArgument(MethodArgument object) {
        // name=ID (':' type=Type)?
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
        sb.append(typeAnnotationFor(object.getType()));
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
        sb.append(String.format("{%n"));
        sb.append(indentedStatements(
            List.of(
                object.getStateVars(),
                object.getTimers(),
                object.getActions(),
                object.getInstantiations(),
                object.getConnections()
            ),
            0
        ));
        sb.append(indentedStatements(
            List.of(object.getReactions()),
            1
        ));
        sb.append("}");
        return sb.toString();
    }

    @Override
    public String caseAction(Action object) {
        // (origin=ActionOrigin)? 'action' name=ID
        // ('(' minDelay=Expression (',' minSpacing=Expression (',' policy=STRING)? )? ')')?
        // (':' type=Type)? ';'?
        StringBuilder sb = new StringBuilder();
        if (object.getOrigin() != null) sb.append(object.getOrigin().getLiteral()).append(" ");
        sb.append("action ");
        sb.append(object.getName());
        if (object.getMinDelay() != null) {
            sb.append("(").append(doSwitch(object.getMinDelay()));
            if (object.getMinSpacing() != null) sb.append(", ").append(doSwitch(object.getMinSpacing()));
            if (object.getPolicy() != null) sb.append(", \"").append(object.getPolicy()).append("\"");
            sb.append(")");
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
        if (!object.getEffects().isEmpty()) {
            sb.append(" ->").append(list(object.getEffects(), ", ", " ", "", true));
        }
        sb.append(" ").append(doSwitch(object.getCode()));
        if (object.getStp() != null) sb.append(" ").append(doSwitch(object.getStp()));
        if (object.getDeadline() != null) sb.append(" ").append(doSwitch(object.getDeadline()));
        return sb.toString();
    }

    @Override
    public String caseTriggerRef(TriggerRef object) {
        // VarRef | startup?='startup' | shutdown?='shutdown'
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
            "%spreamble %s",
            object.getVisibility() != null && object.getVisibility() != Visibility.NONE
                ? object.getVisibility().getLiteral() + " " : "",
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
        sb.append(" ").append(object.getReactorClass().getName());
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
        return String.format("serializer \"%s\"", object.getType());
    }

    @Override
    public String caseKeyValuePairs(KeyValuePairs object) {
        // {KeyValuePairs} '{' (pairs+=KeyValuePair (',' (pairs+=KeyValuePair))* ','?)? '}'
        return list(
            object.getPairs(),
            String.format(",%n    "),
            String.format("{%n    "),
            String.format("%n}"),
            true
        );
    }

    @Override
    public String caseKeyValuePair(KeyValuePair object) {
        // name=Kebab ':' value=Element
        return object.getName() + ": " + doSwitch(object.getValue());
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
        if (object.getKeyvalue() != null) return doSwitch(object.getKeyvalue());
        if (object.getArray() != null) return doSwitch(object.getArray());
        if (object.getLiteral() != null) return object.getLiteral();
        if (object.getId() != null) return object.getId();
        if (object.getUnit() != null) return String.format("%d %s", object.getTime(), object.getUnit());
        return String.valueOf(object.getTime());
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
        sb.append(object.getLhs().getName());
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
            return object.getParameter().getName();
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
        throw new UnsupportedOperationException(String.format(
            "ToText has no case for %s or any of its supertypes, or it does have such a case, but "
                + "the return value of that case was null.",
            object.getClass().getName()
        ));
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

    /**
     * Represent a list of groups of statements.
     * @param statementListList A list of groups of statements.
     * @param extraSeparation Additional vertical separation beyond the bare
     * minimum, to be inserted between everything.
     * @return A string representation of {@code statementListList}.
     */
    private String indentedStatements(List<EList<? extends EObject>> statementListList, int extraSeparation) {
        return statementListList.stream().filter(((Predicate<List<? extends EObject>>) List::isEmpty).negate()).map(
            statementList -> list(
                statementList,
                System.lineSeparator().repeat(1 + extraSeparation),
                "",
                "",
                true
            )
        ).collect(
            Collectors.joining(System.lineSeparator().repeat(2 + extraSeparation), "", "")
        ).indent(INDENTATION);
    }
}
