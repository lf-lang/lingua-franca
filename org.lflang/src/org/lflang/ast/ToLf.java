package org.lflang.ast;

import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.xbase.lib.StringExtensions;

import org.lflang.ASTUtils;
import org.lflang.ast.MalleableString.Builder;
import org.lflang.ast.MalleableString.Joiner;
import org.lflang.lf.Action;
import org.lflang.lf.Array;
import org.lflang.lf.ArraySpec;
import org.lflang.lf.Assignment;
import org.lflang.lf.BuiltinTriggerRef;
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
public class ToLf extends LfSwitch<MalleableString> {

    /** The number of spaces to prepend to a line per indentation level. */
    private static final int INDENTATION = 4;

    /// public instance initialized when loading the class
    public static final ToLf instance = new ToLf();

    // private constructor
    private ToLf() { super(); }

    @Override
    public MalleableString caseArraySpec(ArraySpec spec) {
        return MalleableString.anyOf(
            spec.isOfVariableLength() ? "[]" : "[" + spec.getLength() + "]"
            // TODO: Multiline arrayspec
        );
    }

    @Override
    public MalleableString caseCode(Code code) {
        String content = ToText.instance.doSwitch(code);
        String singleLineRepresentation = String.format("{= %s =}", content.strip());
        String multilineRepresentation = String.format(
            "{=%n%s=}",
            content.strip().indent(INDENTATION)
        );
        if (content.lines().count() > 1 || content.contains("#") || content.contains("//")) {
            return MalleableString.anyOf(multilineRepresentation);
        }
        return MalleableString.anyOf(singleLineRepresentation, multilineRepresentation);
    }

    @Override
    public MalleableString caseHost(Host host) {
        Builder msb = new Builder();
        if (!StringExtensions.isNullOrEmpty(host.getUser())) {
            msb.append(host.getUser()).append("@");
        }
        if (!StringExtensions.isNullOrEmpty(host.getAddr())) {
            msb.append(host.getAddr());
        }
        if (host.getPort() != 0) {
            msb.append(":").append(host.getPort());
        }
        return msb.get();
    }

    @Override
    public MalleableString caseLiteral(Literal l) {
        // STRING | CHAR_LIT | SignedFloat | SignedInt | Boolean
        return MalleableString.anyOf(l.getLiteral());
    }

    @Override
    public MalleableString caseParameterReference(ParameterReference p) {
        // parameter=[Parameter]
        return MalleableString.anyOf(p.getParameter().getName());
    }

    @Override
    public MalleableString caseTime(Time t) {
        // (interval=INT unit=TimeUnit)
        return MalleableString.anyOf(ASTUtils.toTimeValue(t).toString());
    }

    @Override
    public MalleableString caseType(Type type) {
        // time?='time' (arraySpec=ArraySpec)?
        // | id=DottedName ('<' typeParms+=Type (',' typeParms+=Type)* '>')? (stars+='*')* (arraySpec=ArraySpec)?
        // | code=Code
        if (type.getCode() != null) return doSwitch(type.getCode());
        Builder msb = new Builder();
        if (type.isTime()) {
            msb.append("time");
        } else if (type.getId() != null) {
            msb.append(type.getId());  // TODO: Multiline dottedName?
            if (type.getTypeParms() != null) {
                msb.append(list(type.getTypeParms(), ", ", "<", ">", true));
            }
            msb.append("*".repeat(type.getStars().size()));
        }
        if (type.getArraySpec() != null) msb.append(doSwitch(type.getArraySpec()));
        return msb.get();
    }

    @Override
    public MalleableString caseTypeParm(TypeParm t) {
        // literal=TypeExpr | code=Code
        return MalleableString.anyOf(
            !StringExtensions.isNullOrEmpty(t.getLiteral()) ?
            MalleableString.anyOf(t.getLiteral()) : doSwitch(t.getCode())
        );
    }

    @Override
    public MalleableString caseVarRef(VarRef v) {
        // variable=[Variable] | container=[Instantiation] '.' variable=[Variable]
        // | interleaved?='interleaved' '(' (variable=[Variable] | container=[Instantiation] '.' variable=[Variable]) ')'
        if (!v.isInterleaved()) return MalleableString.anyOf(ToText.instance.doSwitch(v));
        // TODO: Break in parens after interleaved?
        return MalleableString.anyOf(String.format("interleaved (%s)", ToText.instance.doSwitch(v)));
    }

    @Override
    public MalleableString caseModel(Model object) {
        // target=TargetDecl
        // (imports+=Import)*
        // (preambles+=Preamble)*
        // (reactors+=Reactor)+
        Builder msb = new Builder();
        msb.append(caseTargetDecl(object.getTarget())).append(System.lineSeparator().repeat(2));
        object.getImports().forEach(i -> msb.append(caseImport(i)).append(System.lineSeparator()));
        if (!object.getImports().isEmpty()) msb.append(System.lineSeparator());
        object.getPreambles().forEach(
            p -> msb.append(casePreamble(p)).append(System.lineSeparator().repeat(2))
        );
        if (!object.getPreambles().isEmpty()) msb.append(System.lineSeparator());
        msb.append(
             object.getReactors().stream().map(this::doSwitch)
                   .collect(new Joiner(System.lineSeparator().repeat(2)))
        ).append(System.lineSeparator());
        return msb.get();
    }

    @Override
    public MalleableString caseImport(Import object) {
        // 'import' reactorClasses+=ImportedReactor (',' reactorClasses+=ImportedReactor)* 'from' importURI=STRING ';'?
        // TODO: Break this. Break string, break at whitespace outside the string.
        return MalleableString.anyOf(String.format(
            "import %s from \"%s\"",
            list(object.getReactorClasses(), ", ", "", "", false),
            object.getImportURI()
        ));
    }

    @Override
    public MalleableString caseReactorDecl(ReactorDecl object) {
        // Reactor | ImportedReactor
        return defaultCase(object);
    }

    @Override
    public MalleableString caseImportedReactor(ImportedReactor object) {
        // reactorClass=[Reactor] ('as' name=ID)?
        if (object.getName() != null) {
            return MalleableString.anyOf(String.format(
                "%s as %s",
                object.getReactorClass().getName(),
                object.getName()
            ));
        }
        return MalleableString.anyOf(object.getReactorClass().getName());
    }

    @Override
    public MalleableString caseReactor(Reactor object) {
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
        Builder msb = new Builder();
        msb.append(reactorHeader(object));
        MalleableString smallFeatures = indentedStatements(
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
        MalleableString bigFeatures = indentedStatements(
            List.of(
                object.getReactions(),
                object.getMethods(),
                object.getMutations(),
                object.getModes()
            ),
            1
        );
        msb.append(smallFeatures);
        if (!smallFeatures.isEmpty() && !bigFeatures.isEmpty()) msb.append(System.lineSeparator());
        msb.append(bigFeatures);
        msb.append("}");
        return msb.get();
    }

    /** Return the signature of the given reactor. */
    private MalleableString reactorHeader(Reactor object) {
        Builder msb = new Builder();
        if (object.isFederated()) msb.append("federated ");
        if (object.isMain()) msb.append("main ");
        if (object.isRealtime()) msb.append("realtime ");
        msb.append("reactor");
        if (object.getName() != null) msb.append(" ").append(object.getName());
        msb.append(list(object.getTypeParms(), ", ", "<", ">", true));
        msb.append(list(object.getParameters()));
        if (object.getHost() != null) msb.append(" at ").append(doSwitch(object.getHost()));
        if (object.getSuperClasses() != null && !object.getSuperClasses().isEmpty()) {
            // TODO: Break here
            msb.append(" extends ").append(
                object.getSuperClasses().stream().map(ReactorDecl::getName).collect(Collectors.joining(", "))
            );
        }
        msb.append(String.format(" {%n"));
        return msb.get();
    }

    @Override
    public MalleableString caseTargetDecl(TargetDecl object) {
        // 'target' name=ID (config=KeyValuePairs)? ';'?
        Builder msb = new Builder();
        msb.append("target ").append(object.getName());
        if (object.getConfig() != null) msb.append(" ").append(doSwitch(object.getConfig()));
        return msb.get();
    }

    @Override
    public MalleableString caseStateVar(StateVar object) {
        // 'state' name=ID (
        //     (':' (type=Type))?
        //     ((parens+='(' (init+=Expression (','  init+=Expression)*)? parens+=')')
        //         | (braces+='{' (init+=Expression (','  init+=Expression)*)? braces+='}')
        //     )?
        // ) ';'?
        Builder msb = new Builder();
        msb.append("state ").append(object.getName());
        msb.append(typeAnnotationFor(object.getType()));
        if (!object.getParens().isEmpty()) msb.append(list(object.getInit()));
        if (!object.getBraces().isEmpty()) msb.append(list(object.getInit(), ", ", "{", "}", true));
        return msb.get();
    }

    @Override
    public MalleableString caseMethod(Method object) {
        // const?='const'? 'method' name=ID
        // '(' (arguments+=MethodArgument (',' arguments+=MethodArgument)*)? ')'
        // (':' return=Type)?
        // code=Code
        // ';'?
        Builder msb = new Builder();
        if (object.isConst()) msb.append("const ");
        msb.append("method ").append(object.getName());
        msb.append(list(object.getArguments(), ", ", "(", ")", false));
        msb.append(typeAnnotationFor(object.getReturn())).append(" ").append(doSwitch(object.getCode()));
        return msb.get();
    }

    @Override
    public MalleableString caseMethodArgument(MethodArgument object) {
        // name=ID (':' type=Type)?
        return MalleableString.anyOf(object.getName() + typeAnnotationFor(object.getType()));
    }

    @Override
    public MalleableString caseInput(Input object) {
        // mutable?='mutable'? 'input' (widthSpec=WidthSpec)? name=ID (':' type=Type)? ';'?
        Builder msb = new Builder();
        if (object.isMutable()) msb.append("mutable ");
        msb.append("input");
        if (object.getWidthSpec() != null) msb.append(doSwitch(object.getWidthSpec()));
        msb.append(" ").append(object.getName()).append(typeAnnotationFor(object.getType()));
        return msb.get();
    }

    @Override
    public MalleableString caseOutput(Output object) {
        // 'output' (widthSpec=WidthSpec)? name=ID (':' type=Type)? ';'?
        Builder msb = new Builder();
        msb.append("output");
        if (object.getWidthSpec() != null) msb.append(doSwitch(object.getWidthSpec()));
        msb.append(" ").append(object.getName());
        msb.append(typeAnnotationFor(object.getType()));
        return msb.get();
    }

    @Override
    public MalleableString caseTimer(Timer object) {
        // 'timer' name=ID ('(' offset=Expression (',' period=Expression)? ')')? ';'?
        Builder msb = new Builder();
        msb.append("timer ").append(object.getName());
        if (object.getOffset() != null) {
            // TODO: Break this param list
            msb.append("(");
            msb.append(doSwitch(object.getOffset()));
            if (object.getPeriod() != null) msb.append(", ").append(doSwitch(object.getPeriod()));
            msb.append(")");
        }
        return msb.get();
    }

    @Override
    public MalleableString caseMode(Mode object) {
        // {Mode} (initial?='initial')? 'mode' (name=ID)?
        // '{' (
        //     (stateVars+=StateVar) |
        //     (timers+=Timer) |
        //     (actions+=Action) |
        //     (instantiations+=Instantiation) |
        //     (connections+=Connection) |
        //     (reactions+=Reaction)
        // )* '}'
        Builder msb = new Builder();
        if (object.isInitial()) msb.append("initial ");
        msb.append("mode ");
        if (object.getName() != null) msb.append(object.getName()).append(" ");
        msb.append(String.format("{%n"));
        msb.append(indentedStatements(
            List.of(
                object.getStateVars(),
                object.getTimers(),
                object.getActions(),
                object.getInstantiations(),
                object.getConnections()
            ),
            0
        ));
        msb.append(indentedStatements(
            List.of(object.getReactions()),
            1
        ));
        msb.append("}");
        return msb.get();
    }

    @Override
    public MalleableString caseAction(Action object) {
        // (origin=ActionOrigin)? 'action' name=ID
        // ('(' minDelay=Expression (',' minSpacing=Expression (',' policy=STRING)? )? ')')?
        // (':' type=Type)? ';'?
        Builder msb = new Builder();
        if (object.getOrigin() != null) msb.append(object.getOrigin().getLiteral()).append(" ");
        msb.append("action ");
        msb.append(object.getName());
        if (object.getMinDelay() != null) {
            // TODO: break this
            msb.append("(").append(doSwitch(object.getMinDelay()));
            if (object.getMinSpacing() != null) msb.append(", ").append(doSwitch(object.getMinSpacing()));
            if (object.getPolicy() != null) msb.append(", \"").append(object.getPolicy()).append("\"");
            msb.append(")");
        }
        msb.append(typeAnnotationFor(object.getType()));
        return msb.get();
    }

    @Override
    public MalleableString caseReaction(Reaction object) {
        // ('reaction')
        // ('(' (triggers+=TriggerRef (',' triggers+=TriggerRef)*)? ')')?
        // (sources+=VarRef (',' sources+=VarRef)*)?
        // ('->' effects+=VarRefOrModeTransition (',' effects+=VarRefOrModeTransition)*)?
        // code=Code
        // (stp=STP)?
        // (deadline=Deadline)?
        Builder msb = new Builder();
        msb.append("reaction");
        msb.append(list(object.getTriggers()));
        msb.append(list(object.getSources(), ", ", " ", "", true));
        if (!object.getEffects().isEmpty()) {
            msb.append(" ->").append(list(object.getEffects(), ", ", " ", "", true));
        }
        msb.append(" ").append(doSwitch(object.getCode()));
        if (object.getStp() != null) msb.append(" ").append(doSwitch(object.getStp()));
        if (object.getDeadline() != null) msb.append(" ").append(doSwitch(object.getDeadline()));
        return msb.get();
    }

    @Override
    public MalleableString caseTriggerRef(TriggerRef object) {
        // BuiltinTriggerRef | VarRef
        throw new UnsupportedOperationException(
            "TriggerRefs are BuiltinTriggerRefs or VarRefs, so the methods "
                + "corresponding to those types should be invoked instead.");
    }

    @Override
    public MalleableString caseBuiltinTriggerRef(BuiltinTriggerRef object) {
        // type = BuiltinTrigger
        return MalleableString.anyOf(object.getType().getLiteral());
    }

    @Override
    public MalleableString caseDeadline(Deadline object) {
        // 'deadline' '(' delay=Expression ')' code=Code
        return new Builder()
            .append("deadline(")// TODO: line break here
            .append(doSwitch(object.getDelay()))
            .append(") ")
            .append(caseCode(object.getCode()))
            .get();
    }

    @Override
    public MalleableString caseSTP(STP object) {
        // 'STP' '(' value=Expression ')' code=Code
        return new Builder()
            .append("STP(") // TODO: Line break here. Also address redundancy with caseDeadline
            .append(doSwitch(object.getValue()))
            .append(") ")
            .append(doSwitch(object.getCode()))
            .get();
    }

    @Override
    public MalleableString caseMutation(Mutation object) {
        // ('mutation')
        // ('(' (triggers+=TriggerRef (',' triggers+=TriggerRef)*)? ')')?
        // (sources+=VarRef (',' sources+=VarRef)*)?
        // ('->' effects+=[VarRef] (',' effects+=[VarRef])*)?
        // code=Code
        Builder msb = new Builder();
        msb.append("mutation");
        if (!object.getTriggers().isEmpty()) {
            // TODO: use list method here
            msb.append(object.getTriggers().stream().map(this::doSwitch).collect(
                new Joiner(", ", "(", ")")
            ));
        }
        return msb.get();
    }

    @Override
    public MalleableString casePreamble(Preamble object) {
        // (visibility=Visibility)? 'preamble' code=Code
        Builder msb = new Builder();
        if (object.getVisibility() != null && object.getVisibility() != Visibility.NONE) {
            msb.append(object.getVisibility().getLiteral()).append(" ");
        }
        return msb.append("preamble ").append(doSwitch(object.getCode())).get();
    }

    @Override
    public MalleableString caseInstantiation(Instantiation object) {
        // name=ID '=' 'new' (widthSpec=WidthSpec)?
        // reactorClass=[ReactorDecl] ('<' typeParms+=TypeParm (',' typeParms+=TypeParm)* '>')? '('
        // (parameters+=Assignment (',' parameters+=Assignment)*)?
        // ')' ('at' host=Host)? ';'?;
        Builder msb = new Builder();
        msb.append(object.getName()).append(" = new");
        if (object.getWidthSpec() != null) msb.append(doSwitch(object.getWidthSpec()));
        msb.append(" ").append(object.getReactorClass().getName());
        if (!object.getTypeParms().isEmpty()) {
            // TODO: Use list method
            msb.append(object.getTypeParms().stream().map(this::doSwitch).collect(
                new Joiner(", ", "<", ">"))
            );
        }
        // TODO: Use list method
        msb.append(object.getParameters().stream().map(this::doSwitch).collect(
            new Joiner(", ", "(", ")"))
        );
        // TODO: Delete the following case when the corresponding feature is removed
        if (object.getHost() != null) msb.append(" at ").append(doSwitch(object.getHost()));
        return msb.get();
    }

    @Override
    public MalleableString caseConnection(Connection object) {
        // ((leftPorts += VarRef (',' leftPorts += VarRef)*)
        //     | ( '(' leftPorts += VarRef (',' leftPorts += VarRef)* ')' iterated ?= '+'?))
        // ('->' | physical?='~>')
        // rightPorts += VarRef (',' rightPorts += VarRef)*
        //     ('after' delay=Expression)?
        // (serializer=Serializer)?
        // ';'?
        Builder msb = new Builder();
        // TODO: break lines here
        if (object.isIterated()) msb.append("(");
        msb.append(object.getLeftPorts().stream().map(this::doSwitch).collect(new Joiner(", ")));
        if (object.isIterated()) msb.append(")+");
        msb.append(object.isPhysical() ? " ~> " : " -> ");
        // TODO: break lines here
        msb.append(object.getRightPorts().stream().map(this::doSwitch).collect(new Joiner(", ")));
        if (object.getDelay() != null) msb.append(" after ").append(doSwitch(object.getDelay()));
        if (object.getSerializer() != null) msb.append(" ").append(doSwitch(object.getSerializer()));
        return msb.get();
    }

    @Override
    public MalleableString caseSerializer(Serializer object) {
        // 'serializer' type=STRING
        return new Builder()
            .append("serializer \"")
            .append(object.getType())
            .append("\"")
            .get();
    }

    @Override
    public MalleableString caseKeyValuePairs(KeyValuePairs object) {
        // {KeyValuePairs} '{' (pairs+=KeyValuePair (',' (pairs+=KeyValuePair))* ','?)? '}'
        return list(object.getPairs(), ", ", "{", "}", true);
    }

    @Override
    public MalleableString caseKeyValuePair(KeyValuePair object) {
        // name=Kebab ':' value=Element
        return new Builder()
            .append(object.getName())
            .append(": ")
            .append(doSwitch(object.getValue()))
            .get();
    }

    @Override
    public MalleableString caseArray(Array object) {
        // '[' elements+=Element (',' (elements+=Element))* ','? ']'
        // TODO: Use list method? and break line
        return object.getElements().stream().map(this::doSwitch).collect(
            new Joiner(", ", "[", "]")
        );
    }

    @Override
    public MalleableString caseElement(Element object) {
        // keyvalue=KeyValuePairs
        // | array=Array
        // | literal=Literal
        // | (time=INT unit=TimeUnit)
        // | id=Path
        if (object.getKeyvalue() != null) return doSwitch(object.getKeyvalue());
        if (object.getArray() != null) return doSwitch(object.getArray());
        if (object.getLiteral() != null) return MalleableString.anyOf(object.getLiteral());
        if (object.getId() != null) return MalleableString.anyOf(object.getId());
        if (object.getUnit() != null) return MalleableString.anyOf(
            String.format("%d %s", object.getTime(), object.getUnit())
        );
        return MalleableString.anyOf(String.valueOf(object.getTime()));
    }

    @Override
    public MalleableString caseTypedVariable(TypedVariable object) {
        // Port | Action
        return defaultCase(object);
    }

    @Override
    public MalleableString caseVariable(Variable object) {
        // TypedVariable | Timer | Mode
        return defaultCase(object);
    }

    @Override
    public MalleableString caseAssignment(Assignment object) {
        // (lhs=[Parameter] (
        //     (equals='=' rhs+=Expression)
        //     | ((equals='=')? (
        //         parens+='(' (rhs+=Expression (',' rhs+=Expression)*)? parens+=')'
        //         | braces+='{' (rhs+=Expression (',' rhs+=Expression)*)? braces+='}'))
        // ));
        Builder msb = new Builder();
        msb.append(object.getLhs().getName());
        if (object.getEquals() != null) msb.append(" = ");
        // TODO: use list method
        if (!object.getParens().isEmpty()) msb.append("(");
        if (!object.getBraces().isEmpty()) msb.append("{");
        msb.append(object.getRhs().stream()
            .map(this::doSwitch)
            .collect(new Joiner(", ", "", "")));
        if (!object.getParens().isEmpty()) msb.append(")");
        if (!object.getBraces().isEmpty()) msb.append("}");
        return msb.get();
    }

    @Override
    public MalleableString caseParameter(Parameter object) {
        // name=ID (':' (type=Type))?
        // ((parens+='(' (init+=Expression (','  init+=Expression)*)? parens+=')')
        // | (braces+='{' (init+=Expression (','  init+=Expression)*)? braces+='}')
        // )?
        return new Builder()
            .append(object.getName())
            .append(typeAnnotationFor(object.getType()))
            .append(list(
                object.getInit(),
                ", ",
                object.getBraces().isEmpty() ? "(" : "{",
                object.getBraces().isEmpty() ? ")" : "}",
                true
            ))
            .get();
    }

    @Override
    public MalleableString caseExpression(Expression object) {
        // {Literal} literal = Literal
        // | Time
        // | ParameterReference
        // | Code
        return defaultCase(object);
    }

    @Override
    public MalleableString casePort(Port object) {
        // Input | Output
        return defaultCase(object);
    }

    @Override
    public MalleableString caseWidthSpec(WidthSpec object) {
        // ofVariableLength?='[]' | '[' (terms+=WidthTerm) ('+' terms+=WidthTerm)* ']';
        if (object.isOfVariableLength()) return MalleableString.anyOf("[]");
        return list(object.getTerms(), " + ", "[", "]", false);
    }

    @Override
    public MalleableString caseWidthTerm(WidthTerm object) {
        // width=INT
        // | parameter=[Parameter]
        // | 'widthof(' port=VarRef ')'
        // | code=Code;
        if (object.getWidth() != 0) {
            return MalleableString.anyOf(object.getWidth());
        } else if (object.getParameter() != null) {
            return MalleableString.anyOf(object.getParameter().getName());
        } else if (object.getPort() != null) {
            // TODO: break line here
            return new Builder().append("widthof(").append(object.getPort()).append(")").get();
        } else if (object.getCode() != null) {
            return doSwitch(object.getCode());
        }
        throw new IllegalArgumentException("A WidthTerm should not be totally nullish/zeroish.");
    }

    @Override
    public MalleableString caseIPV4Host(IPV4Host object) {
        // (user=Kebab '@')? addr=IPV4Addr (':' port=INT)?
        return caseHost(object);
    }

    @Override
    public MalleableString caseIPV6Host(IPV6Host object) {
        // ('[' (user=Kebab '@')? addr=IPV6Addr ']' (':' port=INT)?)
        return caseHost(object);
    }

    @Override
    public MalleableString caseNamedHost(NamedHost object) {
        // (user=Kebab '@')? addr=HostName (':' port=INT)?
        return caseHost(object);
    }

    @Override
    public MalleableString defaultCase(EObject object) {
        throw new UnsupportedOperationException(String.format(
            "ToText has no case for %s or any of its supertypes, or it does have such a case, but "
                + "the return value of that case was null.",
            object.getClass().getName()
        ));
    }

    /**
     * Wrap a multi-line String based on the current state of lineWrap and indentLvl.
     * If lineWrap is 0, returns originalString. Otherwise, breaks lines such that if possible, each line has less than
     * (lineWrap - (indentLvl * INDENTATION) % lineWrap) characters, only breaking at spaces.
     * @param originalString The String to wrap.
     * @return The wrapped String.
     */
    private String wrapLines(String originalString, int lineWrap, int indentLevel) {
        if (lineWrap == 0) return originalString;
        return originalString.lines().map(s -> wrapIndividualLine(s, lineWrap, indentLevel))
            .collect(Collectors.joining(System.lineSeparator()));
    }

    /**
     * Wrap a single line of String based on the current state of lineWrap and indentLvl. See wrapLines().
     * @param line The line to wrap.
     * @return The wrapped line.
     */
    private String wrapIndividualLine(String line, int lineWrap, int indentLevel) {
        int wrapLength = lineWrap - indentLevel * INDENTATION % lineWrap;
        StringBuilder sb = new StringBuilder();
        while (line.length() > wrapLength) {
            // try to wrap at space
            int index = line.lastIndexOf(' ', wrapLength);
            // if unable to find space in limit, extend to the first space we find
            if (index == -1) index = line.indexOf(' ', wrapLength);
            if (index != -1 && line.substring(0, index).isBlank()) {
                // never break out an all-whitespace line unless it is longer than wrapLength
                if(index < wrapLength) {
                    index = line.indexOf(' ', wrapLength);
                } else {
                    // large indent - don't skip over any space so that indents are consistent
                    sb.append(line, 0, index).append(System.lineSeparator());
                    line = line.substring(index);
                    continue;
                }
            }
            if (index != -1) {
                sb.append(line, 0, index).append(System.lineSeparator());
                line = line.substring(index + 1);
            } else {
                // no spaces remaining at all
                sb.append(line);
                line = "";
            }
        }
        if (line.length() > 0) sb.append(line);
        return sb.toString();
    }

    /**
     * Represent the given EList as a string.
     * @param delimiter The delimiter separating elements of the list.
     * @param prefix The token marking the start of the list.
     * @param suffix The token marking the end of the list.
     * @param nothingIfEmpty Whether the result should be simplified to the
     * empty string as opposed to just the prefix and suffix.
     */
    private <E extends EObject> MalleableString list(
        List<E> items,
        String delimiter,
        String prefix,
        String suffix,
        boolean nothingIfEmpty
    ) {
        if (nothingIfEmpty && items.isEmpty()) return MalleableString.anyOf("");
        return items.stream().map(this::doSwitch).collect(new Joiner(delimiter, prefix, suffix));
    }

    private <E extends EObject> MalleableString list(EList<E> items) {
        return list(items, ", ", "(", ")", true);
    }

    private MalleableString typeAnnotationFor(Type type) {
        if (type == null) return MalleableString.anyOf("");
        return new Builder().append(": ").append(doSwitch(type)).get();
    }

    /**
     * Represent a list of groups of statements.
     * @param statementListList A list of groups of statements.
     * @param extraSeparation Additional vertical separation beyond the bare
     * minimum, to be inserted between everything.
     * @return A string representation of {@code statementListList}.
     */
    private MalleableString indentedStatements(List<EList<? extends EObject>> statementListList, int extraSeparation) {
        return statementListList.stream().filter(((Predicate<List<? extends EObject>>) List::isEmpty).negate()).map(
            statementList -> list(
                statementList,
                System.lineSeparator().repeat(1 + extraSeparation),
                "",
                "",
                true
            )
        ).collect(
            new Joiner(System.lineSeparator().repeat(2 + extraSeparation), "", "")
        ).indent(INDENTATION);
    }
}
