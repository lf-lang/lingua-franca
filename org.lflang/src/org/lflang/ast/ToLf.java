package org.lflang.ast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.TerminalRule;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.impl.HiddenLeafNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
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

    /// public instance initialized when loading the class
    public static final ToLf instance = new ToLf();

    // private constructor
    private ToLf() { super(); }

    @Override
    public MalleableString caseArraySpec(ArraySpec spec) {
        if (spec.isOfVariableLength()) return MalleableString.anyOf("[]");
        return list("", "[", "]", false, false, spec.getLength());
    }

    @Override
    public MalleableString doSwitch(EObject eObject) {
        ICompositeNode node = NodeModelUtils.findActualNodeFor(eObject);
        Stream<String> followingComments = getFollowingComments(node);
        List<String> immediatelyPrecedingComments = ASTUtils.getPrecedingComments(
            node,
            false,
            sameLine(node)
        );
        var previous = getNextCompositeSibling(node, INode::getPreviousSibling, true);
        Predicate<INode> doesNotBelongToPrevious = sameLine(node).negate().and(
            previous == null ? n -> true : sameLine(previous).negate()
        );
        Stream<String> singleLinePrecedingComments = ASTUtils.getPrecedingComments(
            node,
           true,
           doesNotBelongToPrevious
        ).stream().map(String::trim);
        Stream<String> multilinePrecedingComments = ASTUtils.getPrecedingComments(
            node,
            false,
            doesNotBelongToPrevious
        ).stream().map(
            it -> it.lines()
                    .map(String::strip)
                    .map(trimmed -> trimmed.startsWith("*") ? " " + trimmed : trimmed)
                    .collect(Collectors.joining(System.lineSeparator()))
        );
        MalleableString representation = super.doSwitch(eObject);
        representation
            .addComments(singleLinePrecedingComments)
            .addComments(multilinePrecedingComments)
            .addComments(immediatelyPrecedingComments)
            .addComments(followingComments);
        return representation;
    }

    private ICompositeNode getNextCompositeSibling(
        INode node,
        Function<INode, INode> getNextSibling,
        boolean traverseUpwards
    ) {
        INode sibling = node;
        while ((sibling = getNextSibling.apply(sibling)) != null) {
            if (sibling instanceof ICompositeNode compositeSibling) return compositeSibling;
        }
        if (node.getParent() != null && traverseUpwards) {
            return getNextCompositeSibling(node.getParent(), getNextSibling, true);
        }
        return null;
    }

    private Stream<INode> getFollowingNonCompositeSiblings(ICompositeNode node) {
        INode sibling = node;
        List<INode> ret = new ArrayList<>();
        while (
            (sibling = sibling.getNextSibling()) != null
                && !(sibling instanceof ICompositeNode)
        ) {
            ret.add(sibling);
        }
        return ret.stream();
    }

    private Stream<String> getFollowingComments(ICompositeNode node) {
        ICompositeNode sibling = getNextCompositeSibling(node, INode::getNextSibling, false);
        Stream<String> followingSiblingComments = getFollowingNonCompositeSiblings(node)
            .filter(
                otherSibling -> otherSibling instanceof HiddenLeafNode hlNode
                    && hlNode.getGrammarElement() instanceof TerminalRule tRule
                    && tRule.getName().endsWith("COMMENT")
            ).map(INode::getText);
        if (sibling == null) return followingSiblingComments;
        Predicate<INode> filter = sameLine(node).and(sameLine(sibling).negate());
        return Stream.concat(followingSiblingComments, Stream.concat(
            ASTUtils.getPrecedingComments(sibling, false, filter).stream(),
            ASTUtils.getPrecedingComments(sibling, true, filter).stream()
        ));
    }

    private Predicate<INode> sameLine(INode node) {
        return other -> node.getStartLine() <= other.getStartLine()
            && other.getStartLine() <= node.getEndLine();
    }

    @Override
    public MalleableString caseCode(Code code) {
        String content = ToText.instance.doSwitch(code).lines()
            .map(String::stripTrailing)
            .collect(Collectors.joining(System.lineSeparator()));
        String singleLineRepresentation = String.format("{= %s =}", content.strip());
        String multilineRepresentation = new Builder()
            .append(String.format("{=%n"))
            .append(MalleableString.anyOf(content).indent(FormattingUtils.INDENTATION))
            .append("=}")
            .get().render().rendering();
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
        // | id=DottedName ('<' typeParms+=Type (',' typeParms+=Type)* '>')? (stars+='*')*
        //     (arraySpec=ArraySpec)?
        // | code=Code
        if (type.getCode() != null) return doSwitch(type.getCode());
        Builder msb = new Builder();
        if (type.isTime()) {
            msb.append("time");
        } else if (type.getId() != null) {
            msb.append(type.getId());  // TODO: Multiline dottedName?
            if (type.getTypeParms() != null) {
                msb.append(list(", ", "<", ">", true, false, type.getTypeParms()));
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
        // | interleaved?='interleaved' '(' (variable=[Variable] | container=[Instantiation] '.'
        //     variable=[Variable]) ')'
        if (!v.isInterleaved()) return MalleableString.anyOf(ToText.instance.doSwitch(v));
        return new Builder()
            .append("interleaved ")
            .append(list(false, ToText.instance.doSwitch(v)))
            .get();
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
        msb.append(
             object.getReactors().stream().map(this::doSwitch)
                   .collect(new Joiner(System.lineSeparator().repeat(2)))
        ).append(System.lineSeparator());
        return msb.get();
    }

    @Override
    public MalleableString caseImport(Import object) {
        // 'import' reactorClasses+=ImportedReactor (',' reactorClasses+=ImportedReactor)*
        //     'from' importURI=STRING ';'?
        // TODO: Break this. Break string, break at whitespace outside the string.
        return new Builder()
            .append("import ")
            // TODO: This is a place where we can use conditional parentheses.
            .append(list(", ", "", "", false, true, object.getReactorClasses()))
            .append(" from \"")
            .append(object.getImportURI())
            .append("\"")
            .get();
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
        // {Reactor} ((federated?='federated' | main?='main')? & realtime?='realtime'?)
        //     'reactor' (name=ID)?
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
        if (!smallFeatures.isEmpty() && !bigFeatures.isEmpty()) {
            msb.append(System.lineSeparator().repeat(1));
        }
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
        msb.append(list(", ", "<", ">", true, false, object.getTypeParms()));
        msb.append(list(true, object.getParameters()));
        if (object.getHost() != null) msb.append(" at ").append(doSwitch(object.getHost()));
        if (object.getSuperClasses() != null && !object.getSuperClasses().isEmpty()) {
            msb.append(
                MalleableString.anyOf(" extends "),
                new Builder()
                    .append(System.lineSeparator())
                    .append(
                        MalleableString.anyOf("extends ").indent(FormattingUtils.INDENTATION * 2)
                    )
                    .get()
               )
               .append(
                object.getSuperClasses().stream()
                    .map(ReactorDecl::getName)
                    .collect(Collectors.joining(", "))
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
        if (object.getConfig() != null && !object.getConfig().getPairs().isEmpty()) {
            msb.append(" ").append(doSwitch(object.getConfig()));
        }
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
        if (!object.getParens().isEmpty()) msb.append(list(true, object.getInit()));
        if (!object.getBraces().isEmpty()) {
            msb.append(list(", ", "{", "}", true, false, object.getInit()));
        }
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
        msb.append(list(false, object.getArguments()));
        msb.append(typeAnnotationFor(object.getReturn()))
            .append(" ")
            .append(doSwitch(object.getCode()));
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
        return new Builder()
            .append("timer ")
            .append(object.getName())
            .append(list(true, object.getOffset(), object.getPeriod()))
            .get();
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
        return msb.append("action ")
            .append(object.getName())
            .append(list(
                true,
                object.getMinDelay(),
                object.getMinSpacing(),
                object.getPolicy() != null ? String.format("\"%s\"", object.getPolicy()) : null
            ))
            .append(typeAnnotationFor(object.getType()))
            .get();
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
        msb.append(list(true, object.getTriggers()));
        msb.append(list(", ", " ", "", true, false, object.getSources()));
        if (!object.getEffects().isEmpty()) {
            msb.append(" ->").append(list(", ", " ", "", true, false, object.getEffects()));
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
            .append("deadline")
            .append(list(false, object.getDelay()))
            .append(" ")
            .append(caseCode(object.getCode()))
            .get();
    }

    @Override
    public MalleableString caseSTP(STP object) {
        // 'STP' '(' value=Expression ')' code=Code
        return new Builder()
            .append("STP")
            // TODO: Address redundancy with caseDeadline?
            .append(list(false, object.getValue()))
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
        return new Builder()
            .append("mutation").append(list(true, object.getTriggers()))
            .get();
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
        msb.append(list(", ", "<", ">", true, false, object.getTypeParms()));
        msb.append(list(false, object.getParameters()));
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
        if (object.isIterated()) {
            msb.append(list(false, object.getLeftPorts())).append("+");
        } else {
            msb.append(
                object.getLeftPorts().stream().map(this::doSwitch).collect(new Joiner(", ")),
                object.getLeftPorts().stream().map(this::doSwitch).collect(
                    new Joiner(String.format(",%n"))
                )
            );
        }
        msb.append(
            "",
            MalleableString.anyOf(System.lineSeparator()).indent(FormattingUtils.INDENTATION)
        );
        msb.append(object.isPhysical() ? " ~> " : " -> ");
        msb.append(minimallyDelimitedList(object.getRightPorts()));
        if (object.getDelay() != null) msb.append(" after ").append(doSwitch(object.getDelay()));
        if (object.getSerializer() != null) {
            msb.append(" ").append(doSwitch(object.getSerializer()));
        }
        return msb.get();
    }

    private MalleableString minimallyDelimitedList(List<? extends EObject> items) {
        return MalleableString.anyOf(
            list(", ", "", "", true, true, items),
            new Builder()
                .append(System.lineSeparator())
                .append(
                    list(String.format(",%n"), "", "", true, true, items)
                        .indent(FormattingUtils.INDENTATION)
                ).append(String.format("%n;")).get()
        );
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
        return list(", ", "{ ", " }", true, false, object.getPairs());
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
        return list(", ", "[", "]", false, false, object.getElements());
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
        String prefix = "";
        String suffix = "";
        if (!object.getParens().isEmpty()) {
            prefix = "(";
            suffix = ")";
        } else if (!object.getBraces().isEmpty()) {
            prefix = "{";
            suffix = "}";
        }
        msb.append(list(", ", prefix, suffix, false, prefix.isBlank(), object.getRhs()));
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
                ", ",
                object.getBraces().isEmpty() ? "(" : "{",
                object.getBraces().isEmpty() ? ")" : "}",
                true,
                false,
                object.getInit()
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
        return list(" + ", "[", "]", false, false, object.getTerms());
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
            return new Builder()
                .append("widthof")
                .append(list(false, object.getPort()))
                .get();
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
     * Represent the given EList as a string.
     * @param suffix The token marking the end of the list.
     * @param separator The separator separating elements of the list.
     * @param prefix The token marking the start of the list.
     * @param nothingIfEmpty Whether the result should be simplified to the
     * empty string as opposed to just the prefix and suffix.
     * @param whitespaceRigid Whether any whitespace appearing in the
     */
    private <E extends EObject> MalleableString list(
        String separator,
        String prefix,
        String suffix,
        boolean nothingIfEmpty,
        boolean whitespaceRigid,
        List<E> items
    ) {
        return list(
            separator,
            prefix,
            suffix,
            nothingIfEmpty,
            whitespaceRigid,
            (Object[]) items.toArray(EObject[]::new)
        );
    }

    private MalleableString list(
        String separator,
        String prefix,
        String suffix,
        boolean nothingIfEmpty,
        boolean whitespaceRigid,
        Object... items
    ) {
        if (nothingIfEmpty && Arrays.stream(items).allMatch(Objects::isNull)) {
            return MalleableString.anyOf("");
        }
        MalleableString rigid = Arrays.stream(items).sequential()
            .filter(Objects::nonNull)
            .map(it -> {
                if (it instanceof MalleableString ms) return ms;
                if (it instanceof EObject eObject) return doSwitch(eObject);
                return MalleableString.anyOf(Objects.toString(it));
            }).collect(new Joiner(separator, prefix, suffix));
        if (whitespaceRigid) return rigid;
        return MalleableString.anyOf(
            rigid,
            new Builder()
                .append(prefix.stripTrailing() + System.lineSeparator())
                .append(list(
                    separator.strip() + System.lineSeparator(),
                    "",
                    System.lineSeparator(),
                    nothingIfEmpty,
                    true,
                    items
                ).indent(FormattingUtils.INDENTATION))
                .append(suffix.stripLeading())
                .get()
        );
    }

    private <E extends EObject> MalleableString list(boolean nothingIfEmpty, EList<E> items) {
        return list(", ", "(", ")", nothingIfEmpty, false, items);
    }

    private MalleableString list(boolean nothingIfEmpty, Object... items) {
        return list(", ", "(", ")", nothingIfEmpty, false, items);
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
    private MalleableString indentedStatements(
        List<EList<? extends EObject>> statementListList,
        int extraSeparation
    ) {
        return statementListList.stream()
            .filter(list -> !list.isEmpty())
            .map(statementList -> list(
                System.lineSeparator().repeat(1 + extraSeparation),
                "",
                System.lineSeparator(),
                true,
                true,
                statementList
            )
        ).collect(
            new Joiner(
                System.lineSeparator().repeat(1 + extraSeparation),
                "",
                ""
            )
        ).indent(FormattingUtils.INDENTATION);
    }
}
