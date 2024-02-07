package org.lflang.ast;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.emf.ecore.EObject;

import org.lflang.ast.ToSExpr.SExpr;
import org.lflang.ast.ToSExpr.SList.SAtom;
import org.lflang.lf.Action;
import org.lflang.lf.Array;
import org.lflang.lf.ArraySpec;
import org.lflang.lf.Assignment;
import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BracketListExpression;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Code;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Connection;
import org.lflang.lf.Deadline;
import org.lflang.lf.Element;
import org.lflang.lf.Expression;
import org.lflang.lf.Host;
import org.lflang.lf.IPV4Host;
import org.lflang.lf.IPV6Host;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.Literal;
import org.lflang.lf.Method;
import org.lflang.lf.MethodArgument;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
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
import org.lflang.lf.Watchdog;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.lf.util.LfSwitch;

public class ToSExpr extends LfSwitch<SExpr> {

//    private SExpr sList(String name, EObject... parts) {
//        var ret = new ArrayList<SExpr>();
//        ret.add(new SAtom(name));
//        for (EObject part : parts) {
//            ret.add(doSwitch(part));
//        }
//        return new SList(ret);
//    }

    private SExpr sList(String name, Object... parts) {
        var ret = new ArrayList<SExpr>();
        ret.add(new SAtom(name));
        for (var part : parts) {
            if (part instanceof List) {
                throw new IllegalArgumentException("List was not expected");
            }
            if (part instanceof EObject object) {
                ret.add(doSwitch(object));
            } else if (part instanceof SExpr sExpr) {
                ret.add(sExpr);
            } else {
                ret.add(new SAtom(String.valueOf(part)));
            }
        }
        return new SList(ret);
    }

    private <T extends EObject> SExpr sList(String name, List<T> parts) {
        var ret = new ArrayList<SExpr>();
        ret.add(new SAtom(name));
        for (EObject part : parts) {
            ret.add(doSwitch(part));
        }
        return new SList(ret);
    }

//    private SExpr sList(String name, SExpr... parts) {
//        var ret = new ArrayList<SExpr>();
//        ret.add(new SAtom(name));
//        ret.addAll(Arrays.asList(parts));
//        return new SList(ret);
//    }

    @Override
    public SExpr caseModel(Model object) {
//        Model:
//        target=TargetDecl
//            (imports+=Import)*
//            (preambles+=Preamble)*
//            (reactors+=Reactor)+
//        ;
        return sList("model",
            doSwitch(object.getTarget()),
            sList("preambles", object.getPreambles()),
            sList("imports", object.getImports()),
            sList("reactors", object.getReactors())
        );
    }

    @Override
    public SExpr caseImport(Import object) {
//        Import: 'import' reactorClasses+=ImportedReactor (',' reactorClasses+=ImportedReactor)* 'from' importURI=STRING ';'?;
        return sList("import", new SAtom(object.getImportURI()), sList("reactors", object.getReactorClasses()));
    }

    @Override
    public SExpr caseReactorDecl(ReactorDecl object) {
//        ReactorDecl: Reactor | ImportedReactor;
        return sList("reactor-decl",
            object.getName()
            );
    }

    @Override
    public SExpr caseImportedReactor(ImportedReactor object) {
//        ImportedReactor: reactorClass=[Reactor] ('as' name=ID)?;
        return sList("imported-reactor", object.getName(), object.getReactorClass().getName(), object.getReactorClass().hashCode());
    }

    @Override
    public SExpr caseReactor(Reactor object) {
//        Reactor:
//        {Reactor} (attributes+=Attribute)* ((federated?='federated' | main?='main')? & realtime?='realtime'?) 'reactor' (name=ID)?
//            ('<' typeParms+=TypeParm (',' typeParms+=TypeParm)* '>')?
//        ('(' parameters+=Parameter (',' parameters+=Parameter)* ')')?
//        ('at' host=Host)?
//        ('extends' (superClasses+=[ReactorDecl] (',' superClasses+=[ReactorDecl])*))?
//        '{'
//        (     (preambles+=Preamble)
//            | (stateVars+=StateVar)
//            | (methods+=Method)
//            | (inputs+=Input)
//            | (outputs+=Output)
//            | (timers+=Timer)
//            | (actions+=Action)
//            | (watchdogs+=Watchdog)
//            | (instantiations+=Instantiation)
//            | (connections+=Connection)
//            | (reactions+=Reaction)
//            | (modes+=Mode)
//        )* '}';
        return sList("reactor",
            new SAtom(object.getName()),
            new SAtom(String.valueOf(object.hashCode())),
            sList("attributes", object.getAttributes()),
            sList("is-main", object.isMain()),
            sList("is-federated", object.isFederated()),
            sList("is-realtime", object.isRealtime()),
            sList("typeParms", object.getTypeParms()),
            sList("parameters", object.getParameters()),
            sList("host", object.getHost()),
            sList("extends", object.getSuperClasses()),
            sList("preambles", object.getPreambles()),
            sList("state", object.getStateVars()),
            sList("methods", object.getMethods()),
            sList("inputs", object.getInputs()),
            sList("outputs", object.getOutputs()),
            sList("timers", object.getTimers()),
            sList("actions", object.getActions()),
            sList("watchdogs", object.getWatchdogs()),
            sList("instantiations", object.getInstantiations()),
            sList("connections", object.getConnections()),
            sList("reactions", object.getReactions()),
            sList("modes", object.getModes())
            );
    }

    @Override
    public SExpr caseTypeParm(TypeParm object) {
//        TypeParm:
//        literal=TypeExpr | code=Code
        return sList("type-parm", object.getLiteral(), object.getCode());
    }

    @Override
    public SExpr caseTargetDecl(TargetDecl object) {
//        TargetDecl:
//        'target' name=ID (config=KeyValuePairs)? ';'?;
        return sList("target-decl", object.getName(), object.getConfig());
    }

    @Override
    public SExpr caseStateVar(StateVar object) {
//        StateVar:
//        (attributes+=Attribute)*
//            (reset?='reset')? 'state' name=ID
//            (':' type=Type)?
//            init=Initializer?
//                ';'?
//        ;
        return sList("state", object.getName(),
            sList("attributes", object.getAttributes()),
            sList("is-reset", object.isReset()),
            object.getType(), object.getInit());
    }

    @Override
    public SExpr caseInitializer(Initializer object) {
//        Initializer:
//        parens?='(' (exprs+=Expression (','  exprs+=Expression)* (trailingComma?=',')?)? ')'
//            | braces?='{' (exprs+=Expression (','  exprs+=Expression)* (trailingComma?=',')?)? '}'
//            | assign?='=' exprs+=Expression
//        ;
        return sList("initializer",
            sList("exprs", object.getExprs()),
            sList("is-braces", object.isBraces()),
            sList("is-parens", object.isParens()),
            sList("is-assign", object.isAssign()),
            sList("is-trailing-comma", object.isTrailingComma())
        );
    }

    @Override
    public SExpr caseMethod(Method object) {
//    const?='const'? 'method' name=ID
//        '(' (arguments+=MethodArgument (',' arguments+=MethodArgument)*)? ')'
//        (':' return=Type)?
//        code=Code
//        ';'?
        return sList("method",
            object.getName(),
            sList("is-const", object.isConst()),
            sList("arguments", object.getArguments()),
            sList("return", object.getReturn()),
            object.getCode()
            );
    }

    @Override
    public SExpr caseMethodArgument(MethodArgument object) {
//        MethodArgument:
//        name=ID (':' type=Type)?
        return sList("method-argument", object.getName(), sList("type", object.getType()));
    }

    @Override
    public SExpr caseInput(Input object) {
//        Input:
//        (attributes+=Attribute)* mutable?='mutable'? 'input' (widthSpec=WidthSpec)? name=ID (':' type=Type)? ';'?;
        return sList("input",
            object.getName(),
            sList("attributes", object.getAttributes()),
            sList("is-mutable", object.isMutable()),
            object.getWidthSpec(),
            object.getType()
            );
    }

    @Override
    public SExpr caseOutput(Output object) {
//        Output:
//        (attributes+=Attribute)* 'output' (widthSpec=WidthSpec)? name=ID (':' type=Type)? ';'?;
        return sList("output",
            object.getName(),
            sList("attributes", object.getAttributes()),
            object.getWidthSpec(),
            object.getType()
        );
    }

    @Override
    public SExpr caseTimer(Timer object) {
//        Timer:
//        (attributes+=Attribute)* 'timer' name=ID ('(' offset=Expression (',' period=Expression)? ')')? ';'?;
        return sList("timer",
            object.getName(),
            sList("attributes", object.getAttributes()),
            sList("offset", object.getOffset()),
            sList("period", object.getPeriod())
            );
    }

    @Override
    public SExpr caseMode(Mode object) {
//        Mode:
//        {Mode} (initial?='initial')? 'mode' (name=ID)?
//            '{' (
//            (stateVars+=StateVar) |
//                (timers+=Timer) |
//                (actions+=Action) |
//                (watchdogs+=Watchdog) |
//                (instantiations+=Instantiation) |
//                (connections+=Connection) |
//                (reactions+=Reaction)
//        )* '}';
        return sList("mode",
            object.getName(),
            sList("is-initial", object.isInitial()),
            sList("state", object.getStateVars()),
            sList("timers", object.getTimers()),
            sList("actions", object.getActions()),
            sList("watchdogs", object.getWatchdogs()),
            sList("instantiations", object.getInstantiations()),
            sList("connections", object.getConnections()),
            sList("reactions", object.getReactions())
            );
    }

    @Override
    public SExpr caseAction(Action object) {
//        Action:
//        (attributes+=Attribute)*
//            (origin=ActionOrigin)? 'action' name=ID
//            ('(' minDelay=Expression (',' minSpacing=Expression (',' policy=STRING)? )? ')')?
//            (':' type=Type)? ';'?;
        return sList("action",
            object.getName(),
            sList("attributes", object.getAttributes()),
            object.getOrigin(),
            sList("min-delay", object.getMinDelay()),
            sList("min-spacing", object.getMinSpacing()),
            sList("policy", object.getPolicy()),
            object.getType()
            );
    }

    @Override
    public SExpr caseReaction(Reaction object) {
//        Reaction:
//        (attributes+=Attribute)*
//            (('reaction') | mutation ?= 'mutation')
//        (name=ID)?
//            ('(' (triggers+=TriggerRef (',' triggers+=TriggerRef)*)? ')')
//        ( => sources+=VarRef (',' sources+=VarRef)*)?
//        ('->' effects+=VarRefOrModeTransition (',' effects+=VarRefOrModeTransition)*)?
//        (code=Code)? (stp=STP)? (deadline=Deadline)? (delimited?=';')?
//        ;
        return sList(
            "reaction",
            object.getName(),
            sList("attributes", object.getAttributes()),
            sList("is-mutation", object.isMutation()),
            sList("triggers", object.getTriggers()),
            sList("sources", object.getSources()),
            sList("effects", object.getEffects()),
            object.getCode(),
            object.getStp(),
            object.getDeadline(),
            sList("is-delimited", object.isDelimited())
            );
    }

    @Override
    public SExpr caseTriggerRef(TriggerRef object) {
//        TriggerRef:
//        BuiltinTriggerRef | VarRef;
//        return sList("trigger-ref", object.getVariable());
        throw new RuntimeException("not implemented");
    }

    @Override
    public SExpr caseBuiltinTriggerRef(BuiltinTriggerRef object) {
//        BuiltinTriggerRef:
//        type = BuiltinTrigger;
        return sList("builtin-trigger-ref", object.getType());
    }

    @Override
    public SExpr caseDeadline(Deadline object) {
//        Deadline:
//        'deadline' '(' delay=Expression ')' code=Code;
        return sList("deadline", object.getDelay(), object.getCode());
    }

    @Override
    public SExpr caseWatchdog(Watchdog object) {
//        Watchdog:
//        'watchdog' name=ID '(' timeout=Expression ')'
//        ('->' effects+=VarRefOrModeTransition (',' effects+=VarRefOrModeTransition)*)?
//        code=Code;
        return sList("watchdog",
            object.getName(),
            sList("timeout", object.getTimeout()),
            sList("effects", object.getEffects()),
            object.getCode()
            );
    }

    @Override
    public SExpr caseSTP(STP object) {
//        STP:
//        'STP' '(' value=Expression ')' code=Code;
        return sList("stp", object.getValue(), object.getCode());
    }

    @Override
    public SExpr casePreamble(Preamble object) {
//        Preamble:
//        (visibility=Visibility)? 'preamble' code=Code;
        return sList("preamble", object.getVisibility(), object.getCode());
    }

    @Override
    public SExpr caseInstantiation(Instantiation object) {
//        Instantiation:
//        (attributes+=Attribute)*
//            name=ID '=' 'new' (widthSpec=WidthSpec)?
//            reactorClass=[ReactorDecl] ('<' typeArgs+=Type (',' typeArgs+=Type)* '>')? '('
//        (parameters+=Assignment (',' parameters+=Assignment)*)?
//            ')' (('at' host=Host ';') | ';'?);
        return sList("instantiation",
            object.getName(),
            sList("attributes", object.getAttributes()),
            object.getWidthSpec(),
            object.getReactorClass(),
            sList("typeArgs", object.getTypeArgs()),
            sList("parameters", object.getParameters()),
            object.getHost()
            );
    }

    @Override
    public SExpr caseConnection(Connection object) {
//        Connection:
//        ((leftPorts += VarRef (',' leftPorts += VarRef)*)
//            | ( '(' leftPorts += VarRef (',' leftPorts += VarRef)* ')' iterated ?= '+'?))
//        ('->' | physical?='~>')
//        rightPorts += VarRef (',' rightPorts += VarRef)*
//            ('after' delay=Expression)?
//        (serializer=Serializer)?
//            ';'?
        return sList("connection",
            sList("left-ports", object.getLeftPorts()),
            sList("right-ports", object.getRightPorts()),
            sList("is-iterated", object.isIterated()),
            sList("is-physical", object.isPhysical()),
            sList("delay", object.getDelay()),
            object.getSerializer()
            );
    }

    @Override
    public SExpr caseSerializer(Serializer object) {
//        Serializer:
//        'serializer' type=STRING
//        ;
        return sList("serializer", object.getType());
    }

    @Override
    public SExpr caseAttribute(Attribute object) {
//        Attribute:
//        '@' attrName=ID ('(' (attrParms+=AttrParm (',' attrParms+=AttrParm)* ','?)? ')')?
//        ;
        return sList("attribute", object.getAttrName(), sList("attr-parms", object.getAttrParms()));
    }

    @Override
    public SExpr caseAttrParm(AttrParm object) {
//        AttrParm:
//        (name=ID '=')? value=Literal;
        return sList("attr-parm", object.getName(), object.getValue());
    }

    @Override
    public SExpr caseKeyValuePairs(KeyValuePairs object) {
//        KeyValuePairs:
//        {KeyValuePairs} '{' (pairs+=KeyValuePair (',' (pairs+=KeyValuePair))* ','?)? '}';
        return sList("key-value-pairs", object.getPairs());
    }

    @Override
    public SExpr caseKeyValuePair(KeyValuePair object) {
//        KeyValuePair:
//        name=(Kebab|STRING) ':' value=Element;
        return sList("pair", object.getName(), object.getValue());
    }

    @Override
    public SExpr caseArray(Array object) {
//        Array: // todo allow empty array in grammar, replace with validator error
//        '[' elements+=Element (',' (elements+=Element))* ','? ']';
        return sList("array", object.getElements());
    }

    @Override
    public SExpr caseElement(Element object) {
//        Element:
//        keyvalue=KeyValuePairs
//            | array=Array
//            | literal=Literal
//            | (time=INT unit=TimeUnit)
//    | id=Path;
        return sList("element", object.getKeyvalue(), object.getArray(), object.getLiteral(), sList("time", object.getTime(), object.getUnit()), object.getId());
    }

    @Override
    public SExpr caseTypedVariable(TypedVariable object) {
//        TypedVariable:
//        Port | Action
//        ;
        throw new RuntimeException("not implemented");
    }

    @Override
    public SExpr caseVariable(Variable object) {
//        Variable:
//        TypedVariable | Timer | Mode | Watchdog;
        throw new RuntimeException("not implemented");
    }

    @Override
    public SExpr caseVarRef(VarRef object) {
//        VarRef:
//        (variable=[Variable] | container=[Instantiation] '.' variable=[Variable]
//    | interleaved?='interleaved' '(' (variable=[Variable] | container=[Instantiation] '.' variable=[Variable]) ')') ('as' (alias=ID))?
//        ;
        return sList("var-ref", object.getVariable(), object.getContainer(), sList("is-interleaved", object.isInterleaved()), sList("alias", object.getAlias()));
    }

    @Override
    public SExpr caseAssignment(Assignment object) {
//        Assignment:
//        lhs=[Parameter]
//        rhs=AssignmentInitializer
//        ;
        return sList("assignment", object.getLhs(), object.getRhs());
    }

    @Override
    public SExpr caseParameter(Parameter object) {
//        Parameter:
//        (attributes+=Attribute)*
//            name=ID (':' type=Type)?
//            init=Initializer?
//        ;
        return sList("parameter",
            object.getName(),
            sList("attributes", object.getAttributes()),
            object.getType(),
            object.getInit()
            );
    }

    @Override
    public SExpr caseExpression(Expression object) {
//        Expression:
//        {Literal} literal=Literal
//            | Time
//            | ParameterReference
//            | {CodeExpr} code=Code
//            | BracedListExpression
//            | BracketListExpression
//        ;
        throw new RuntimeException("not implemented");
//        return sList("expression", object.getLiteral(), object.getTime(), object.getParameter(), object.getCode(), object.getBracedListExpression(), object.getBracketListExpression());
    }

    @Override
    public SExpr caseBracedListExpression(BracedListExpression object) {
//        BracedListExpression:
//        '{' {BracedListExpression} (items+=Expression (',' items+=Expression)*)? ','? '}'
//        ;
        return sList("braced-list-expression", object.getItems());
    }

    @Override
    public SExpr caseBracketListExpression(BracketListExpression object) {
//        BracketListExpression:
//        '[' {BracketListExpression} (items+=Expression (',' items+=Expression)*)? ','? ']'
//        ;
        return sList("bracket-list-expression", object.getItems());
    }

    @Override
    public SExpr caseParameterReference(ParameterReference object) {
//        ParameterReference:
//        parameter=[Parameter]
//        ;
        return sList("parameter-reference", object.getParameter());
    }

    @Override
    public SExpr caseTime(Time object) {
//        Time:
//        (interval=INT unit=TimeUnit)
//        ;
        return sList("time", object.getInterval(), object.getUnit());
    }

    @Override
    public SExpr casePort(Port object) {
//        Port:
//        Input | Output;
        throw new RuntimeException("not implemented");
    }

    @Override
    public SExpr caseType(Type object) {
//        Type:
//        time?='time' (arraySpec=ArraySpec)?
//   | id=DottedName ('<' typeArgs+=Type (',' typeArgs+=Type)* '>')? (stars+='*')* (arraySpec=ArraySpec)?
//   | code=Code
//        ;
        return sList("type", object.getId(), sList("stars", object.getStars().size()), object.getArraySpec(), object.getCode(), sList("is-time", object.isTime()));
    }

    @Override
    public SExpr caseArraySpec(ArraySpec object) {
//        ArraySpec:
//        '[' ( ofVariableLength?=']' | length=INT ']' );
        return sList("array-spec", sList("length", object.getLength()), sList("is-of-variable-length", object.isOfVariableLength()));
    }

    @Override
    public SExpr caseWidthSpec(WidthSpec object) {
//        WidthSpec:
//        '[' ( ofVariableLength?=']' | (terms+=WidthTerm) ('+' terms+=WidthTerm)* ']' );
        return sList("width-spec", sList("terms", object.getTerms()), sList("is-of-variable-length", object.isOfVariableLength()));
    }

    @Override
    public SExpr caseWidthTerm(WidthTerm object) {
//        WidthTerm:
//        width=INT
//            | parameter=[Parameter]
//    | 'widthof(' port=VarRef ')'
//            | code=Code;
        return sList("width-term", sList("width", object.getWidth()), object.getParameter(), sList("width-of", object.getPort()), object.getCode());
    }

    @Override
    public SExpr caseIPV4Host(IPV4Host object) {
//        IPV4Host:
//        (user=Kebab '@')? addr=IPV4Addr (':' port=INT)?
//        ;
        return sList("ipv4-host", sList("user", object.getUser()), object.getAddr(), sList("port", object.getPort()));
    }

    @Override
    public SExpr caseIPV6Host(IPV6Host object) {
//        IPV6Host:
//        ('[' (user=Kebab '@')? addr=IPV6Addr ']' (':' port=INT)?)
//        ;
        return sList("ipv6-host", sList("user", object.getUser()), object.getAddr(), sList("port", object.getPort()));
    }

    @Override
    public SExpr caseNamedHost(NamedHost object) {
//        NamedHost:
//        (user=Kebab '@')? addr=HostName (':' port=INT)?
//        ;
        return sList("named-host", sList("user", object.getUser()), object.getAddr(), sList("port", object.getPort()));
    }

    @Override
    public SExpr caseHost(Host object) {
//        Host:
//        IPV4Host | IPV6Host | NamedHost
//        ;
        // return super.caseHost(object);
        throw new RuntimeException("not implemented");
    }

    @Override
    public SExpr caseCode(Code object) {
//        Code:
//        // {Code} '{=' (tokens+=Token)* '=}'
//        {Code} '{=' body=Body '=}'
//        ;
        return sList("code", object.getBody());
    }

    @Override
    public SExpr caseLiteral(Literal object) {
//        Literal:
//        STRING | CHAR_LIT | SignedFloat | SignedInt | Boolean
//        ;
        return sList("literal", object.getLiteral());
    }

    @Override
    public SExpr caseCodeExpr(CodeExpr object) {
        return doSwitch(object.getCode());
    }

    @Override
    public SExpr defaultCase(EObject object) {
        throw new RuntimeException("this should be unreachable");
    }

    public interface SExpr {}
    public record SList(List<SExpr> parts) implements SExpr {
        @Override
        public String toString() {
            var ret = new StringBuilder();
            ret.append('(');
            var flat = parts.stream().allMatch(it -> it instanceof SAtom);
            var first = true;
            for (var part : parts) {
                if (!first) {
                    ret.append(',');
                    if (!flat) {
                        ret.append('\n');
                    } else {
                        ret.append(' ');
                    }
                }
                var s = part.toString();
                if (!flat && !first) {
                    s = s.indent(1);
                }
                ret.append(s);
                first = false;
            }
            ret.append(')');
            return ret.toString();
        }
        public record SAtom(String s) implements SExpr {

            @Override
            public String toString() {
                return "\"" + s.replaceAll("\"", "\uD83C\uDCA0") + "\"";
            }
        }
    }
}
