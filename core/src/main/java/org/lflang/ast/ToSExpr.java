package org.lflang.ast;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.util.LineAndColumn;
import org.lflang.ast.ToSExpr.SExpr;
import org.lflang.ast.ToSExpr.SList.Fingerprint;
import org.lflang.ast.ToSExpr.SList.Metadata;
import org.lflang.ast.ToSExpr.SList.SAtom;
import org.lflang.ast.ToSExpr.SList.Sym;
import org.lflang.generator.Position;
import org.lflang.generator.Range;
import org.lflang.lf.Action;
import org.lflang.lf.Array;
import org.lflang.lf.Assignment;
import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BracketListExpression;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.CStyleArraySpec;
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
import org.lflang.lf.MaxWait;
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

/**
 * Converts an LF model to an S-expression.
 *
 * @ingroup Utilities
 */
public class ToSExpr extends LfSwitch<SExpr> {

  /**
   * The eObjects in the syntax tree on the path from the root up to and including the current
   * eObject.
   */
  private final List<EObject> callStack = new ArrayList<>();

  @Override
  public SExpr doSwitch(EObject theEObject) {
    callStack.add(theEObject);
    var ret = doSwitchHelper(theEObject);
    callStack.remove(callStack.size() - 1);
    return ret;
  }

  private <T> boolean inside(Class<T> tClass) {
    return callStack.size() >= 2 && tClass.isInstance(callStack.get(callStack.size() - 2));
  }

  private boolean inVarRef() {
    // uses variable, typedvariable, timer, mode, watchdog, port, input, output, action
    return inside(VarRef.class);
  }

  //  private boolean inReaction() {
  //    // uses triggerref
  //    return inside(Reaction.class);
  //  }

  private SExpr doSwitchHelper(EObject theEObject) {
    var node = NodeModelUtils.getNode(theEObject);
    var range = getLfRange(theEObject);
    var metadata =
        new Metadata(
            theEObject.eResource().getURI(),
            node.getTotalOffset(),
            range.getStartInclusive().getZeroBasedLine(),
            range.getStartInclusive().getZeroBasedColumn(),
            range.getEndExclusive().getZeroBasedLine(),
            range.getEndExclusive().getZeroBasedColumn());
    var ret = super.doSwitch(theEObject);
    ret.setMetadata(metadata);
    return ret;
  }

  private static Range getLfRange(EObject astNode) {
    final INode node = NodeModelUtils.getNode(astNode);
    final LineAndColumn oneBasedLfLineAndColumn =
        NodeModelUtils.getLineAndColumn(node, node.getTotalOffset());
    Position lfStart =
        Position.fromOneBased(
            oneBasedLfLineAndColumn.getLine(), oneBasedLfLineAndColumn.getColumn());
    return new Range(lfStart, lfStart.plus(node.getText()));
  }

  private SExpr sym(String name) {
    return new SAtom<>(new Sym(name));
  }

  private SExpr fingerprint(EObject eObject) {
    try {
      var md = MessageDigest.getInstance("SHA");
      var range =
          getLfRange(
              eObject); // No two distinct syntactic elements can have the exact same source code
      // range
      var locBuffer = ByteBuffer.allocate(16); // 4 32-bit ints
      locBuffer.putInt(range.getStartInclusive().getZeroBasedLine());
      locBuffer.putInt(range.getStartInclusive().getZeroBasedColumn());
      locBuffer.putInt(range.getEndExclusive().getZeroBasedLine());
      locBuffer.putInt(range.getEndExclusive().getZeroBasedColumn());
      md.update(locBuffer.array());
      md.update(eObject.eResource().getURI().toString().getBytes(StandardCharsets.UTF_8));
      md.update(
          NodeModelUtils.getNode(eObject)
              .getText()
              .getBytes(
                  StandardCharsets
                      .UTF_8)); // FIXME: this is probably redundant, but I am leaving it in for
      // good measure until we validate this machinery more rigorously
      return new SAtom<>(new Fingerprint(md.digest()));
    } catch (NoSuchAlgorithmException e) {
      throw new RuntimeException("should be impossible; SHA is a supported algorithm");
    }
  }

  private SExpr sList(String name, Object... parts) {
    var ret = new ArrayList<SExpr>();
    ret.add(sym(name));
    for (var part : parts) {
      if (part == null) {
        continue;
      }
      if (part instanceof List) {
        throw new IllegalArgumentException("List was not expected");
      }
      if (part instanceof EObject object) {
        ret.add(doSwitch(object));
      } else if (part instanceof SExpr sExpr) {
        ret.add(sExpr);
      } else {
        ret.add(new SAtom<>(part));
      }
    }
    return new SList(ret);
  }

  private <T extends EObject> SExpr sList(String name, List<T> parts) {
    var ret = new ArrayList<SExpr>();
    ret.add(sym(name));
    for (EObject part : parts) {
      ret.add(doSwitch(part));
    }
    if (ret.size() == 1) {
      return null;
    }
    return new SList(ret);
  }

  @Override
  public SExpr caseModel(Model object) {
    //        Model:
    //        target=TargetDecl
    //            (imports+=Import)*
    //            (preambles+=Preamble)*
    //            (reactors+=Reactor)+
    //        ;
    return sList(
        "model",
        doSwitch(object.getTarget()),
        sList("imports", object.getImports()),
        sList("preambles", object.getPreambles()),
        sList("reactors", object.getReactors()));
  }

  @Override
  public SExpr caseImport(Import object) {
    //        Import: 'import' reactorClasses+=ImportedReactor (','
    // reactorClasses+=ImportedReactor)* 'from' importURI=STRING ';'?;
    return sList(
        "import",
        new SAtom<>(
            object.getImportURI() != null ? object.getImportURI() : object.getImportPackage()),
        sList("reactors", object.getReactorClasses()));
  }

  @Override
  public SExpr caseReactorDecl(ReactorDecl object) {
    //        ReactorDecl: Reactor | ImportedReactor;
    return sList("reactor-decl", object.getName());
  }

  @Override
  public SExpr caseImportedReactor(ImportedReactor object) {
    //        ImportedReactor: reactorClass=[Reactor] ('as' name=ID)?;
    return sList(
        "imported-reactor",
        object.getName(),
        fingerprint(object),
        object.getReactorClass().getName(),
        fingerprint(object.getReactorClass()));
  }

  @Override
  public SExpr caseReactor(Reactor object) {
    //        Reactor:
    //        {Reactor} (attributes+=Attribute)* ((federated?='federated' | main?='main')? &
    // realtime?='realtime'?) 'reactor' (name=ID)?
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
    return sList(
        "reactor",
        new SAtom<>(object.getName()),
        fingerprint(object),
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
        sList("modes", object.getModes()));
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
    return sList(
        "state",
        object.getName(),
        sList("attributes", object.getAttributes()),
        sList("is-reset", object.isReset()),
        object.getType(),
        object.getInit());
  }

  @Override
  public SExpr caseInitializer(Initializer object) {
    //        Initializer:
    //        parens?='(' (exprs+=Expression (','  exprs+=Expression)* (trailingComma?=',')?)? ')'
    //            | braces?='{' (exprs+=Expression (','  exprs+=Expression)* (trailingComma?=',')?)?
    // '}'
    //            | assign?='=' exprs+=Expression
    //        ;
    return sList(
        "initializer", sList("expr", object.getExpr()), sList("is-assign", object.isAssign()));
  }

  @Override
  public SExpr caseMethod(Method object) {
    //    const?='const'? 'method' name=ID
    //        '(' (arguments+=MethodArgument (',' arguments+=MethodArgument)*)? ')'
    //        (':' return=Type)?
    //        code=Code
    //        ';'?
    return sList(
        "method",
        object.getName(),
        sList("is-const", object.isConst()),
        sList("arguments", object.getArguments()),
        sList("return", object.getReturn()),
        object.getCode());
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
    //        (attributes+=Attribute)* mutable?='mutable'? 'input' (widthSpec=WidthSpec)? name=ID
    // (':' type=Type)? ';'?;
    if (inVarRef()) {
      return sList("input", object.getName(), fingerprint(object));
    }
    return sList(
        "input",
        object.getName(),
        fingerprint(object),
        sList("attributes", object.getAttributes()),
        sList("is-mutable", object.isMutable()),
        object.getWidthSpec(),
        object.getType());
  }

  @Override
  public SExpr caseOutput(Output object) {
    //        Output:
    //        (attributes+=Attribute)* 'output' (widthSpec=WidthSpec)? name=ID (':' type=Type)?
    // ';'?;
    if (inVarRef()) {
      return sList("output", object.getName(), fingerprint(object));
    }
    return sList(
        "output",
        object.getName(),
        fingerprint(object),
        sList("attributes", object.getAttributes()),
        object.getWidthSpec(),
        object.getType());
  }

  @Override
  public SExpr caseTimer(Timer object) {
    //        Timer:
    //        (attributes+=Attribute)* 'timer' name=ID ('(' offset=Expression (','
    // period=Expression)? ')')? ';'?;
    if (inVarRef()) {
      return sList("timer", object.getName());
    }
    return sList(
        "timer",
        object.getName(),
        fingerprint(object),
        sList("attributes", object.getAttributes()),
        sList("offset", object.getOffset()),
        sList("period", object.getPeriod()));
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
    if (inVarRef()) {
      return sList("mode", object.getName(), fingerprint(object));
    } else {
      return sList(
          "mode",
          object.getName(),
          fingerprint(object),
          sList("is-initial", object.isInitial()),
          sList("state", object.getStateVars()),
          sList("timers", object.getTimers()),
          sList("actions", object.getActions()),
          sList("watchdogs", object.getWatchdogs()),
          sList("instantiations", object.getInstantiations()),
          sList("connections", object.getConnections()),
          sList("reactions", object.getReactions()));
    }
  }

  @Override
  public SExpr caseAction(Action object) {
    //        Action:
    //        (attributes+=Attribute)*
    //            (origin=ActionOrigin)? 'action' name=ID
    //            ('(' minDelay=Expression (',' minSpacing=Expression (',' policy=STRING)? )? ')')?
    //            (':' type=Type)? ';'?;
    if (inVarRef()) {
      return sList("action", object.getName(), fingerprint(object));
    }
    return sList(
        "action",
        object.getName(),
        fingerprint(object),
        sList("attributes", object.getAttributes()),
        object.getOrigin(),
        sList("min-delay", object.getMinDelay()),
        sList("min-spacing", object.getMinSpacing()),
        sList("policy", object.getPolicy()),
        object.getType());
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
    //        (code=Code)? (maxwait=MaxWait)? (deadline=Deadline)? (delimited?=';')?
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
        object.getMaxWait(),
        object.getDeadline(),
        sList("is-delimited", object.isDelimited()));
  }

  @Override
  public SExpr caseTriggerRef(TriggerRef object) {
    //        TriggerRef:
    //        BuiltinTriggerRef | VarRef;
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
    if (inVarRef()) {
      return sList("watchdog", object.getName(), fingerprint(object));
    }
    return sList(
        "watchdog",
        object.getName(),
        fingerprint(object),
        sList("timeout", object.getTimeout()),
        sList("effects", object.getEffects()),
        object.getCode());
  }

  @Override
  public SExpr caseMaxWait(MaxWait object) {
    //        maxwait:
    //        'maxwait' ()'(' value=Expression ')')? code=Code;
    return sList("maxwait", object.getValue(), object.getCode());
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
    return sList(
        "instantiation",
        object.getName(),
        sList("attributes", object.getAttributes()),
        object.getWidthSpec(),
        sList("reactor", object.getReactorClass().getName(), fingerprint(object.getReactorClass())),
        sList("typeArgs", object.getTypeArgs()),
        sList("parameters", object.getParameters()),
        object.getHost());
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
    return sList(
        "connection",
        sList("left-ports", object.getLeftPorts()),
        sList("right-ports", object.getRightPorts()),
        sList("is-iterated", object.isIterated()),
        sList("is-physical", object.isPhysical()),
        sList("delay", object.getDelay()),
        object.getSerializer());
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
    //            | time=Time
    //            | id=Path;
    return sList(
        "element",
        object.getKeyvalue(),
        object.getArray(),
        object.getLiteral(),
        object.getTime() == null
            ? null
            : object.getTime().getForever() != null
                ? "forever"
                : object.getTime().getNever() != null
                    ? "never"
                    : sList("time", object.getTime().getInterval(), object.getTime().getUnit()),
        object.getId());
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
    //    | interleaved?='interleaved' '(' (variable=[Variable] | container=[Instantiation] '.'
    // variable=[Variable]) ')') ('as' (alias=ID))?
    //        ;
    return sList(
        "var-ref",
        object.getVariable(),
        object.getContainer(),
        sList("is-interleaved", object.isInterleaved()),
        sList("alias", object.getAlias()));
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
    return sList(
        "parameter",
        object.getName(),
        sList("attributes", object.getAttributes()),
        object.getType(),
        object.getInit());
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
    //        return sList("expression", object.getLiteral(), object.getTime(),
    // object.getParameter(), object.getCode(), object.getBracedListExpression(),
    // object.getBracketListExpression());
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
    //   | id=DottedName ('<' typeArgs+=Type (',' typeArgs+=Type)* '>')? (stars+='*')*
    // (arraySpec=ArraySpec)?
    //   | code=Code
    //        ;
    return sList(
        "type",
        object.getId(),
        sList("stars", object.getStars().size()),
        object.getCStyleArraySpec(),
        object.getCode(),
        sList("is-time", object.isTime()));
  }

  @Override
  public SExpr caseCStyleArraySpec(CStyleArraySpec object) {
    //        ArraySpec:
    //        '[' ( ofVariableLength?=']' | length=INT ']' );
    return sList(
        "array-spec",
        sList("length", object.getLength()),
        sList("is-of-variable-length", object.isOfVariableLength()));
  }

  @Override
  public SExpr caseWidthSpec(WidthSpec object) {
    //        WidthSpec:
    //        '[' ( ofVariableLength?=']' | (terms+=WidthTerm) ('+' terms+=WidthTerm)* ']' );
    return sList(
        "width-spec",
        sList("terms", object.getTerms()),
        sList("is-of-variable-length", object.isOfVariableLength()));
  }

  @Override
  public SExpr caseWidthTerm(WidthTerm object) {
    //        WidthTerm:
    //        width=INT
    //            | parameter=[Parameter]
    //    | 'widthof(' port=VarRef ')'
    //            | code=Code;
    return sList(
        "width-term",
        sList("width", object.getWidth()),
        object.getParameter(),
        sList("width-of", object.getPort()),
        object.getCode());
  }

  @Override
  public SExpr caseIPV4Host(IPV4Host object) {
    //        IPV4Host:
    //        (user=Kebab '@')? addr=IPV4Addr (':' port=INT)?
    //        ;
    return sList(
        "ipv4-host",
        sList("user", object.getUser()),
        object.getAddr(),
        sList("port", object.getPort()));
  }

  @Override
  public SExpr caseIPV6Host(IPV6Host object) {
    //        IPV6Host:
    //        ('[' (user=Kebab '@')? addr=IPV6Addr ']' (':' port=INT)?)
    //        ;
    return sList(
        "ipv6-host",
        sList("user", object.getUser()),
        object.getAddr(),
        sList("port", object.getPort()));
  }

  @Override
  public SExpr caseNamedHost(NamedHost object) {
    //        NamedHost:
    //        (user=Kebab '@')? addr=HostName (':' port=INT)?
    //        ;
    return sList(
        "named-host",
        sList("user", object.getUser()),
        object.getAddr(),
        sList("port", object.getPort()));
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

  public abstract static class SExpr {
    private Optional<Metadata> m = Optional.empty();

    protected abstract String display();

    public void setMetadata(Metadata m) {
      //      noinspection StatementWithEmptyBody
      if (this.m.isPresent()) {
        // It is actually possible for doSwitch to be
        // invoked twice, so this can be called redundantly
        // in non-error cases. Don't ask me why. It creates
        // redundant work, but this hasn't been shown to a
        // performance bottleneck, so it's fine.
        // throw new IllegalStateException("tried to set the metadata twice for object " +
        // display());
      } else {
        this.m = Optional.of(m);
      }
    }

    @Override
    public String toString() {
      return m.map(
              metadata -> String.format("(%s\n%s)", metadata, display().indent(1).stripTrailing()))
          .orElseGet(
              () -> {
                var indented = display().indent(1);
                return String.format("(()%s)", indented.stripTrailing());
              });
    }
  }

  public class SList extends SExpr {
    private final List<SExpr> parts;

    public SList(List<SExpr> parts) {
      this.parts = parts;
    }

    protected String display() {
      var ret = new StringBuilder();
      ret.append('(');
      var flat = parts.stream().allMatch(it -> it instanceof SAtom);
      var first = true;
      for (var part : parts) {
        if (!first) {
          if (!flat) {
            ret.append('\n');
          } else {
            ret.append(' ');
          }
        }
        var s = part.toString();
        if (!flat && !first) {
          s = s.indent(1).stripTrailing();
        }
        ret.append(s);
        first = false;
      }
      ret.append(')');
      return ret.toString();
    }

    public static class SAtom<T> extends SExpr {
      private final T x;

      public SAtom(T x) {
        this.x = x;
      }

      public String display() {
        if (x instanceof String s) {
          return "\"" + s.replaceAll("\"", "\uD83C\uDCA0") + "\"";
        } else if (x instanceof Boolean b) {
          return b ? "#t" : "#f";
        } else {
          return String.valueOf(x);
        }
      }
    }

    public record Sym(String s) {
      @Override
      public String toString() {
        return s;
      }
    }

    public record Fingerprint(byte[] digest) {
      @Override
      public String toString() {
        var ret = new StringBuilder();
        ret.append("#u8(");
        var first = true;
        for (byte c : digest) {
          if (!first) {
            ret.append(' ');
          }
          ret.append(Byte.toUnsignedInt(c));
          first = false;
        }
        ret.append(")");
        return ret.toString();
      }
    }

    public record Metadata(
        URI uri, int offset, int startLine, int startCol, int endLine, int endCol) {
      @Override
      public String toString() {
        return String.format(
            "(\"%s\" %d %d %d %d %d)",
            uri.toFileString(), offset, startLine, startCol, endLine, endCol);
      }
    }
  }
}
