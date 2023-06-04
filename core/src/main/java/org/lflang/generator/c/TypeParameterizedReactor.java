package org.lflang.generator.c;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

import com.google.common.collect.ImmutableMap;
import org.eclipse.emf.common.util.URI;
import org.lflang.InferredType;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.*;

/**
 * A reactor class combined with concrete type arguments bound to its type parameters.
 */
public class TypeParameterizedReactor {
  /** The syntactic reactor class definition. */
  private final Reactor reactor;
  /** The type arguments associated with this particular variant of the reactor class. */
  private final Map<String, Type> typeArgs;
  private final List<String> typeParams;
  private final ImmutableMap<String, Map<URI, Integer>> nameMap;

  /**
   * Construct the TPR corresponding to the given instantiation which syntactically appears within
   * the definition corresponding to {@code parent}.
   *
   * @param i An instantiation of the TPR to be constructed.
   * @param parent The reactor in which {@code i} appears, or {@code null} if type variables are
   *     permitted instead of types in this TPR.
   */
  public TypeParameterizedReactor(Instantiation i, TypeParameterizedReactor parent) {
    this(i, parent, parent.nameMap);
  }

  public TypeParameterizedReactor(Instantiation i, List<Reactor> reactors) {
    this(i, null, getNameMap(reactors));
  }

  private static Map<String, Map<URI, Integer>> getNameMap(List<Reactor> reactors) {
    Map<String, Map<URI, Integer>> nameMap = new HashMap<>();
    Map<String, Integer> countMap = new HashMap<>();
    for (var reactor : reactors) {
      var def = ASTUtils.toDefinition(reactor);
      if (nameMap.containsKey(def.getName())) {
        nameMap.get(def.getName()).put(def.eResource().getURI(), countMap.get(def.getName()));
        countMap.put(def.getName(), countMap.get(def.getName()));
      } else {
        nameMap.put(def.getName(), new HashMap<>());
        nameMap.get(def.getName()).put(def.eResource().getURI(), 0);
        countMap.put(def.getName(), 1);
      }
    }
    return nameMap;
  }

  private String uniqueName(ReactorDecl decl) {
    var name = decl.getName();
    return name + (Objects.requireNonNull(nameMap.get(name)).get(decl.eResource().getURI()) == 0 ? "" : nameMap.get(name));
  }

  private TypeParameterizedReactor(Instantiation i, TypeParameterizedReactor parent, Map<String, Map<URI, Integer>> nameMap) {
    reactor = ASTUtils.toDefinition(i.getReactorClass());
    var definition = ASTUtils.toDefinition(i.getReactorClass());
    typeParams = definition.getTypeParms().stream().map(TypeParm::getLiteral).toList();
    typeArgs = addTypeArgs(i, parent);
    this.nameMap = ImmutableMap.copyOf(nameMap);
  }

  private Map<String, Type> addTypeArgs(
      Instantiation instantiation, TypeParameterizedReactor parent) {
    HashMap<String, Type> ret = new HashMap<>();
    if (instantiation.getTypeArgs() != null) {
      for (int i = 0; i < typeParams.size(); i++) {
        var arg = instantiation.getTypeArgs().get(i);
        ret.put(
            typeParams.get(i), parent == null ? arg : parent.resolveType(arg));
      }
    }
    return ret;
  }

  /** Return the name of the reactor given its type arguments. */
  public String getName() {
    // FIXME: Types that are not just a single token need to be escaped or hashed
    return reactor.getName()
        + typeArgs.values().stream().map(ASTUtils::toOriginalText).collect(Collectors.joining("_"));
  }

  /** #define type names as concrete types. */
  public void doDefines(CodeBuilder b) {
    typeArgs.forEach(
        (literal, concreteType) ->
            b.pr(
                "#if defined "
                    + literal
                    + "\n"
                    + "#undef "
                    + literal
                    + "\n"
                    + "#endif // "
                    + literal
                    + "\n"
                    + "#define "
                    + literal
                    + " "
                    + ASTUtils.toOriginalText(concreteType)));
  }

  /** Resolve type arguments if the given type is defined in terms of generics. */
  public Type resolveType(Type t) {
    if (t.getId() != null && typeArgs.get(t.getId()) != null) return typeArgs.get(t.getId());
    if (t.getCode() == null) return t;
    var arg = typeArgs.get(t.getCode().getBody());
    if (arg != null) return arg;
    return t;
  }

  /** Resolve type arguments if the given type is defined in terms of generics. */
  public InferredType resolveType(InferredType t) {
    if (t.astType == null) return t;
    return InferredType.fromAST(resolveType(t.astType));
  }

  /**
   * Return a name that is unique to this TypeParameterizedReactor (up to structural equality) and
   * that is prefixed with exactly one underscore and that does not contain any upper-case letters.
   */
  public String uniqueName() {
    var resolved = ASTUtils.toDefinition(reactor);
    return "_" + uniqueName(resolved) + typeParams.stream().map(it -> it + "_" + typeArgs.get(it)).collect(Collectors.joining("_"));
  }

  @Override
  public int hashCode() {
    return reactor.hashCode() * 31 + typeArgs.hashCode();
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof TypeParameterizedReactor other
        && reactor.equals(other.reactor)
        && typeArgs.equals(other.typeArgs);
  }

  public Reactor reactor() {
    return reactor;
  }
}
