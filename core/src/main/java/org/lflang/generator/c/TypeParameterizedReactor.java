package org.lflang.generator.c;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;
import org.lflang.InferredType;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;

/**
 * A reactor class combined with concrete type arguments bound to its type parameters.
 *
 * @param reactor The syntactic reactor class definition
 * @param typeArgs The type arguments associated with this particular variant of the reactor class.
 */
public record TypeParameterizedReactor(Reactor reactor, Map<String, Type> typeArgs) {

  private static final Map<TypeParameterizedReactor, String> uniqueNames = new HashMap<>();
  private static final Map<String, Integer> nameCounts = new HashMap<>();

  /**
   * Construct the TPR corresponding to the given instantiation which syntactically appears within
   * the definition corresponding to {@code parent}.
   *
   * @param i An instantiation of the TPR to be constructed.
   * @param parent The reactor in which {@code i} appears, or {@code null} if type variables are
   *     permitted instead of types in this TPR.
   */
  public TypeParameterizedReactor(Instantiation i, TypeParameterizedReactor parent) {
    this(
        ASTUtils.toDefinition(i.getReactorClass()),
        addTypeArgs(i, ASTUtils.toDefinition(i.getReactorClass()), parent));
  }

  private static Map<String, Type> addTypeArgs(
      Instantiation instantiation, Reactor r, TypeParameterizedReactor parent) {
    HashMap<String, Type> ret = new HashMap<>();
    if (instantiation.getTypeArgs() != null) {
      for (int i = 0; i < r.getTypeParms().size(); i++) {
        var arg = instantiation.getTypeArgs().get(i);
        ret.put(
            r.getTypeParms().get(i).getLiteral(), parent == null ? arg : parent.resolveType(arg));
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
  public synchronized String uniqueName() {
    String name = reactor.getName().toLowerCase();
    if (uniqueNames.containsKey(this)) return uniqueNames.get(this);
    if (nameCounts.containsKey(name)) {
      int currentCount = nameCounts.get(name);
      nameCounts.put(name, currentCount + 1);
      uniqueNames.put(this, "_" + name + currentCount);
      return uniqueName();
    }
    nameCounts.put(name, 1);
    uniqueNames.put(this, "_" + name);
    return uniqueName();
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
}
