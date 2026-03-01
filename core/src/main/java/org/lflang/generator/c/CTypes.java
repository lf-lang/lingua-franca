package org.lflang.generator.c;

import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.lflang.InferredType;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TargetTypes;
import org.lflang.lf.ParameterReference;

/**
 * C-specific type information.
 *
 * @ingroup Generator
 */
public class CTypes implements TargetTypes {

  // Regular expression pattern for array types.
  // For example, for "foo[10]", the first match will be "foo" and the second "[10]".
  // For "foo[]", the first match will be "foo" and the second "".
  static final Pattern arrayPattern =
      Pattern.compile("^\\s*(?:/\\*.*?\\*/)?\\s*(\\w+)\\s*\\[([0-9]*)]\\s*$");
  private static final CTypes INSTANCE = new CTypes();

  public CTypes() {}

  @Override
  public boolean supportsGenerics() {
    return false;
  }

  @Override
  public String getTargetTimeType() {
    return "interval_t";
  }

  @Override
  public String getTargetTagType() {
    return "tag_t";
  }

  @Override
  public String getTargetUndefinedType() {
    return "/*undefined*/";
  }

  /**
   * Given a type, return a C representation of the type. Note that C types are very idiosyncratic.
   * For example, `int[]` is not always accepted as a type, and `int*` must be used
   * instead, except when initializing a variable using a static initializer, as in `int[] foo
   * = {1, 2, 3`;}. When initializing a variable using a static initializer, use {@link
   * #getVariableDeclaration(TypeParameterizedReactor, InferredType, String, boolean)} instead.
   *
   * @param type The type.
   */
  @Override
  public String getTargetType(InferredType type) {
    var result = TargetTypes.super.getTargetType(type);
    Matcher matcher = arrayPattern.matcher(result);
    if (matcher.find()) {
      return matcher.group(1) + '*';
    }
    return result;
  }

  @Override
  public String getTargetParamRef(ParameterReference expr, InferredType typeOrNull) {
    throw new UnsupportedOperationException("No context defined");
  }

  @Override
  public String getTargetTimeExpr(TimeValue time) {
    if (time.unit != null) {
      return cMacroName(time.unit) + "(" + time.getMagnitude() + ")";
    } else {
      return Long.valueOf(time.getMagnitude()).toString();
    }
  }

  /**
   * Return a variable declaration of the form "`type name`". The type is as returned by
   * {@link #getTargetType(InferredType)}, except with the array syntax `[size]` preferred
   * over the pointer syntax `*` (if applicable). This also includes the variable name because
   * C requires the array type specification to be placed after the variable name rather than with
   * the type. The result is of the form `type variable_name[size]` or `type
   * variable_name` depending on whether the given type is an array type, unless the array type has
   * no size (it is given as `[]`. In that case, the returned form depends on the third
   * argument, initializer. If true, the then the returned declaration will have the form
   * `type variable_name[]`, and otherwise it will have the form `type* variable_name`.
   *
   * @param tpr The type-parameterized reactor.
   * @param type The type.
   * @param variableName The name of the variable.
   * @param initializer True to return a form usable in a static initializer.
   */
  public String getVariableDeclaration(
      TypeParameterizedReactor tpr, InferredType type, String variableName, boolean initializer) {
    String t = TargetTypes.super.getTargetType(tpr.resolveType(type));
    Matcher matcher = arrayPattern.matcher(t);
    String declaration = String.format("%s %s", t, variableName);
    if (matcher.find()) {
      // For array types, we have to move the []
      // because C is very picky about where this goes. It has to go
      // after the variable name. Also, in an initializer, it has to have
      // form [], and in a struct definition, it has to use *.
      if (matcher.group(2).equals("") && !initializer) {
        declaration = String.format("%s* %s", matcher.group(1), variableName);
      } else {
        declaration = String.format("%s %s[%s]", matcher.group(1), variableName, matcher.group(2));
      }
    }
    return declaration;
  }

  // note that this is moved out by #544
  public static String cMacroName(TimeUnit unit) {
    return unit.getCanonicalName().toUpperCase();
  }

  public static CTypes getInstance() {
    return INSTANCE;
  }

  public static CTypes generateParametersIn(ReactorInstance instance) {
    return new CTypes() {
      @Override
      public String getTargetParamRef(ParameterReference expr, InferredType typeOrNull) {
        return CUtil.reactorRef(instance) + "->" + expr.getParameter().getName();
      }
    };
  }
}
