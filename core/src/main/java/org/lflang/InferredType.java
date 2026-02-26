package org.lflang;

import java.util.function.Function;
import org.eclipse.emf.ecore.EObject;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Type;

/**
 * A helper class that represents an inferred type.
 *
 * <p>This class helps to separate the rules of type inference from code generation for types. It is
 * particularly useful in cases when the type is not given directly in LF code, but is inferred from
 * the context. In this case it could happen that no ASTNode representing the type does not exist.
 * For instance when a parameter type is inferred from a time value. All in all, this class provides
 * a clean interface between type inference in ASTUtils and code generation.
 *
 * <p>ASTUtils provides functionality to create an inferred type from Lingua Franca variables
 * (getInferredType). This inferred type can than be translated to target code using the code
 * generators or be converted to a general textual representation using toText().
 *
 * @author Christian Menard
 * @ingroup Utilities
 */
public class InferredType {

  /** The AST node representing the inferred type if such a node exists. */
  public final Type astType;

  /** A flag indicating whether the inferred type has the base type time. */
  public final boolean isTime;

  /** Private constructor */
  private InferredType(Type astType, boolean isTime) {
    this.astType = astType;
    this.isTime = isTime;
  }

  /** Check if the inferred type is undefined. */
  public boolean isUndefined() {
    return astType == null && !isTime;
  }

  /**
   * Convert the inferred type to its textual representation as it would appear in LF code, with
   * CodeMap tags inserted.
   */
  public String toText() {
    return toTextHelper(ASTUtils::toText);
  }

  /**
   * Convert the inferred type to its textual representation as it would appear in LF code, without
   * CodeMap tags inserted.
   */
  public String toOriginalText() {
    return toTextHelper(ASTUtils::toOriginalText);
  }

  private String toTextHelper(Function<EObject, String> toText) {
    if (astType != null) {
      return toText.apply(astType);
    } else if (isTime) {
      return "time";
    }
    return "";
  }

  /**
   * Convert the inferred type to its textual representation while ignoring any list qualifiers or
   * type arguments.
   *
   * @return Textual representation of this inferred type without list qualifiers
   */
  public String baseType() {
    if (astType != null) {
      return ASTUtils.baseType(astType);
    } else if (isTime) {
      return "time";
    }
    return "";
  }

  /**
   * Create an inferred type from an AST node.
   *
   * @param type an AST node
   * @return A new inferred type representing the given AST node
   */
  public static InferredType fromAST(Type type) {
    if (type == null) {
      return undefined();
    }
    return new InferredType(type, type.isTime());
  }

  /** Create an undefined inferred type. */
  public static InferredType undefined() {
    return new InferredType(null, false);
  }

  /** Create an inferred type representing time. */
  public static InferredType time() {
    return new InferredType(null, true);
  }
}
