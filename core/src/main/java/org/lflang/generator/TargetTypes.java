package org.lflang.generator;

import java.util.List;
import java.util.stream.Collectors;
import org.lflang.InferredType;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Action;
import org.lflang.lf.BracedListExpression;
import org.lflang.lf.BracketListExpression;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Expression;
import org.lflang.lf.Initializer;
import org.lflang.lf.Literal;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.ParenthesisListExpression;
import org.lflang.lf.Port;
import org.lflang.lf.StateVar;
import org.lflang.lf.Time;
import org.lflang.lf.Type;

/**
 * Information about the types of a target language. Contains utilities to convert LF expressions
 * and types to the target language. Each code generator is expected to use at least one
 * language-specific instance of this interface.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
public interface TargetTypes {

  /**
   * Return true if the target supports generics (i.e., parametric polymorphism), false otherwise.
   */
  boolean supportsGenerics();

  /** Return the type of time durations. */
  String getTargetTimeType();

  /** Return the type of tags. */
  String getTargetTagType();

  default String getTargetParamRef(ParameterReference expr, InferredType typeOrNull) {
    return escapeIdentifier(expr.getParameter().getName());
  }

  /** Translate the braced list expression into target language syntax. */
  default String getTargetBracedListExpr(BracedListExpression expr, InferredType typeOrNull) {
    InferredType t = typeOrNull == null ? InferredType.undefined() : typeOrNull;
    return expr.getItems().stream()
        .map(e -> getTargetExpr(e, t))
        .collect(Collectors.joining(",", "{", "}"));
  }

  /** Translate the bracket list expression into target language syntax. */
  default String getTargetBracketListExpr(BracketListExpression expr, InferredType typeOrNull) {
    InferredType t = typeOrNull == null ? InferredType.undefined() : typeOrNull;
    return expr.getItems().stream()
        .map(e -> getTargetExpr(e, t))
        .collect(Collectors.joining(", ", "[", "]"));
  }

  /** Translate the parenthesis list expression into target language syntax. */
  default String getTargetParenthesistListExpr(
      ParenthesisListExpression expr, InferredType typeOrNull) {
    InferredType t = typeOrNull == null ? InferredType.undefined() : typeOrNull;
    return expr.getItems().stream()
        .map(e -> getTargetExpr(e, t))
        .collect(Collectors.joining(", ", "(", ")"));
  }

  /** Return an "unknown" type which is used as a default when a type cannot be inferred. */
  String getTargetUndefinedType();

  /**
   * Returns a version of the given LF identifier that is escaped properly for insertion into a
   * piece of target code.
   */
  default String escapeIdentifier(String ident) {
    return ident;
  }

  /**
   * Returns an expression in the target language that corresponds to the given time value ({@link
   * #getTargetTimeType()}).
   */
  String getTargetTimeExpr(TimeValue timeValue);

  /**
   * Returns the expression that is used to replace a missing expression in the source language. The
   * expression may for instance be a type-agnostic default value (e.g. Rust's {@code
   * Default::default()}), or produce a compiler error (e.g. Rust's {@code compiler_error!("missing
   * initializer")}).
   *
   * @throws UnsupportedGeneratorFeatureException If the target does not support this
   */
  default String getMissingExpr(InferredType type) {
    throw new UnsupportedGeneratorFeatureException("Missing initializers");
  }

  /**
   * Returns a target type inferred from the type node, or the initializer list. If both are absent,
   * then the undefined type is returned.
   */
  default String getTargetType(Type type, Initializer init) {
    return getTargetType(ASTUtils.getInferredType(type, init));
  }

  /**
   * Returns the target type of the type node. This just provides a default parameter for {@link
   * #getTargetType(Type, Initializer)}. If the parameter is null, then the undefined type is
   * returned.
   */
  default String getTargetType(Type type) {
    return getTargetType(type, null);
  }

  /** Return a string representing the specified type in the target language. */
  default String getTargetType(InferredType type) {
    if (type.isUndefined()) {
      return getTargetUndefinedType();
    } else if (type.isTime) {
      return getTargetTimeType();
    } else if (!type.astType.getTypeArgs().isEmpty()) {
      List<String> args = type.astType.getTypeArgs().stream().map(this::getTargetType).toList();
      return getGenericType(type.baseType(), args);
    }
    return type.toOriginalText();
  }

  /** Build a generic type. The type args list must not be empty. */
  default String getGenericType(String base, List<String> args) {
    assert !args.isEmpty() : "Empty type arguments for " + base;
    return base + "<" + String.join(", ", args) + ">";
  }

  /** Return a string representing the type of the given parameter. */
  default String getTargetType(Parameter p) {
    return getTargetType(ASTUtils.getInferredType(p));
  }

  /** Return a string representing the type of the given state variable. */
  default String getTargetType(StateVar s) {
    return getTargetType(ASTUtils.getInferredType(s));
  }

  /** Return a string representing the type of the given action. */
  default String getTargetType(Action a) {
    return getTargetType(ASTUtils.getInferredType(a));
  }

  /** Return a string representing the type of the given port. */
  default String getTargetType(Port p) {
    return getTargetType(ASTUtils.getInferredType(p));
  }

  /**
   * Returns the representation of the given initializer expression in target code. The given type,
   * if non-null, may inform the code generation.
   *
   * @param init Initializer node (nullable)
   * @param type Declared type of the expression (nullable)
   */
  default String getTargetInitializer(Initializer init, Type type) {
    var inferredType = ASTUtils.getInferredType(type, init);
    if (init == null) {
      return getMissingExpr(inferredType);
    }
    return getTargetExpr(init.getExpr(), inferredType);
  }

  /**
   * Returns the representation of the given expression in target code. The given type, if non-null,
   * may inform the code generation.
   */
  default String getTargetExpr(Expression expr, InferredType type) {
    if (ASTUtils.isZero(expr) && type != null && type.isTime) {
      return getTargetTimeExpr(TimeValue.ZERO);
    } else if (expr instanceof ParameterReference) {
      return getTargetParamRef((ParameterReference) expr, type);
    } else if (expr instanceof Time) {
      return getTargetTimeExpr((Time) expr);
    } else if (expr instanceof Literal) {
      return ASTUtils.addZeroToLeadingDot(((Literal) expr).getLiteral()); // here we don't escape
    } else if (expr instanceof CodeExpr) {
      return ASTUtils.toText(((CodeExpr) expr).getCode());
//    } else if (expr instanceof BracedListExpression) {
//      return getTargetBracedListExpr((BracedListExpression) expr, type);
//    } else if (expr instanceof BracketListExpression) {
//      return getTargetBracketListExpr((BracketListExpression) expr, type);
//    } else if (expr instanceof ParenthesisListExpression) {
//      return getTargetParenthesistListExpr((ParenthesisListExpression) expr, type);
    } else {
      throw new IllegalStateException("Invalid value " + expr);
    }
  }

  /** Returns the representation of the given time value in target code. */
  default String getTargetTimeExpr(Time t) {
    return getTargetTimeExpr(ASTUtils.toTimeValue(t));
  }
}
