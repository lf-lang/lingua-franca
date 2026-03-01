package org.lflang.target.property.type;

import java.util.function.Predicate;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

/**
 * Primitive types for target properties, each with a description used in error messages and
 * predicate used for validating values.
 *
 * @author Marten Lohstroh
 */
public enum PrimitiveType implements TargetPropertyType {
  BOOLEAN(
      "'true' or 'false'",
      v ->
          ASTUtils.elementToSingleString(v).equalsIgnoreCase("true")
              || ASTUtils.elementToSingleString(v).equalsIgnoreCase("false")),
  INTEGER(
      "an integer",
      v -> {
        try {
          Integer.parseInt(ASTUtils.elementToSingleString(v));
        } catch (NumberFormatException e) {
          return false;
        }
        return true;
      }),
  NON_NEGATIVE_INTEGER(
      "a non-negative integer",
      v -> {
        try {
          int result = Integer.parseInt(ASTUtils.elementToSingleString(v));
          if (result < 0) return false;
        } catch (NumberFormatException e) {
          return false;
        }
        return true;
      }),
  TIME_VALUE(
      "a time value with units",
      v ->
          v.getKeyvalue() == null
              && v.getArray() == null
              && v.getLiteral() == null
              && v.getId() == null
              && v.getTime() != null),
  STRING(
      "a string",
      v -> v.getLiteral() != null && !isCharLiteral(v.getLiteral()) || v.getId() != null),
  FILE("a path to a file", STRING.validator);

  /** A description of this type, featured in error messages. */
  private final String description;

  /** A predicate for determining whether a given Element conforms to this type. */
  public final Predicate<Element> validator;

  /**
   * Private constructor to create a new primitive type.
   *
   * @param description A textual description of the type that should start with "a/an".
   * @param validator A predicate that returns true if a given Element conforms to this type.
   */
  PrimitiveType(String description, Predicate<Element> validator) {
    this.description = description;
    this.validator = validator;
  }

  /** Return true if the given Element is a valid instance of this type. */
  public boolean validate(Element e) {
    return this.validator.test(e);
  }

  /**
   * Check (recursively) the given Element against its associated type(s) and add found problems to
   * the given list of errors.
   *
   * @param e The element to type check.
   * @param name The name of the target property.
   * @param v The LFValidator to append errors to.
   */
  public boolean check(Element e, String name, MessageReporter v) {
    var valid = this.validate(e);
    if (!valid) {
      v.at(e).error(String.format("%s is required to be %s.", name, this));
      return false;
    } else {
      return true;
    }
  }

  /** Return a textual description of this type. */
  @Override
  public String toString() {
    return this.description;
  }

  private static boolean isCharLiteral(String s) {
    return s.length() > 2 && '\'' == s.charAt(0) && '\'' == s.charAt(s.length() - 1);
  }
}
