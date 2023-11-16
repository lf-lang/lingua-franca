package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.lflang.MessageReporter;
import org.lflang.lf.Element;
import org.lflang.target.Target;

/**
 * A type that can assume one of several types.
 *
 * @author Marten Lohstroh
 */
public enum UnionType implements TargetPropertyType {
  STRING_OR_STRING_ARRAY(Arrays.asList(PrimitiveType.STRING, ArrayType.STRING_ARRAY)),
  PLATFORM_STRING_OR_DICTIONARY(List.of(new PlatformType(), DictionaryType.PLATFORM_DICT)),
  FILE_OR_FILE_ARRAY(Arrays.asList(PrimitiveType.FILE, ArrayType.FILE_ARRAY)),
  DOCKER_UNION(Arrays.asList(PrimitiveType.BOOLEAN, DictionaryType.DOCKER_DICT)),
  TRACING_UNION(Arrays.asList(PrimitiveType.BOOLEAN, DictionaryType.TRACING_DICT)),
  SCHEDULER_UNION_OR_DICTIONARY(List.of(new SchedulerType(), DictionaryType.SCHEDULER_DICT)),
  ;

  /** The constituents of this type union. */
  public final List<TargetPropertyType> options;

  /**
   * Private constructor for creating unions types.
   *
   * @param options The types that are part of the union.
   */
  UnionType(List<TargetPropertyType> options) {
    this.options = options;
  }

  /**
   * Return the type among those in this type union that matches the given name.
   *
   * @param name The string to match against.
   * @return The matching dictionary element (or null if there is none).
   */
  public TargetPropertyType forName(String name) {
    return Target.match(name, options);
  }

  /** Recursively check that the passed in element conforms to the rules of this union. */
  @Override
  public boolean check(Element e, String name, MessageReporter r) {
    var match = this.match(e);
    if (match.isPresent()) {
      return match.get().check(e, name, r);
    }
    return false;
  }

  /**
   * Internal method for matching a given element against the allowable types.
   *
   * @param e AST node that represents the value of a target property.
   * @return The matching type wrapped in an Optional object.
   */
  private Optional<TargetPropertyType> match(Element e) {
    return this.options.stream().filter(option -> option.validate(e)).findAny();
  }

  /**
   * Return true if this union has an option that matches the given element.
   *
   * @param e The element to match against this type.
   */
  @Override
  public boolean validate(Element e) {
    if (this.match(e).isPresent()) {
      return true;
    }
    return false;
  }

  /**
   * Return a human-readable description of this type. If three is a default option, then indicate
   * it.
   */
  @Override
  public String toString() {
    return "one of the following: "
        + options.stream().map(option -> option.toString()).collect(Collectors.joining(", "));
  }
}
