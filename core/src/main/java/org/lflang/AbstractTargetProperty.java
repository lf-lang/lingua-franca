package org.lflang;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.type.TargetPropertyType;

/**
 * A base class for target properties.
 *
 * <p>After implementing this class to create a new target property, add a corresponding entry to
 * {@code TargetConfig} and hook it into the {@code TargetProperty} enum.
 *
 * @param <T> The data type of the value assigned to the target property.
 */
public abstract class AbstractTargetProperty<T, S extends TargetPropertyType> {

  /** The type of values that can be assigned to this property. */
  public final S type;

  /** Whether (after initialization) this property has been set. */
  protected boolean isSet;

  /**
   * The value assigned to the target property, initialized using the {@code initialValue()} method.
   */
  private T value = initialValue();

  /**
   * Construct a new target property.
   *
   * @param type The type of the value that can be assigned to the property.
   */
  public AbstractTargetProperty(S type) {
    this.type = type;
  }

  /**
   * If this target property is not supported by the given target, report a warning through the
   * message reporter at the location of the given key-value pair.
   *
   * @param pair The ast node that matches this target property.
   * @param target The target to check against.
   * @param reporter The reporter to issue a warning through if this property is not supported by
   *     the given target.
   */
  public void checkSupport(KeyValuePair pair, Target target, MessageReporter reporter) {
    if (!this.isSupported(target)) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .warning(
              String.format(
                  "The target property: %s is not supported by the %s target and will thus be"
                      + " ignored.",
                  pair.getName(), target));
    }
  }

  /**
   * If the given key-value pair does not match the type required by this target property, report an
   * error through the given reporter.
   *
   * @param pair The ast node that matches this target property.
   * @param reporter The reporter to issue an error through if the given key-value pair does not
   *     match the type required by this property.
   */
  public void checkType(KeyValuePair pair, MessageReporter reporter) {
    if (!this.type.check(pair.getValue(), pair.getName(), reporter)) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
          .error("Target property '" + pair.getName() + "' is required to be " + type + ".");
    }
  }

  /**
   * Return {@code true} if this target property has been set (past initialization), {@code false}
   * otherwise.
   */
  public boolean isSet() {
    return isSet;
  }

  /**
   * Return {@code true} if this target property is supported by the given target, {@code false}
   * otherwise.
   *
   * @param target The target to check against.
   */
  public final boolean isSupported(Target target) {
    return supportedTargets().contains(target);
  }

  /**
   * Manually override the value of this target property.
   *
   * @param value The value to assign to this target property.
   */
  public void override(T value) {
    this.isSet = true;
    this.value = value;
  }

  /** Reset this target property to its initial value. */
  public void reset() {
    this.value = initialValue();
    this.isSet = false;
  }

  /**
   * Parse the given AST node into the given target config. Encountered errors are reported via the
   * given reporter.
   *
   * @param node The AST node to derive a newly assigned value from.
   * @param reporter A reporter for reporting errors.
   */
  public void set(Element node, MessageReporter reporter) {
    var parsed = this.fromAst(node, reporter);
    if (parsed != null) {
      this.isSet = true;
      this.value = parsed;
    }
  }

  /**
   * Parse the given element into the given target config. May use the error reporter to report
   * format errors.
   */
  public void set(String value, MessageReporter err) {
    var parsed = this.fromString(value, err);
    if (parsed != null) {
      this.isSet = true;
      this.value = parsed;
    }
  }

  @Override
  public String toString() {
    return value == null ? "" : value.toString();
  }

  /**
   * Override this method to implement additional checks. The base implementation does nothing.
   *
   * <p>This method is meant to perform additional validation above and beyond checking target
   * support and type checking which are done automatically.
   *
   * @param pair The key-value pair to type check.
   * @param ast The root of the abstract syntax tree.
   * @param reporter A reporter for reporting errors.
   */
  public void validate(KeyValuePair pair, Model ast, MessageReporter reporter) {}

  /** Return the initial value to assign to this target property. */
  public abstract T initialValue();

  /** Return the value currently assigned to this target property. */
  public T get() {
    return value;
  }

  /**
   * Given an AST node, produce a corresponding value that is assignable to this target property, or
   * report an error via the given reporter in case any problems are encountered.
   *
   * @param node The AST node to derive a value of type {@code T} from.
   * @param reporter A reporter for reporting errors.
   * @return A value of type {@code T}.
   */
  protected abstract T fromAst(Element node, MessageReporter reporter);

  /**
   * Given a string, produce a corresponding value that is assignable to this target property, or
   * report an error via the given reporter in case any problems are encountered.
   *
   * @param string The string to derive a value of type {@code T} from.
   * @param reporter A reporter for reporting errors.
   * @return A value of type {@code T}.
   */
  protected abstract T fromString(String string, MessageReporter reporter);

  /** Return a list of targets that support this target property. */
  public abstract List<Target> supportedTargets();

  /**
   * Return an AST node that represents this target property and the value currently assigned to it.
   */
  public abstract Element toAstElement();

  /** Return the name of this target property (in kebab case). */
  public abstract String name();

  public static List<AbstractTargetProperty> getAllTargetProperties(Object object) {
    var fields = object.getClass().getDeclaredFields();

    List<AbstractTargetProperty> properties =
        Arrays.stream(fields)
            .filter(f -> AbstractTargetProperty.class.isAssignableFrom(f.getType()))
            .map(
                f -> {
                  try {
                    return (AbstractTargetProperty) f.get(object);
                  } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                  }
                })
            .collect(Collectors.toList());

    return properties;
  }
}
