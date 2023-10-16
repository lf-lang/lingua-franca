package org.lflang;

import java.util.List;
import java.util.Optional;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.TargetConfig;
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

  /** The type of values assignable to this target property. */
  public final S type;

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
   * Return {@code true} if this target property is supported by the given target, {@code false}
   * otherwise.
   *
   * @param target The target to check against.
   */
  public final boolean isSupported(Target target) {
    return supportedTargets().contains(target);
  }

  @Override
  public String toString() {
    return this.name();
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
  public abstract Element toAstElement(T value);

  public Optional<Element> astElementFromConfig(TargetConfig config) {
    var value = toAstElement(config.get(this));
    if (value != null) {
      return Optional.of(value);
    } else {
      return Optional.empty();
    }
  }

  /** Return the name of this target property (in kebab case). */
  public abstract String name();

  public final void override(TargetConfig config, T value) {
    config.set(this, value);
  }

  protected void update(TargetConfig config, T value) {
    override(config, value);
  }

  public final void update(TargetConfig config, Element node, MessageReporter reporter) {
    this.update(config, fromAst(node, reporter));
  }

  public final void update(TargetConfig config, String value, MessageReporter reporter) {
    this.update(config, fromString(value, reporter));
  }

  /**
   * Return true if the given object is an instance of a class with the same name. FIXME: make this
   * a singleton class and remove this override https://www.baeldung.com/kotlin/singleton-classes
   * https://stackoverflow.com/questions/24214148/java-getinstance-vs-static
   *
   * @param obj The object to compare this instance to.
   */
  @Override
  public boolean equals(Object obj) {
    return obj.getClass().getName().equals(this.getClass().getName());
  }

  @Override
  public int hashCode() {
    return this.getClass().getName().hashCode();
  }

  /**
   * Retrieve a key-value pair from the given AST that matches the given target property.
   *
   * @param ast The AST retrieve the key-value pair from.
   * @param property The target property of interest.
   * @return The found key-value pair, or {@code null} if no matching pair could be found.
   */
  public static KeyValuePair getKeyValuePair(Model ast, AbstractTargetProperty<?, ?> property) {
    var targetProperties = ast.getTarget().getConfig();
    List<KeyValuePair> properties =
        targetProperties.getPairs().stream()
            .filter(pair -> pair.getName().equals(property.name()))
            .toList();
    assert properties.size() <= 1;
    return properties.size() > 0 ? properties.get(0) : null;
  }
}
