package org.lflang.target.property;

import com.google.gson.JsonObject;
import java.util.List;
import java.util.Optional;
import org.lflang.MessageReporter;
import org.lflang.generator.GeneratorArguments;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.TargetPropertyType;

/**
 * A abstract base class for target properties.
 *
 * @param <T> The data type of the value assigned to the target property.
 * @author Marten Lohstroh
 */
public abstract class TargetProperty<T, S extends TargetPropertyType> {

  /** The type of values assignable to this target property. */
  public final S type;

  /**
   * Construct a new target property.
   *
   * @param type The type of the value that can be assigned to the property.
   */
  protected TargetProperty(S type) {
    this.type = type;
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

  /**
   * Replace the value assigned to this target property in the given config with the given value.
   *
   * @param config The configuration to change.
   * @param value The new value to assign.
   */
  public final void override(TargetConfig config, T value) {
    config.set(this, value);
  }

  public void update(TargetConfig config, GeneratorArguments args, MessageReporter reporter) {
    var value = value(args);
    if (value != null) {
      update(config, value);
    } else if (args.jsonObject != null) {
      update(config, fromJSON(args.jsonObject, reporter));
    }
  }

  /**
   * Update the given configuration using the given value. The default implementation simply assigns
   * the given value, overriding whatever value might have been assigned before.
   *
   * @param config The configuration to update.
   * @param value The value to perform the update with.
   */
  public void update(TargetConfig config, T value) {
    override(config, value);
  }

  /**
   * Update the given configuration based on the given corresponding AST node.
   *
   * @param config The configuration to update.
   * @param node The node to perform the update with.
   * @param reporter A reporter to report issues.
   */
  public final void update(TargetConfig config, Element node, MessageReporter reporter) {
    this.update(config, fromAst(node, reporter));
  }

  /**
   * Update the given configuration based on the given corresponding AST node.
   *
   * @param config The configuration to update.
   * @param value The node to perform the update with.
   * @param reporter A reporter to report issues.
   */
  public final void update(TargetConfig config, String value, MessageReporter reporter) {
    this.update(config, fromString(value, reporter));
  }

  /**
   * Return true if the given object is an instance of a class with the same name.
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

  protected T fromJSON(JsonObject jsonObject, MessageReporter reporter) {
    T value = null;
    if (jsonObject.has(name())) {
      value = this.fromString(jsonObject.get(name()).getAsString(), reporter);
    }
    return value;
  }

  /**
   * Retrieve a key-value pair from the given AST that matches the given target property.
   *
   * @param ast The AST retrieve the key-value pair from.
   * @param property The target property of interest.
   * @return The found key-value pair, or {@code null} if no matching pair could be found.
   */
  public static KeyValuePair getKeyValuePair(Model ast, TargetProperty<?, ?> property) {
    var targetProperties = ast.getTarget().getConfig();
    List<KeyValuePair> properties =
        targetProperties.getPairs().stream()
            .filter(pair -> pair.getName().equals(property.name()))
            .toList();
    assert properties.size() <= 1;
    return properties.size() > 0 ? properties.get(0) : null;
  }

  public T value(GeneratorArguments args) {
    return null;
  }
}
