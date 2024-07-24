package org.lflang.target.property;

import com.google.gson.JsonElement;
import java.util.Optional;
import org.lflang.MessageReporter;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.TargetPropertyType;

/**
 * An abstract base class for target properties.
 *
 * @param <T> The Java type of the value assigned to the target property.
 * @param <S> The LF type of the value assigned to the target property.
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
  public boolean checkType(KeyValuePair pair, MessageReporter reporter) {
    boolean isOk = this.type.check(pair.getValue(), pair.getName(), reporter);
    if (!isOk) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
          .error("Target property '" + pair.getName() + "' is required to be " + type + ".");
    }
    return isOk;
  }

  @Override
  public String toString() {
    return this.name();
  }

  /** Return the initial value to assign to this target property. */
  public abstract T initialValue();

  /**
   * Override this method to implement additional checks. The base implementation does nothing.
   *
   * <p>This method is meant to perform additional validation above and beyond checking target
   * support and type checking which are done automatically.
   *
   * @param config The target configuration to check against.
   * @param reporter A reporter for reporting errors.
   */
  public void validate(TargetConfig config, MessageReporter reporter) {}

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
   * Return {@code true} if this property is to be loaded from imported files, {@code false}
   * otherwise.
   */
  public boolean loadFromImport() {
    return false;
  }

  /**
   * Return {@code true} if this property is to be loaded by federates when specified at the level
   * of the federation, {@code false} otherwise.
   */
  public boolean loadFromFederation() {
    return true;
  }

  /**
   * Return {@code true} if this property is to be loaded from imported federates, {@code false}
   * otherwise.
   */
  public boolean loadFromFederate() {
    return false;
  }

  /**
   * Replace the value assigned to this target property in the given config with the given value.
   *
   * @param config The configuration to change.
   * @param value The new value to assign.
   */
  public final void override(TargetConfig config, T value) {
    config.set(this, value);
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
   * Update the given configuration based on the given corresponding AST node. If the given
   * configuration does not belong in the AST, then report an error using the error message given by
   * {@code TargetConfig.NOT_IN_LF_SYNTAX_MESSAGE}.
   *
   * @param config The configuration to update.
   * @param pair The pair that holds the value to perform the update with.
   * @param reporter A reporter to report issues.
   */
  public final void update(TargetConfig config, KeyValuePair pair, MessageReporter reporter) {
    this.update(config, fromAst(pair.getValue(), reporter));
  }

  /**
   * Update the given configuration based on the given JSON element.
   *
   * @param config The configuration to update.
   * @param element The JSON element that holds the value to perform the update with.
   * @param reporter A reporter to report issues.
   */
  public final void update(TargetConfig config, JsonElement element, MessageReporter reporter) {
    this.update(config, fromJSON(element, reporter));
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

  /**
   * Return a value based on the given JSON element.
   *
   * @param element The JSON element to produce a value from/
   * @param reporter A message reporter for reporting issues.
   */
  protected T fromJSON(JsonElement element, MessageReporter reporter) {
    T value = null;
    if (element.isJsonPrimitive()) {
      value = this.fromString(element.getAsString(), reporter);
    } else {
      reporter.nowhere().error("Encountered non-primitive JSON element; no method for handling it");
    }
    return value;
  }
}
