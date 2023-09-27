package org.lflang;

import java.util.List;

import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.validation.ValidationReporter;

/**
 * Extend this class to manage a non-trivial target property configuration, i.e.:
 * <ul>
 *     <li>it requires additional validation (override {@code validate});</li>
 *     <li>it uses elaborate datastructures (define as inner classes); or</li>
 *     <li>it performs non-trivial updates (override {@code update}.</li>
 * </ul>
 * @param <T> The type of the configuration value.
 */
public abstract class TargetPropertyConfig<T> {

  protected T value = initialValue();

  protected boolean isSet;

  /* The type of values that can be assigned to this property. */
  public final TargetPropertyType type;

  public TargetPropertyConfig(TargetPropertyType type) {
    this.type = type;
  }

  public void override(T value) {
    this.isSet = true;
    this.value = value;
  }

  public abstract T initialValue();

  /**
   * Parse the given element into the given target config. May use the error reporter to report
   * format errors.
   */
  public void set(Element value, MessageReporter err) {
    var parsed = this.fromAst(value, err);
    if (parsed != null) {
      this.isSet = true;
      this.value = parsed;
    }
  }

  public void set(String value, MessageReporter err) {
    var parsed = this.fromString(value, err);
    if (parsed != null) {
      this.isSet = true;
      this.value = parsed;
    }
  }

  public void reset() {
    this.value = initialValue();
    this.isSet = false;
  }

  /**
   * Return the current configuration.
   */
  public T get() {
    return value;
  }

  protected abstract T fromAst(Element value, MessageReporter err);

  protected abstract T fromString(String value, MessageReporter err);

  public abstract List<Target> supportedTargets();

  public final boolean isSupported(Target target) {
    if (supportedTargets().contains(target)) {
      return true;
    }
    return false;
  }

  // FIXME: config may not be needed.
  public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
    // Check whether the property is supported by the target.
    if (!this.isSupported(config.target)) {
      reporter.warning(
          "The target parameter: "
              + pair.getName()
              + " is not supported by the "
              + config.target
              + " target and will thus be ignored.",
          pair,
          Literals.KEY_VALUE_PAIR__NAME);
    }

    if (!this.type.check(pair.getValue(), pair.getName(), reporter)) {
      reporter.error("Target property '" + pair.getName() + "' is required to be " + type + ".", pair, Literals.KEY_VALUE_PAIR__VALUE);
    }

  }

  /**
   * Read this property from the target config and build an element which represents it for the AST.
   * May return null if the given value of this property is the same as the default.
   */
  public abstract Element toAstElement();

  public boolean isSet() {
    return isSet;
  }

  @Override
  public String toString() { return value.toString(); }
}
