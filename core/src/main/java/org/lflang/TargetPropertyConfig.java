package org.lflang;

import java.util.List;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.type.TargetPropertyType;

/**
 * Extend this class to manage the configuration of a target property.
 *
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

  /** Return the current configuration. */
  public T get() {
    return value;
  }

  protected abstract T fromAst(Element value, MessageReporter err);

  protected abstract T fromString(String value, MessageReporter err);

  public abstract List<Target> supportedTargets();

  public final boolean isSupported(Target target) {
    return supportedTargets().contains(target);
  }

  public void validate(
      KeyValuePair pair, Model ast, TargetConfig config, MessageReporter reporter) {
    // FIXME: Make abstract?
    // FIXME: consider not passing in config
  }

  public void checkSupport(KeyValuePair pair, TargetConfig config, MessageReporter reporter) {
    if (!this.isSupported(config.target)) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .warning(
              String.format(
                  "The target parameter: %s is not supported by the %s target and will thus be"
                      + " ignored.",
                  pair.getName(), config.target));
    }
  }

  public void checkType(KeyValuePair pair, TargetConfig config, MessageReporter reporter) {
    if (!this.type.check(pair.getValue(), pair.getName(), reporter)) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
          .error("Target property '" + pair.getName() + "' is required to be " + type + ".");
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
  public String toString() {
    return value == null ? null : value.toString();
  }

  public void markSet() {
    this.isSet = true;
  }
}
