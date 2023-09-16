package org.lflang;

import java.util.Properties;

import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.validation.ValidationReporter;

public abstract class TargetPropertyConfig<T> { //implements TargetPropertyConfigurator<T> {

  protected T value = initialize();

  protected boolean setByUser;

  public void override(T value) { // FIXME: do all overrides imply setByUser?
    this.value = value;
  }

  public abstract T initialize();

  /**
   * Parse the given element into the given target config. May use the error reporter to report
   * format errors.
   */
  public void set(Element value, MessageReporter err) {
    var parsed = this.parse(value); // FIXME pass in error reporter. Maybe also rename to load?
    if (parsed != null) {
      this.setByUser = true;
      this.value = parsed;
    }
  }

  public void update(Element value, MessageReporter err) {
    this.set(value, err);
  }

  public void update(Properties cliArgs) {
    this.setByUser = true;
  }

  /**
   * Return the current configuration.
   */
  public T get() {
    return value;
  }

  protected abstract T parse(Element value);

  // FIXME: config may not be needed.
  public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {

  }

  /**
   * Read this property from the target config and build an element which represents it for the AST.
   * May return null if the given value of this property is the same as the default.
   */
  public abstract Element export();

  public boolean isSetByUser() {
    return setByUser;
  }
}
