package org.lflang;

import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.validation.ValidationReporter;

public interface TargetPropertyConfig<T> {

  /**
   * Parse the given element into the given target config. May use the error reporter to report
   * format errors.
   */
  void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err);

  T parse(Element value);

  // FIXME: config may not be needed.
  void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter);

  /**
   * Read this property from the target config and build an element which represents it for the AST.
   * May return null if the given value of this property is the same as the default.
   */
  Element getPropertyElement(TargetConfig config);
}
