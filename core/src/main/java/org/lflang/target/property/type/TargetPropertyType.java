package org.lflang.target.property.type;

import org.lflang.MessageReporter;
import org.lflang.lf.Element;

/**
 * An interface for types associated with target properties.
 *
 * @author Marten Lohstroh
 */
public interface TargetPropertyType {

  /**
   * Return true if the given Element is a valid instance of this type.
   *
   * @param e The Element to validate.
   * @return True if the element conforms to this type, false otherwise.
   */
  boolean validate(Element e);

  /**
   * Check (recursively) the given Element against its associated type(s) and add found problems to
   * the given list of errors.
   *
   * @param e The Element to type check.
   * @param name The name of the target property.
   * @param r A reference to the validator to report errors to.
   */
  boolean check(Element e, String name, MessageReporter r);
}
