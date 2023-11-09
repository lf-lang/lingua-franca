package org.lflang.target.property.type;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.lf.Array;
import org.lflang.lf.Element;

/**
 * An array type of which the elements confirm to a given type.
 *
 * @author Marten Lohstroh
 */
public enum ArrayType implements TargetPropertyType {
  STRING_ARRAY(PrimitiveType.STRING),
  FILE_ARRAY(PrimitiveType.FILE);

  /** Type parameter of this array type. */
  public final TargetPropertyType type;

  /**
   * Private constructor to create a new array type.
   *
   * @param type The type of elements in the array.
   */
  ArrayType(TargetPropertyType type) {
    this.type = type;
  }

  /**
   * Check that the passed in element represents an array and ensure that its elements are all of
   * the correct type.
   */
  @Override
  public boolean check(Element e, String name, MessageReporter r) {
    Array array = e.getArray();
    if (array != null) {
      List<Element> elements = array.getElements();
      var valid = true;
      for (int i = 0; i < elements.size(); i++) {
        valid &= this.type.check(elements.get(i), "Entry", r);
      }
      return valid;
    }
    return false;
  }

  /** Return true of the given element is an array. */
  @Override
  public boolean validate(Element e) {
    return e.getArray() != null;
  }

  /** Return a human-readable description of this type. */
  @Override
  public String toString() {
    return "an array of which each element is " + this.type.toString();
  }
}
