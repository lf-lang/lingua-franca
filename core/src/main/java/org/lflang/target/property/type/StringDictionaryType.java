package org.lflang.target.property.type;

import org.lflang.MessageReporter;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;

/** Dictionary type that allows for keys that will be interpreted as strings and string values. */
public enum StringDictionaryType implements TargetPropertyType {
  COMPILE_DEFINITION();

  @Override
  public boolean validate(Element e) {
    if (e.getKeyvalue() != null) {
      return true;
    }
    return false;
  }

  @Override
  public boolean check(Element e, String name, MessageReporter v) {
    KeyValuePairs kv = e.getKeyvalue();
    if (kv != null) {
      var valid = true;
      for (KeyValuePair pair : kv.getPairs()) {
        String key = pair.getName();
        Element val = pair.getValue();

        // Make sure the type is string
        valid &= PrimitiveType.STRING.check(val, "Entry", v);
      }
      return valid;
    }
    return false;
  }

  @Override
  public String toString() {
    return "a dictionary that maps strings keys to string values";
  }
}
