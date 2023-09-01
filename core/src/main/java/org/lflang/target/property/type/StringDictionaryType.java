package org.lflang.target.property.type;

import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.validation.LFValidator;

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
    public void check(Element e, String name, LFValidator v) {
        KeyValuePairs kv = e.getKeyvalue();
        if (kv == null) {
            TargetPropertyType.produceError(name, this.toString(), v);
        } else {
            for (KeyValuePair pair : kv.getPairs()) {
                String key = pair.getName();
                Element val = pair.getValue();

                // Make sure the type is string
                PrimitiveType.STRING.check(val, name + "." + key, v);
            }
        }
    }
}