package org.lflang.target.property.type;

import java.util.List;

import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.validation.LFValidator;

/**
 * An array type of which the elements confirm to a given type.
 *
 * @author Marten Lohstroh
 */
public enum ArrayType implements TargetPropertyType {
    STRING_ARRAY(PrimitiveType.STRING),
    FILE_ARRAY(PrimitiveType.FILE);

    /** Type parameter of this array type. */
    public TargetPropertyType type;

    /**
     * Private constructor to create a new array type.
     *
     * @param type The type of elements in the array.
     */
    private ArrayType(TargetPropertyType type) {
        this.type = type;
    }

    /**
     * Check that the passed in element represents an array and ensure that its elements are all of
     * the correct type.
     */
    @Override
    public void check(Element e, String name, LFValidator v) {
        Array array = e.getArray();
        if (array == null) {
            TargetPropertyType.produceError(name, this.toString(), v);
        } else {
            List<Element> elements = array.getElements();
            for (int i = 0; i < elements.size(); i++) {
                this.type.check(elements.get(i), name + "[" + i + "]", v);
            }
        }
    }

    /** Return true of the given element is an array. */
    @Override
    public boolean validate(Element e) {
        if (e.getArray() != null) {
            return true;
        }
        return false;
    }

    /** Return a human-readable description of this type. */
    @Override
    public String toString() {
        return "an array of which each element is " + this.type.toString();
    }
}