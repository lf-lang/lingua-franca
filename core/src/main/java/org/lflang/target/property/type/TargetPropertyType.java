package org.lflang.target.property.type;

import java.util.function.Predicate;

import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.validation.LFValidator;

/**
 * An interface for types associated with target properties.
 *
 * @author Marten Lohstroh
 */
public interface TargetPropertyType {

    /**
     * Return true if the the given Element is a valid instance of this type.
     *
     * @param e The Element to validate.
     * @return True if the element conforms to this type, false otherwise.
     */
    public boolean validate(Element e);

    /**
     * Check (recursively) the given Element against its associated type(s) and add found problems
     * to the given list of errors.
     *
     * @param e The Element to type check.
     * @param name The name of the target property.
     * @param v A reference to the validator to report errors to.
     */
    public void check(Element e, String name, LFValidator v);

    /**
     * Helper function to produce an error during type checking.
     *
     * @param name The description of the target property.
     * @param description The description of the type.
     * @param v A reference to the validator to report errors to.
     */
    public static void produceError(String name, String description, LFValidator v) {

        v.reportTargetPropertyError(
            "Target property '" + name + "' is required to be " + description + ".");
    }
}


