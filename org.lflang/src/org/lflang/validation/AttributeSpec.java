/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.validation;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import org.lflang.lf.AttrParm;
import org.lflang.lf.Attribute;
import org.lflang.lf.LfPackage.Literals;

/**
 * Specification of the structure of an annotation.
 * 
 * @author{Clément Fournier, TU Dresden, INSA Rennes}
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
class AttributeSpec {

    private final Map<String, AttrParamSpec> paramSpecByName;

    public static final String VALUE_ATTR = "value";

    /** A map from a string to a supported AttributeSpec */
    public static final Map<String, AttributeSpec> ATTRIBUTE_SPECS_BY_NAME = new HashMap<>();

    public AttributeSpec(List<AttrParamSpec> params) {
        paramSpecByName = params.stream().collect(Collectors.toMap(it -> it.name, it -> it));
    }

    /**
     * Check that the attribute conforms to this spec and whether
     * attr has the correct name.
     */
    public void check(LFValidator validator, Attribute attr) {
        Set<String> seen;
        // Check to see if there is one or multiple parameters.
        if (attr.getAttrParms().size() == 1 && attr.getAttrParms().get(0).getName() == null) {
            // If we are in this branch,
            // then the user has provided @attr("value"),
            // which is a shorthand for @attr(value="value").
            AttrParamSpec valueSpec = paramSpecByName.get(VALUE_ATTR);
            if (valueSpec == null) {
                validator.error("Attribute doesn't have a 'value' parameter.", Literals.ATTR_PARM__NAME);
                return;
            }

            valueSpec.check(validator, attr.getAttrParms().get(0));
            seen = Set.of(VALUE_ATTR);
        } else {
            // Process multiple attributes, each of which has to be named.
            seen = processNamedAttrParms(validator, attr);
        }

        // Check if there are any missing parameters required by this attribute.
        Map<String, AttrParamSpec> missingParams = new HashMap<>(paramSpecByName);
        missingParams.keySet().removeAll(seen);
        missingParams.forEach((name, paramSpec) -> {
            if (!paramSpec.isOptional()) {
                validator.error("Missing required attribute parameter '" + name + "'.", Literals.ATTRIBUTE__ATTR_PARMS);
            }
        });
    }

    /**
     * Check if the attribute parameters are named, whether
     * these names are known, and whether the named parameters
     * conform to the param spec (whether the param has the
     * right type, etc.).
     * 
     * @param validator The current validator in use
     * @param attr The attribute being checked
     * @return A set of named attribute parameters the user provides
     */
    private Set<String> processNamedAttrParms(LFValidator validator, Attribute attr) {
        Set<String> seen = new HashSet<>();
        for (AttrParm parm : attr.getAttrParms()) {
            if (parm.getName() == null) {
                validator.error("Missing name for attribute parameter.", Literals.ATTRIBUTE__ATTR_NAME);
                continue;
            }

            AttrParamSpec parmSpec = paramSpecByName.get(parm.getName());
            if (parmSpec == null) {
                validator.error("\"" + parm.getName() + "\"" + " is an unknown attribute parameter.",
                                Literals.ATTRIBUTE__ATTR_NAME);
                continue;
            }
            // Check whether a parameter conforms to its spec.
            parmSpec.check(validator, parm);
            seen.add(parm.getName());
        }
        return seen;
    }

    /**
     * The specification of the attribute parameter.
     * 
     * @param name The name of the attribute parameter
     * @param type The type of the parameter
     * @param defaultValue If non-null, parameter is optional.
     */
    record AttrParamSpec(String name, AttrParamType type, Object defaultValue) {

        private boolean isOptional() {
            return defaultValue == null;
        }

        // Check if a parameter has the right type.
        // Currently only String, Int, Boolean, and Float are supported.
        public void check(LFValidator validator, AttrParm parm) {
            switch(type) {
                case STRING:
                    if (parm.getValue().getStr() == null) {
                        validator.error("Incorrect type: \"" + parm.getName() + "\"" + " should have type String.",
                                        Literals.ATTRIBUTE__ATTR_NAME);
                    }
                    break;
                case INT:
                    if (parm.getValue().getInt() == null) {
                        validator.error("Incorrect type: \"" + parm.getName() + "\"" + " should have type Int.",
                                        Literals.ATTRIBUTE__ATTR_NAME);
                    }
                    break;
                case BOOLEAN:
                    if (parm.getValue().getBool() == null) {
                        validator.error("Incorrect type: \"" + parm.getName() + "\"" + " should have type Boolean.",
                                        Literals.ATTRIBUTE__ATTR_NAME);
                    }
                    break;
                case FLOAT:
                    if (parm.getValue().getFloat() == null) {
                        validator.error("Incorrect type: \"" + parm.getName() + "\"" + " should have type Float.",
                                        Literals.ATTRIBUTE__ATTR_NAME);
                    }
                    break;
            }
            
        }
    }

    /**
     * The type of attribute parameters currently supported.
     */
    enum AttrParamType {
        STRING,
        INT,
        BOOLEAN,
        FLOAT
    }

    /**
     * The specs of the known annotations are declared here.
     * Note: If an attribute only has one parameter, the parameter name should be "value."
     */
    static {
        // @label("value")
        ATTRIBUTE_SPECS_BY_NAME.put("label", new AttributeSpec(
            List.of(new AttrParamSpec(AttributeSpec.VALUE_ATTR, AttrParamType.STRING, null))
        ));
    }
}
