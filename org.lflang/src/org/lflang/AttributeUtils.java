/*
Copyright (c) 2022, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.lflang;

import java.util.List;

import org.eclipse.emf.ecore.EObject;

import org.lflang.lf.Action;
import org.lflang.lf.Attribute;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.Timer;

/**
 * A helper class for processing attributes in the AST.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 * @author{Clément Fournier, TU Dresden, INSA Rennes}
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
public class AttributeUtils {

    /**
     * Return the attributes declared on the given node. Throws
     * if the node does not support declaring attributes.
     *
     * @throws IllegalArgumentException If the node cannot have attributes
     */
    public static List<Attribute> getAttributes(EObject node) {
        if (node instanceof Reactor) {
            return ((Reactor) node).getAttributes();
        } else if (node instanceof Reaction) {
            return ((Reaction) node).getAttributes();
        } else if (node instanceof Action) {
            return ((Action) node).getAttributes();
        } else if (node instanceof Timer) {
            return ((Timer) node).getAttributes();
        } else if (node instanceof StateVar) {
            return ((StateVar) node).getAttributes();
        } else if (node instanceof Parameter) {
            return ((Parameter) node).getAttributes();
        } else if (node instanceof Input) {
            return ((Input) node).getAttributes();
        } else if (node instanceof Output) {
            return ((Output) node).getAttributes();
        }
        throw new IllegalArgumentException("Not annotatable: " + node);
    }

    /**
     * Return the value of the attribute with the given name
     * if present, otherwise return null.
     *
     * @throws IllegalArgumentException If the node cannot have attributes
     */
    public static String findAttributeByName(EObject node, String name) {
        List<Attribute> attrs = getAttributes(node);
        return attrs.stream()
                    .filter(it -> it.getAttrName().equalsIgnoreCase(name)) // case-insensitive search (more user-friendly)
                    .map(it -> it.getAttrParms().get(0).getValue().getStr())
                    .findFirst()
                    .orElse(null);
    }
    
    /**
     * Return the value of the {@code @label} attribute if
     * present, otherwise return null.
     *
     * @throws IllegalArgumentException If the node cannot have attributes
     */
    public static String findLabelAttribute(EObject node) {
        return findAttributeByName(node, "label");
    }

    /**
     * Return true if the specified node is an Input and has an {@code @sparse}
     * attribute.
     * @param node An AST node.
     */
    public static boolean isSparse(EObject node) {
        if (node instanceof Input) {
            for (var attribute : getAttributes(node)) {
                if (attribute.getAttrName().equalsIgnoreCase("sparse")) return true;
            }
        }
        return false;
    }

    /**
     * Return the declared label of the node, as given by the @label
     * annotation (or an @label comment).
     *
     * @throws IllegalArgumentException If the node cannot have attributes
     */
    public static String label(EObject n) {
        String fromAttr = findLabelAttribute(n);
        if (fromAttr == null) {
            return ASTUtils.findAnnotationInComments(n, "@label");
        }
        return fromAttr;
    }

    /**
     * Search for an `@label` annotation for a given reaction.
     *
     * @param n the reaction for which the label should be searched
     * @return The annotated string if an `@label` annotation was found. `null` otherwise.
     */
    public static String label(Reaction n) {
        return label((EObject) n);
    }
}
