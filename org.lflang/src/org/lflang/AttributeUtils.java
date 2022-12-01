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
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.resource.XtextResource;

import org.lflang.lf.Action;
import org.lflang.lf.Attribute;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.Timer;
import org.lflang.util.StringUtil;

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
        } else if (node instanceof Instantiation) {
            return ((Instantiation) node).getAttributes();
        }
        throw new IllegalArgumentException("Not annotatable: " + node);
    }

    /**
     * Return the attribute with the given name
     * if present, otherwise return null.
     *
     * @throws IllegalArgumentException If the node cannot have attributes
     */
    public static Attribute findAttributeByName(EObject node, String name) {
        List<Attribute> attrs = getAttributes(node);
        return attrs.stream()
                    .filter(it -> it.getAttrName().equalsIgnoreCase(name)) // case-insensitive search (more user-friendly)
                    .findFirst()
                    .orElse(null);
    }

    /**
     * Return the first argument specified for the attribute.
     *
     * This should be used if the attribute is expected to have a single argument.
     * If there is no argument, null is returned.
     */
    public static String getFirstArgumentValue(Attribute attr) {
        if (attr == null || attr.getAttrParms().isEmpty()) {
            return null;
        }
        return StringUtil.removeQuotes(attr.getAttrParms().get(0).getValue());
    }

    /**
     * Search for an attribute with the given name on the given AST node and return its first
     * argument as a String.
     *
     * This should only be used on attributes that are expected to have a single argument.
     *
     * Returns null if the attribute is not found or if it does not have any arguments.
     */
    public static String getAttributeValue(EObject node, String attrName) {
        final var attr = findAttributeByName(node, attrName);
        String value = getFirstArgumentValue(attr);
        // Attribute annotations in comments are deprecated, but we still check for then for backwards
        // compatibility
        if (value == null) {
            return findAnnotationInComments(node, "@" + attrName);
        }
        return value;
    }

    /**
     * Retrieve a specific annotation in a comment associated with the given model element in the AST.
     *
     * This will look for a comment. If one is found, it searches for the given annotation `key`.
     * and extracts any string that follows the annotation marker.
     *
     * @param object the AST model element to search a comment for
     * @param key the specific annotation key to be extracted
     * @return `null` if no JavaDoc style comment was found or if it does not contain the given key.
     *     The string immediately following the annotation marker otherwise.
     */
    public static String findAnnotationInComments(EObject object, String key) {
        if (!(object.eResource() instanceof XtextResource)) return null;
        ICompositeNode node = NodeModelUtils.findActualNodeFor(object);
        return ASTUtils.getPrecedingComments(node, n -> true).flatMap(String::lines)
            .filter(line -> line.contains(key))
            .map(String::trim)
            .map(it -> it.substring(it.indexOf(key) + key.length()))
            .map(it -> it.endsWith("*/") ? it.substring(0, it.length() - "*/".length()) : it)
            .findFirst().orElse(null);
    }

    /**
     * Return true if the specified node is an Input and has an {@code @sparse}
     * attribute.
     * @param node An AST node.
     */
    public static boolean isSparse(EObject node) {
        return findAttributeByName(node, "sparse") != null;
    }

    /**
     * Return the declared label of the node, as given by the @label annotation.
     */
    public static String getLabel(EObject node) {
        return getAttributeValue(node, "label");
    }

    /**
     * Return the declared icon of the node, as given by the @icon annotation.
     */
    public static  String getIconPath(EObject node) {
        return getAttributeValue(node, "icon");
    }

}
