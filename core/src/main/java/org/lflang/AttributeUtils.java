package org.lflang;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.resource.XtextResource;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.*;
import org.lflang.util.StringUtil;
import org.lflang.validation.AttributeSpec;

/**
 * A helper class for processing attributes in the AST.
 *
 * <p>An attribute is an annotation using the `@` syntax. For example, `@label("My Label")` is an
 * attribute. It is associated with whatever node comes immediately after it.
 *
 * @author Shaokai Lin
 * @author Clément Fournier
 * @author Alexander Schulz-Rosengarten
 * @ingroup Utilities
 */
public class AttributeUtils {

  /**
   * Return the attributes declared on the given node or null if the node does not support declaring
   * attributes.
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
    } else if (node instanceof Watchdog) {
      return ((Watchdog) node).getAttributes();
    }
    return null;
  }

  /**
   * Return the attribute with the given name if present, otherwise return null.
   *
   * <p>If there are multiple attributes with the same name, this returns the first one.
   *
   * @param node The node to search for the attribute.
   * @param name The name of the attribute to search for.
   * @return The attribute with the given name or null if it is not found.
   * @see findAttributesByName
   */
  public static Attribute findAttributeByName(EObject node, String name) {
    List<Attribute> attrs = getAttributes(node);
    if (attrs == null) {
      return null;
    }
    return attrs.stream()
        .filter(
            it ->
                it.getAttrName()
                    .equalsIgnoreCase(name)) // case-insensitive search (more user-friendly)
        .findFirst()
        .orElse(null);
  }

  /**
   * Return all attributes with the given name or null if the node does not support declaring
   * attributes.
   *
   * @see findAttributeByName
   */
  public static List<Attribute> findAttributesByName(EObject node, String name) {
    List<Attribute> attrs = getAttributes(node);
    if (attrs == null) {
      return null;
    }
    return attrs.stream()
        .filter(
            it ->
                it.getAttrName()
                    .equalsIgnoreCase(name)) // case-insensitive search (more user-friendly)
        .toList();
  }

  /**
   * Return the first argument specified for the attribute or null if the attribute is not found or
   * if it does not have any arguments. This should be used only if the attribute is expected to
   * have a single argument.
   *
   * @param attr The attribute to get the first argument from.
   * @return The first argument of the attribute or null if the attribute is not found or if it does
   *     not have any arguments.
   */
  public static String getFirstArgumentValue(Attribute attr) {
    if (attr == null || attr.getAttrParms().isEmpty()) {
      return null;
    }
    return StringUtil.removeQuotes(attr.getAttrParms().get(0).getValue());
  }

  /**
   * Search for an attribute with the given name on the given AST node and return its first argument
   * as a String or null if the attribute is not found or if it does not have any arguments. This
   * should only be used on attributes that are expected to have a single argument.
   *
   * @param node The node to search for the attribute.
   * @param attrName The name of the attribute to search for.
   * @return The first argument of the attribute or null if the attribute is not found or if it does
   *     not have any arguments.
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
   * For an attribute with the given name on the given AST node, return a map of the attribute
   * parameters to their values. This should only be used on attributes that are expected to have a
   * single argument. This returns null if the given node does not support declaring attributes, and
   * returns an empty map if the attribute is not found or if it does not have any arguments.
   *
   * @param node The node to search for the attribute.
   * @param attrName The name of the attribute to search for.
   * @return A map of the attribute parameters to their values or null if the attribute is not found
   *     or if it does not have any arguments.
   */
  public static Map<String, String> getAttributeValues(EObject node, String attrName) {
    final List<Attribute> attrs = findAttributesByName(node, attrName);
    if (attrs == null) {
      return null;
    }
    HashMap<String, String> layoutOptions = new HashMap<>();
    for (Attribute attribute : attrs) {
      layoutOptions.put(
          StringUtil.removeQuotes(attribute.getAttrParms().get(0).getValue()),
          StringUtil.removeQuotes(attribute.getAttrParms().get(1).getValue()));
    }
    return layoutOptions;
  }

  /**
   * Retrieve a specific annotation in a comment associated with the given model element in the AST.
   * This will look for a comment. If one is found, it searches for the given annotation `key`
   * and extracts any string that follows the annotation marker. Note that annotations in comments
   * are deprecated, but we still check for them for backwards compatibility.
   *
   * @param object The AST model element to search a comment for.
   * @param key The specific annotation key to be extracted.
   * @return `null` if no JavaDoc style comment was found or if it does not contain the given
   *     key. The string immediately following the annotation marker otherwise.
   */
  public static String findAnnotationInComments(EObject object, String key) {
    if (!(object.eResource() instanceof XtextResource)) return null;
    ICompositeNode node = NodeModelUtils.findActualNodeFor(object);
    return ASTUtils.getPrecedingComments(node, n -> true)
        .flatMap(String::lines)
        .filter(line -> line.contains(key))
        .map(String::trim)
        .map(it -> it.substring(it.indexOf(key) + key.length()))
        .map(it -> it.endsWith("*/") ? it.substring(0, it.length() - "*/".length()) : it)
        .findFirst()
        .orElse(null);
  }

  /**
   * Return the parameter of the given attribute with the given name. Return null if no such
   * parameter is found.
   *
   * @param attribute The attribute to get the parameter from.
   * @param parameterName The name of the parameter to get.
   * @return The parameter value or null if no such parameter is found.
   */
  public static String getAttributeParameter(Attribute attribute, String parameterName) {
    return (attribute == null)
        ? null
        : attribute.getAttrParms().stream()
            .filter(param -> Objects.equals(param.getName(), parameterName))
            .map(AttrParm::getValue)
            .map(StringUtil::removeQuotes)
            .findFirst()
            .orElse(null);
  }

  /**
   * Return the parameter of the given attribute with the given name and interpret it as a boolean.
   * Returns null if no such parameter is found.
   *
   * @param attribute The attribute to get the parameter from.
   * @param parameterName The name of the parameter to get.
   * @return The parameter value or null if no such parameter is found.
   */
  public static Boolean getBooleanAttributeParameter(Attribute attribute, String parameterName) {
    if (attribute == null || parameterName == null) {
      return null;
    }
    final var param = getAttributeParameter(attribute, parameterName);
    if (param == null) {
      return null;
    }
    return param.equalsIgnoreCase("true");
  }

  /**
   * Return true if the specified node is an Input and has an `@sparse` attribute.
   *
   * @param node An AST node.
   * @return True if the specified node is an Input and has an `@sparse` attribute.
   */
  public static boolean isSparse(EObject node) {
    return findAttributeByName(node, "sparse") != null;
  }

  /**
   * Return true if the reactor is marked to be a federate.
   *
   * @param reactor The reactor to check.
   * @return True if the reactor is marked to be a federate.
   */
  public static boolean isFederate(Reactor reactor) {
    return findAttributeByName(reactor, "_fed_config") != null;
  }

  /**
   * Return true if the reaction is marked to have a C code body. Currently, this is only used for
   * synthesized reactions in the context of federated execution in Python.
   *
   * @param reaction The reaction to check.
   * @return True if the reaction is marked to have a C code body.
   */
  public static boolean hasCBody(Reaction reaction) {
    return findAttributeByName(reaction, "_c_body") != null;
  }

  /**
   * Return the declared label of the node, as given by the `@label` annotation.
   *
   * @param node The node to get the label from.
   * @return The label of the node or null if there is no such annotation.
   */
  public static String getLabel(EObject node) {
    return getAttributeValue(node, "label");
  }

  /**
   * Return the declared icon of the node, as given by the `@icon` annotation, or null if there is no
   * such annotation.
   *
   * @param node The node to get the icon path from.
   * @return The icon path of the node or null if there is no such annotation.
   */
  public static String getIconPath(EObject node) {
    return getAttributeValue(node, "icon");
  }

  /**
   * Return the `@side` annotation for the given node (presumably a port) or null if there is
   * no such annotation.
   *
   * @param node The node to get the port side from.
   * @return The port side of the node or null if there is no such annotation.
   */
  public static String getPortSide(EObject node) {
    return getAttributeValue(node, "side");
  }

  /**
   * Return the `layout` annotation for the given element or null if there is no such
   * annotation.
   *
   * @param node The node to get the layout option from.
   * @return The layout option of the node or null if there is no such annotation.
   */
  public static Map<String, String> getLayoutOption(EObject node) {
    return getAttributeValues(node, "layout");
  }

  /**
   * Return the `@enclave` attribute annotated on the given node. Return null if there is no
   * such attribute.
   *
   * @param node The node to get the enclave attribute from.
   * @return The enclave attribute of the node or null if there is no such attribute.
   */
  public static Attribute getEnclaveAttribute(Instantiation node) {
    return findAttributeByName(node, "enclave");
  }

  /**
   * Return true if the specified instance has an `@enclave` attribute.
   *
   * @param node The node to check.
   * @return True if the specified instance has an `@enclave` attribute.
   */
  public static boolean isEnclave(Instantiation node) {
    return getEnclaveAttribute(node) != null;
  }

  /**
   * Return true if the specified instantiation is of an EnclaveConnection reactor.
   *
   * @param node The node to check.
   * @return True if the specified instantiation is of an EnclaveConnection reactor.
   */
  public static boolean isEnclaveConnection(Instantiation node) {
    return findAttributeByName(node.getReactorClass(), "_enclave_connection") != null;
  }

  /**
   * Retrieve the number of worker parameter from an enclave attribute. Return 1 if not specified or
   * has illegal value
   *
   * @param node The node to get the number of workers from.
   * @return The number of workers or 1 if not specified or has illegal value.
   */
  public static int getEnclaveNumWorkersFromAttribute(Instantiation node) {
    Attribute enclaveAttr = getEnclaveAttribute(node);
    if (enclaveAttr != null) {
      for (AttrParm attrParm : enclaveAttr.getAttrParms()) {
        if (attrParm.getName().equals(AttributeSpec.WORKERS_ATTR)) {
          int value = Integer.valueOf(attrParm.getValue());
          if (value > 0) {
            return value;
          } else {
            return 1;
          }
        }
      }
    }
    return 1; // Not specified
  }
}
