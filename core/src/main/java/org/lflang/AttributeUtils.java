package org.lflang;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.eclipse.emf.ecore.EObject;
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
   * Return a list of attributes declared on the given node. An empty list is returned if the node
   * does not have any attributes.
   *
   * @param node The node to get the attributes from.
   * @throws IllegalArgumentException If the node cannot have attributes.
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
    } else if (node instanceof Connection) {
      return ((Connection) node).getAttributes();
    }
    throw new IllegalArgumentException("Not annotatable: " + node);
  }

  /**
   * Return the attribute with the given name if present, otherwise return null.
   * If there are multiple attributes with the same name, this returns the first one.
   *
   * @param node The node to search for the attribute.
   * @param name The name of the attribute to search for.
   * @return The attribute with the given name or null if it is not found.
   */
  public static Attribute findAttributeByName(EObject node, String name) {
    try {
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
    } catch (IllegalArgumentException e) {
      return null;
    }
  }

  /**
   * Return a list of attributes with the given name. An empty list is returned if the node does not
   * have any attributes with the given name.
   *
   * @param node The node to get the attributes from.
   * @param name The name of the attributes to get.
   * @throws IllegalArgumentException If the node cannot have attributes
   */
  public static List<Attribute> findAttributesByName(EObject node, String name)
      throws IllegalArgumentException {
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
   * Return the first argument specified for the first attribute with the given name. If there is no
   * attribute with the given name, return null. If the attribute has no arguments, or if the
   * arguments are not of a suitable form, return null. This ignores any argument name, if one is
   * given, and only returns arguments of the form of strings ("foo"), numbers (123), boolean values
   * (true, false), or the time values `forever` or `never`. Time values with units are not returned
   * (the return value will be null).
   *
   * <p>This is a convenience method for common use cases where an attribute is specified with a
   * single argument of a suitable form. The validator should check that the arguments are of a
   * suitable form.
   *
   * @param node The node to get the attribute value from.
   * @param attrName The name of the attribute to get the value from.
   */
  public static String getAttributeValue(EObject node, String attrName) {
    final var attr = findAttributeByName(node, attrName);
    if (attr == null || attr.getAttrParms().isEmpty()) {
      return null;
    }
    return StringUtil.removeQuotes(attr.getAttrParms().get(0).getValue());
  }

  /**
   * Return the parameter with the given name for the specified attribute or null if no such
   * parameter is found.
   *
   * @param attribute The attribute to get the parameter from.
   * @param parameterName The name of the parameter to get.
   */
  public static AttrParm getAttributeParameter(Attribute attribute, String parameterName) {
    return (attribute == null)
        ? null
        : attribute.getAttrParms().stream()
            .filter(param -> Objects.equals(param.getName(), parameterName))
            .findFirst()
            .orElse(null);
  }

  /**
   * Return true if there is a parameter of the given attribute with the given name whose value is
   * "true" (case-insensitive) and return false otherwise.
   *
   * @param attribute The attribute to get the parameter from.
   * @param parameterName The name of the parameter to get.
   */
  public static boolean getBooleanAttributeParameter(Attribute attribute, String parameterName) {
    if (attribute == null || parameterName == null) {
      return false;
    }
    final var param = getAttributeParameter(attribute, parameterName);
    if (param == null || param.getValue() == null) {
      return false;
    }
    return param.getValue().equalsIgnoreCase("true");
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
   * Return the layout annotations for the given element or null if there is no such annotation.
   * Layout annotations have the form:
   *
   * <pre>{@code
   * @layout(option="string", value="any")
   * }</pre>
   *
   * For example:
   *
   * <pre>{@code
   * @layout(option="port.side", value="WEST")
   * }</pre>
   *
   * This will return all such annotations for the specified node in the form of a map from the option
   * name to the value.
   */
  public static Map<String, String> getLayoutOption(EObject node) {
    final List<Attribute> attrs = findAttributesByName(node, "layout");
    HashMap<String, String> result = new HashMap<>();
    for (Attribute attribute : attrs) {
      result.put(
          // FIXME: This assumes the parameters are in the correct order.
          StringUtil.removeQuotes(attribute.getAttrParms().get(0).getValue()),
          StringUtil.removeQuotes(attribute.getAttrParms().get(1).getValue()));
    }
    return result;
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

  /**
   * Return the value of the `@maxwait` attribute of the given node or TimeValue.ZERO if does not
   * have one.
   *
   * @param node The AST node (Instantiation or Connection).
   */
  public static TimeValue getMaxWait(EObject node) {
    final var attr = findAttributeByName(node, "maxwait");
    if (attr != null) {
      // The attribute is expected to have a single argument of type Time or the literal "0".
      // The validator checks this.
      final var time = attr.getAttrParms().get(0).getTime();
      if (time != null) {
        return ASTUtils.toTimeValue(time);
      }
    }
    return TimeValue.ZERO;
  }

  /**
   * Return the value of the `@absent_after` attribute of the given node or TimeValue.ZERO if does not
   * have one.
   *
   * @param node The AST node (a Connection).
   */
  public static TimeValue getAbsentAfter(EObject node) {
    final var attr = findAttributeByName(node, "absent_after");
    if (attr != null) {
      // The attribute is expected to have a single argument of type Time or the literal "0".
      // The validator checks this.
      final var time = attr.getAttrParms().get(0).getTime();
      if (time != null) {
        return ASTUtils.toTimeValue(time);
      }
    }
    return TimeValue.ZERO;
  }
}
