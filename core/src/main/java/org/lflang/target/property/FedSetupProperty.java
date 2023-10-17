package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.TargetProperty;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.util.StringUtil;

/**
 * Directs the C or Python target to include the associated C file used for setting up federated
 * execution before processing the first tag.
 */
public class FedSetupProperty extends TargetProperty<String, PrimitiveType> {

  /** Singleton target property instance. */
  public static final FedSetupProperty INSTANCE = new FedSetupProperty();

  private FedSetupProperty() {
    super(PrimitiveType.FILE);
  }

  @Override
  public String initialValue() {
    return null;
  }

  @Override
  protected String fromAst(Element node, MessageReporter reporter) {
    return StringUtil.removeQuotes(ASTUtils.elementToSingleString(node));
  }

  @Override
  protected String fromString(String string, MessageReporter reporter) {
    return string;
  }

  @Override
  public Element toAstElement(String value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "_fed_setup"; // FIXME: follow kebab case convention
  }
}
