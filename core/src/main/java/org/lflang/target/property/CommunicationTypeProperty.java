package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.CommunicationTypeType;
import org.lflang.target.property.type.CommunicationTypeType.CommunicationType;

/** Directive to specify the target communication type such as 'TCP', 'SST', or 'MQTT'. */
public final class CommunicationTypeProperty extends TargetProperty<CommunicationType, CommunicationTypeType> {

  /** Singleton target property instance. */
  public static final CommunicationTypeProperty INSTANCE = new CommunicationTypeProperty();

  private CommunicationTypeProperty() {
    super(new CommunicationTypeType());
  }

  @Override
  public Element toAstElement(CommunicationType value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public CommunicationType initialValue() {
    return CommunicationType.TCP;
  }

  @Override
  public CommunicationType fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  @Override
  protected CommunicationType fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public String name() {
    return "comm-type";
  }
}
