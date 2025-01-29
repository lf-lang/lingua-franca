package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.CommunicationModeType;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

public final class CommunicationModeProperty
    extends TargetProperty<CommunicationMode, CommunicationModeType> {

  /** Singleton target property instance. */
  public static final CommunicationModeProperty INSTANCE = new CommunicationModeProperty();

  private CommunicationModeProperty() {
    super(new CommunicationModeType());
  }

  @Override
  public Element toAstElement(CommunicationMode value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public CommunicationMode initialValue() {
    return CommunicationMode.TCP;
  }

  @Override
  public CommunicationMode fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  @Override
  protected CommunicationMode fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public String name() {
    return "comm-type";
  }
}
