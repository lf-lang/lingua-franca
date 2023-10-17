package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.CoordinationModeType;
import org.lflang.target.property.type.CoordinationModeType.CoordinationMode;

/**
 * The type of coordination used during the execution of a federated program. The default is
 * 'centralized'.
 */
public final class CoordinationProperty
    extends TargetProperty<CoordinationMode, CoordinationModeType> {

  /** Singleton target property instance. */
  public static final CoordinationProperty INSTANCE = new CoordinationProperty();

  private CoordinationProperty() {
    super(new CoordinationModeType());
  }

  @Override
  public CoordinationMode initialValue() {
    return CoordinationMode.CENTRALIZED;
  }

  @Override
  public CoordinationMode fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  @Override
  protected CoordinationMode fromString(String string, MessageReporter reporter) {
    return ((CoordinationModeType) this.type).forName(string);
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement(CoordinationMode value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public String name() {
    return "coordination";
  }
}
