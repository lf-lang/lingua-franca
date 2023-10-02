package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.CoordinationProperty.CoordinationMode;
import org.lflang.target.property.type.UnionType;

public class CoordinationProperty extends AbstractTargetProperty<CoordinationMode> {

  public CoordinationProperty() {
    super(UnionType.COORDINATION_UNION);
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
    return (CoordinationMode) UnionType.COORDINATION_UNION.forName(string);
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.get().toString());
  }

  @Override
  public String name() {
    return "coordination";
  }

  /**
   * Enumeration of coordination types.
   *
   * @author Marten Lohstroh
   */
  public enum CoordinationMode {
    CENTRALIZED,
    DECENTRALIZED;

    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }
  }
}
