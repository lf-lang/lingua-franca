package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.target.property.CoordinationModeProperty.CoordinationMode;
import org.lflang.target.property.type.UnionType;

public class CoordinationModeProperty extends TargetPropertyConfig<CoordinationMode> {

  public CoordinationModeProperty() {
    super(UnionType.COORDINATION_UNION);
  }

  @Override
  public CoordinationMode initialValue() {
    return CoordinationMode.CENTRALIZED;
  }

  @Override
  public CoordinationMode fromAst(Element value, MessageReporter err) {
    return fromString(ASTUtils.elementToSingleString(value), err);
  }

  @Override
  protected CoordinationMode fromString(String value, MessageReporter err) {
    return (CoordinationMode) UnionType.COORDINATION_UNION.forName(value);
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public void validate(
      KeyValuePair pair, Model ast, TargetConfig config, MessageReporter reporter) {}

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.value.toString());
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
