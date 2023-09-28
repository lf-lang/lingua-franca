package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.BuildConfig.BuildType;
import org.lflang.target.property.type.UnionType;

public class BuildTypeProperty extends TargetPropertyConfig<BuildType> {

  public BuildTypeProperty() {
    super(UnionType.BUILD_TYPE_UNION);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.value.toString());
  }

  @Override
  public BuildType initialValue() {
    return BuildType.RELEASE;
  }

  @Override
  public BuildType fromAst(Element value, MessageReporter err) {
    return fromString(ASTUtils.elementToSingleString(value), err);
  }

  @Override
  protected BuildType fromString(String value, MessageReporter err) {
    return (BuildType) UnionType.BUILD_TYPE_UNION.forName(value);
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Rust);
  }
}
