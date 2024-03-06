package org.lflang.generator.ts;

import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.TargetTypes;
import org.lflang.lf.StateVar;

public class TSTypes implements TargetTypes {

  private static TSTypes INSTANCE = new TSTypes();

  private TSTypes() {}

  @Override
  public String getTargetType(StateVar s) {
    var type = TargetTypes.super.getTargetType(s);
    if (!ASTUtils.isInitialized(s)) {
      return "%s | undefined".formatted(type);
    } else {
      return type;
    }
  }

  @Override
  public boolean supportsGenerics() {
    return true;
  }

  @Override
  public String getTargetTimeType() {
    return "TimeValue";
  }

  @Override
  public String getTargetTagType() {
    return "TimeValue";
  }

  @Override
  public String getTargetUndefinedType() {
    return "unknown";
  }

  public String getTargetTimeExpr(TimeValue value) {
    if (value.unit != null) {
      return "TimeValue.%s(%s)".formatted(value.unit.getCanonicalName(), value.time);
    } else {
      // The value must be zero.
      return "TimeValue.zero()";
    }
  }

  public static TSTypes getInstance() {
    return INSTANCE;
  }
}
