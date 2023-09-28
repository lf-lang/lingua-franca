package org.lflang.target.property;

import java.util.List;
import java.util.Objects;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.target.property.ClockSyncModeProperty.ClockSyncMode;
import org.lflang.target.property.type.UnionType;

public class ClockSyncModeProperty extends TargetPropertyConfig<ClockSyncMode> {

  public ClockSyncModeProperty() {
    super(UnionType.CLOCK_SYNC_UNION);
  }

  @Override
  public ClockSyncMode initialValue() {
    return ClockSyncMode.INIT;
  }

  @Override
  public ClockSyncMode fromAst(Element value, MessageReporter err) {
    UnionType.CLOCK_SYNC_UNION.validate(value);
    var mode = fromString(ASTUtils.elementToSingleString(value), err);
    return Objects.requireNonNullElse(mode, ClockSyncMode.INIT);
  }

  @Override
  protected ClockSyncMode fromString(String value, MessageReporter err) {
    return (ClockSyncMode) UnionType.CLOCK_SYNC_UNION.forName(value);
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public void validate(
      KeyValuePair pair, Model ast, TargetConfig config, MessageReporter reporter) {
    super.validate(pair, ast, config, reporter);
    if (pair != null) {
      boolean federatedExists = false;
      for (Reactor reactor : ast.getReactors()) {
        if (reactor.isFederated()) {
          federatedExists = true;
        }
      }
      if (!federatedExists) {
        reporter
            .at(pair, Literals.KEY_VALUE_PAIR__NAME)
            .warning("The clock-sync target property is incompatible with non-federated programs.");
      }
    }
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.value.toString());
  }

  /**
   * Enumeration of clock synchronization modes.
   *
   * <ul>
   *   <li>OFF: The clock synchronization is universally off.
   *   <li>STARTUP: Clock synchronization occurs at startup only.
   *   <li>ON: Clock synchronization occurs at startup and at runtime.
   * </ul>
   *
   * @author Edward A. Lee
   */
  public enum ClockSyncMode {
    OFF,
    INIT,
    ON;
    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }
  }
}
