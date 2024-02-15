package org.lflang.target.property;

import java.util.Objects;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.ClockSyncModeType;
import org.lflang.target.property.type.ClockSyncModeType.ClockSyncMode;

/** The mode of clock synchronization to be used in federated programs. The default is 'initial'. */
public final class ClockSyncModeProperty extends TargetProperty<ClockSyncMode, ClockSyncModeType> {

  /** Singleton target property instance. */
  public static final ClockSyncModeProperty INSTANCE = new ClockSyncModeProperty();

  private ClockSyncModeProperty() {
    super(new ClockSyncModeType());
  }

  @Override
  public ClockSyncMode initialValue() {
    return ClockSyncMode.INIT;
  }

  @Override
  public ClockSyncMode fromAst(Element node, MessageReporter reporter) {
    var mode = fromString(ASTUtils.elementToSingleString(node), reporter);
    return Objects.requireNonNullElse(mode, ClockSyncMode.INIT);
  }

  @Override
  protected ClockSyncMode fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public Element toAstElement(ClockSyncMode value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public String name() {
    return "clock-sync";
  }
}
