package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.ThreadPolicyType;
import org.lflang.target.property.type.ThreadPolicyType.ThreadPolicy;

/**
 * Directive to specify the target build type such as 'Release' or 'Debug'. This is also used in the
 * Rust target to select a Cargo profile.
 */
public final class ThreadPolicyProperty extends TargetProperty<ThreadPolicy, ThreadPolicyType> {

  /** Singleton target property instance. */
  public static final ThreadPolicyProperty INSTANCE = new ThreadPolicyProperty();

  private ThreadPolicyProperty() {
    super(new ThreadPolicyType());
  }

  @Override
  public Element toAstElement(ThreadPolicy value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public ThreadPolicy initialValue() {
    return ThreadPolicy.NORMAL;
  }

  @Override
  public ThreadPolicy fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  @Override
  protected ThreadPolicy fromString(String string, MessageReporter reporter) {
    return this.type.forName(string);
  }

  @Override
  public String name() {
    return "thread-policy";
  }
}
