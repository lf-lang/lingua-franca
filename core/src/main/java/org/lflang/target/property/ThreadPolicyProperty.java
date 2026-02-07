package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.ThreadPolicyType;
import org.lflang.target.property.type.ThreadPolicyType.ThreadPolicy;

/**
 * Directive to specify the thread scheduling policy.
 *
 * <p>This property controls how worker threads are scheduled by the operating system:
 * <ul>
 *   <li>"normal" (LF_SCHED_FAIR) - Default non real-time scheduling</li>
 *   <li>"rt-rr" (LF_SCHED_TIMESLICE) - Real-time round-robin scheduling</li>
 *   <li>"rt-fifo" (LF_SCHED_PRIORITY) - Real-time FIFO priority scheduling</li>
 * </ul>
 *
 * <p>Note: Real-time scheduling policies typically require elevated privileges (e.g., root on Linux).
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
    return ThreadPolicy.getDefault();
  }

  @Override
  public ThreadPolicy fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  @Override
  protected ThreadPolicy fromString(String string, MessageReporter reporter) {
    ThreadPolicy result = this.type.forName(string);
    if (result == null) {
      reporter
          .nowhere()
          .error(
              "Invalid thread policy: '"
                  + string
                  + "'. "
                  + "Allowed values are: "
                  + this.type.optionsString());
      return initialValue();
    }
    return result;
  }

  @Override
  public String name() {
    return "thread-policy";
  }
}

