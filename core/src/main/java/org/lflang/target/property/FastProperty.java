package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Reactor;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;

/**
 * If true, configure the execution environment such that it does not wait for physical time to
 * match logical time. The default is false.
 */
public final class FastProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final FastProperty INSTANCE = new FastProperty();

  private FastProperty() {
    super();
  }

  @Override
  public String name() {
    return "fast";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var pair = config.lookup(this);
    if (config.isSet(this) && config.isFederated()) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .error("The fast target property is incompatible with federated programs.");
    }

    if (config.target != Target.CPP) {
      // Check for physical actions
      for (Reactor reactor : ASTUtils.getAllReactors(config.getMainResource())) {
        // Check to see if the program has a physical action in a reactor
        for (Action action : reactor.getActions()) {
          if (action.getOrigin().equals(ActionOrigin.PHYSICAL)) {
            reporter
                .at(pair, Literals.KEY_VALUE_PAIR__NAME)
                .error(
                    String.format(
                        "In the %s target, the fast target property is incompatible with physical"
                            + " actions.",
                        config.target.toString()));
            break;
          }
        }
      }
    }
  }
}
