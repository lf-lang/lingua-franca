package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;

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
  public void validate(KeyValuePair pair, Model ast, MessageReporter reporter) {
    if (pair != null) {
      // Check for federated
      for (Reactor reactor : ast.getReactors()) {
        // Check to see if the program has a federated reactor
        if (reactor.isFederated()) {
          reporter
              .at(pair, Literals.KEY_VALUE_PAIR__NAME)
              .error("The fast target property is incompatible with federated programs.");
          break;
        }
      }

      final var target = ASTUtils.getTarget(ast);
      if (target != Target.CPP) {
        // Check for physical actions
        for (Reactor reactor : ast.getReactors()) {
          // Check to see if the program has a physical action in a reactor
          for (Action action : reactor.getActions()) {
            if (action.getOrigin().equals(ActionOrigin.PHYSICAL)) {
              reporter
                  .at(pair, Literals.KEY_VALUE_PAIR__NAME)
                  .error(
                      String.format(
                          "In the %s target, the fast target property is incompatible with physical"
                              + " actions.",
                          target.toString()));
              break;
            }
          }
        }
      }
    }
  }
}
