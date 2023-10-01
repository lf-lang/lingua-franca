package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;

public class KeepaliveProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return Target.ALL;
  }

  @Override
  public void validate(KeyValuePair pair, Model ast, MessageReporter reporter) {
    if (pair != null && ASTUtils.getTarget(ast) == Target.CPP) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .warning(
              "The keepalive property is inferred automatically by the C++ "
                  + "runtime and the value given here is ignored");
    }
  }
}
