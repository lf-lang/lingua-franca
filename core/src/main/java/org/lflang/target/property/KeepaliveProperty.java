package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;

public class KeepaliveProperty extends DefaultBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return Target.ALL;
  }

  @Override
  public void validate(
      KeyValuePair pair, Model ast, TargetConfig config, MessageReporter reporter) {
    super.validate(pair, ast, config, reporter);
    if (pair != null && config.target == Target.CPP) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .warning(
              "The keepalive property is inferred automatically by the C++ "
                  + "runtime and the value given here is ignored");
    }
  }
}
