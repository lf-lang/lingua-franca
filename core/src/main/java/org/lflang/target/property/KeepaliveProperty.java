package org.lflang.target.property;

import static org.lflang.TargetProperty.KEEPALIVE;

import java.util.List;
import java.util.Properties;

import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.DefaultBooleanProperty;
import org.lflang.validation.ValidationReporter;

public class KeepaliveProperty extends DefaultBooleanProperty {

    @Override
    public void update(Properties cliArgs) {
        super.update(cliArgs);
        var key = KEEPALIVE.toString();
        if (cliArgs.containsKey(key)) {
            this.override(Boolean.parseBoolean(cliArgs.getProperty(KEEPALIVE.toString())));
        }
    }

    @Override
    public List<Target> supportedTargets() {
        return Target.ALL;
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        super.validate(pair, ast, config, reporter);
        if (pair != null && config.target == Target.CPP) {
            reporter.warning(
                "The keepalive property is inferred automatically by the C++ "
                    + "runtime and the value given here is ignored",
                pair,
                Literals.KEY_VALUE_PAIR__NAME);
        }
    }

}
