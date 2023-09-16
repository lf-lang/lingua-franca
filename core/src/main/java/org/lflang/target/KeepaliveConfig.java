package org.lflang.target;

import java.util.Properties;

import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.validation.ValidationReporter;

public class KeepaliveConfig extends TargetPropertyConfig<Boolean> {

    @Override
    public Boolean initialize() {
        return false;
    }

    @Override
    public void update(Properties cliArgs) {
        super.update(cliArgs);
        var key = TargetProperty.KEEPALIVE.toString();
        if (cliArgs.containsKey(key)) {
            this.override(Boolean.parseBoolean(cliArgs.getProperty(TargetProperty.KEEPALIVE.description)));
        }
    }

    @Override
    public Boolean parse(Element value) {
        return ASTUtils.toBoolean(value);
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        if (pair != null && config.target == Target.CPP) {
            reporter.warning(
                "The keepalive property is inferred automatically by the C++ "
                    + "runtime and the value given here is ignored",
                pair,
                Literals.KEY_VALUE_PAIR__NAME);
        }
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value);
    }
}
