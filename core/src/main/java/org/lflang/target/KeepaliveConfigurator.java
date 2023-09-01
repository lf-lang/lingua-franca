package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.validation.LFValidator.ValidationReporter;

public class KeepaliveConfigurator implements TargetPropertyConfig<Boolean> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {
        config.keepalive = this.parse(value);
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
    public Element getPropertyElement(TargetConfig config) {
        return ASTUtils.toElement(config.keepalive);
    }
}
