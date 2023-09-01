package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.validation.LFValidator.ValidationReporter;

public class AuthConfigurator implements TargetPropertyConfig<Boolean> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {
        config.auth = this.parse(value);
    }

    @Override
    public Boolean parse(Element value) {
        return ASTUtils.toBoolean(value);
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {

    }

    @Override
    public Element getPropertyElement(TargetConfig config) {
        return ASTUtils.toElement(config.auth);
    }
}
