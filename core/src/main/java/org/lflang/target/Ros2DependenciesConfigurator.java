package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.validation.LFValidator.ValidationReporter;

public class Ros2DependenciesConfigurator implements TargetPropertyConfig<Boolean> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {

    }

    @Override
    public Boolean parse(Element value) {
        return null;
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        var ros2enabled = TargetProperty.getKeyValuePair(ast, TargetProperty.ROS2);
        if (pair != null && (ros2enabled == null || !ASTUtils.toBoolean(ros2enabled.getValue()))) {
            reporter.warning(
                "Ignoring ros2-dependencies as ros2 compilation is disabled",
                pair,
                Literals.KEY_VALUE_PAIR__NAME);
        }
    }

    @Override
    public Element getPropertyElement(TargetConfig config) {
        return null;
    }
}
