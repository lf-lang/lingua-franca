package org.lflang.target;

import java.util.ArrayList;
import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.validation.ValidationReporter;

public class Ros2DependenciesConfig extends TargetPropertyConfig<List<String>> {


    @Override
    public List<String> initialize() {
        return new ArrayList<>();
    }

    @Override
    public List parse(Element value) {
        return ASTUtils.elementToListOfStrings(value);
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
    public Element export() {
        return ASTUtils.toElement(value);
    }

}
