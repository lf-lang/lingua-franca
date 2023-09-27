package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;

import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.type.ArrayType;
import org.lflang.validation.ValidationReporter;

public class Ros2DependenciesProperty extends TargetPropertyConfig<List<String>> {

    public Ros2DependenciesProperty() {
        super(ArrayType.STRING_ARRAY);
    }

    @Override
    public List<String> initialValue() {
        return new ArrayList<>();
    }

    @Override
    public List fromAstElement(Element value) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    public List<Target> supportedTargets() {
        return List.of(Target.CPP);
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        super.validate(pair, ast, config, reporter);
        var ros2enabled = TargetProperty.getKeyValuePair(ast, TargetProperty.ROS2);
        if (pair != null && (ros2enabled == null || !ASTUtils.toBoolean(ros2enabled.getValue()))) {
            reporter.warning(
                "Ignoring ros2-dependencies as ros2 compilation is disabled",
                pair,
                Literals.KEY_VALUE_PAIR__NAME);
        }
    }

    @Override
    public Element toAstElement() {
        return ASTUtils.toElement(value);
    }

}
