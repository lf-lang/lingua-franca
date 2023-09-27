package org.lflang.target.property;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

public class BuildCommandsProperty extends TargetPropertyConfig<List<String>> {


    public BuildCommandsProperty() {
        super(UnionType.STRING_OR_STRING_ARRAY);
    }

    @Override
    public List<String> initialize() {
        return new ArrayList<>();
    }

    @Override
    public List<String> parse(Element value) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.C, Target.CCPP);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value.toString());
    }

}
