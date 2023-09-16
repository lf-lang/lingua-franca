package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;

import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

public class BuildCommandsConfig extends TargetPropertyConfig<List<String>> {

    @Override
    public List<String> initialize() {
        return new ArrayList<>();
    }

    @Override
    public List<String> parse(Element value) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value.toString());
    }

}
