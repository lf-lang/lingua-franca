package org.lflang.target.property;


import java.util.ArrayList;
import java.util.List;

import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;


public abstract class DefaultStringListProperty extends TargetPropertyConfig<List<String>> {

    public DefaultStringListProperty() {
        super(UnionType.STRING_OR_STRING_ARRAY);
    }

    @Override
    public List<String> initialValue() {
        return new ArrayList<>();
    }

    @Override
    public List<String> parse(Element value) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(value);
    }

}
