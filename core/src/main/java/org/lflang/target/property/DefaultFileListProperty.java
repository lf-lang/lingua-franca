package org.lflang.target.property;


import java.util.ArrayList;
import java.util.List;

import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;


public abstract class DefaultFileListProperty extends TargetPropertyConfig<List<String>> {

    public DefaultFileListProperty() {
        super(UnionType.FILE_OR_FILE_ARRAY);
    }

    @Override
    public void override(List<String> value) { // FIXME: should this be override or update?
        this.setByUser = true;
        this.value.addAll(value);
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
    public Element export() {
        return ASTUtils.toElement(value);
    }
}
