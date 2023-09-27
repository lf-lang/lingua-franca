package org.lflang.target.property;


import java.util.List;

import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;


public abstract class DefaultStringConfig extends TargetPropertyConfig<String> {

    public DefaultStringConfig() {
        super(PrimitiveType.STRING);
    }

    @Override
    public String initialize() {
        return "";
    }

    @Override
    public String parse(Element value) {
        return ASTUtils.elementToSingleString(value);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(value);
    }
}
