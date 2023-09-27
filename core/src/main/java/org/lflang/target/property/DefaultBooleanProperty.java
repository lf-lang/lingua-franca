package org.lflang.target.property;


import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;


public abstract class DefaultBooleanProperty extends TargetPropertyConfig<Boolean> {

    public DefaultBooleanProperty() {
        super(PrimitiveType.BOOLEAN);
    }

    @Override
    public Boolean initialValue() {
        return false;
    }

    @Override
    public Boolean parse(Element value) {
        return ASTUtils.toBoolean(value);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(value);
    }
}
