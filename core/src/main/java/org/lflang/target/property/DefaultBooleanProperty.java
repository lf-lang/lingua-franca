package org.lflang.target.property;


import org.lflang.MessageReporter;
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
    public Boolean fromAst(Element value, MessageReporter err) {
        return ASTUtils.toBoolean(value);
    }

    @Override
    protected Boolean fromString(String value, MessageReporter err) {
        return Boolean.parseBoolean(value);
    }

    @Override
    public Element toAstElement() {
        return ASTUtils.toElement(value);
    }
}
