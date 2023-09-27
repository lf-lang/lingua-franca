package org.lflang.target.property;


import java.util.List;

import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;


public class TimeOutProperty extends TargetPropertyConfig<TimeValue> {

    public TimeOutProperty() {
        super(PrimitiveType.TIME_VALUE);
    }

    @Override
    public TimeValue initialize() {
        return null;
    }

    @Override
    public TimeValue parse(Element value) {
        return ASTUtils.toTimeValue(value);
    }

    @Override
    public List<Target> supportedTargets() {
        return Target.ALL;
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(value);
    }
}
