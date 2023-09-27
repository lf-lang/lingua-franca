package org.lflang.target.property;

import java.util.List;

import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

public class WorkersProperty extends TargetPropertyConfig<Integer> {

    public WorkersProperty() {
        super(PrimitiveType.NON_NEGATIVE_INTEGER);
    }

    @Override
    public Integer initialize() {
        return 0;
    }

    @Override
    protected Integer parse(Element value) {
        return ASTUtils.toInteger(value);
    }

    @Override
    public List<Target> supportedTargets() {
        return List.of(Target.C, Target.CCPP, Target.Python, Target.CPP, Target.Rust);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(value);
    }

}
