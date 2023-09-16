package org.lflang.target;

import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

public class AuthConfig extends TargetPropertyConfig<Boolean> {

    @Override
    public Boolean initialize() {
        return false;
    }

    @Override
    public Boolean parse(Element value) {
        return ASTUtils.toBoolean(value);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value.toString());
    }

}
