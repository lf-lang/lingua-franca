package org.lflang.target.property.type;

import java.util.List;

import org.lflang.Target;
import org.lflang.target.property.DefaultStringConfig;

public class CompilerConfig extends DefaultStringConfig {

    @Override
    public List<Target> supportedTargets() {
        return Target.ALL;
    }
}
