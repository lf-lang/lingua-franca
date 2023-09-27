package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;

import org.lflang.Target;

public class NoCompileProperty extends DefaultBooleanProperty {

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.C, Target.CPP, Target.CCPP, Target.Python);
    }

}
