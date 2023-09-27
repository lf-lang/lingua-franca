package org.lflang.target.property;

import java.util.List;

import org.lflang.Target;

public class ThreadingProperty extends DefaultBooleanProperty {

    @Override
    public List<Target> supportedTargets() {
        return List.of(Target.C, Target.CCPP, Target.Python);
    }


    @Override
    public Boolean initialize() {
        return true;
    }
}
