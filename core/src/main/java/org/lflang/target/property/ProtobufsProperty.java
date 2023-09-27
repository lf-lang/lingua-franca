package org.lflang.target.property;

import java.util.List;

import org.lflang.Target;

public class ProtobufsProperty extends DefaultFileListProperty {

    @Override
    public List<Target> supportedTargets() {
        return List.of(Target.C, Target.CCPP, Target.TS, Target.Python);
    }

}
