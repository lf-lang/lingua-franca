package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * Directive for specifying .proto files that need to be compiled and their code included in the
 * sources.
 */
public class ProtobufsProperty extends AbstractFileListProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.TS, Target.Python);
  }

  @Override
  public String name() {
    return "protobufs";
  }
}
