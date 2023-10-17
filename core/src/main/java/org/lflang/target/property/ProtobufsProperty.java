package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * Directive for specifying .proto files that need to be compiled and their code included in the
 * sources.
 */
public final class ProtobufsProperty extends FileListProperty {

  /** Singleton target property instance. */
  public static final ProtobufsProperty INSTANCE = new ProtobufsProperty();

  private ProtobufsProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.TS, Target.Python);
  }

  @Override
  public String name() {
    return "protobufs";
  }
}
