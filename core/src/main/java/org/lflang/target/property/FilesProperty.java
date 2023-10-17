package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to stage particular files on the class path to be processed by the code generator. */
public final class FilesProperty extends FileListProperty {

  /** Singleton target property instance. */
  public static final FilesProperty INSTANCE = new FilesProperty();

  private FilesProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public String name() {
    return "files";
  }
}
