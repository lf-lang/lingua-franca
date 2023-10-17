package org.lflang.target.property;

/** Directive to stage particular files on the class path to be processed by the code generator. */
public final class FilesProperty extends FileListProperty {

  /** Singleton target property instance. */
  public static final FilesProperty INSTANCE = new FilesProperty();

  private FilesProperty() {
    super();
  }

  @Override
  public String name() {
    return "files";
  }
}
