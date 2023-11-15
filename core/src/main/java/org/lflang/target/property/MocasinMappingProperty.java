package org.lflang.target.property;

/** Directive to stage particular files on the class path to be processed by the code generator. */
public final class MocasinMappingProperty extends FileListProperty {

  /** Singleton target property instance. */
  public static final MocasinMappingProperty INSTANCE = new MocasinMappingProperty();

  private MocasinMappingProperty() {
    super();
  }

  @Override
  public String name() {
    return "mocasin-mapping";
  }
}
