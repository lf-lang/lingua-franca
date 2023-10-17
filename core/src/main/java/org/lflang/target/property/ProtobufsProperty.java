package org.lflang.target.property;

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
  public String name() {
    return "protobufs";
  }
}
