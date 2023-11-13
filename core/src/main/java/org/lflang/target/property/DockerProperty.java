package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.target.property.DockerProperty.DockerOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

/**
 * Directive to generate a Dockerfile. This is either a boolean, true or false, or a dictionary of
 * options.
 */
public final class DockerProperty extends TargetProperty<DockerOptions, UnionType> {

  /** Singleton target property instance. */
  public static final DockerProperty INSTANCE = new DockerProperty();

  private DockerProperty() {
    super(UnionType.DOCKER_UNION);
  }

  @Override
  public DockerOptions initialValue() {
    return new DockerOptions(false);
  }

  @Override
  public DockerOptions fromAst(Element node, MessageReporter reporter) {
    var options = new DockerOptions(false);
    if (node.getLiteral() != null) {
      if (ASTUtils.toBoolean(node)) {
        options.enabled = true;
      }
    } else if (node.getKeyvalue() != null) {

      options.enabled = true;
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        DockerOption option = (DockerOption) DictionaryType.DOCKER_DICT.forName(entry.getName());
        if (option != null && option.equals(DockerOption.FROM)) {
          options.from = ASTUtils.elementToSingleString(entry.getValue());
        }
      }
    }
    return options;
  }

  @Override
  protected DockerOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement(DockerOptions value) {
    if (!value.enabled) {
      return null;
    } else if (value.equals(new DockerOptions(true))) {
      // default configuration
      return ASTUtils.toElement(true);
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      for (DockerOption opt : DockerOption.values()) {
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(opt.toString());
        if (opt == DockerOption.FROM) {
          if (value.from == null) {
            continue;
          }
          pair.setValue(ASTUtils.toElement(value.from));
        }
        kvp.getPairs().add(pair);
      }
      e.setKeyvalue(kvp);
      if (kvp.getPairs().isEmpty()) {
        return null;
      }
      return e;
    }
  }

  @Override
  public String name() {
    return "docker";
  }

  /** Settings related to Docker options. */
  public static class DockerOptions {

    public boolean enabled;

    public DockerOptions(boolean enabled) {
      this.enabled = enabled;
    }

    /**
     * The base image and tag from which to build the Docker image. The default is "alpine:latest".
     */
    public String from = "alpine:latest";

    @Override
    public boolean equals(Object o) {
      if (this == o) {
        return true;
      }
      if (o == null || getClass() != o.getClass()) {
        return false;
      }
      DockerOptions that = (DockerOptions) o;
      return from.equals(that.from);
    }
  }

  /**
   * Docker options.
   *
   * @author Edward A. Lee
   */
  public enum DockerOption implements DictionaryElement {
    FROM("FROM", PrimitiveType.STRING);

    public final PrimitiveType type;

    private final String description;

    DockerOption(String alias, PrimitiveType type) {
      this.description = alias;
      this.type = type;
    }

    /** Return the description of this dictionary element. */
    @Override
    public String toString() {
      return this.description;
    }

    /** Return the type associated with this dictionary element. */
    public TargetPropertyType getType() {
      return this.type;
    }
  }
}
