package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.target.property.DockerProperty.DockerOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

public class DockerProperty extends TargetPropertyConfig<DockerOptions> {

  public DockerProperty() {
    super(UnionType.DOCKER_UNION);
  }

  @Override
  public DockerOptions initialValue() {
    return new DockerOptions(false);
  }

  @Override
  public DockerOptions fromAst(Element value, MessageReporter err) {
    var options = new DockerOptions(false);
    if (value.getLiteral() != null) {
      if (ASTUtils.toBoolean(value)) {
        options.enabled = true;
      }
    } else {
      for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
        options.enabled = true;
        DockerOption option = (DockerOption) DictionaryType.DOCKER_DICT.forName(entry.getName());
        if (Objects.requireNonNull(option) == DockerOption.FROM) {
          options.from = ASTUtils.elementToSingleString(entry.getValue());
        }
      }
    }
    return options;
  }

  @Override
  protected DockerOptions fromString(String value, MessageReporter err) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.Python, Target.TS);
  }

  @Override
  public Element toAstElement() {
    if (!this.value.enabled) {
      return null;
    } else if (this.value.equals(new DockerOptions(true))) {
      // default configuration
      return ASTUtils.toElement(true);
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      for (DockerOption opt : DockerOption.values()) {
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(opt.toString());
        if (opt == DockerOption.FROM) {
          if (this.value.from == null) {
            continue;
          }
          pair.setValue(ASTUtils.toElement(this.value.from));
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
