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
    var enabled = false;
    var from = "";
    var rti = DockerOptions.DOCKERHUB_RTI_IMAGE;
    var preBuildScript = "";
    var runScript = "";

    if (node.getLiteral() != null) {
      if (ASTUtils.toBoolean(node)) {
        enabled = true;
      }
    } else if (node.getKeyvalue() != null) {
      enabled = true;
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        DockerOption option = (DockerOption) DictionaryType.DOCKER_DICT.forName(entry.getName());
        if (option == null) {
          continue;
        }
        switch (option) {
          case FROM:
            from = ASTUtils.elementToSingleString(entry.getValue());
            break;
          case RTI_IMAGE:
            rti = ASTUtils.elementToSingleString(entry.getValue());
            break;
          case PRE_BUILD_SCRIPT:
            preBuildScript = ASTUtils.elementToSingleString(entry.getValue());
            break;
          case RUN_SCRIPT:
            runScript = ASTUtils.elementToSingleString(entry.getValue());
            break;
        }
      }
    }
    return new DockerOptions(enabled, from, rti, preBuildScript, runScript);
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
        if (opt == DockerOption.RTI_IMAGE) {
          if (value.rti.equals(DockerOptions.DOCKERHUB_RTI_IMAGE)) {
            continue;
          }
          pair.setValue(ASTUtils.toElement(value.rti));
        }
        if (opt == DockerOption.RUN_SCRIPT) {
          if (!value.runScript.isEmpty()) {
            continue;
          }
          pair.setValue(ASTUtils.toElement(value.runScript));
        }
        if (opt == DockerOption.PRE_BUILD_SCRIPT) {
          if (!value.preBuildScript.isEmpty()) {
            continue;
          }
          pair.setValue(ASTUtils.toElement(value.preBuildScript));
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
  public record DockerOptions(
      boolean enabled, String from, String rti, String preBuildScript, String runScript) {

    /** Default location to pull the rti from. */
    public static final String DOCKERHUB_RTI_IMAGE = "lflang/rti:rti";

    /** String to indicate a local build of the rti. */
    public static final String LOCAL_RTI_IMAGE = "rti:local";

    public DockerOptions(boolean enabled) {
      this(enabled, "", DOCKERHUB_RTI_IMAGE, "", "");
    }
  }

  /**
   * Docker options.
   *
   * @author Edward A. Lee
   */
  public enum DockerOption implements DictionaryElement {
    FROM("FROM", PrimitiveType.STRING),
    RTI_IMAGE("rti-image", PrimitiveType.STRING),
    PRE_BUILD_SCRIPT("pre-build-script", PrimitiveType.STRING),
    RUN_SCRIPT("run-script", PrimitiveType.STRING);

    public final PrimitiveType type;

    public final String option;

    DockerOption(String option, PrimitiveType type) {
      this.option = option;
      this.type = type;
    }

    /** Return the type associated with this dictionary element. */
    public TargetPropertyType getType() {
      return this.type;
    }

    /** Return the description of this dictionary element. */
    @Override
    public String toString() {
      return this.option;
    }
  }
}
