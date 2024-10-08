package org.lflang.target.property;

import org.lflang.LocalStrings;
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
    var noBuild = false;
    var builderBase = "";
    var runnerBase = "";
    var rti = DockerOptions.DOCKERHUB_RTI_IMAGE;
    var shell = DockerOptions.DEFAULT_SHELL;
    var preBuildScript = "";
    var postBuildScript = "";
    var runScript = "";
    var envFile = "";
    var dockerConfigFile = "";

    if (node.getLiteral() != null) {
      if (ASTUtils.toBoolean(node)) {
        enabled = true;
      }
    } else if (node.getKeyvalue() != null) {
      enabled = true;
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        DockerOption option = (DockerOption) DictionaryType.DOCKER_DICT.forName(entry.getName());
        switch (option) {
          case NO_BUILD -> noBuild = ASTUtils.toBoolean(entry.getValue());
          case BUILDER_BASE -> builderBase = ASTUtils.elementToSingleString(entry.getValue());
          case RUNNER_BASE -> runnerBase = ASTUtils.elementToSingleString(entry.getValue());
          case PRE_BUILD_SCRIPT ->
              preBuildScript = ASTUtils.elementToSingleString(entry.getValue());
          case PRE_RUN_SCRIPT -> runScript = ASTUtils.elementToSingleString(entry.getValue());
          case POST_BUILD_SCRIPT ->
              postBuildScript = ASTUtils.elementToSingleString(entry.getValue());
          case RTI_IMAGE -> rti = ASTUtils.elementToSingleString(entry.getValue());
          case ENV_FILE -> envFile = ASTUtils.elementToSingleString(entry.getValue());
          case DOCKER_CONFIG_FILE ->
              dockerConfigFile = ASTUtils.elementToSingleString(entry.getValue());
        }
      }
    }
    return new DockerOptions(
        enabled,
        noBuild,
        builderBase,
        runnerBase,
        rti,
        shell,
        preBuildScript,
        postBuildScript,
        runScript,
        envFile,
        dockerConfigFile);
  }

  @Override
  protected DockerOptions fromString(String string, MessageReporter reporter) {
    if (string.equalsIgnoreCase("true")) {
      return new DockerOptions(true);
    } else if (string.equalsIgnoreCase("false")) {
      return new DockerOptions(false);
    } else {
      throw new UnsupportedOperationException(
          "Docker options other than \"true\" and \"false\" are not supported.");
    }
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
        switch (opt) {
          case NO_BUILD -> pair.setValue(ASTUtils.toElement(value.noBuild));
          case BUILDER_BASE -> pair.setValue(ASTUtils.toElement(value.builderBase));
          case RUNNER_BASE -> pair.setValue(ASTUtils.toElement(value.runnerBase));
          case PRE_BUILD_SCRIPT -> pair.setValue(ASTUtils.toElement(value.preBuildScript));
          case PRE_RUN_SCRIPT -> pair.setValue(ASTUtils.toElement(value.preRunScript));
          case POST_BUILD_SCRIPT -> pair.setValue(ASTUtils.toElement(value.postBuildScript));
          case RTI_IMAGE -> pair.setValue(ASTUtils.toElement(value.rti));
          case ENV_FILE -> pair.setValue(ASTUtils.toElement(value.envFile));
          case DOCKER_CONFIG_FILE -> pair.setValue(ASTUtils.toElement(value.dockerConfigFile));
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
      boolean enabled,
      boolean noBuild,
      String builderBase,
      String runnerBase,
      String rti,
      String shell,
      String preBuildScript,
      String postBuildScript,
      String preRunScript,
      String envFile,
      String dockerConfigFile) {

    /** Default location to pull the rti from. */
    public static final String DOCKERHUB_RTI_IMAGE =
        "lflang/rti:" + LocalStrings.VERSION.toLowerCase();

    public static final String DEFAULT_SHELL = "/bin/sh";

    /** String to indicate a local build of the rti. */
    public static final String LOCAL_RTI_IMAGE = "rti:local";

    public DockerOptions(boolean enabled) {
      this(enabled, false, "", "", DOCKERHUB_RTI_IMAGE, DEFAULT_SHELL, "", "", "", "", "");
    }
  }

  /**
   * Docker options.
   *
   * @author Edward A. Lee
   */
  public enum DockerOption implements DictionaryElement {
    NO_BUILD("no-build", PrimitiveType.BOOLEAN),
    BUILDER_BASE("builder-base", PrimitiveType.STRING),
    ENV_FILE("env-file", PrimitiveType.STRING),
    RUNNER_BASE("runner-base", PrimitiveType.STRING),
    RTI_IMAGE("rti-image", PrimitiveType.STRING),
    PRE_BUILD_SCRIPT("pre-build-script", PrimitiveType.STRING),
    PRE_RUN_SCRIPT("pre-run-script", PrimitiveType.STRING),
    POST_BUILD_SCRIPT("post-build-script", PrimitiveType.STRING),
    DOCKER_CONFIG_FILE("docker-compose-override", PrimitiveType.STRING);

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
