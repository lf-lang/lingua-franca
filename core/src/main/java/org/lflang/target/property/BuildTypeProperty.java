package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.BuildTypeProperty.BuildType;
import org.lflang.target.property.type.UnionType;

/**
 * Directive to specify the target build type such as 'Release' or 'Debug'. This is also used in the
 * Rust target to select a Cargo profile.
 */
public class BuildTypeProperty extends AbstractTargetProperty<BuildType> {

  public BuildTypeProperty() {
    super(UnionType.BUILD_TYPE_UNION);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.get().toString());
  }

  @Override
  public BuildType initialValue() {
    return BuildType.RELEASE;
  }

  @Override
  public BuildType fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  @Override
  protected BuildType fromString(String string, MessageReporter reporter) {
    return (BuildType) UnionType.BUILD_TYPE_UNION.forName(string);
  }

  @Override
  public String name() {
    return "build-type";
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Rust);
  }

  /**
   * Enumeration of Cmake build types. These are also mapped to Cargo profiles for the Rust target
   * (see {@link org.lflang.generator.rust.RustTargetConfig})
   *
   * @author Christian Menard
   */
  public enum BuildType {
    RELEASE("Release"),
    DEBUG("Debug"),
    TEST("Test"),
    REL_WITH_DEB_INFO("RelWithDebInfo"),
    MIN_SIZE_REL("MinSizeRel");

    /** Alias used in toString method. */
    private final String alias;

    /** Private constructor for Cmake build types. */
    BuildType(String alias) {
      this.alias = alias;
    }

    /** Return the alias. */
    @Override
    public String toString() {
      return this.alias;
    }
  }
}
