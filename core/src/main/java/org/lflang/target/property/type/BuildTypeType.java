package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.target.property.type.BuildTypeType.BuildType;

/** Enumeration of supported platforms */
public class BuildTypeType extends OptionsType<BuildType> {

  @Override
  protected Class<BuildType> enumClass() {
    return BuildType.class;
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

    public static List<BuildType> optionsList() {
      return Arrays.stream(BuildType.values()).collect(Collectors.toList());
    }

    public static BuildType getDefault() {
      return BuildType.DEBUG;
    }
  }
}
