package org.lflang.target.property;

public class BuildConfig {

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
