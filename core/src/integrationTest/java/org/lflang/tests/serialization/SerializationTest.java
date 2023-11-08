package org.lflang.tests.serialization;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.type.BuildTypeType.BuildType;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers;

public class SerializationTest extends TestBase {

  protected SerializationTest() {
    super(Target.ALL);
  }

  @Override
  protected void applyDefaultConfiguration(TargetConfig config) {
    super.applyDefaultConfiguration(config);
    // Use the Debug build type as coverage generation does not work for the serialization tests
    BuildTypeProperty.INSTANCE.override(config, BuildType.DEBUG);
  }

  @Test
  public void runSerializationTestsWithThreadingOff() {
    Assumptions.assumeTrue(supportsSingleThreadedExecution(), Message.NO_SINGLE_THREADED_SUPPORT);
    runTestsForTargets(
        Message.DESC_SERIALIZATION,
        TestCategory.SERIALIZATION::equals,
        Transformers::noChanges,
        Configurators::disableThreading,
        TestLevel.EXECUTION,
        false);
  }

  @Test
  public void runSerializationTests() {
    Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
    runTestsForTargets(
        Message.DESC_SERIALIZATION,
        TestCategory.SERIALIZATION::equals,
        Transformers::noChanges,
        Configurators::noChanges,
        TestLevel.EXECUTION,
        false);
  }
}
