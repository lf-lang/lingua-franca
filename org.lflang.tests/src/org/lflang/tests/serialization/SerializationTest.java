package org.lflang.tests.serialization;

import java.util.Properties;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;

public class SerializationTest extends TestBase {

    protected SerializationTest() {
        super(Target.ALL);        
    }

    @Override
    protected void addExtraLfcArgs(Properties args) {
        super.addExtraLfcArgs(args);
        // Use the Debug build type as coverage generation does not work for the serialization tests
        args.setProperty("build-type", "Debug");
    }
    
    @Test
    public void runSerializationTestsWithThreadingOff() {
        Assumptions.assumeTrue(supportsSingleThreadedExecution(), Message.NO_SINGLE_THREADED_SUPPORT);
        runTestsForTargets(Message.DESC_SERIALIZATION,
                TestCategory.SERIALIZATION::equals, Configurators::disableThreading,
                false);
    }
    
    @Test
    public void runSerializationTests() {
        Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
        runTestsForTargets(Message.DESC_SERIALIZATION,
                TestCategory.SERIALIZATION::equals, Configurators::noChanges,
                false);
    }
}
