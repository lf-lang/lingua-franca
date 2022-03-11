package org.lflang.tests.serialization;

import java.util.List;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.AbstractTest;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestBase.Message;
import org.lflang.tests.TestBase.TestLevel;
import org.lflang.tests.TestRegistry.TestCategory;

public class SerializationTest extends TestBase {

    protected SerializationTest() {
        super(Target.ALL);        
    }

    @Test
    public void runSerializationTests() {
        Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
        runTestsForTargets(Message.DESC_SERIALIZATION,
                TestCategory.SERIALIZATION::equals, Configurators::noChanges,
                TestLevel.EXECUTION, false);
    }
}
