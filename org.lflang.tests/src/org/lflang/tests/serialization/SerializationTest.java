package org.lflang.tests.serialization;

import java.util.List;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.Target;
import org.lflang.tests.AbstractTest;
import org.lflang.tests.TestBase.Message;

public class SerializationTest extends AbstractTest {

    protected SerializationTest() {
        super(Target.ALL);        
    }

    @Test
    @Override
    public void runSerializationTests() {
        Assumptions.assumeFalse(isWindows(), Message.NO_WINDOWS_SUPPORT);
        super.runSerializationTests();
    }
}
