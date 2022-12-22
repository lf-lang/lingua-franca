package org.lflang.tests.compiler;

import java.nio.file.Files;
import java.nio.file.Path;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.Target;
import org.lflang.ast.FormattingUtils;
import org.lflang.ast.IsEqual;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.LfParsingUtil;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class RoundTripTests {

    @Test
    public void roundTripTest() throws Exception {
        for (Target target : Target.values()) {
            for (TestCategory category : TestCategory.values()) {
                for (LFTest test : TestRegistry.getRegisteredTests(target, category, false)) {
                    run(test.getSrcPath());
                }
            }
        }
    }

    private void run(Path file) throws Exception {
        Model originalModel = LfParsingUtil.parse(file);
        Assertions.assertTrue(originalModel.eResource().getErrors().isEmpty());
        // TODO: Check that the output is a fixed point
        final int smallLineLength = 20;
        final String squishedTestCase = FormattingUtils.render(originalModel, smallLineLength);
        final Model resultingModel = LfParsingUtil.parseSourceAsIfInDirectory(file.getParent(), squishedTestCase);

        Assertions.assertTrue(
            new IsEqual(originalModel).doSwitch(resultingModel),
            String.format(
                "The reformatted version of %s was not equivalent to the original file.%n"
                    + "Formatted file:%n%s%n%n",
                file,
                squishedTestCase
            )
        );
        final String normalTestCase = FormattingUtils.render(originalModel);
        Assertions.assertEquals(
            Files.readString(file).replaceAll("\\r\\n?", "\n"),
            normalTestCase,
            "File is not formatted properly, or formatter is bugged. Check " + file
        );
    }

}
