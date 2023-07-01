package org.lflang.tests.compiler;

import static java.util.Collections.emptyList;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;

import com.google.inject.Inject;
import java.net.URI;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;
import org.lflang.Target;
import org.lflang.ast.FormattingUtil;
import org.lflang.ast.IsEqual;
import org.lflang.ast.LfParsingHelper;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LFTest;
import org.lflang.tests.LfParsingTestHelper;
import org.lflang.tests.TestRegistry;
import org.lflang.tests.TestRegistry.TestCategory;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
@Execution(ExecutionMode.CONCURRENT)
public class RoundTripTests {
  @Inject private LfParsingHelper parser;
  @Inject private TestRegistry testRegistry;

  @TestFactory
  public Collection<DynamicTest> roundTripTestFactory() {
    List<DynamicTest> result = new ArrayList<>();
    Path cwd = Paths.get(".").toAbsolutePath();
    for (Target target : Target.values()) {
      for (TestCategory category : TestCategory.values()) {
        for (LFTest test : testRegistry.getRegisteredTests(target, category, false)) {
          URI testSourceUri = test.getSrcPath().toUri();
          result.add(
              DynamicTest.dynamicTest(
                  "Round trip " + cwd.relativize(test.getSrcPath()),
                  testSourceUri,
                  () -> run(parser, test.getSrcPath())));
        }
      }
    }
    return result;
  }

  private static void run(LfParsingHelper parser, Path file) {
    Model originalModel = parser.parse(file);
    assertThat(originalModel.eResource().getErrors(), equalTo(emptyList()));
    // TODO: Check that the output is a fixed point
    final int smallLineLength = 20;
    final String squishedTestCase = FormattingUtil.render(originalModel, smallLineLength);
    final Model resultingModel =
        parser.parseSourceAsIfInDirectory(file.getParent(), squishedTestCase);
    LfParsingTestHelper.checkValid(file.toString(), resultingModel);

    assertThat(resultingModel.eResource().getErrors(), equalTo(emptyList()));
    Assertions.assertTrue(
        new IsEqual(originalModel).doSwitch(resultingModel),
        String.format(
            "The reformatted version of %s was not equivalent to the original file.%n"
                + "Formatted file:%n%s%n%n",
            file, squishedTestCase));
  }
}
