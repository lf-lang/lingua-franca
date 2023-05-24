package org.lflang.tests.compiler;

import static java.util.Collections.emptyList;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.junit.jupiter.api.Assertions.fail;

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
  public void roundTripTest() {
    for (Target target : Target.values()) {
      for (TestCategory category : TestCategory.values()) {
        for (LFTest test : TestRegistry.getRegisteredTests(target, category, false)) {
          try {
            run(test.getSrcPath());
          } catch (Throwable thrown) {
            fail("Test case " + test.getSrcPath() + " failed", thrown);
          }
        }
      }
    }
  }

  private void run(Path file) throws Exception {
    Model originalModel = LfParsingUtil.parse(file);
    System.out.println(file);
    assertThat(originalModel.eResource().getErrors(), equalTo(emptyList()));
    // TODO: Check that the output is a fixed point
    final int smallLineLength = 20;
    final String squishedTestCase = FormattingUtils.render(originalModel, smallLineLength);
    final Model resultingModel =
        LfParsingUtil.parseSourceAsIfInDirectory(file.getParent(), squishedTestCase);

    assertThat(resultingModel.eResource().getErrors(), equalTo(emptyList()));
    Assertions.assertTrue(
        new IsEqual(originalModel).doSwitch(resultingModel),
        String.format(
            "The reformatted version of %s was not equivalent to the original file.%n"
                + "Formatted file:%n%s%n%n",
            file, squishedTestCase));
  }
}
