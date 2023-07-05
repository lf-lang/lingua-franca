package org.lflang.tests.compiler;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.ast.IsEqual;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LfParsingTestHelper;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class EquivalenceUnitTests {

  @Test
  public void testSimple() {
    assertSelfEquivalence(
        """
                target C

                reactor Destination {
                    input ok: bool
                    input in: int
                    state last_invoked: tag_t({= NEVER_TAG_INITIALIZER =})
                }
                """);
  }

  @Test
  public void testCodeExprEqItselfModuloIndent() {
    assertEquivalent(
        """
                target C
                reactor Destination {
                    state s: tag_t({=
                        NEVER_TAG_INITIALIZER
                    =})
                }
                """,
        """
                target C
                reactor Destination {
                    state s: tag_t({= NEVER_TAG_INITIALIZER =})
                }
                """);
  }

  @Test
  public void testInitializerParensAreIrrelevantInAssignment() {
    assertEquivalent(
        """
                target C
                reactor A(a: int(0)) {}
                main reactor {
                    a = new A(a = 1)
                }
                """,
        """
                target C
                reactor A(a: int(0)) {}
                main reactor {
                    a = new A(a = (1)) // mind the parens here.
                }
                """);
  }

  private void assertSelfEquivalence(String input) {
    LfParsingTestHelper parser = new LfParsingTestHelper();
    Model inputModel = parser.parseValidModel("test input", input);
    // need to parse twice otherwise they are trivially equivalent
    // because they're the same object.
    Model otherModel = parser.parseValidModel("other", input);

    // test equivalence of the models.
    Assertions.assertTrue(
        new IsEqual(inputModel).doSwitch(otherModel),
        String.format("Model is not equivalent to itself. Source:%n%s", input));
  }

  private void assertEquivalent(String input, String other) {
    LfParsingTestHelper parser = new LfParsingTestHelper();
    Model inputModel = parser.parseValidModel("test input", input);
    Model outputModel = parser.parseValidModel("other", other);

    // test equivalence of the models.
    Assertions.assertTrue(
        new IsEqual(inputModel).doSwitch(outputModel),
        String.format(
            "The reformatted model is not equivalent to the original file.%n"
                + "Input file:%n%s%n%n"
                + "Comparand file:%n%s%n%n",
            input, other));
  }
}
