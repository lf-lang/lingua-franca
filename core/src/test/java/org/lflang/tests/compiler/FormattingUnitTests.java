package org.lflang.tests.compiler;

import com.google.inject.Inject;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.ast.FormattingUtil;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LfParsingTestHelper;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public class FormattingUnitTests {

  @Test
  public void testSimple() {
    assertFormatsTo(
        """
                target C
                reactor Main{ }
                """,
        """
                target C

                reactor Main {
                }
                """);
  }

  @Test
  public void testAssignments() {
    assertFormatsTo(
        """
                target C

                reactor Destination {
                    input ok: bool
                    input in: int
                    state last_invoked: tag_t({= NEVER_TAG_INITIALIZER =})
                }
                """,
        """
                target C

                reactor Destination {
                  input ok: bool
                  input in: int
                  state last_invoked: tag_t = {= NEVER_TAG_INITIALIZER =}
                }
                """);
  }

  @Test
  public void testState() {
    assertFormatsTo(
        """
                target Python

                reactor Destination {
                    state  one_init: tag_t( {= NEVER_TAG_INITIALIZER =})
                    state no_init:   tag_t
                     state list_init(1,2) // this syntax is deprecated
                }
                """,
        """
                target Python

                reactor Destination {
                  state one_init: tag_t = {= NEVER_TAG_INITIALIZER =}
                  state no_init: tag_t
                  state list_init(1, 2)  # this syntax is deprecated
                }
                """);
  }

  @Test
  public void testCppInits() {
    assertIsFormatted(
        """
                target Cpp

                reactor Destination {
                  state one_init: tag_t({= NEVER_TAG_INITIALIZER =})
                  state no_init: tag_t
                  state assign: int = 0
                  state paren: int(0)
                  state brace: std::vector<int>{1, 2}
                  state paren_list: std::vector<int>(1, 2)
                }
                """);
  }

  @Inject LfParsingTestHelper parser;

  private void assertIsFormatted(String input) {
    assertFormatsTo(input, input);
  }

  private void assertFormatsTo(String input, String expectedOutput) {
    Model inputModel = parser.parseValidModel("test input", input);
    String formattedString = FormattingUtil.render(inputModel);
    Assertions.assertEquals(
        expectedOutput, formattedString, "Formatted output is different from what was expected");
  }
}
