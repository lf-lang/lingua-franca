package org.lflang.tests.compiler;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.ast.FormattingUtils;
import org.lflang.ast.IsEqual;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;
import org.lflang.tests.LfParsingUtil;

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
                """
        );
    }

    @Test
    public void testAssignments() {
        assertIsFormatted(
            """
                target C

                reactor Destination {
                    input ok: bool
                    input in: int
                    state last_invoked: tag_t({= NEVER_TAG_INITIALIZER =})
                }
                """
        );
    }

    @Test
    public void testState() {
        assertFormatsTo(
            """
                target Python

                reactor Destination {
                    state  one_init: tag_t( {= NEVER_TAG_INITIALIZER =})
                    state no_init:   tag_t
                     state list_init(1,2)
                }
                """,
            """
                target Python

                reactor Destination {
                    state one_init: tag_t({= NEVER_TAG_INITIALIZER =})
                    state no_init: tag_t
                    state list_init(1, 2)
                }
                """
        );
    }

    private void assertIsFormatted(String input) {
        assertFormatsTo(input, input);
    }

    private void assertFormatsTo(String input, String expectedOutput) {
        Model inputModel = LfParsingUtil.parseValidModel("test input", input);
        String formattedString = FormattingUtils.render(inputModel);
        Assertions.assertEquals(
            expectedOutput,
            formattedString,
            "Formatted output is different from what was expected"
        );
    }


}
