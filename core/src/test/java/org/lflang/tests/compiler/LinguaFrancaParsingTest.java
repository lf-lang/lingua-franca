package org.lflang.tests.compiler;

import com.google.inject.Inject;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;

/**
 * Test harness for ensuring that grammar captures all corner cases.
 * @ingroup Tests
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
class LinguaFrancaParsingTest {
  @Inject ParseHelper<Model> parser;

  @Test
  public void checkForTarget() throws Exception {
    String testCase =
        """
            targett C;
            reactor Foo {
            }
        """;
    Model result = parser.parse(testCase);
    Assertions.assertNotNull(result);
    Assertions.assertFalse(
        result.eResource().getErrors().isEmpty(), "Failed to catch misspelled target keyword.");
  }

  @Test
  public void testAttributes() throws Exception {
    String testCase =
        """
            target C;
            @label("somethign", "else")
            @ohio()
            @a
            @bdebd(a="b")
            @bd("abc")
            @bd("abc",)
            @a(a="a", b="b")
            @a(a="a", b="b",)
            main reactor {

            }
        """;
    parseWithoutError(testCase);
  }

  @Test
  public void testAttributeContexts() throws Exception {
    String testCase =
        """
            target C;
            @a
            main reactor(@b parm: int) {

                @ohio reaction() {==}
                @ohio logical action f;
                @ohio timer t;
                @ohio input q: int;
                @ohio output q2: int;
            }
        """;
    parseWithoutError(testCase);
  }

  @Test
  public void testTokenizeEmptyWidth() throws Exception {
    String testCase =
        """
            target C;
            main reactor {
                state foo: int[];
                state foo: int[   ]; //spaces are allowed
            }
        """;
    parseWithoutError(testCase);
  }

  private Model parseWithoutError(String s) throws Exception {
    Model model = parser.parse(s);
    Assertions.assertNotNull(model);
    Assertions.assertTrue(
        model.eResource().getErrors().isEmpty(),
        "Encountered unexpected error while parsing: " + model.eResource().getErrors());
    return model;
  }
}
