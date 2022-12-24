/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.lflang.tests.compiler;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.lf.Model;
import org.lflang.tests.LFInjectorProvider;

import com.google.inject.Inject;

/**
 * Test harness for ensuring that grammar captures
 * all corner cases.
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
class LinguaFrancaParsingTest {
    @Inject
    ParseHelper<Model> parser;

    @Test
    public void checkForTarget() throws Exception {
        String testCase = """
            targett C;
            reactor Foo {
            }
        """;
        Model result = parser.parse(testCase);
        Assertions.assertNotNull(result);
        Assertions.assertFalse(result.eResource().getErrors().isEmpty(), "Failed to catch misspelled target keyword.");
    }

    @Test
    public void testAttributes() throws Exception {
        String testCase = """
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
        String testCase = """
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
        String testCase = """
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
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
                              "Encountered unexpected error while parsing: " +
                                  model.eResource().getErrors());
        return model;
    }

}
