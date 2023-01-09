/* ASTUtils Unit Tests. */

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

import static org.lflang.ASTUtils.isInitialized;
import static org.lflang.util.IteratorUtil.asStream;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import javax.inject.Inject;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.ASTUtils;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Literal;
import org.lflang.lf.Model;
import org.lflang.lf.Parameter;
import org.lflang.lf.StateVar;
import org.lflang.tests.LFInjectorProvider;

/**
 * Collection of unit tests on the ASTutils.
 * 
 * @author Christian Menard
 */

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)

class LinguaFrancaASTUtilsTest {
    @Inject
    ParseHelper<Model> parser;
    
    /**
     * Test that isInititialized returns true for inititialized state variables
     */
    @Test
    public void initializedState() throws Exception {
// Java 17:
//        Model model = parser.parse("""
//            target Cpp;
//            main reactor Foo {
//                state a();
//                state b:int(3);
//                state c:int[](1,2,3);
//                state d(1 sec);
//                state e(1 sec, 2 sec, 3 sec);
//            }
//        """);
// Java 11:
        Model model = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target Cpp;", 
            "main reactor {",
            "    state a();",
            "    state b:int(3);",
            "    state c:int[](1,2,3);",
            "    state d(1 sec);",
            "    state e(1 sec, 2 sec, 3 sec);",
            "}"
        ));
        


        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
            "Encountered unexpected error while parsing: " +
                model.eResource().getErrors());
                
        model.eAllContents().forEachRemaining((obj) -> {
            if (obj instanceof StateVar) {
                Assertions.assertTrue(isInitialized((StateVar)obj));
            }
        });
    }
    
    /**
     * Test that isInititialized returns false for uninititialized state variables
     */
    @Test
    public void uninitializedState() throws Exception {
// Java 17:
//        Model model = parser.parse("""
//            target Cpp;
//            main reactor Foo {
//                state a;
//                state b:int;
//                state c:int[];
//                state d:time;
//                state e:time[];
//            }
//        '''
// Java 11:
        Model model = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target Cpp;", 
            "main reactor {",
            "    state a;",
            "    state b:int;",
            "    state c:int[];",
            "    state d:time;",
            "    state e:time;",
            "}"
        ));
        
        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
            "Encountered unexpected error while parsing: " +
                model.eResource().getErrors());

        model.eAllContents().forEachRemaining((obj) -> {
            if (obj instanceof StateVar) {
                Assertions.assertFalse(isInitialized((StateVar)obj));
            }
        });

    }
    /**
     * Return a map from strings to instantiations given a model.
     * 
     * @param model The model to discover instantiations in.
     */
    private Map<String, Instantiation> getInsts(Model model) {
       return asStream(model.eAllContents())
                .filter(obj -> obj instanceof Instantiation)
                .map(obj -> (Instantiation) obj)
                .collect(Collectors.toMap(Instantiation::getName, it -> it));
    }
    
    /**
     * Test reading initial values of parameters.
     * This checks that the example given in the documentation of the
     * ASTUtils.initialValue() function behaves as stated in the docs.
     */
     @Test
     public void initialValue() throws Exception {

// Java 17:        
//        Model model = parser.parse("""
//            target C;
//            reactor A(x:int(1)) {}
//            reactor B(y:int(2)) {
//                a1 = new A(x = y);
//                a2 = new A(x = -1);
//            }
//            reactor C(z:int(3)) {
//                b1 = new B(y = z);
//                b2 = new B(y = -2);
//            }
//        """
// Java 11:
        
        Model model = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target C;",
            "reactor A(x:int(1)) {}", 
            "reactor B(y:int(2)) {",
            "    a1 = new A(x = y);",
            "    a2 = new A(x = -1);",
            "}",
            "reactor C(z:int(3)) {",
            "    b1 = new B(y = z);",
            "    b2 = new B(y = -2);",
            "}"
        ));
        
        Assertions.assertNotNull(model);
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
                "Encountered unexpected error while parsing: " +
                model.eResource().getErrors());
        
        var map = getInsts(model);
        
        /* Check for this:
         *     initialValue(x, null) returns 1
         *     initialValue(x, [a1]) returns 2
         *     initialValue(x, [a2]) returns -1
         *     initialValue(x, [a1, b1]) returns 3
         *     initialValue(x, [a2, b1]) returns -1
         *     initialValue(x, [a1, b2]) returns -2
         *     initialValue(x, [a2, b2]) returns -1
         * 
         *     initialValue(y, null) returns 2
         *     initialValue(y, [a1]) throws an IllegalArgumentException
         *     initialValue(y, [b1]) returns 3
         *     initialValue(y, [b2]) returns -2
         */
        
        model.eAllContents().forEachRemaining((obj) -> {
            if (obj instanceof Parameter) {
                Parameter parameter = (Parameter)obj;
                if (parameter.getName() == "x") {
                    var values = ASTUtils.initialValue(parameter, null);
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "1");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("a1")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "2");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("a2")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "-1");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("a1"), map.get("b1")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "3");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("a2"), map.get("b1")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "-1");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("a1"), map.get("b2")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "-2");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("a2"), map.get("b2")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "-1");
                } else if (parameter.getName() == "y") {
                    var values = ASTUtils.initialValue(parameter, null);
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "2");

                    Assertions.assertThrows(IllegalArgumentException.class,
                                            () -> ASTUtils.initialValue(parameter, List.of(map.get("a1"))));

                    values = ASTUtils.initialValue(parameter, List.of(map.get("b1")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "3");

                    values = ASTUtils.initialValue(parameter, List.of(map.get("b2")));
                    Assertions.assertInstanceOf(Literal.class, values.get(0));
                    Assertions.assertEquals(((Literal)values.get(0)).getLiteral(), "-2");
                }
            }
        });
    }
}
