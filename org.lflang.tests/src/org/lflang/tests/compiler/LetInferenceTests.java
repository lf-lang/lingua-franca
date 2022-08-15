package org.lflang.tests.compiler;/* Parsing unit tests. */

/*************
 Copyright (c) 2022, The University of California at Berkeley.

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

import java.nio.file.Path;
import javax.inject.Inject;

import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.eclipse.xtext.testing.util.ParseHelper;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import org.lflang.ASTUtils;
import org.lflang.DefaultErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CGenerator;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.tests.LFInjectorProvider;
import static org.lflang.ASTUtils.*;

@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)

/**
 * Test for getting minimum delay in reactions.
 * Checking the actions and port's delay,then get the minimum reaction delay.
 * @author{Wonseo Choi <wonsuh1202@hanyang.ac.kr>}
 * @author{Yunsang Cho <snsc7878@hanyang.ac.kr>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Hokeun Kim <hokeunkim@berkeley.edu>}
 */
class LetInferenceTest  {

    @Inject
    ParseHelper<Model> parser;


    @Test
    public void testLet() throws Exception {
        Model model = parser.parse(String.join(
            System.getProperty("line.separator"),
            "target C;",
            "main reactor {",
            "    ramp = new Ramp();",
            "    print = new Print();",
            "    print2 = new Print();",
            "    ramp.y -> print.x after 20 msec;",
            "    ramp.y -> print2.x after 30 msec;",
            "}",
            "reactor Ramp {",
            "    logical action a(60 msec):int;",
            "    logical action b(100 msec):int;",
            "    input x:int;",
            "    output y:int;",
            "    output z:int;",
            "    reaction(startup) -> y, z, a, b{=",
            "    =}",
            "}",
            "reactor Print {",
            "    input x:int;",
            "    output z:int;",
            "    reaction(x) -> z {=",
            "    =}",
            "}"
        ));

        Assertions.assertNotNull(model);
        ASTUtils.insertGeneratedDelays(model.eResource(), new CGenerator(new FileConfig(model.eResource(), Path.of("./a/"), true), new DefaultErrorReporter()));
        Assertions.assertTrue(model.eResource().getErrors().isEmpty(),
                              "Encountered unexpected error while parsing: " +
                                  model.eResource().getErrors());

        Instantiation mainDef = null;

        TreeIterator<EObject> it = model.eResource().getAllContents();
        while (it.hasNext()) {
            EObject obj = it.next();
            if (!(obj instanceof Reactor)) {
                continue;
            }
            Reactor reactor = (Reactor) obj;
            if (reactor.isMain()) {
                mainDef = LfFactory.eINSTANCE.createInstantiation();
                mainDef.setName(reactor.getName());
                mainDef.setReactorClass(reactor);
            }
        }

        ReactorInstance mainInstance = new ReactorInstance(toDefinition(mainDef.getReactorClass()), new DefaultErrorReporter());

        for (ReactorInstance reactorInstance : mainInstance.children) {
            if (reactorInstance.isGeneratedDelay()) {
                for (ReactionInstance reactionInstance : reactorInstance.reactions) {
                    Assertions.assertEquals(reactionInstance.assignLogicalExecutionTime(), TimeValue.ZERO);
                }
            } else if (reactorInstance.getName().contains("ramp")) {
                for (ReactionInstance reactionInstance : reactorInstance.reactions) {
                    Assertions.assertEquals(new TimeValue(20L, TimeUnit.MILLI), reactionInstance.assignLogicalExecutionTime());
                }
            } else if (reactorInstance.getName().contains("print")) {
                for (ReactionInstance reactionInstance : reactorInstance.reactions) {
                    Assertions.assertEquals(TimeValue.ZERO, reactionInstance.assignLogicalExecutionTime());
                }
            }
        }
    }
}
