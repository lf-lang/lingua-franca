/*************
 * Copyright (c) 2023, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.tests.runtime;

import java.util.List;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.Configurators;
import org.lflang.tests.TestBase;
import org.lflang.tests.TestRegistry.TestCategory;
import org.lflang.tests.Transformers;

public class CFlexPRETTest extends TestBase {

  public CFlexPRETTest() {
    super(Target.C);
  }

  @Test
  public void buildFlexPRETConcurrent() {
    Assumptions.assumeTrue(isLinux(), "FlexPRET tests only supported on Linux");
    super.runTestsFor(
        List.of(Target.C),
        "Build concurrent tests for FlexPRET.",
        TestCategory.CONCURRENT::equals,
        Transformers::noChanges,
        Configurators::makeFlexPRETCompatible,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildFlexPRETBasicTestsUnthreaded() {
    Assumptions.assumeTrue(isLinux(), "FlexPRET tests only supported on Linux");
    super.runTestsFor(
        List.of(Target.C),
        "Build basic tests for FlexPRET in single threaded mode.",
        TestCategory.BASIC::equals,
        Transformers::noChanges,
        Configurators::makeFlexPRETCompatibleUnthreaded,
        TestLevel.BUILD,
        false);
  }

  @Test
  public void buildFlexPRETBasicTests() {
    Assumptions.assumeTrue(isLinux(), "FlexPRET tests only supported on Linux");
    super.runTestsFor(
        List.of(Target.C),
        "Build basic tests for FlexPRET.",
        TestCategory.BASIC::equals,
        Transformers::noChanges,
        Configurators::makeFlexPRETCompatible,
        TestLevel.BUILD,
        false);
  }
}
