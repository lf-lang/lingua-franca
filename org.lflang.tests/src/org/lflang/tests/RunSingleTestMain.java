/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.tests;

import java.io.FileNotFoundException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.lflang.Target;
import org.lflang.tests.runtime.CCppTest;
import org.lflang.tests.runtime.CTest;
import org.lflang.tests.runtime.CppTest;
import org.lflang.tests.runtime.PythonTest;
import org.lflang.tests.runtime.RustTest;
import org.lflang.tests.runtime.TypeScriptTest;

/**
 * Execute a single test case. Use it with the gradle task
 * {@code gradle runSingleTest --args test/Python/src/Minimal.lf}
 *
 * @author Clément Fournier
 */
public class RunSingleTestMain {


    private static final Pattern TEST_FILE_PATTERN = Pattern.compile("(test/(\\w+))/src/([^/]++/)*(\\w+.lf)");

    public static void main(String[] args) throws FileNotFoundException {
        if (args.length != 1) {
            throw new IllegalArgumentException("Expected 1 path to an LF file");
        }
        var path = Paths.get(args[0]);
        if (!Files.exists(path)) {
            throw new FileNotFoundException("No such test file: " + path);
        }

        Matcher matcher = TEST_FILE_PATTERN.matcher(args[0]);
        if (!matcher.matches()) {
            throw new FileNotFoundException("Not a test: " + path);
        }

        Target target = Target.forName(matcher.group(2)).get();

        Class<? extends TestBase> testClass = getTestInstance(target);

        LFTest testCase = new LFTest(target, path.toAbsolutePath());

        TestBase.runSingleTestAndPrintResults(testCase, testClass, TestBase.pathToLevel(path));
    }

    private static Class<? extends TestBase> getTestInstance(Target target) {
        switch (target) {
        case C:
            return CTest.class;
        case CCPP:
            return CCppTest.class;
        case CPP:
            return CppTest.class;
        case TS:
            return TypeScriptTest.class;
        case Python:
            return PythonTest.class;
        case Rust:
            return RustTest.class;
        default:
            throw new IllegalArgumentException();
        }
    }
}
