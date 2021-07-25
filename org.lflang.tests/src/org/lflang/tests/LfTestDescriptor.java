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

import java.nio.file.Path;

import org.lflang.tests.TestRegistry.TestCategory;

/**
 *
 */
public class LfTestDescriptor {

    private final Path srcFile;
    private final String testName;
    private final TestCategory testCategory;
    private final Path packageRoot;


    public LfTestDescriptor(Path srcFile, String testName, TestCategory testCategory, Path packageRoot) {
        this.srcFile = srcFile;
        this.testName = testName;
        this.testCategory = testCategory;
        this.packageRoot = packageRoot;
    }


    public Path getPackageRoot() {
        return packageRoot;
    }


    public Path getSrcFile() {
        return srcFile;
    }

    public TestCategory getTestCategory() {
        return testCategory;
    }

    public boolean shouldRun() {
        return packageRoot != TestRegistry.LF_EXAMPLE_PATH;
    }

    public String getTestName() {
        return testName;
    }
}
