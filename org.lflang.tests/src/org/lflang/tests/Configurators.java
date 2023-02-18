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

import org.lflang.tests.TestRegistry.TestCategory;

/**
 * Configuration procedures for {@link TestBase} methods.
 *
 * @author Clément Fournier
 * @author Marten Lohstroh
 */
public class Configurators {

    /** Test configuration function. */
    @FunctionalInterface
    public interface Configurator {

        /**
         * Apply a side effect to the given test case to change its default configuration.
         * Return true if configuration succeeded, false otherwise.
         */
        boolean configure(LFTest test);
    }

    /**
     * Configure the given test to use single-threaded execution.
     *
     * For targets that provide a threaded and an unthreaded runtime,
     * this configures using the unthreaded runtime. For targets that
     * do not distinguish threaded and unthreaded runtime, the number
     * of workers is set to 1.
     *
     * @param test The test to configure.
     * @return True if successful, false otherwise.
     */
    public static boolean disableThreading(LFTest test) {
        test.getContext().getArgs().setProperty("threading", "false");
        test.getContext().getArgs().setProperty("workers", "1");
        return true;
    }

    // TODO: In the future we want to execute to the test with QEMU
    //  but it requires parsing QEMU output until either:
    //  1) A timeout
    //  2) "exit" is printed to stdout. Then look if there is a FATAL ERROR printed somewhere
    // So it would requre continously parsing the stdout and waiting for exit keyword
    public static boolean platformZephyrQemuNoFlash(LFTest test) {
        test.getContext().getArgs().setProperty("threading", "false");
        // TODO: How can I set platform as a dictionary (platform.board platform.flash etc)
        // test.getContext().getArgs().setProperty("platform", "Zephyr");
        return true;
    }
    /**
     * Make no changes to the configuration.
     *
     * @param ignoredTest The test to configure.
     * @return True
     */
    public static boolean noChanges(LFTest ignoredTest) {
        return true;
    }

    /**
     * Given a test category, return true if it is compatible with single-threaded execution.
     */
    public static boolean compatibleWithThreadingOff(TestCategory category) {

        // CONCURRENT, FEDERATED, DOCKER_FEDERATED, DOCKER
        // are not compatible with single-threaded execution.
        // ARDUINO and ZEPHYR have their own test suites, so we don't need to rerun.
        boolean excluded = category == TestCategory.CONCURRENT
            || category == TestCategory.SERIALIZATION
            || category == TestCategory.FEDERATED
            || category == TestCategory.DOCKER_FEDERATED
            || category == TestCategory.DOCKER
            || category == TestCategory.ARDUINO
            || category == TestCategory.ZEPHYR;

        // SERIALIZATION and TARGET tests are excluded on Windows.
        excluded |= TestBase.isWindows() && (category == TestCategory.TARGET);
        return !excluded;
    }
}
