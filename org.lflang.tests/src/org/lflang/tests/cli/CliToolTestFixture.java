/*
 * Copyright (c) 2022, TU Dresden.
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

package org.lflang.tests.cli;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.file.Path;
import java.util.concurrent.Callable;
import java.util.function.Consumer;

import org.hamcrest.Matcher;
import org.opentest4j.AssertionFailedError;

import org.lflang.cli.Io;

/**
 * Test utilities for a CLI tool, eg {@link org.lflang.cli.Lfc},
 * {@link org.lflang.cli.Lff}.
 *
 * @author Clément Fournier
 */
abstract class CliToolTestFixture {

    /**
     * Override to call the relevant main.
     */
    protected abstract void runCliProgram(Io io, String[] args);


    /**
     * Run the tool with the given arguments, in the system
     * working directory.
     *
     * @param args Arguments
     * @return The execution result
     */
    public ExecutionResult run(String... args) {
        return run(Io.SYSTEM.getWd(), args);
    }

    /**
     * Run the tool with the given arguments, in the given
     * working directory.
     *
     * @param wd working directory
     * @param args Arguments
     * @return The execution result
     */
    public ExecutionResult run(Path wd, String... args) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        ByteArrayOutputStream err = new ByteArrayOutputStream();

        Io testIo = new Io(
            new PrintStream(err),
            new PrintStream(out),
            wd
        );
        int exitCode = testIo.fakeSystemExit(io -> runCliProgram(io, args));

        return new ExecutionResult(out, err, exitCode);
    }

    /**
     * The result of an execution of a CLI program like LFC.
     *
     * @param out Output stream
     * @param err Error stream
     * @param exitCode Exit code of the process
     */
    record ExecutionResult(
        ByteArrayOutputStream out,
        ByteArrayOutputStream err,
        int exitCode
    ) {

        public String getOut() {
            return out.toString();
        }

        public String getErr() {
            return err.toString();
        }


        public void checkOk() {
            assertEquals(0, exitCode);
        }

        public void checkFailed() {
            assertEquals(1, exitCode);
        }

        public void checkNoErrorOutput() {
            checkStdErr(equalTo(""));
        }

        public void checkStdOut(Matcher<? super String> matcher) {
            assertThat(getOut(), matcher);
        }

        public void checkStdErr(Matcher<? super String> matcher) {
            assertThat(getErr(), matcher);
        }

        /**
         * Use this method to wrap assertions.
         */
        public void verify(ThrowingConsumer<ExecutionResult> actions) {
            try {
                actions.accept(this);
            } catch (Exception | AssertionFailedError e) {
                System.out.println("TEST FAILED");
                System.out.println("> Return code: " + exitCode);
                System.out.println("> Standard output -------------------------");
                System.err.println(out.toString());
                System.out.println("> Standard error --------------------------");
                System.err.println(err.toString());
                System.out.println("> -----------------------------------------");

                if (e instanceof Exception) {
                    throw new AssertionFailedError("Expected no exception to be thrown", e);
                }
                throw (AssertionFailedError) e;
            }
        }


        @FunctionalInterface
        interface ThrowingConsumer<T> {

            void accept(T t) throws Exception;
        }
    }
}
