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

package org.lflang.tests;

import static org.hamcrest.MatcherAssert.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Predicate;

import org.hamcrest.BaseMatcher;
import org.hamcrest.Description;
import org.hamcrest.Matcher;

/**
 * @author Clément Fournier
 */
public class TestUtils {

    private static Matcher<Path> pathMatcher(String description, Predicate<Path> predicate) {
        return new BaseMatcher<>() {
            @Override
            public boolean matches(Object item) {
                return item instanceof Path && predicate.test((Path) item);
            }

            @Override
            public void describeTo(Description describer) {
                describer.appendText(description);
            }
        };
    }

    public static Matcher<Path> isDirectory() {
        return pathMatcher("is a directory", Files::isDirectory);
    }

    public static Matcher<Path> isRegularFile() {
        return pathMatcher("is a regular file", Files::isRegularFile);
    }

    public static Matcher<Path> exists() {
        return pathMatcher("exists", Files::exists);
    }

    /**
     * Builder for a directory. Useful to create a fake LF project.
     */
    public static class TempDirBuilder {

        private final Path curDir;

        private TempDirBuilder(Path path) {
            this.curDir = path;
            if (!Files.isDirectory(path)) {
                throw new IllegalArgumentException("Not a directory: " + path);
            }
        }

        public static TempDirBuilder dirBuilder(Path path) {
            return new TempDirBuilder(path);
        }

        /** Create a directory at the given path. Return a new temp dir builder for that subdir. */
        public TempDirBuilder cd(String relativePath) throws IOException {
            Path relPath = Paths.get(relativePath);
            if (relPath.isAbsolute()) {
                throw new IllegalArgumentException("Should be a relative path: " + relativePath);
            }
            Path dir = curDir.resolve(relPath);
            Files.createDirectories(dir);
            return new TempDirBuilder(dir);
        }

        /** Create a directory at the given path. Return this instance. */
        public TempDirBuilder mkdirs(String relativePath) throws IOException {
            Path relPath = Paths.get(relativePath);
            if (relPath.isAbsolute()) {
                throw new IllegalArgumentException("Should be a relative path: " + relativePath);
            }
            Path dir = curDir.resolve(relPath);
            Files.createDirectories(dir);
            return this;
        }

        /** Create a file in the given subpath. Return this instance. */
        public TempDirBuilder file(String relativePath, String contents) throws IOException {
            Path relPath = Paths.get(relativePath);
            if (relPath.isAbsolute()) {
                throw new IllegalArgumentException("Should be a relative path: " + relativePath);
            }
            Path filePath = curDir.resolve(relPath);
            Files.createDirectories(filePath.getParent());
            Files.writeString(filePath, contents);
            return this;
        }
    }

    /**
     * Builder for a directory. Useful to create a fake LF project.
     */
    public static class TempDirChecker {

        private final Path curDir;

        private TempDirChecker(Path path) {
            this.curDir = path;
            if (!Files.isDirectory(path)) {
                throw new IllegalArgumentException("Not a directory: " + path);
            }
        }

        public static TempDirChecker dirChecker(Path path) {
            return new TempDirChecker(path);
        }

        /** Create a directory at the given path. Return a new temp dir builder for that subdir. */
        public TempDirBuilder cd(String relativePath) throws IOException {
            Path relPath = Paths.get(relativePath);
            if (relPath.isAbsolute()) {
                throw new IllegalArgumentException("Should be a relative path: " + relativePath);
            }
            Path dir = curDir.resolve(relPath);
            Files.createDirectories(dir);
            return new TempDirBuilder(dir);
        }

        /**
         * Check the contents of the file match the matcher.
         * The file should be a UTF-8 encoded text file. Return
         * this instance.
         */
        public TempDirChecker checkContentsOf(String relativePath, Matcher<? super String> contentsMatcher) throws IOException {
            Path relPath = Paths.get(relativePath);
            if (relPath.isAbsolute()) {
                throw new IllegalArgumentException("Should be a relative path: " + relativePath);
            }
            Path filePath = curDir.resolve(relPath);

            assertThat(Files.readString(filePath), contentsMatcher);
            return this;
        }

        public TempDirChecker check(String relativePath, Matcher<? super Path> pathMatcher) throws IOException {
            Path relPath = Paths.get(relativePath);
            if (relPath.isAbsolute()) {
                throw new IllegalArgumentException("Should be a relative path: " + relativePath);
            }
            Path filePath = curDir.resolve(relPath);

            assertThat(filePath, pathMatcher);
            return this;
        }
    }
}
