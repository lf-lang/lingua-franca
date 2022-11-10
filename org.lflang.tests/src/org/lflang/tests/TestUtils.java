package org.lflang.tests;

import java.nio.file.Files;
import java.nio.file.Path;

import org.hamcrest.BaseMatcher;
import org.hamcrest.Description;
import org.hamcrest.Matcher;

/**
 * @author Clément Fournier
 */
public class TestUtils {

    public static Matcher<Path> isDirectory() {
        return new BaseMatcher<>() {
            @Override
            public boolean matches(Object item) {
                return item instanceof Path && Files.isDirectory((Path) item);
            }

            @Override
            public void describeTo(Description description) {
                description.appendText("is a directory");
            }
        };
    }

    public static Matcher<Path> exists() {
        return new BaseMatcher<>() {
            @Override
            public boolean matches(Object item) {
                return item instanceof Path && Files.exists((Path) item);
            }

            @Override
            public void describeTo(Description description) {
                description.appendText("exists");
            }
        };
    }
}
