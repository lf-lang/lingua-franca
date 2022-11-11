package org.lflang.tests.cli;

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.lflang.LocalStrings;
import org.lflang.cli.Io;
import org.lflang.cli.Lff;
import org.lflang.tests.cli.CliToolTestFixture.ExecutionResult;

/**
 * @author Clément Fournier
 */
public class LffCliTest {

    private static final String FILE_BEFORE_REFORMAT = """
        target Python;
        main   reactor {
            reaction(startup) {=
               =}
        }
        """;
    private static final String FILE_AFTER_REFORMAT = """
        target Python
            
        main reactor {
            reaction(startup) {=  =}
        }
        """;
    LffTestFixture lffTester = new LffTestFixture();


    @Test
    public void testHelpArg() {
        ExecutionResult result = lffTester.run("--help", "--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(containsString("usage: lff"));
    }

    @Test
    public void testVersion() {
        ExecutionResult result = lffTester.run("--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(equalTo("lff " + LocalStrings.VERSION + "\n"));
    }


    @Test
    public void testWrongCliArg() {
        ExecutionResult result = lffTester.run("--notanargument", "File.lf");
        result.checkStdErr(containsString("Unrecognized option: --notanargument"));
        result.checkStdErr(containsString("fatal error"));
        result.checkFailed();
    }

    @Test
    public void testFormatSingleFileInPlace(@TempDir Path tempDir) throws IOException {
        Files.createDirectories(tempDir.resolve("src"));
        Path srcFile = tempDir.resolve("src/File.lf");
        Files.writeString(srcFile, FILE_BEFORE_REFORMAT);

        ExecutionResult result = lffTester.run(tempDir, "src/File.lf");

        result.checkOk();

        assertThat(Files.readString(srcFile), equalTo(FILE_AFTER_REFORMAT));
    }


    @Test
    public void testFormatDirectory(@TempDir Path tempDir) throws IOException {
        Files.createDirectories(tempDir.resolve("src"));
        Path srcFile = tempDir.resolve("src/File.lf");
        Files.writeString(srcFile, FILE_BEFORE_REFORMAT);

        ExecutionResult result = lffTester.run(tempDir, "src");

        result.checkOk();

        assertThat(Files.readString(srcFile), equalTo(FILE_AFTER_REFORMAT));
    }


    @Test
    public void testFormatDirectoryVerbose(@TempDir Path tempDir) throws IOException {
        Files.createDirectories(tempDir.resolve("src"));
        Path srcFile = tempDir.resolve("src/File.lf");
        Files.writeString(srcFile, FILE_BEFORE_REFORMAT);

        ExecutionResult result = lffTester.run(tempDir, "-v", "src");

        result.checkOk();

        result.checkStdOut(containsString("Formatted src/File.lf"));
        assertThat(Files.readString(srcFile), equalTo(FILE_AFTER_REFORMAT));
    }

    @Test
    public void testNoSuchFile(@TempDir Path tempDir) {
        ExecutionResult result = lffTester.run(tempDir, "-v", "nosuchdir");

        result.checkFailed();

        result.checkStdErr(containsString(
            tempDir.resolve("nosuchdir") + ": No such file or directory"));
    }


    static class LffTestFixture extends CliToolTestFixture {


        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lff.main(io, args);
        }
    }

}
