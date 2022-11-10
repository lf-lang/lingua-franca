package org.lflang.tests.cli;

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.lflang.tests.TestUtils.exists;
import static org.lflang.tests.TestUtils.isDirectory;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.lflang.LocalStrings;
import org.lflang.cli.Io;
import org.lflang.cli.Lfc;
import org.lflang.tests.cli.CliToolTestFixture.ExecutionResult;

/**
 * @author Clément Fournier
 */
public class LfcCliTest {

    LfcTestFixture lfcTester = new LfcTestFixture();

    static final String LF_PYTHON_FILE = """
        target Python
        main reactor {
            reaction(startup) {==}
        }
        """;


    @Test
    public void testHelpArg() {
        ExecutionResult result = lfcTester.run("--help", "--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(containsString("usage: lfc"));
    }

    @Test
    public void testVersion() {
        ExecutionResult result = lfcTester.run("--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(equalTo("lfc " + LocalStrings.VERSION + "\n"));
    }


    @Test
    public void testWrongCliArg() {
        ExecutionResult result = lfcTester.run("--notanargument", "File.lf");
        result.checkStdErr(containsString("Unrecognized option: --notanargument"));
        result.checkStdErr(containsString("fatal error"));
        result.checkFailed();
    }

    // todo is there a way to test this without invoking an actual
    // generator?
    @Test
    public void testGenInSrcDir(@TempDir Path tempDir) throws IOException {
        Files.createDirectories(tempDir.resolve("src"));
        Path srcFile = tempDir.resolve("src/File.lf");
        Files.writeString(srcFile, LF_PYTHON_FILE);

        ExecutionResult result = lfcTester.run(tempDir, "src/File.lf");

        result.checkOk();
        assertThat(tempDir.resolve("src-gen"), isDirectory());
        assertThat(tempDir.resolve("bin"), isDirectory());
        assertThat(tempDir.resolve("src-gen/File/File.py"), exists());
    }


    static class LfcTestFixture extends CliToolTestFixture {


        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lfc.main(io, args);
        }
    }

}
