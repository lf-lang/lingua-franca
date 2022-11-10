package org.lflang.tests.cli;

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.equalTo;

import org.junit.jupiter.api.Test;

import org.lflang.LocalStrings;
import org.lflang.cli.Io;
import org.lflang.cli.Lfc;
import org.lflang.tests.cli.CliToolTestFixture.ExecutionResult;

/**
 * @author Clément Fournier
 */
public class LfcCliTest {

    LfcTestFixture lfcTester = new LfcTestFixture();


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


    static class LfcTestFixture extends CliToolTestFixture {


        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lfc.main(io, args);
        }
    }

}
