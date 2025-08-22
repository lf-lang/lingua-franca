package org.lflang.tests;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.FileNotFoundException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;
import org.lflang.target.Target;
import org.lflang.tests.runtime.CCppTest;
import org.lflang.tests.runtime.CTest;
import org.lflang.tests.runtime.CppTest;
import org.lflang.tests.runtime.PythonTest;
import org.lflang.tests.runtime.RustTest;
import org.lflang.tests.runtime.TypeScriptTest;

/**
 * Execute a single test case.
 *
 * Use it with the gradle task `./gradlew singleTest -DsingleTest=test/C/src/Minimal.lf`
 *
 * @author Clément Fournier
 * @ingroup Tests
 */
public class RunSingleTest {

  private static final Pattern TEST_FILE_PATTERN =
      Pattern.compile("(test\\W(\\w+))\\Wsrc\\W(\\w++\\W)*(\\w+.lf)");

  @Test
  public void runSingleTest() throws FileNotFoundException {
    String singleTestPath = System.getProperty("singleTest");
    assumeTrue(singleTestPath != null && !singleTestPath.isBlank());

    var path = Paths.get(singleTestPath);
    if (!Files.exists(path)) {
      throw new FileNotFoundException("No such test file: " + path);
    }

    Matcher matcher = TEST_FILE_PATTERN.matcher(path.toString());
    if (!matcher.matches()) {
      throw new FileNotFoundException("Not a test: " + path);
    }

    Target target = Target.forName(matcher.group(2)).get();

    Class<? extends TestBase> testClass = getTestInstance(target);

    LFTest testCase = new LFTest(path.toAbsolutePath());

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
