package org.lflang.tests;

import org.junit.jupiter.api.Assertions;
import org.lflang.ast.LfParsingHelper;
import org.lflang.lf.Model;

/**
 * Utility to parse LF classes. Not static so that we can reuse the injector, as dependency
 * injection takes a lot of time.
 *
 * @author Clément Fournier
 */
public class LfParsingTestHelper {

  private final LfParsingHelper parser = new LfParsingHelper();

  /** Parse the given file, asserts that there are no parsing errors. */
  public Model parseValidModel(String fileName, String reformattedTestCase) {
    Model resultingModel = parser.parse(reformattedTestCase);
    checkValid(fileName, resultingModel);
    return resultingModel;
  }

  public static void checkValid(String fileName, Model resultingModel) {
    Assertions.assertNotNull(resultingModel);
    if (!resultingModel.eResource().getErrors().isEmpty()) {
      resultingModel.eResource().getErrors().forEach(System.err::println);
      Assertions.assertTrue(
          resultingModel.eResource().getErrors().isEmpty(), "Parsing errors in " + fileName);
    }
  }
}
