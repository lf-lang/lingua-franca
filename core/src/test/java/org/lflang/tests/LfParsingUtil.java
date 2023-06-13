package org.lflang.tests;

import org.junit.jupiter.api.Assertions;
import org.lflang.ast.ParsingUtils;
import org.lflang.lf.Model;

/**
 * @author Clément Fournier
 */
public class LfParsingUtil {

  /** Parse the given file, asserts that there are no parsing errors. */
  public static Model parseValidModel(String fileName, String reformattedTestCase) {
    Model resultingModel = ParsingUtils.parse(reformattedTestCase);
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
