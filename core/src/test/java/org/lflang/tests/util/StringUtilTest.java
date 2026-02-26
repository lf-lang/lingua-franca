package org.lflang.tests.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.lflang.util.StringUtil.camelToSnakeCase;
import static org.lflang.util.StringUtil.removeQuotes;

import org.junit.jupiter.api.Test;

/**
 * Test StringUtil functions.
 * @ingroup Tests
 */
public class StringUtilTest {

  @Test
  public void testRemoveQuotes() {
    assertEquals("abc", removeQuotes("\"abc\""));
    assertEquals("a", removeQuotes("'a'"));
    assertEquals("'a", removeQuotes("'a"));
    assertEquals("a\"", removeQuotes("a\""));
    assertEquals("\"", removeQuotes("\""));
    assertEquals("'", removeQuotes("'"));
    assertEquals("", removeQuotes(""));
    assertNull(removeQuotes(null));
  }

  @Test
  public void testCamelToSnakeCase() {
    assertEquals("some_string", camelToSnakeCase("someString"));
    assertEquals("abc_str", camelToSnakeCase("AbcStr"));
    assertEquals("ast", camelToSnakeCase("AST"));
    assertEquals("ast_builder", camelToSnakeCase("ASTBuilder"));
    assertEquals("something_with_a_preamble", camelToSnakeCase("SomethingWithAPreamble"));
  }
}
