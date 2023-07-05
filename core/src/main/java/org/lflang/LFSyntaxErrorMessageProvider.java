package org.lflang;

import com.google.inject.Inject;
import java.util.Set;
import java.util.stream.Collectors;
import org.antlr.runtime.RecognitionException;
import org.antlr.runtime.Token;
import org.eclipse.xtext.GrammarUtil;
import org.eclipse.xtext.IGrammarAccess;
import org.eclipse.xtext.nodemodel.SyntaxErrorMessage;
import org.eclipse.xtext.parser.antlr.SyntaxErrorMessageProvider;

/**
 * Custom error message provider that intercepts syntax errors.
 *
 * @author Marten Lohstroh
 */
public class LFSyntaxErrorMessageProvider extends SyntaxErrorMessageProvider {

  /** Issue code for syntax error due to misused keyword. */
  public static String USED_RESERVED_KEYWORD = "USED_RESERVED_KEYWORD";

  /** Issue code for syntax error due to misused single quotes. */
  public static String SINGLY_QUOTED_STRING = "SINGLE_QUOTED_STRING";

  /** Helper that provides access to the grammar. */
  @Inject IGrammarAccess grammarAccess;

  /**
   * Set of keywords that otherwise would be valid identifiers. For example, 'reaction' is part of
   * this set, but '{=' is not.
   */
  public Set<String> keywords;

  /** Customize intercepted error messages. @Override */
  public SyntaxErrorMessage getSyntaxErrorMessage(IParserErrorContext context) {

    if (context != null) {
      String message = context.getDefaultMessage();

      // Describe situation where an unexpected token was encountered where a closing single quote
      // was expected.
      if (message.contains("expecting '''")) {
        return new SyntaxErrorMessage(
            "Single quotes can only be used around single characters, not strings. "
                + "Please use double quotes instead.",
            SINGLY_QUOTED_STRING);
      }

      RecognitionException e = context.getRecognitionException();
      if (e != null) {
        Token token = e.token;
        if (token != null) {
          String text = token.getText();
          if (text != null) {
            // Update keywords if not yet set.
            if (keywords == null) {
              // ('a'..'z'|'A'..'Z'|'_')('a'..'z'|'A'..'Z'|'_'|'0'..'9')*
              this.keywords =
                  GrammarUtil.getAllKeywords(grammarAccess.getGrammar()).stream()
                      .filter(k -> k.matches("^([a-z]|[A-Z]|_)+(\\w|_)*$"))
                      .collect(Collectors.toSet());
            }
            // Describe situation where a keyword is used as an identifier.
            if (keywords.contains(text)) {
              return new SyntaxErrorMessage(
                  "'"
                      + text
                      + "' is a reserved keyword "
                      + "which cannot be used as an identifier.",
                  USED_RESERVED_KEYWORD);
            }
          }
        }
      }
    }
    return super.getSyntaxErrorMessage(context);
  }
}
