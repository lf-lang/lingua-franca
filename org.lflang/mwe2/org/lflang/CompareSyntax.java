package org.lflang;

import org.eclipse.emf.mwe2.runtime.workflow.IWorkflowContext;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.emf.mwe2.runtime.workflow.IWorkflowComponent;

/**
 * This class is intended to catch at least some inconsistencies
 * that could arise between the Lingua Franca grammar that is used
 * for parsing LF source files and the TextMate-style grammar that
 * is used for syntax highlighting. It is strictly a developer
 * tool for use only in the MWE2 workflow.
 */
public class CompareSyntax implements IWorkflowComponent {
    private static final Pattern KEYWORD_IN_GRAMMAR = Pattern.compile("(?<!\\.\\.)'(?<keyword>[^\\s'\\.]*?)'(?!\\.\\.)");
    
    private String rootPath;
    private String grammarPath;
    private String textmateGrammarPath;
    private final Set<String> ignore = new HashSet<>();
    
    @Override
    public void invoke(IWorkflowContext ctx) {
        String textmateGrammar = getTextmateGrammar();
        for (String keyword : getLfKeywords()) {
            if (!textmateGrammar.contains(keyword)) {
                System.err.println(
                    "WARNING: The token '" + keyword + "' appears in " + grammarPath
                    + ", but not in " + textmateGrammarPath + ". Consider updating "
                    + textmateGrammarPath + " or silencing the warning by ignoring "
                    + "'" + keyword + "'" + " in GenerateLinguaFranca.mwe2."
                );
            }
        }
    }

    @Override
    public void preInvoke() {
        // Do nothing.
    }

    @Override
    public void postInvoke() {
        // Do nothing.
    }
    
    /**
     * Informs this instance that <code>nonKeyWord</code> is not a
     * keyword.
     * @param nonKeyword a string that is not a keyword
     */
    public void addIgnore(String nonKeyword) {
        ignore.add(nonKeyword);
    }
    
    /**
     * Sets the path to the root directory relative to which all
     * other paths are given.
     * @param path the path to the root directory. This path must
     * be given with Unix-style formatting.
     */
    public void setRootPath(String path) {
        rootPath = matchPlatform(path);
    }
    
    /**
     * Sets the path to the Xtext grammar for Lingua Franca.
     * @param path the path to the Xtext grammar for Lingua Franca.
     * This path must be given with Unix-style formatting.
     */
    public void setGrammarPath(String path) {
        grammarPath = matchPlatform(path);
    }
    
    /**
     * Sets the path to the TextMate-style grammar that is used for
     * syntax highlighting.
     * @param path the path to the TextMate-style grammar that is
     * used for syntax highlighting. This path must be given with
     * Unix-style formatting.
     */
    public void setTextmateGrammarPath(String path) {
        textmateGrammarPath = matchPlatform(path);
    }
    
    /**
     * Returns a set of all keywords that appear in the Lingua Franca
     * Xtext grammar.
     */
    private Set<String> getLfKeywords() {
        Path p = Paths.get(
            Objects.requireNonNull(rootPath) + File.separator +
            Objects.requireNonNull(grammarPath)
        );
        String contents;
        try {
            contents = Files.readString(p);
        } catch (IOException e) {
            throw new RuntimeException("Failed to read file at \"" + p + "\":\n" + e);
        }
        Set<String> keywords = new HashSet<>();
        Matcher m = KEYWORD_IN_GRAMMAR.matcher(contents);
        while (m.find()) {
            if (!ignore.contains(m.group("keyword"))) keywords.add(m.group("keyword"));
        }
        return keywords;
    }
    
    /**
     * Returns the contents of the TextMate-style grammar.
     * @return the contents of the TextMate-style grammar
     */
    private String getTextmateGrammar() {
        Path p = Paths.get(
            Objects.requireNonNull(rootPath) + File.separator +
            Objects.requireNonNull(textmateGrammarPath)
        );
        String contents;
        try {
            contents = Files.readString(p);
        } catch (IOException e) {
            throw new RuntimeException("Failed to read file at \"" + p + "\":\n" + e);
        }
        return contents;
    }
    
    /**
     * Converts <code>path</code> to a path representation that is
     * consistent with the current platform.
     */
    private String matchPlatform(String path) {
	return path.replace("/", File.separator);
    }
}
