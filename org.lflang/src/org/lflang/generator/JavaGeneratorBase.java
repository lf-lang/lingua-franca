package org.lflang.generator;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Stream;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;

import org.lflang.lf.Code;
import org.lflang.validation.AbstractLFValidator;

/**
 * Generator base class for shared code between code generators.
 * This is created to ease our migration from Xtend.
 */
public abstract class JavaGeneratorBase extends AbstractLFValidator {
    ////////////////////////////////////////////
    //// Protected fields.

    /**
     * All code goes into this string buffer.
     */
    protected StringBuilder code = new StringBuilder();

    /////////////////////////////////////////////
    ///// Private fields.

    /**
     * Map from builder to its current indentation.
     */
    private final Map<StringBuilder, String> indentation = new HashMap<>();

    /////////////////////////////////////////////
    ///// Protected methods.

    /**
     * Get the code produced so far.
     * @return The code produced so far as a String.
     */
    protected String getCode() {
        return code.toString();
    }

    /**
     * Increase the indentation of the output code produced.
     */
    protected void indent() {
        indent(code);
    }

    /**
     * Increase the indentation of the output code produced
     * on the specified builder.
     * @param builder The builder to indent.
     */
    protected void indent(StringBuilder builder) {
        String prefix = indentation.get(builder);
        if (prefix == null) {
            prefix = "";
        }
        prefix += "    ";
        indentation.put(builder, prefix);
    }

    /** Reduce the indentation by one level for generated code
     *  in the default code buffer.
     */
    protected void unindent() {
        unindent(code);
    }

    /** Reduce the indentation by one level for generated code
     *  in the specified code buffer.
     */
    protected void unindent(StringBuilder builder) {
        String indent = indentation.get(builder);
        if (indent != null) {
            indentation.put(builder, indent.substring(0, Math.max(0, indent.length() - 4)));
        }
    }

    /**
     * Append the specified text plus a final newline to the current
     * code buffer.
     * @param format A format string to be used by {@code String.format} or
     * the text to append if no further arguments are given.
     * @param args Additional arguments to pass to the formatter.
     */
    protected void pr(String format, Object... args) {
        pr(
            code,
            (args != null && args.length > 0) ? String.format(format, args) : format
        );
    }

    /**
     * Append the specified text plus a final newline to the specified
     * code buffer.
     * @param builder The code buffer.
     * @param text The text to append.
     */
    protected void pr(StringBuilder builder, Object text) {
        String string = text.toString();
        String indent = indentation.get(builder);
        if (indent == null) {
            indent = "";
        }
        string = string.replaceAll("\t", "    ");
        String[] split = string.split("\n");
        int offset = Stream.of(split).skip(1)
                           .mapToInt(line -> line.indexOf(line.trim()))
                           .min()
                           .orElse(0);
        // Now make a pass for each line, replacing the offset leading
        // spaces with the current indentation.
        boolean firstLine = true;
        for (String line : split) {
            builder.append(indent);
            // Do not trim the first line
            if (firstLine) {
                builder.append(line);
                firstLine = false;
            } else {
                builder.append(line.substring(offset));
            }
            builder.append("\n");
        }
    }

    /**
     * Leave a marker in the generated code that indicates the original line
     * number in the LF source.
     * @param eObject The node.
     */
    protected void prSourceLineNumber(EObject eObject) {
        pr(code, String.format(
            "// %d",
            NodeModelUtils.getNode(eObject).getStartLine() + (eObject instanceof Code ? 1 : 0)
        ));
    }

    /**
     * Print a comment to the generated file.
     * Particular targets will need to override this if comments
     * start with something other than '//'.
     * @param comment The comment.
     */
    protected void prComment(String comment) {
        pr(code, "// " + comment);
    }
}
