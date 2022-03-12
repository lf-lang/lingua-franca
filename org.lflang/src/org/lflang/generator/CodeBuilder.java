package org.lflang.generator;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.stream.Stream;

import org.eclipse.emf.common.CommonPlugin;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Code;

/**
 * Helper class for printing code with indentation.
 * This class is backed by a StringBuilder and is used to accumulate
 * code to be printed to a file. Its main function is to handle indentation.
 * 
 * @author Edward A. Lee
 * @author Peter Donovan
 */
public class CodeBuilder {
    
    /**
     * Construct a new empty code emitter.
     */
    public CodeBuilder() {}
    
    /**
     * Construct a new code emitter with the text and indentation
     * of the specified code emitter.
     * @param model The model code emitter.
     */
    public CodeBuilder(CodeBuilder model) {
        indentation = model.indentation;
        code.append(model.toString());
    }
    
    /////////////////////////////////////////////
    ///// Public methods.

    /**
     * Get the code produced so far.
     * @return The code produced so far as a String.
     */
    public String getCode() {
        return code.toString();
    }

    /**
     * Increase the indentation of the output code produced.
     */
    public void indent() {
        indentation += "    ";
    }
    
    /**
     * Insert the specified text at the specified position.
     * @param position The position.
     * @param text The text.
     */
    public void insert(int position, String text) {
        code.insert(position, text);
    }
    
    /**
     * Return the length of the code in characters.
     */
    public int length() {
        return code.length();
    }
    
    /**
     * Add a new line.
     */
    public void newLine() {
        this.pr("");
    }

    /**
     * Append the specified text plus a final newline.
     * @param format A format string to be used by {@code String.format} or
     *  the text to append if no further arguments are given.
     * @param args Additional arguments to pass to the formatter.
     */
    public void pr(String format, Object... args) {
        pr(
            (args != null && args.length > 0) ? String.format(format, args) : format
        );
    }

    /**
     * Append the specified text plus a final newline to the specified
     * code buffer. This also replaces tabs with four spaces.
     * @param text The the object whose toString() method provides the text.
     */
    public void pr(Object text) {
        String string = text.toString();
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
            code.append(indentation);
            // Do not trim the first line
            if (firstLine) {
                code.append(line);
                firstLine = false;
            } else {
                code.append(line.substring(offset));
            }
            code.append("\n");
        }
    }

    /**
     * Version of pr() that prints a source line number using a #line
     * prior to each line of the output. Use this when multiple lines of
     * output code are all due to the same source line in the .lf file.
     * @param eObject The AST node that this source line is based on.
     * @param text The text to append.
     */
    public void pr(EObject eObject, Object text) {
        var split = text.toString().split("\n");
        for (String line : split) {
            prSourceLineNumber(eObject);
            pr(line);
        }
    }
    
    /** Print the #line compiler directive with the line number of
     *  the specified object.
     *  @param eObject The node.
     */
    public void prSourceLineNumber(EObject eObject) {
        var node = NodeModelUtils.getNode(eObject);
        if (node != null) {
            // For code blocks (delimited by {= ... =}, unfortunately,
            // we have to adjust the offset by the number of newlines before {=.
            // Unfortunately, this is complicated because the code has been
            // tokenized.
            var offset = 0;
            if (eObject instanceof Code) {
                offset += 1;
            }
            // Extract the filename from eResource, an astonishingly difficult thing to do.
            URI resolvedURI = CommonPlugin.resolve(eObject.eResource().getURI());
            // pr(output, "#line " + (node.getStartLine() + offset) + ' "' + FileConfig.toFileURI(fileConfig.srcFile) + '"')
            pr("#line " + (node.getStartLine() + offset) + " \"" + resolvedURI + "\"");
        }
    }

    /**
     * Append a single-line, C-style comment to the code.
     * @param comment The comment.
     */
    public void prComment(String comment) {
        pr("// " + comment);
    }
    
    /**
     * Remove all lines that start with the specified prefix
     * and return a new CodeBuilder with the result.
     * @param prefix The prefix.
     */
    public CodeBuilder removeLines(String prefix) {
        String separator = "\n";
        String[] lines = toString().split(separator);
        
        CodeBuilder builder = new CodeBuilder();
        
        for(String line : lines) {
            String trimmedLine = line.trim();
            if(!trimmedLine.startsWith(prefix)) {
                builder.pr(line);
            }
        }
        return builder;
    }

    /**
     * Start a scoped block, which is a section of code
     * surrounded by curley braces and indented.
     * This must be followed by an {@link endScopedBlock(StringBuilder)}.
     * @param builder The code emitter into which to write.
     */
    public void startScopedBlock() {
        pr("{");
        indent();
    }

    /**
     * Start a scoped block for the specified reactor.
     * If the reactor is a bank, then this starts a for loop
     * that iterates over the bank members using a standard index
     * variable whose name is that returned by {@link CUtil.bankIndex(ReactorInstance)}.
     * If the reactor is null or is not a bank, then this simply
     * starts a scoped block by printing an opening curly brace.
     * This also adds a declaration of a pointer to the self
     * struct of the reactor or bank member.
     * 
     * This block is intended to be nested, where each block is
     * put within a similar block for the reactor's parent.
     * This ensures that all (possibly nested) bank index variables
     * are defined within the block.
     * 
     * This must be followed by an {@link endScopedBlock(StringBuilder)}.
     * 
     * @param builder The place to write the code.
     * @param reactor The reactor instance.
     * @param restrict For federated execution only, if this is true, then
     *  skip iterations where the topmost bank member is not in the federate.
     */
    public void startScopedBlock(
        ReactorInstance reactor, 
        FederateInstance federate,
        boolean isFederated,
        boolean restrict
    ) {
        // NOTE: This is protected because it is used by the PythonGenerator.
        if (reactor != null && reactor.isBank()) {
            var index = CUtil.bankIndexName(reactor);
            if (reactor.depth == 1 && isFederated && restrict) {
                // Special case: A bank of federates. Instantiate only the current federate.
                startScopedBlock();
                pr("int "+index+" = "+federate.bankIndex+";");
            } else {
                pr("// Reactor is a bank. Iterate over bank members.");
                pr("for (int "+index+" = 0; "+index+" < "+reactor.width+"; "+index+"++) {");
                indent();
            }
        } else {
            startScopedBlock();
        }
    }

    /**
     * If the specified port is a multiport, then start a specified iteration
     * over the channels of the multiport using as the channel index the
     * variable name returned by {@link CUtil.channelIndex(PortInstance)}.
     * If the port is not a multiport, do nothing.
     * This is required to be followed by {@link endChannelIteration(StringBuilder, PortInstance}.
     * @param builder Where to write the code.
     * @param port The port.
     */
    public void startChannelIteration(PortInstance port) {
        if (port.isMultiport) {
            var channel = CUtil.channelIndexName(port);
            pr("// Port "+port.getFullName()+" is a multiport. Iterate over its channels.");
            pr("for (int "+channel+" = 0; "+channel+" < "+port.width+"; "+channel+"++) {");
            indent();
        }
    }

    /**
     * Start a scoped block to iterate over bank members and
     * channels for the specified port with a a variable with
     * the name given by count counting the iterations.
     * If this port is a multiport, then the channel index
     * variable name is that returned by {@link CUtil.channelIndex(PortInstance)}.
     *
     * This block is intended to be nested, where each block is
     * put within a similar block for the reactor's parent.
     *
     * This is required to be followed by a call to
     * {@link endScopedBankChannelIteration(StringBuilder, PortInstance, String)}.
     * @param builder Where to write the code.
     * @param port The port.
     * @param count The variable name to use for the counter, or
     *  null to not provide a counter.
     */
    public void startScopedBankChannelIteration(
        PortInstance port, FederateInstance currentFederate,
        String count, boolean isFederated
    ) {
        if (count != null) {
            startScopedBlock();
            pr("int "+count+" = 0;");
        }
        startScopedBlock(port.parent, currentFederate, isFederated, true);
        startChannelIteration(port);
    }

    /**
     * End a scoped block.
     * @param builder The place to write the code.
     */
    public void endScopedBlock() {
        // NOTE: This is protected because it is used by the PythonGenerator.
        unindent();
        pr("}");
    }

    /**
     * If the specified port is a multiport, then start a specified iteration
     * over the channels of the multiport using as the channel index the
     * variable name returned by {@link CUtil.channelIndex(PortInstance)}.
     * If the port is not a multiport, do nothing.
     * This is required to be followed by {@link endChannelIteration(StringBuilder, PortInstance}.
     * @param builder Where to write the code.
     * @param port The port.
     */
    public void endChannelIteration(PortInstance port) {
        if (port.isMultiport) {
            unindent();
            pr("}");
        }
    }

    /**
     * End a scoped block to iterate over bank members and
     * channels for the specified port with a a variable with
     * the name given by count counting the iterations.
     * @param builder Where to write the code.
     * @param port The port.
     * @param count The variable name to use for the counter, or
     *  null to not provide a counter.
     */
    public void endScopedBankChannelIteration(
        PortInstance port, String count
    ) {
        if (count != null) {
            pr(count + "++;");
        }
        endChannelIteration(port);
        endScopedBlock();
        if (count != null) {
            endScopedBlock();
        }
    }

    /**
     * Return the code as a string.
     */
    @Override
    public String toString() {
        return code.toString();
    }

    /** 
     * Reduce the indentation by one level for generated code/
     */
    public void unindent() {
        indentation = indentation.substring(0, Math.max(0, indentation.length() - 4));
    }

    /**
     * Write the text to a file.
     * @param path The file to write the code to.
     */
    public void writeToFile(String path) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(path))) {
            for (int i = 0; i < code.length(); i++) {
                writer.write(code.charAt(i));
            }
        }
    }

    ////////////////////////////////////////////
    //// Private fields.

    /** Place to store the code. */
    private StringBuilder code = new StringBuilder();
    
    /** Current indentation. */
    private String indentation = "";
}
