package org.lflang.generator;

import java.nio.file.Path;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.emf.common.CommonPlugin;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Code;
import org.lflang.util.FileUtil;
import static org.lflang.generator.c.CMixedRadixGenerator.*;
import static org.lflang.util.StringUtil.joinObjects;

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
     * Append the given text to the code buffer at the current indentation level.
     */
    public void pr(CharSequence text) {
        for (String line : (Iterable<? extends String>) () -> text.toString().lines().iterator()) {
            code.append(indentation).append(line).append(System.lineSeparator());
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
     * Start a scoped block that iterates over the specified range of port channels.
     * 
     * This must be followed by a call to
     * {@link #endScopedRangeBlock(StringBuilder, RuntimeRange<PortInstance>)}.
     *
     * This block should NOT be nested, where each block is
     * put within a similar block for the reactor's parent.
     * Within the created block, every use of
     * {@link CUtil.reactorRef(ReactorInstance, String)}
     * must provide the second argument, a runtime index variable name,
     * that must match the runtimeIndex parameter given here.
     * 
     * @param builder Where to write the code.
     * @param range The range of port channels.
     * @param runtimeIndex A variable name to use to index the runtime instance of
     *  either port's parent or the port's parent's parent (if nested is true), or
     *  null to use the default, "runtime_index".
     * @param bankIndex A variable name to use to index the bank of the port's parent or null to use the
     *  default, the string returned by {@link CUtil.bankIndexName(ReactorInstance)}.
     * @param channelIndex A variable name to use to index the channel or null to
     *  use the default, the string returned by {@link CUtil.channelIndexName(PortInstance)}.
     * @param nested If true, then the runtimeIndex variable will be set
     *  to the bank index of the port's parent's parent rather than the
     *  port's parent.
     * @param restrict For federated execution (only), if this argument
     *  is true, then the iteration will skip over bank members that
     *  are not in the current federate.
     */
    public void startScopedRangeBlock(
        FederateInstance currentFederate,
        RuntimeRange<PortInstance> range, 
        String runtimeIndex,
        String bankIndex,
        String channelIndex,
        boolean nested,
        boolean isFederated,
        boolean restrict
    ) {
        
        pr("// Iterate over range "+range.toString()+".");
        var ri = (runtimeIndex == null)? "runtime_index" : runtimeIndex;
        var ci = (channelIndex == null)? CUtil.channelIndexName(range.instance) : channelIndex;
        var bi = (bankIndex == null)? CUtil.bankIndexName(range.instance.parent) : bankIndex;
        var rangeMR = range.startMR();
        var sizeMR = rangeMR.getDigits().size();
        var nestedLevel = (nested) ? 2 : 1;

        startScopedBlock();
        if (range.width > 1) {
            pr(String.join("\n", 
                "int range_start[] =  { "+joinObjects(rangeMR.getDigits(), ", ")+" };",
                "int range_radixes[] = { "+joinObjects(rangeMR.getRadixes(), ", ")+" };",
                "int permutation[] = { "+joinObjects(range.permutation(), ", ")+" };",
                "mixed_radix_int_t range_mr = {",
                "    "+sizeMR+",",
                "    range_start,",
                "    range_radixes,",
                "    permutation",
                "};",
                "for (int range_count = "+range.start+"; range_count < "+range.start+" + "+range.width+"; range_count++) {"
            ));
            indent();
            pr(String.join("\n", 
                "int "+ri+" = mixed_radix_parent(&range_mr, "+nestedLevel+"); // Runtime index.",
                "int "+ci+" = range_mr.digits[0]; // Channel index.",
                "int "+bi+" = "+(sizeMR <= 1 ? "0" : "range_mr.digits[1]")+"; // Bank index."
            ));
            if (isFederated) {
                if (restrict) {
                    // In case we have a bank of federates. Need that iteration
                    // only cover the one federate. The last digit of the mixed-radix
                    // number is the bank index (or 0 if this is not a bank of federates).
                    pr("if (range_mr.digits[range_mr.size - 1] == "+currentFederate.bankIndex+") {");
                    indent();
                } else {
                    startScopedBlock();
                }
            }
        } else {
            var ciValue = rangeMR.getDigits().get(0);
            var riValue = rangeMR.get(nestedLevel);
            var biValue = (sizeMR > 1)? rangeMR.getDigits().get(1) : 0;
            if (isFederated) {
                if (restrict) {
                    // Special case. Have a bank of federates. Need that iteration
                    // only cover the one federate. The last digit of the mixed-radix
                    // number identifies the bank member (or is 0 if not within a bank).
                    pr("if ("+rangeMR.get(sizeMR - 1)+" == "+currentFederate.bankIndex+") {");
                    indent();
                } else {
                    startScopedBlock();
                }
            }
            pr(String.join("\n", 
                "int "+ri+" = "+riValue+"; // Runtime index.",
                "int "+ci+" = "+ciValue+"; // Channel index.",
                "int "+bi+" = "+biValue+"; // Bank index.",
                "int range_count = 0;"
            ));
        }
    }

    /**
     * Start a scoped block that iterates over the specified pair of ranges.
     * The destination range can be wider than the source range, in which case the
     * source range is reused until the destination range is filled.
     * The following integer variables will be defined within the scoped block:
     * 
     * * src_channel: The channel index for the source.
     * * src_bank: The bank index of the source port's parent.
     * * src_runtime: The runtime index of the source port's parent or
     *   the parent's parent (if the source is an input).
     * 
     * * dst_channel: The channel index for the destination.
     * * dst_bank: The bank index of the destination port's parent.
     * * dst_runtime: The runtime index of the destination port's parent or
     *   the parent's parent (if destination is an output).
     * 
     * For convenience, the above variable names are defined in the private
     * class variables sc, sb, sr, and dc, db, dr.
     *  
     * This block should NOT be nested, where each block is
     * put within a similar block for the reactor's parent.
     * Within the created block, every use of
     * {@link CUtil.reactorRef(ReactorInstance, String, String)}
     * and related functions must provide the above variable names.
     * 
     * This must be followed by a call to
     * {@link #endScopedRangeBlock(StringBuilder, SendRange, RuntimeRange<PortInstance>)}.
     * 
     * @param builder Where to write the code.
     * @param srcRange The send range.
     * @param dstRange The destination range.
     */
    public void startScopedRangeBlock(
        FederateInstance currentFederate,
        SendRange srcRange, 
        RuntimeRange<PortInstance> dstRange,
        boolean isFederated
    ) {
        var srcRangeMR = srcRange.startMR();
        var srcSizeMR = srcRangeMR.getRadixes().size();
        var srcNestedLevel = (srcRange.instance.isInput()) ? 2 : 1;
        var dstNested = dstRange.instance.isOutput();
        
        pr("// Iterate over ranges "+srcRange.toString()+" and "+dstRange.toString()+".");
        
        if (isFederated && srcRange.width == 1) {
            // Skip this whole block if the src is not in the federate.
            pr("if ("+srcRangeMR.get(srcRangeMR.numDigits() - 1)+" == "+currentFederate.bankIndex+") {");
            indent();
        } else {
            startScopedBlock();
        }
        
        if (srcRange.width > 1) {
            pr(String.join("\n", 
                "int src_start[] =  { "+joinObjects(srcRangeMR.getDigits(), ", ")+" };",
                "int src_value[] =  { "+joinObjects(srcRangeMR.getDigits(), ", ")+" }; // Will be incremented.",
                "int src_radixes[] = { "+joinObjects(srcRangeMR.getRadixes(), ", ")+" };",
                "int src_permutation[] = { "+joinObjects(srcRange.permutation(), ", ")+" };",
                "mixed_radix_int_t src_range_mr = {",
                "    "+srcSizeMR+",",
                "    src_value,",
                "    src_radixes,",
                "    src_permutation",
                "};"
            ));
        } else {
            var ciValue = srcRangeMR.getDigits().get(0);
            var biValue = (srcSizeMR > 1)? srcRangeMR.getDigits().get(1) : 0;
            var riValue = srcRangeMR.get(srcNestedLevel);
            pr(String.join("\n", 
                "int "+sr+" = "+riValue+"; // Runtime index.",
                "int "+sc+" = "+ciValue+"; // Channel index.",
                "int "+sb+" = "+biValue+"; // Bank index."
            ));
        }
        
        startScopedRangeBlock(currentFederate, dstRange, dr, db, dc, dstNested, isFederated, true);

        if (srcRange.width > 1) {
            pr(String.join("\n", 
                "int "+sr+" = mixed_radix_parent(&src_range_mr, "+srcNestedLevel+"); // Runtime index.",
                "int "+sc+" = src_range_mr.digits[0]; // Channel index.",
                "int "+sb+" = "+(srcSizeMR <= 1 ? "0" : "src_range_mr.digits[1]")+"; // Bank index."
            ));
        }
        
        // The above startScopedRangeBlock() call will skip any iteration where the destination
        // is a bank member is not in the federation. Here, we skip any iteration where the
        // source is a bank member not in the federation.
        if (isFederated && srcRange.width > 1) {
            // The last digit of the mixed radix
            // number identifies the bank (or is 0 if no bank).
            pr("if (src_range_mr.digits[src_range_mr.size - 1] == "+currentFederate.bankIndex+") {");
            indent();
        }
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
     * End a scoped block for the specified range.
     * @param builder Where to write the code.
     * @param range The send range.
     */
    public void endScopedRangeBlock(
        RuntimeRange<PortInstance> range,
        boolean isFederated
    ) {
        if (isFederated) {
            // Terminate the if statement or block (if not restrict).
            endScopedBlock();
        }
        if (range.width > 1) {
            pr("mixed_radix_incr(&range_mr);");
            endScopedBlock(); // Terminate for loop.
        }
        endScopedBlock();
    }

    /**
     * End a scoped block that iterates over the specified pair of ranges.
     * 
     * @param builder Where to write the code.
     * @param srcRange The send range.
     * @param dstRange The destination range.
     */
    public void endScopedRangeBlock(
        SendRange srcRange, 
        RuntimeRange<PortInstance> dstRange,
        boolean isFederated
    ) {
        // Do not use endScopedRangeBlock because we need things nested.
        if (isFederated) {
            if (srcRange.width > 1) {
                // Terminate the if statement.
                endScopedBlock();
            }
            // Terminate the if statement or block (if not restrict).
            endScopedBlock();
        }
        if (srcRange.width > 1) {
            pr(String.join("\n", 
                "mixed_radix_incr(&src_range_mr);",
                "if (mixed_radix_to_int(&src_range_mr) >= "+srcRange.start+" + "+srcRange.width+") {",
                "    // Start over with the source.",
                "    for (int i = 0; i < src_range_mr.size; i++) {",
                "        src_range_mr.digits[i] = src_start[i];",
                "    }",
                "}"
            ));
        }
        if (dstRange.width > 1) {
            pr("mixed_radix_incr(&range_mr);");
            endScopedBlock(); // Terminate for loop.
        }
        // Terminate unconditional scope block in startScopedRangeBlock calls.
        endScopedBlock();
        endScopedBlock();
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
    public CodeMap writeToFile(String path) throws IOException {
        CodeMap ret = CodeMap.fromGeneratedCode(code.toString());
        FileUtil.writeToFile(ret.getGeneratedCode(), Path.of(path), true);
        return ret;
    }

    ////////////////////////////////////////////
    //// Private fields.

    /** Place to store the code. */
    private StringBuilder code = new StringBuilder();
    
    /** Current indentation. */
    private String indentation = "";
}
