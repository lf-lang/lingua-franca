package org.lflang.generator;

import static org.lflang.generator.c.CMixedRadixGenerator.*;
import static org.lflang.util.StringUtil.joinObjects;

import java.io.IOException;
import java.nio.file.Path;
import org.eclipse.emf.common.CommonPlugin;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Code;
import org.lflang.util.FileUtil;

/**
 * Helper class for printing code with indentation. This class is backed by a StringBuilder and is
 * used to accumulate code to be printed to a file. Its main function is to handle indentation.
 *
 * @author Edward A. Lee
 * @author Peter Donovan
 */
public class CodeBuilder {

  private static final String END_SOURCE_LINE_NUMBER_TAG =
      "/* END PR SOURCE LINE NUMBER 9sD0aiwE01RcMWl */";

  /** Construct a new empty code emitter. */
  public CodeBuilder() {}

  /**
   * Construct a new code emitter with the text and indentation of the specified code emitter.
   *
   * @param model The model code emitter.
   */
  public CodeBuilder(CodeBuilder model) {
    indentation = model.indentation;
    code.append(model);
  }

  /////////////////////////////////////////////
  ///// Public methods.

  /**
   * Get the code produced so far.
   *
   * @return The code produced so far as a String.
   */
  public String getCode() {
    return code.toString();
  }

  /** Increase the indentation of the output code produced. */
  public void indent() {
    indentation += "    ";
  }

  /**
   * Insert the specified text at the specified position.
   *
   * @param position The position.
   * @param text The text.
   */
  public void insert(int position, String text) {
    code.insert(position, text);
  }

  /** Return the length of the code in characters. */
  public int length() {
    return code.length();
  }

  /** Add a new line. */
  public void newLine() {
    this.pr("");
  }

  /**
   * Append the specified text plus a final newline.
   *
   * @param format A format string to be used by {@code String.format} or the text to append if no
   *     further arguments are given.
   * @param args Additional arguments to pass to the formatter.
   */
  public void pr(String format, Object... args) {
    pr((args != null && args.length > 0) ? String.format(format, args) : format);
  }

  /** Append the given text to the code buffer at the current indentation level. */
  public void pr(CharSequence text) {
    // append newline on empty input string
    if (text.toString().equals("")) code.append("\n");
    for (String line : (Iterable<? extends String>) () -> text.toString().lines().iterator()) {
      code.append(indentation).append(line).append("\n");
    }
  }

  /**
   * Print the #line compiler directive with the line number of the specified object.
   *
   * @param eObject The node.
   * @param suppress Do nothing if true.
   */
  public void prSourceLineNumber(EObject eObject, boolean suppress) {
    if (suppress) {
      return;
    }
    var node = NodeModelUtils.getNode(eObject);
    if (node != null) {
      // For code blocks (delimited by {= ... =}, unfortunately,
      // we have to adjust the offset by the number of newlines before {=.
      // Unfortunately, this is complicated because the code has been
      // tokenized.
      var offset = 0;
      if (eObject instanceof Code code) {
        var text = ASTUtils.toOriginalText(code).lines().findFirst();
        if (text.isPresent()) {
          var firstLine = text.get().trim();
          var exact = NodeModelUtils.getNode(code).getText().replaceAll("\\{=", "");
          for (var line : (Iterable<String>) () -> exact.lines().iterator()) {
            if (line.trim().equals(firstLine)) {
              break;
            }
            offset += 1;
          }
        }
      }
      // Extract the filename from eResource, an astonishingly difficult thing to do.
      String filePath =
          CommonPlugin.resolve(eObject.eResource().getURI()).path().replace("\\", "\\\\");
      pr("#line " + (node.getStartLine() + offset) + " \"" + filePath + "\"");
    }
  }

  /**
   * Print a tag marking the end of a block corresponding to the source LF file.
   *
   * @param suppress Do nothing if true.
   */
  public void prEndSourceLineNumber(boolean suppress) {
    if (!suppress) pr(END_SOURCE_LINE_NUMBER_TAG);
  }

  /**
   * Append a single-line, C-style comment to the code.
   *
   * @param comment The comment.
   */
  public void prComment(String comment) {
    pr("// " + comment);
  }

  /**
   * Remove all lines that start with the specified prefix and return a new CodeBuilder with the
   * result.
   *
   * @param prefix The prefix.
   */
  public CodeBuilder removeLines(String prefix) {
    String separator = "\n";
    String[] lines = toString().split(separator);

    CodeBuilder builder = new CodeBuilder();

    for (String line : lines) {
      String trimmedLine = line.trim();
      if (!trimmedLine.startsWith(prefix)) {
        builder.pr(line);
      }
    }
    return builder;
  }

  /**
   * Start a scoped block, which is a section of code surrounded by curley braces and indented. This
   * must be followed by an {@link #endScopedBlock()}.
   */
  public void startScopedBlock() {
    pr("{");
    indent();
  }

  /**
   * Start a scoped block for the specified reactor. If the reactor is a bank, then this starts a
   * for loop that iterates over the bank members using a standard index variable whose name is that
   * returned by {@link CUtil#bankIndex(ReactorInstance)}. If the reactor is null or is not a bank,
   * then this simply starts a scoped block by printing an opening curly brace. This also adds a
   * declaration of a pointer to the self struct of the reactor or bank member.
   *
   * <p>This block is intended to be nested, where each block is put within a similar block for the
   * reactor's parent. This ensures that all (possibly nested) bank index variables are defined
   * within the block.
   *
   * <p>This must be followed by an {@link #endScopedBlock()}.
   *
   * @param reactor The reactor instance.
   */
  public void startScopedBlock(ReactorInstance reactor) {
    if (reactor != null && reactor.isBank()) {
      var index = CUtil.bankIndexName(reactor);
      pr("// Reactor is a bank. Iterate over bank members.");
      pr("for (int " + index + " = 0; " + index + " < " + reactor.width + "; " + index + "++) {");
      indent();
    } else {
      startScopedBlock();
    }
  }

  /**
   * If the specified port is a multiport, then start a specified iteration over the channels of the
   * multiport using as the channel index the variable name returned by {@link
   * CUtil#channelIndex(PortInstance)}. If the port is not a multiport, do nothing. This is required
   * to be followed by {@link #endChannelIteration(PortInstance)}.
   *
   * @param port The port.
   */
  public void startChannelIteration(PortInstance port) {
    if (port.isMultiport) {
      var channel = CUtil.channelIndexName(port);
      pr("// Port " + port.getFullName() + " is a multiport. Iterate over its channels.");
      pr(
          "for (int "
              + channel
              + " = 0; "
              + channel
              + " < "
              + port.width
              + "; "
              + channel
              + "++) {");
      indent();
    }
  }

  /**
   * Start a scoped block to iterate over bank members and channels for the specified port with a
   * variable with the name given by count counting the iterations. If this port is a multiport,
   * then the channel index variable name is that returned by {@link
   * CUtil#channelIndex(PortInstance)}.
   *
   * <p>This block is intended to be nested, where each block is put within a similar block for the
   * reactor's parent.
   *
   * <p>This is required to be followed by a call to {@link
   * #endScopedBankChannelIteration(PortInstance, String)}.
   *
   * @param port The port.
   * @param count The variable name to use for the counter, or null to not provide a counter.
   */
  public void startScopedBankChannelIteration(PortInstance port, String count) {
    if (count != null) {
      startScopedBlock();
      pr("int " + count + " = 0;");
    }
    startScopedBlock(port.parent);
    startChannelIteration(port);
  }

  /**
   * Start a scoped block that iterates over the specified range of port channels.
   *
   * <p>This must be followed by a call to {@link #endScopedRangeBlock(RuntimeRange)}.
   *
   * <p>This block should NOT be nested, where each block is put within a similar block for the
   * reactor's parent. Within the created block, every use of {@link
   * CUtil#reactorRef(ReactorInstance, String)} must provide the second argument, a runtime index
   * variable name, that must match the runtimeIndex parameter given here.
   *
   * @param range The range of port channels.
   * @param runtimeIndex A variable name to use to index the runtime instance of either port's
   *     parent or the port's parent's parent (if nested is true), or null to use the default,
   *     "runtime_index".
   * @param bankIndex A variable name to use to index the bank of the port's parent or null to use
   *     the default, the string returned by {@link CUtil#bankIndexName(ReactorInstance)}.
   * @param channelIndex A variable name to use to index the channel or null to use the default, the
   *     string returned by {@link CUtil#channelIndexName(PortInstance)}.
   * @param nested If true, then the runtimeIndex variable will be set to the bank index of the
   *     port's parent's parent rather than the port's parent.
   */
  public void startScopedRangeBlock(
      RuntimeRange<PortInstance> range,
      String runtimeIndex,
      String bankIndex,
      String channelIndex,
      boolean nested) {

    pr("// Iterate over range " + range.toString() + ".");
    var ri = (runtimeIndex == null) ? "runtime_index" : runtimeIndex;
    var ci = (channelIndex == null) ? CUtil.channelIndexName(range.instance) : channelIndex;
    var bi = (bankIndex == null) ? CUtil.bankIndexName(range.instance.parent) : bankIndex;
    var rangeMR = range.startMR();
    var sizeMR = rangeMR.getDigits().size();
    var nestedLevel = (nested) ? 2 : 1;

    startScopedBlock();
    if (range.width > 1) {
      pr(
          String.join(
              "\n",
              "int range_start[] =  { " + joinObjects(rangeMR.getDigits(), ", ") + " };",
              "int range_radixes[] = { " + joinObjects(rangeMR.getRadixes(), ", ") + " };",
              "int permutation[] = { " + joinObjects(range.permutation(), ", ") + " };",
              "mixed_radix_int_t range_mr = {",
              "    " + sizeMR + ",",
              "    range_start,",
              "    range_radixes,",
              "    permutation",
              "};",
              "for (int range_count = "
                  + range.start
                  + "; range_count < "
                  + range.start
                  + " + "
                  + range.width
                  + "; range_count++) {"));
      indent();
      pr(
          String.join(
              "\n",
              "int "
                  + ri
                  + " = mixed_radix_parent(&range_mr, "
                  + nestedLevel
                  + "); // Runtime index.",
              "SUPPRESS_UNUSED_WARNING(" + ri + ");",
              "int " + ci + " = range_mr.digits[0]; // Channel index.",
              "SUPPRESS_UNUSED_WARNING(" + ci + ");",
              "int " + bi + " = " + (sizeMR <= 1 ? "0" : "range_mr.digits[1]") + "; // Bank index.",
              "SUPPRESS_UNUSED_WARNING(" + bi + ");"));

    } else {
      var ciValue = rangeMR.getDigits().get(0);
      var riValue = rangeMR.get(nestedLevel);
      var biValue = (sizeMR > 1) ? rangeMR.getDigits().get(1) : 0;
      pr(
          String.join(
              "\n",
              "int "
                  + ri
                  + " = "
                  + riValue
                  + "; SUPPRESS_UNUSED_WARNING("
                  + ri
                  + "); // Runtime index.",
              "int "
                  + ci
                  + " = "
                  + ciValue
                  + "; SUPPRESS_UNUSED_WARNING("
                  + ci
                  + "); // Channel index.",
              "int "
                  + bi
                  + " = "
                  + biValue
                  + "; SUPPRESS_UNUSED_WARNING("
                  + bi
                  + "); // Bank index.",
              "int range_count = 0; SUPPRESS_UNUSED_WARNING(range_count);"));
    }
  }

  /**
   * Start a scoped block that iterates over the specified pair of ranges. The destination range can
   * be wider than the source range, in which case the source range is reused until the destination
   * range is filled. The following integer variables will be defined within the scoped block:
   *
   * <ul>
   *   <li>src_channel: The channel index for the source.
   *   <li>src_bank: The bank index of the source port&#39;s parent.
   *   <li>src_runtime: The runtime index of the source port&#39;s parent or the parent&#39;s parent
   *       (if the source is an input).
   * </ul>
   *
   * <ul>
   *   <li>dst_channel: The channel index for the destination.
   *   <li>dst_bank: The bank index of the destination port&#39;s parent.
   *   <li>dst_runtime: The runtime index of the destination port&#39;s parent or the parent&#39;s
   *       parent (if destination is an output).
   * </ul>
   *
   * <p>For convenience, the above variable names are defined in the private class variables sc, sb,
   * sr, and dc, db, dr.
   *
   * <p>This block should NOT be nested, where each block is put within a similar block for the
   * reactor's parent. Within the created block, every use of {@link
   * CUtil#reactorRef(ReactorInstance, String)} and related functions must provide the above
   * variable names.
   *
   * <p>This must be followed by a call to {@link #endScopedRangeBlock(SendRange, RuntimeRange)}.x
   *
   * @param srcRange The send range.
   * @param dstRange The destination range.
   */
  public void startScopedRangeBlock(SendRange srcRange, RuntimeRange<PortInstance> dstRange) {
    var srcRangeMR = srcRange.startMR();
    var srcSizeMR = srcRangeMR.getRadixes().size();
    var srcNestedLevel = (srcRange.instance.isInput()) ? 2 : 1;
    var dstNested = dstRange.instance.isOutput();

    pr("// Iterate over ranges " + srcRange + " and " + dstRange + ".");
    startScopedBlock();
    if (srcRange.width > 1) {
      pr(
          String.join(
              "\n",
              "int src_start[] =  { " + joinObjects(srcRangeMR.getDigits(), ", ") + " };",
              "int src_value[] =  { "
                  + joinObjects(srcRangeMR.getDigits(), ", ")
                  + " }; // Will be incremented.",
              "int src_radixes[] = { " + joinObjects(srcRangeMR.getRadixes(), ", ") + " };",
              "int src_permutation[] = { " + joinObjects(srcRange.permutation(), ", ") + " };",
              "mixed_radix_int_t src_range_mr = {",
              "    " + srcSizeMR + ",",
              "    src_value,",
              "    src_radixes,",
              "    src_permutation",
              "};"));
    } else {
      var ciValue = srcRangeMR.getDigits().get(0);
      var biValue = (srcSizeMR > 1) ? srcRangeMR.getDigits().get(1) : 0;
      var riValue = srcRangeMR.get(srcNestedLevel);
      pr(
          String.join(
              "\n",
              "int " + sr + " = " + riValue + "; // Runtime index.",
              "SUPPRESS_UNUSED_WARNING(" + sr + ");",
              "int " + sc + " = " + ciValue + "; // Channel index.",
              "SUPPRESS_UNUSED_WARNING(" + sc + ");",
              "int " + sb + " = " + biValue + "; // Bank index.",
              "SUPPRESS_UNUSED_WARNING(" + sb + ");"));
    }

    startScopedRangeBlock(dstRange, dr, db, dc, dstNested);

    if (srcRange.width > 1) {
      pr(
          String.join(
              "\n",
              "int "
                  + sr
                  + " = mixed_radix_parent(&src_range_mr, "
                  + srcNestedLevel
                  + "); // Runtime index.",
              "SUPPRESS_UNUSED_WARNING(" + sr + ");",
              "int " + sc + " = src_range_mr.digits[0]; // Channel index.",
              "SUPPRESS_UNUSED_WARNING(" + sc + ");",
              "int "
                  + sb
                  + " = "
                  + (srcSizeMR <= 1 ? "0" : "src_range_mr.digits[1]")
                  + "; // Bank index.",
              "SUPPRESS_UNUSED_WARNING(" + sb + ");"));
    }
  }

  public void endScopedBlock() {
    unindent();
    pr("}");
  }

  /**
   * If the specified port is a multiport, then start a specified iteration over the channels of the
   * multiport using as the channel index the variable name returned by {@link
   * CUtil#channelIndex(PortInstance)}. If the port is not a multiport, do nothing.
   *
   * @param port The port.
   */
  public void endChannelIteration(PortInstance port) {
    if (port.isMultiport) {
      unindent();
      pr("}");
    }
  }

  /**
   * End a scoped block to iterate over bank members and channels for the specified port with a
   * variable with the name given by count counting the iterations.
   *
   * @param port The port.
   * @param count The variable name to use for the counter, or null to not provide a counter.
   */
  public void endScopedBankChannelIteration(PortInstance port, String count) {
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
   *
   * @param range The send range.
   */
  public void endScopedRangeBlock(RuntimeRange<PortInstance> range) {
    if (range.width > 1) {
      pr("mixed_radix_incr(&range_mr);");
      endScopedBlock(); // Terminate for loop.
    }
    endScopedBlock();
  }

  /**
   * End a scoped block that iterates over the specified pair of ranges.
   *
   * @param srcRange The send range.
   * @param dstRange The destination range.
   */
  public void endScopedRangeBlock(SendRange srcRange, RuntimeRange<PortInstance> dstRange) {
    if (srcRange.width > 1) {
      pr(
          String.join(
              "\n",
              "mixed_radix_incr(&src_range_mr);",
              "if (mixed_radix_to_int(&src_range_mr) >= "
                  + srcRange.start
                  + " + "
                  + srcRange.width
                  + ") {",
              "    // Start over with the source.",
              "    for (int i = 0; i < src_range_mr.size; i++) {",
              "        src_range_mr.digits[i] = src_start[i];",
              "    }",
              "}"));
    }
    if (dstRange.width > 1) {
      pr("mixed_radix_incr(&range_mr);");
      endScopedBlock(); // Terminate for loop.
    }
    // Terminate unconditional scope block in startScopedRangeBlock calls.
    endScopedBlock();
    endScopedBlock();
  }

  /** Return the code as a string. */
  @Override
  public String toString() {
    return code.toString();
  }

  /** Reduce the indentation by one level for generated code/ */
  public void unindent() {
    indentation = indentation.substring(0, Math.max(0, indentation.length() - 4));
  }

  /**
   * Write the text to a file.
   *
   * @param path The file to write the code to.
   */
  public CodeMap writeToFile(String path) throws IOException {
    String s = code.toString();
    int lineNumber = 1;
    StringBuilder out = new StringBuilder();
    for (var line : (Iterable<String>) () -> s.lines().iterator()) {
      lineNumber++;
      if (line.contains(END_SOURCE_LINE_NUMBER_TAG) && !path.endsWith(".ino")) {
        out.append("#line ")
            .append(lineNumber)
            .append(" \"")
            .append(path.replace("\\", "\\\\"))
            .append("\"");
      } else {
        out.append(line);
      }
      out.append('\n');
    }
    CodeMap ret = CodeMap.fromGeneratedCode(out.toString());
    FileUtil.writeToFile(ret.getGeneratedCode(), Path.of(path), true);
    return ret;
  }

  ////////////////////////////////////////////
  //// Private fields.

  /** Place to store the code. */
  private final StringBuilder code = new StringBuilder();

  /** Current indentation. */
  private String indentation = "";
}
