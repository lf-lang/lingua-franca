package org.lflang.pretvm.profiles;

import org.lflang.TimeValue;

/**
 * WCET profile for FlexPRET, assuming 100 MHz (10 ns) clock and 4 hardware threads.
 *
 * <p>FIXME: Values are placeholders; actual values should be determined experimentally.
 */
public record FlexPRETProfile() {

  /** Return the worst-case execution time for the given instruction opcode. */
  public static TimeValue getInstWCET(String opcode) {
    return switch (opcode) {
      case "ADD" -> TimeValue.fromNanoSeconds(1200L);
      case "ADDI" -> TimeValue.fromNanoSeconds(1040L);
      case "BEQ" -> TimeValue.fromNanoSeconds(1200L);
      case "BGE" -> TimeValue.fromNanoSeconds(1200L);
      case "BLT" -> TimeValue.fromNanoSeconds(1280L);
      case "BNE" -> TimeValue.fromNanoSeconds(1200L);
      case "DU" -> TimeValue.fromNanoSeconds(3120L);
      case "EXE" -> TimeValue.fromNanoSeconds(400L);
      case "JAL" -> TimeValue.fromNanoSeconds(880L);
      case "JALR" -> TimeValue.fromNanoSeconds(1040L);
      case "STP" -> TimeValue.fromNanoSeconds(320L);
      case "WLT" -> TimeValue.fromNanoSeconds(400L);
      case "WU" -> TimeValue.fromNanoSeconds(400L);
      default -> throw new IllegalArgumentException("Unknown opcode: " + opcode);
    };
  }
}
