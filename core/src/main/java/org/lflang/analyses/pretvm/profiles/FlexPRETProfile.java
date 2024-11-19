package org.lflang.analyses.pretvm.profiles;

import org.lflang.TimeValue;
import org.lflang.analyses.pretvm.instructions.Instruction.Opcode;

public record FlexPRETProfile() {
  // FIXME: This is a placeholder for the FlexPRET profile. The actual
  // values should be determined experimentally.
  public static TimeValue getInstWCET(Opcode opcode) {
    // assuming 100MHz (10ns) Frequency and 4 Hardware Threads
    return switch (opcode) {
      case ADD -> TimeValue.fromNanoSeconds(1200L);
      case ADDI -> TimeValue.fromNanoSeconds(1040L);
      case BEQ -> TimeValue.fromNanoSeconds(1200L);
      case BGE -> TimeValue.fromNanoSeconds(1200L);
      case BLT -> TimeValue.fromNanoSeconds(1280L);
      case BNE -> TimeValue.fromNanoSeconds(1200L);
      case DU -> TimeValue.fromNanoSeconds(3120L);
      case EXE -> TimeValue.fromNanoSeconds(400L);
      case JAL -> TimeValue.fromNanoSeconds(880L);
      case JALR -> TimeValue.fromNanoSeconds(1040L);
      case STP -> TimeValue.fromNanoSeconds(320L);
      case WLT -> TimeValue.fromNanoSeconds(400L);
      case WU -> TimeValue.fromNanoSeconds(400L);
      default -> throw new IllegalArgumentException("Unknown opcode: " + opcode);
    };
  }
}
