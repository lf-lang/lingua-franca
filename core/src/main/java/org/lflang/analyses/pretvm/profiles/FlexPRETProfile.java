package org.lflang.analyses.pretvm.profiles;

import org.lflang.TimeValue;
import org.lflang.analyses.pretvm.instructions.Instruction.Opcode;

public record FlexPRETProfile() {
    // FIXME: This is a placeholder for the FlexPRET profile. The actual
    // values should be determined experimentally.
    public static TimeValue getInstWCET(Opcode opcode) {
        return switch (opcode) {
            case ADD    -> TimeValue.fromNanoSeconds(1000L);
            case ADDI   -> TimeValue.fromNanoSeconds(1000L);
            case ADV    -> TimeValue.fromNanoSeconds(1000L);
            case ADVI   -> TimeValue.fromNanoSeconds(1000L);
            case BEQ    -> TimeValue.fromNanoSeconds(1000L);
            case BGE    -> TimeValue.fromNanoSeconds(1000L);
            case BLT    -> TimeValue.fromNanoSeconds(1000L);
            case BNE    -> TimeValue.fromNanoSeconds(1000L);
            case DU     -> TimeValue.fromNanoSeconds(1000L);
            case EXE    -> TimeValue.fromNanoSeconds(1000L);
            case JAL    -> TimeValue.fromNanoSeconds(1000L);
            case JALR   -> TimeValue.fromNanoSeconds(1000L);
            case STP    -> TimeValue.fromNanoSeconds(1000L);
            case WLT    -> TimeValue.fromNanoSeconds(1000L);
            case WU     -> TimeValue.fromNanoSeconds(1000L);
        };
    }
}
