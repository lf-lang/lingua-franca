package org.lflang.analyses.evm;

import org.lflang.generator.ReactionInstance;

public class InstructionEXE implements Instruction {
    
    /** Opcode of this instruction */
    final private Opcode opcode = Opcode.EXE;

    /** Reaction to be executed */
    public ReactionInstance reaction;

    /** Constructor */
    public InstructionEXE(ReactionInstance reaction) {
        this.reaction = reaction;
    }

	@Override
	public Opcode getOpcode() {
		return this.opcode;
	}

    @Override
    public String toString() {
        return opcode + ": " + this.reaction;
    }
}
