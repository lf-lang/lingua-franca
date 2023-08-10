# Steps for adding a new instruction

## Compiler
1. Add a new opcode in `Instruction.java`.
2. Create a new instruction class under `pretvm`.
3. Generate new instructions in `InstructionGenerator.java`.
4. Generate C code in `InstructionGenerator.java`.

## C Runtime
1. Add a new enum in `scheduler_instructions.h`.
2. Add an implementation for the new instruction in `scheduler_static.c`.
