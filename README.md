# ARMv8-CPU
This is a 5-stage pipelined CPU that implements the ARM LEGV8 instruction set. This CPU is based on the 5-stage pipelined MIPS CPU on this page. The differences between the CPU using the MIPS instruction set and the CPU using the ARM LEGV8 instruction set include the setting of the N, Z and V flags, utilizing 64-bit registers and busses, extending and modifying the opcode decoder to account for the new instructions and setting the new instruction flags, modifying the location of the zero register (changing it from R0 to R31), and adding modified shifting logic for shift instructions.

# Stages
1. Instruction fetch
2. Instruction decode
3. Execute
4. Memory
5. Write back
