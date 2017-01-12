#ifndef CPUCORE_V1_H
#define CPUCORE_V1_H

#include <Arduino.h>

extern byte     Regs[8];
extern uint16_t regSP;
extern uint16_t regPC;

#define regB Regs[0]
#define regC Regs[1]
#define regD Regs[2]
#define regE Regs[3]
#define regH Regs[4]
#define regL Regs[5]
#define regS Regs[6]
#define regA Regs[7]

typedef void (*CPUFUN)(byte);
extern CPUFUN cpu_opcodes[256];
#define CPU_EXEC(opcode) (cpu_opcodes[opcode])(opcode)

#endif
