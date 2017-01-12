#ifndef CPUCORE_V2_H
#define CPUCORE_V2_H

#include <Arduino.h>

extern union unionBC
{
  struct { byte C, B; };
  uint16_t BC;
} regBC;

extern union unionDE
{
  struct { byte E, D; };
  uint16_t DE;
} regDE;

extern union unionHL
{
  struct { byte L, H; };
  uint16_t HL;
} regHL;

extern union unionPC
{
  struct { byte L, H; };
  uint16_t PC;
} regPCU;

extern byte regS;
extern byte regA;
extern uint16_t regSP;

#define regB  regBC.B
#define regC  regBC.C
#define regD  regDE.D
#define regE  regDE.E
#define regH  regHL.H
#define regL  regHL.L
#define regPC regPCU.PC

typedef void (*CPUFUN)();
extern CPUFUN cpu_opcodes[256];
#define CPU_EXEC(opcode) (cpu_opcodes[opcode])()

#endif
