#ifndef CPUCORE_H
#define CPUCORE_H

#include <Arduino.h>

#define PS_CARRY       0x01
#define PS_PARITY      0x04
#define PS_HALFCARRY   0x10
#define PS_ZERO        0x40
#define PS_SIGN        0x80

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
