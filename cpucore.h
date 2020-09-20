// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef CPUCORE_H
#define CPUCORE_H

#include <Arduino.h>
#include "config.h"

#define PS_CARRY       0x01
#define PS_PARITY      0x04
#define PS_HALFCARRY   0x10
#define PS_ZERO        0x40
#define PS_SIGN        0x80


// The emulated Z80 runs at 2MHz (true Z80, not Z80a or Z80b)
#define CPU_CLOCK_I8080 2000
#define CPU_CLOCK_Z80   2000


extern union unionAF
{
  struct { byte A, F; };
  uint16_t AF;
} regAF;

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

extern uint16_t regSP;

#define regA  regAF.A
#define regS  regAF.F
#define regB  regBC.B
#define regC  regBC.C
#define regD  regDE.D
#define regE  regDE.E
#define regH  regHL.H
#define regL  regHL.L
#define regPC regPCU.PC

#define PROC_I8080 0
#define PROC_Z80   1

#if USE_Z80==0 

  // fixed I8080 CPU
  #define cpu_opcodes cpucore_i8080_opcodes
  #define cpu_clock_KHz()     CPU_CLOCK_I8080
  #define cpu_get_processor() PROC_I8080

#elif USE_Z80==1 

  // fixed Z80 CPU
  extern byte regRL;
  #define cpu_opcodes cpucore_z80_opcodes
  #define cpu_clock_KHz()     CPU_CLOCK_Z80
  #define cpu_get_processor() PROC_Z80

#else 

  // CPU is switchable
  extern byte regRL;
  void cpu_set_processor(int processor);
  int  cpu_get_processor();
  int  cpu_clock_KHz();

#endif

typedef void (*CPUFUN)();
extern CPUFUN cpu_opcodes[256];
#define CPU_EXEC(opcode) (cpu_opcodes[opcode])();

void cpu_setup();
void cpu_print_registers();

#endif
