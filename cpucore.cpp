// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2018 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
// -----------------------------------------------------------------------------

#include "cpucore.h"
#include "Altair8800.h"
#include "cpucore_z80.h"
#include "cpucore_i8080.h"

// registers shared between i8080 and z80 implementation
union unionAF regAF;
union unionBC regBC;
union unionDE regDE;
union unionHL regHL;
union unionPC regPCU;
uint16_t regSP;

#if USE_Z80==0 // fixed I8080 CPU

void cpu_setup() {}
void cpu_print_registers() { cpucore_i8080_print_registers(); }

#elif USE_Z80==1 // fixed Z80 CPU

void cpu_setup() {}
void cpu_print_registers() { cpucore_z80_print_registers(); }

#elif USE_Z80==2 // CPU is switchable


CPUFUN cpu_opcodes[256];
static int processor = -1;
static int clock_KHz = 0;


void cpu_setup()
{
  cpu_set_processor(config_use_z80() ? PROC_Z80 : PROC_I8080);
}


void cpu_set_processor(int p)
{
  processor = p;
  clock_KHz = processor==PROC_I8080 ? CPU_CLOCK_I8080 : CPU_CLOCK_Z80;
  memcpy(cpu_opcodes, processor==PROC_I8080 ? cpucore_i8080_opcodes : cpucore_z80_opcodes, 256*sizeof(CPUFUN));
}


int cpu_get_processor()
{
  return processor;
}


int cpu_clock_KHz()
{
  return clock_KHz;
}


void cpu_print_registers()
{
  if( processor==PROC_I8080 )
    cpucore_i8080_print_registers();
  else
    cpucore_z80_print_registers();
}

#else
#error INVALID USE_Z80 setting
#endif
