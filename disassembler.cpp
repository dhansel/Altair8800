// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
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

#include <Arduino.h>
#include "disassembler.h"
#include "disassembler_i8080.h"
#include "disassembler_z80.h"
#include "cpucore.h"


byte disassemble(byte *Mem, uint16_t PC, bool print_bytes)
{
#if USE_Z80==0 // fixed I8080 CPU
  return disassemble_i8080(Mem, PC, print_bytes);
#elif USE_Z80==1 // fixed Z80 CPU
  return disassemble_z80(Mem, PC, print_bytes);
#else // CPU is switchable
  if( cpu_get_processor() == PROC_I8080 )
    return disassemble_i8080(Mem, PC, print_bytes);
  else
    return disassemble_z80(Mem, PC, print_bytes);
#endif
}
