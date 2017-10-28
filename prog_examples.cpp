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
#include "config.h"
#include "prog_basic.h"
#include "mem.h"

#if defined(__AVR_ATmega2560__)

// not enough room in MEGA to store ASM examples (or PS2 assembler anyways)
#include "prog_examples_basic_mega.h"
const char * const asm_programs[] = {};
#define READ_EX_BYTE(pv,pi,pc) pgm_read_byte(((const char*) pgm_read_word(pv+(pi)))+(pc))

#else

#include "prog_examples_basic_due.h"
#include "prog_examples_asm.h"
#define READ_EX_BYTE(pv,pi,pc) ((byte) pv[pi][pc])

#endif

static byte     prog_idx = 0;
static uint16_t prog_ctr = 0;
static byte     NULs     = 0;


bool prog_examples_read_start(byte idx)
{
  if( idx < sizeof(basic_programs)/sizeof(char *) ||
      idx >= 0x80 && idx < 0x80+(sizeof(asm_programs)/sizeof(char *)) ||
      idx == 0xc0 )
    {
      prog_idx = idx;
      prog_ctr = 0;
      NULs     = 0;

      if( idx == 0xc0 )
        {
          // 4k BASIC will get into an infinite loop if a full 64k RAM are
          // available => purposely reduce the RAM size by 1 byte
          mem_set_ram_limit_sys(0xfffe);
        }

      return true;
    }
  else
    return false;
}


bool prog_examples_read_next(byte dev, byte *b)
{
  if( NULs>0 )
    {
      NULs--;
      *b = 0;
      return true;
    }
  else if( prog_idx < 0x80 )
    *b = READ_EX_BYTE(basic_programs, prog_idx, prog_ctr);
  else if( prog_idx == 0xC0 )
    return prog_basic_read_4k(prog_ctr++, b);
  else
    *b = READ_EX_BYTE(asm_programs, prog_idx-0x80, prog_ctr);

  if( *b=='\r' ) NULs = config_serial_playback_example_nuls(dev);
  if( *b>0 ) prog_ctr++;
  return *b != 0;
}
