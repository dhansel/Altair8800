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

#include "breakpoint.h"
#include "numsys.h"
#include "Altair8800.h"
#include "cpucore.h"

#if MAX_BREAKPOINTS > 0

byte numBreakpoints = 0;
uint16_t breakpoints[MAX_BREAKPOINTS];

void break_check_do(uint16_t addr)
{
  byte i;
  for(i=0; i<numBreakpoints; i++)
    if( addr == breakpoints[i] )
      {
        Serial.print(F("\n\n--- Reached breakpoint at "));
        numsys_print_word(addr);
        Serial.print(F(" ---\n\n"));
        altair_interrupt(INT_SW_STOP);
      }
}


void breakpoint_add(uint16_t addr)
{
  breakpoints[numBreakpoints++] = addr;
}


void breakpoint_remove_last()
{
  if( numBreakpoints>0 )
    numBreakpoints--;
}


void breakpoint_print()
{
  for(byte b=0; b<numBreakpoints; b++)
    {
      numsys_print_word(breakpoints[b]);
      Serial.print(' ');
    }
}

#endif

