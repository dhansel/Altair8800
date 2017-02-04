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

