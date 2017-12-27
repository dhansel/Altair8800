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
#include "profile.h"
#include "disassembler.h"
#include "timer.h"
#include "config.h"
#include "host.h"

#if USE_PROFILING_DETAIL>0
unsigned long prof_detail_counter, prof_opcode_count[256];

void prof_reset_details()
{
  prof_detail_counter = 0;
  for(int i=0; i<256; i++) prof_opcode_count[i]=0;
}

void prof_print_details()
{
  int i;
  byte dummymem[3] = {0, 0, 0};
  unsigned long total;
  byte maxIdx;
  float totalpct = 0.0;

  total = 0;
  for(i=0; i<256; i++) total += prof_opcode_count[i];

  int num = 0;
  while( totalpct<.99 )
    {
      maxIdx = 0;
      for(i=1; i<256; i++)
        if( prof_opcode_count[i] > prof_opcode_count[maxIdx] )
          maxIdx = i;

      if( prof_opcode_count[maxIdx]==0 )
        break;
      else
        {
          float pct = ((float) (prof_opcode_count[maxIdx])) / ((float) total);
          totalpct += pct;
          dummymem[0] = maxIdx;
          Serial.print(prof_opcode_count[maxIdx]);
          Serial.print(F(" = "));
          Serial.print(pct * 100.0);
          Serial.print(F("% = "));
          Serial.print(totalpct * 100.0);
          Serial.print(F("% : "));
          disassemble(dummymem, 0);
          Serial.print('\n');
          prof_opcode_count[maxIdx] = 0;
        }
    }
}

#endif


static uint32_t prof_time;
static uint32_t prof_cycles;
extern uint16_t throttle_delay;

void prof_print()
{
  uint32_t now = micros();

  if( (now-prof_time) >= 1000000 )
    {
      uint32_t d   = now - prof_time;
      uint32_t c   = timer_get_cycles() - prof_cycles;
      float mhz = ((float) c) / ((float) d);
      Serial.print(F("\033[s\033[H\033[32mPerformance: "));
      Serial.print(c);
      Serial.print(F(" cycles in "));
      Serial.print(((float) d)/1000.0);
      Serial.print(F(" msec = "));
      Serial.print(mhz);
      Serial.print(F(" MHz = "));
      Serial.print(int(mhz/0.02+.5));
      Serial.print('%');
#if USE_THROTTLE>0
      Serial.print(F(" (d="));
      Serial.print(throttle_delay);
      Serial.print(')');
#endif
      Serial.print(F("\033[K\033[37m\033[u"));
      
#if USE_PROFILING_DETAIL>0
      if( ++prof_detail_counter == 10 )
        {
          prof_print_details();
          prof_reset_details();
        }
#endif
      
      prof_time   = now;
      prof_cycles = timer_get_cycles();
    }
}


void profile_enable(bool b)
{
  if( b )
    {
      prof_time   = micros();
      prof_cycles = timer_get_cycles();
      timer_start(TIMER_PROFILE, 0, true);
    }
  else
    timer_stop(TIMER_PROFILE);

#if USE_PROFILING_DETAIL>0
  prof_reset_details();
#endif
}


void profile_setup()
{
  timer_setup(TIMER_PROFILE, 100000, prof_print);
}
