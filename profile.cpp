#include <Arduino.h>
#include "profile.h"
#include "disassembler.h"
#include "host.h"

#ifndef PROF_DISPLAY_INTERVAL
#define PROF_DISPLAY_INTERVAL 500000
#endif

#if USE_PROFILING>0

word profiling = false;
unsigned long prof_timer, prof_cycle_counter, prof_counter;

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


static void prof_reset()
{
#if USE_THROTTLE>0
  // if using throttling then we call prof_check only every 10000
  // instructions, so scale the counter accordingly
  prof_counter = PROF_DISPLAY_INTERVAL / 10000;
#else
  prof_counter = PROF_DISPLAY_INTERVAL;
#endif
  
  prof_cycle_counter = 0;
  prof_timer = millis();
}


void prof_print()
{
  unsigned long d = millis() - prof_timer;
  float mhz = ((float) prof_cycle_counter) / ((float) d * 1000.0);
  Serial.print(F("Performance: "));
  Serial.print(prof_cycle_counter);
  Serial.print(F(" cycles in "));
  Serial.print(((float) d)/1000.0);
  Serial.print(F(" sec = "));
  Serial.print(mhz);
  Serial.print(F(" MHz = "));
  Serial.print(int(mhz/0.02+.5));
  Serial.println(F("%"));
  prof_reset();

#if USE_PROFILING_DETAIL>0
  if( ++prof_detail_counter == 10 )
    {
      prof_print_details();
      prof_reset_details();
    }
#endif
}


void prof_toggle()
{
  profiling = !profiling;
  Serial.print(F("\nProfiling is "));
  Serial.print(profiling ? F("ON\n") : F("off\n"));
  prof_reset();
}

#else

#if USE_THROTTLE>0
#error USE_THROTTLE requires USE_PROFILING
#endif

#endif

