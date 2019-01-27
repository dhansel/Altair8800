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

#include "timer.h"
#include "config.h"

#ifdef __AVR_ATmega2560__
#define MAX_TIMERS 9
#else
#define MAX_TIMERS 13
#endif


#define DEBUG 0

uint32_t timer_cycle_counter        = 0;
uint32_t timer_cycle_counter_offset = 0;
uint32_t timer_next_expire_cycles   = 0xffffffff;
byte     timer_next_expire_tid      = 0xff;


struct TimerData {
  TimerFnTp timer_fn;
  bool      running;
  bool      recurring;
  uint32_t  cycles_period;
  uint32_t  cycles_count;
};

struct TimerData timer_data[MAX_TIMERS];
byte timer_queue[MAX_TIMERS];
byte timer_queue_len = 0;

#if DEBUG>1
static void print_queue()
{
  printf("TIMER QUEUE: [");
  for(byte i=0; i<timer_queue_len; i++)
    printf("%i=%i/%i ", timer_queue[i], timer_data[timer_queue[i]].cycles_count, timer_data[timer_queue[i]].cycles_period);
  if( timer_queue_len>0 )
    printf("] / next timer %i in %u\n", timer_next_expire_tid, timer_next_expire_cycles);
  else
    printf("]\n");
}
#else
#define print_queue() while(0)
#endif

static void print_queue2()
{
  printf("TIMER QUEUE: [");
  for(byte i=0; i<timer_queue_len; i++)
    printf("%i=%i/%i ", timer_queue[i], timer_data[timer_queue[i]].cycles_count, timer_data[timer_queue[i]].cycles_period);
  if( timer_queue_len>0 )
    printf("] / next timer %i in %u\n", timer_next_expire_tid, timer_next_expire_cycles);
  else
    printf("]\n");
}


static void timer_reset_counter()
{
#if DEBUG>1
  printf("timer_reset_counter()\n");
#endif
  for(byte i=0; i<timer_queue_len; i++)
    {
      if( timer_data[timer_queue[i]].cycles_count > timer_cycle_counter )
        timer_data[timer_queue[i]].cycles_count -= timer_cycle_counter;
      else
        timer_data[timer_queue[i]].cycles_count = 0;
    }

  timer_cycle_counter_offset += timer_cycle_counter;
  timer_next_expire_cycles   -= timer_cycle_counter;
  timer_cycle_counter = 0;

  print_queue();
}


static void timer_queue_get_next()
{
#if DEBUG>1
  printf("timer_queue_get_next()\n");
#endif
  if( timer_queue_len<2 )
    {
      timer_next_expire_tid    = 0xff;
      timer_next_expire_cycles = 0xffffffff;
      timer_queue_len          = 0;
      timer_cycle_counter_offset += timer_cycle_counter;
      timer_cycle_counter = 0;
    }
  else
    {
      // remove first element from queue
      memmove(timer_queue, timer_queue+1, timer_queue_len-1);
      timer_queue_len--;

      // reset cycle counter
      for(byte i=0; i<timer_queue_len; i++)
        {
          if( timer_data[timer_queue[i]].cycles_count > timer_cycle_counter )
            timer_data[timer_queue[i]].cycles_count -= timer_cycle_counter;
          else
            timer_data[timer_queue[i]].cycles_count = 0;
        }

      timer_cycle_counter_offset += timer_cycle_counter;
      timer_cycle_counter = 0;

      // set next expiration
      timer_next_expire_tid    = timer_queue[0];
      timer_next_expire_cycles = timer_data[timer_next_expire_tid].cycles_count;
    }

  print_queue();
}


static void timer_queue_add(byte tid)
{
#if DEBUG>1
  printf("timer_queue_add()\n");
#endif

  timer_reset_counter();
  uint32_t cycles_count = timer_data[tid].cycles_count;

  byte i;
  for(i=0; i<timer_queue_len; i++)
    if( timer_data[timer_queue[i]].cycles_count > cycles_count )
      break;

  memmove(timer_queue+i+1, timer_queue+i, timer_queue_len-i);
  timer_queue[i] = tid;
  timer_queue_len++;

  if( i==0 )
    {
      timer_next_expire_tid    = tid;
      timer_next_expire_cycles = timer_data[tid].cycles_count;
    }

  print_queue();
}


static void timer_queue_remove(byte tid)
{
#if DEBUG>1
  printf("timer_queue_remove()\n");
#endif

  byte i;
  for(i=0; i<timer_queue_len; i++)
    if( timer_queue[i] == tid )
      {
        memmove(timer_queue+i, timer_queue+i+1, timer_queue_len-i-1);
        timer_queue_len--;
        
        if( timer_queue_len==0 )
          {
            timer_next_expire_tid    = 0xff;
            timer_next_expire_cycles = 0xffffffff;
          }
        else if( i==0 )
          {
            timer_next_expire_tid    = timer_queue[0];
            timer_next_expire_cycles = timer_data[timer_next_expire_tid].cycles_count;
          }
        
        print_queue();
        return;
      }
}


void timer_check()
{
  byte tid = timer_next_expire_tid;
  bool show = false;

  while( tid<0xff )
    {
#if DEBUG>0
      printf("%u: timer %i expired\n", timer_get_cycles(), tid);
      print_queue();
#endif
      if( !timer_data[tid].recurring ) 
        timer_data[tid].running = false;

      timer_queue_get_next();

      if( timer_data[tid].recurring )
        {
#if DEBUG>0
          printf("re-scheduling timer %i\n", tid);
#endif
          timer_data[tid].cycles_count = timer_data[tid].cycles_period;
          timer_queue_add(tid);
        }

#if DEBUG>1
      printf("calling timer %i function\n", tid);
#endif
      (timer_data[tid].timer_fn)();
#if DEBUG>1
      printf("returned from timer %i function\n", tid);
#endif
      
      // check if more timers expired
      if( timer_queue_len>0 && timer_data[timer_queue[0]].cycles_count==0 )
        tid = timer_queue[0];
      else
        tid = 0xff;
    }
}


void timer_setup(byte tid, uint32_t microseconds, TimerFnTp timer_fn)
{
  if( tid<MAX_TIMERS )
    {
      bool running = timer_data[tid].running;
      if( running ) timer_stop(tid);
      timer_data[tid].timer_fn      = timer_fn;
      timer_data[tid].cycles_period = microseconds*2;
      if( running ) timer_start(tid);
    }
}


void timer_start(byte tid, uint32_t microseconds, bool recurring)
{
  if( tid<MAX_TIMERS )
    {
      if( microseconds>0 ) timer_data[tid].cycles_period = microseconds*2;

#if DEBUG>0
      printf("%u: starting timer %i: %i microseconds\n", timer_get_cycles(), tid, timer_data[tid].cycles_period/2);
#endif
      if( timer_data[tid].running ) timer_queue_remove(tid);

      timer_data[tid].recurring    = recurring;
      timer_data[tid].cycles_count = timer_data[tid].cycles_period;
      timer_data[tid].running      = true;
      
      timer_queue_add(tid);
    }
}


void timer_stop(byte tid)
{
  if( tid < MAX_TIMERS && timer_data[tid].running )
    {
#if DEBUG>0
      printf("%u: stopping timer %i\n", timer_get_cycles(), tid);
#endif
      timer_queue_remove(tid);
      timer_data[tid].running = false;
    }
}

uint32_t timer_get_period(byte tid)
{
  return tid < MAX_TIMERS ? timer_data[tid].cycles_period / 2 : 0;
}

bool timer_running(byte tid)
{
  return tid < MAX_TIMERS ? timer_data[tid].running : false;
}


void timer_setup()
{
  timer_queue_len  = 0;
  timer_cycle_counter = 0;
  timer_cycle_counter_offset = 0;
  for(byte tid=0; tid<MAX_TIMERS; tid++)
    {
      timer_data[tid].running  = false;
      timer_data[tid].timer_fn = NULL;
    }
}
