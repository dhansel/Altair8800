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

#include "Altair8800.h"
#include "mem.h"
#include "host.h"

byte Mem[MEMSIZE];


byte MEM_READ_STEP(uint16_t a)
{
  if( altair_isreset() )
    {
      byte v = MREAD(a);
      host_set_status_leds_READMEM();
      altair_set_outputs(a, v);
      altair_wait_step();
      v = host_read_data_leds(); // CPU reads whatever is on the data bus at this point
      host_clr_status_led_MEMR();
      return v;
    }
  else
    return 0x00;
}


void MEM_WRITE_STEP(uint16_t a, byte v)
{ 
  if( altair_isreset() )
    {
      MWRITE(a, v);
      host_set_status_leds_WRITEMEM();
      altair_set_outputs(a, 0xff);
      altair_wait_step();
      host_clr_status_led_WO();
    }
}


#if USE_PROTECT>0

word protected_flag = 0;
byte protected_flags[32];

void mem_protect(uint16_t a)
{
  protected_flags[((a)>>8)/8] |= (1<<(((a)>>8)&0x07));
  protected_flag = 1;
}

void mem_unprotect(uint16_t a)
{
  byte i;
  protected_flags[((a)>>8)/8] &= ~(1<<(((a)>>8)&0x07));
  for(i=0; i<32 && protected_flags[i]==0; i++);
  protected_flag = i<32;
}


#endif

word        mem_ram_limit      = 0xFFFF;
static word mem_ram_limit_user = 0xFFFF;
static word mem_ram_limit_sys  = 0xFFFF;

void mem_set_ram_limit_sys(uint16_t a)
{
  if( a < MEMSIZE )
    mem_ram_limit_sys = a;
  else
    mem_ram_limit_sys = MEMSIZE-1;

  mem_ram_limit = min(mem_ram_limit_user, mem_ram_limit_sys);
}


void mem_set_ram_limit_usr(uint16_t a)
{
  if( a < mem_ram_limit_user )
    {
      // areas without RAM read 0xFF
      for(uint16_t i=a; i<MEMSIZE-1 && i<mem_ram_limit_sys-1; i++)
        Mem[i+1] = 0xFF;
    }
  else if( a > mem_ram_limit_user )
    {
      // initialize newly "installed" RAM with 0
      for(uint16_t i=mem_ram_limit_user+1; i<MEMSIZE-1 && i<a && i<mem_ram_limit_sys; i++)
        Mem[i+1] = 0x00;
    }

  if( a < MEMSIZE )
    mem_ram_limit_user = a;
  else
    mem_ram_limit_user = MEMSIZE-1;

  mem_ram_limit = min(mem_ram_limit_user, mem_ram_limit_sys);
}


uint16_t mem_get_ram_limit_usr()
{
  return mem_ram_limit_user;
}


void mem_clr_ram_limit()
{
  mem_ram_limit = mem_ram_limit_user;
}


void mem_setup()
{
#if USE_PROTECT>0
  for(int i=0; i<32; i++) protected_flags[i]=0x00;
  protected_flag = 0;
#endif

  mem_ram_limit_user = MEMSIZE-1;
  mem_ram_limit_sys  = 0xFFFF;
  mem_clr_ram_limit();
}
