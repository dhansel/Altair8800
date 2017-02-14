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
#include "prog.h"
#include "Altair8800.h"
#include "prog_basic.h"
#include "prog_tools.h"
#include "prog_games.h"
#include "prog_ps2.h"
#include "numsys.h"
#include "host.h"


struct prog_info_struct {
  PGM_P name;
  uint16_t (*load)(byte *);
  bool run;
};


uint16_t prog_print_dir(byte *mem);


struct prog_info_struct get_prog_info(byte i)
{
  struct prog_info_struct programs[] =
    {
      {PSTR("[print this directory]"),     prog_print_dir,                false},
      {PSTR("Calculator"),                 prog_tools_copy_calc,          true},
      {PSTR("Kill-the-Bit"),               prog_games_copy_killbits,      true},
      {PSTR("Pong (LEDs)"),                prog_games_copy_pong,          true},
      {PSTR("Pong (Terminal)"),            prog_games_copy_pongterm,      true},
      {PSTR("4k Basic"),                   prog_basic_copy_4k,            true},
      {PSTR("16k ROM Basic"),              prog_basic_copy_16k,           true},
#if !defined(__AVR_ATmega2560__)
      {PSTR("MITS Programming System II"), prog_ps2_copy_monitor,         true},
      {PSTR("Disk boot ROM"),              prog_tools_copy_diskboot,      true},
      {PSTR("ALTAIR Turnkey Monitor"),     prog_tools_copy_turnmon,       true},
      {PSTR("Music ('Daisy')"),            prog_games_copy_daisy,         true},
#endif
      {PSTR("CPU Diagnostic"),             prog_tools_copy_diag,          true},
      {PSTR("CPU Exerciser"),              prog_tools_copy_exerciser,     true},
      //{PSTR("Status lights test"),         prog_tools_copy_statustest,    false},
      //{PSTR("Serial echo using IRQ"),      prog_tools_copy_serialirqtest, false},
      {NULL, NULL, false}
    };

  return programs[i];
}


uint16_t prog_print_dir(byte *mem)
{
  int i = 0;

  Serial.println();
  while( get_prog_info(i).name!=NULL )
    {
      numsys_print_byte_bin(i);
      Serial.print(F(") "));
      Serial.println(FP(get_prog_info(i).name));
      i++;
    }

  Serial.println(F("10nnnnnn) [load memory page, nnnnnn=file number]"));
  Serial.println(F("11nnnnnn) [save memory page, nnnnnn=file number]"));
}


byte prog_find(const char *name)
{
  for(byte i=0; i<0xff; i++) 
    {
      if( get_prog_info(i).name == NULL )
        return 0;
      else if( strcmp_P(name, get_prog_info(i).name)==0 )
        return i;
    }

  return 0;
}


const char *prog_get_name(byte n)
{
  // check that program #n exists
  for(int i=0; i<n; i++) 
    if( get_prog_info(i).name == NULL )
      return NULL;

  return get_prog_info(n).name;
}


bool prog_load(byte n, uint16_t *pc, byte *mem)
{
  // check that program #n exists
  for(int i=0; i<=n; i++) 
    if( get_prog_info(i).name == NULL )
      return false;
  
  uint16_t addr = get_prog_info(n).load(mem);
  if( addr<0xffff )
    {
      *pc = addr;
      return get_prog_info(n).run;
    }
  else
    return false;
}
