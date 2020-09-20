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
#include "prog_dazzler.h"
#include "numsys.h"
#include "host.h"
#include "mem.h"


struct prog_info_struct {
  PGM_P name;
  uint16_t (*load)();
  bool run;
};


uint16_t prog_print_dir();


struct prog_info_struct get_prog_info(byte i)
{
  // To save global memory (mostly for the Mega) we build up this table on the
  // stack and then pick the entry we need and return it. Note that this returns
  // the whole entry, not just a pointer to it.
  const struct prog_info_struct programs[] =
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
#if NUM_DRIVES>0
      {PSTR("Disk boot ROM"),              prog_tools_copy_diskboot,      true},
#endif
      {PSTR("ALTAIR Turnkey Monitor"),     prog_tools_copy_turnmon,       true},
      {PSTR("Music ('Daisy')"),            prog_games_copy_daisy,         true},
#endif
      {PSTR("CPU Diagnostic"),             prog_tools_copy_diag,          true},
      {PSTR("CPU Exerciser"),              prog_tools_copy_exerciser,     true},
#if !defined(__AVR_ATmega2560__)
      {PSTR("Music system"),               prog_tools_copy_musicsys,      true},
#if NUM_HDSK_UNITS>0
      {PSTR("Hard disk boot ROM"),         prog_tools_copy_hdbl,          true},
#endif
      {PSTR("Multi-boot loader ROM"),      prog_tools_copy_multiboot,     true},
#if NUM_TDRIVES>0
      {PSTR("Tarbell disk boot ROM"),      prog_tools_copy_tdiskboot,     true},
#endif
#if NUM_CDRIVES>0
      {PSTR("Cromemco RDOS 1.0 ROM"),      prog_tools_copy_rdos10,        true},
#endif
#endif
#if USE_DAZZLER>0
      {PSTR("Dazzler Kaleidoscope"),       prog_dazzler_copy_kaleidoscope,true},
#if 0
      {PSTR("DazzleDoodle"),               prog_dazzler_copy_doodle,      true},
      {PSTR("DazzleMation"),               prog_dazzler_copy_animation,   true},
      {PSTR("DazzleWriter"),               prog_dazzler_copy_writer,      true},
      {PSTR("Dazzler Life"),               prog_dazzler_copy_life,        true},
      {PSTR("Dazzler Track"),              prog_dazzler_copy_track,       true},
      {PSTR("Dazzler Chase"),              prog_dazzler_copy_chase,       true},
      {PSTR("Dazzler Spacewar"),           prog_dazzler_copy_spacewar,    true},
      {PSTR("Dazzler Gotcha"),             prog_dazzler_copy_gotcha,      true},
      {PSTR("Dazzler 4d Tic Tac Toe"),     prog_dazzler_copy_tic_tac_toe, true},
#endif
#endif
#if USE_VDM1>0
      {PSTR("Cuter for VDM-1"),            prog_tools_copy_vdmcuter,      true},
#endif
      //{PSTR("ADEXER"),                     prog_tools_copy_adexer,        true},
      //{PSTR("Status lights test"),         prog_tools_copy_statustest,    false},
      //{PSTR("Serial echo using IRQ"),      prog_tools_copy_serialirqtest, false},
      {NULL, NULL, false}
    };

  return programs[i];
}


uint16_t prog_print_dir()
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

  Serial.println(F("01xxxxxx) [Read Intel HEX data from primary host interface]"));
  Serial.println(F("10nnnnnn) [load memory page, nnnnnn=file number]"));
  Serial.println(F("11nnnnnn) [save memory page, nnnnnn=file number]"));
  return 0;
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


bool prog_load(byte n, uint16_t *pc)
{
  // check that program #n exists
  for(int i=0; i<=n; i++) 
    if( get_prog_info(i).name == NULL )
      return false;
  
  uint16_t addr = get_prog_info(n).load();
  if( n>0 )
    {
      if( addr==0xFFFF )
        Serial.print(F("[Unable to load "));
      else if( get_prog_info(n).run )
        Serial.print(F("[Running "));
      else
        Serial.print(F("[Loading "));

      Serial.print(FP(get_prog_info(n).name));
      Serial.println(']');
      
      if( addr!=0xFFFF )
        {
          *pc = addr;
          return get_prog_info(n).run;
        }
    }
  
  return false;
}


bool prog_copy_to_ram(uint16_t ramdst, const void *src, uint32_t length)
{
  if( mem_is_writable(ramdst, ramdst+length-1) ) 
    {
      host_copy_flash_to_ram(Mem+ramdst, src, length);
      return true;
    }
  else
    return false;
}


bool prog_create_temporary_rom(uint16_t ramdst, const void *src, uint32_t length, const char *name)
{
#if MAX_NUM_ROMS==0
  return prog_copy_to_ram(ramdst, src, length);
#else
  if( ramdst+length <= MEMSIZE )
    {
      if( memcmp(Mem+ramdst, src, length)==0 )
        return true;
      else if( mem_add_rom(ramdst, length, name, MEM_ROM_FLAG_TEMP) )
        {
          host_copy_flash_to_ram(Mem+ramdst, src, length);
          return true;
        }
    }

  return false;
#endif
}


// C implementation of the Altair checksum loader
// returns 0xffff on failure, otherwise returns the start address
uint16_t prog_checksum_loader(const byte *tape, unsigned int tape_length)
{
  byte d, n, cs;
  uint16_t addr;
  
  while( 1 )
    {
      if( tape_length<1 ) return 0xffff; else tape_length--;
      d = pgm_read_byte(tape++);
      switch( d )
        {
        case 0074:
          {
            // program load record
            if( tape_length < 3 ) return 0xffff; else tape_length -= 3;
            n     = pgm_read_byte(tape++);
            cs    = pgm_read_byte(tape++);
            d     = pgm_read_byte(tape++);
            addr  = cs + d * 256;
            cs    = cs + d;

            if( tape_length < ((unsigned int) n)+1u )
              return 0xffff;
            else
              tape_length -= ((unsigned int) n)+1u;

            do {
              d = pgm_read_byte(tape++);
              cs += d;
              MWRITE(addr, d);
              if( MREAD(addr) != d ) return 0xffff; // no memory at this address
              addr++;
              n--;
            }
            while( n>0 );
            d = pgm_read_byte(tape++);
            if( cs != d ) return 0xffff; // checksum error
            break;
          }

        case 0170:
          {
            // EOF Record
            if( tape_length < 2 ) return 0xffff; else tape_length -= 2;
            addr  = pgm_read_byte(tape++);
            addr += pgm_read_byte(tape++) * 256;
            return addr;
          }
        }
    }
  
  return 0xffff; // should never get here
}
