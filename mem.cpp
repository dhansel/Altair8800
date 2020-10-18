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
#include "numsys.h"
#include "filesys.h"
#include "config.h"


word mem_ram_limit = 0xFFFF, mem_protected_limit = 0xFFFF;
byte mem_protected_flags[32];

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
#if SHOW_MWRITE_OUTPUT>0
      altair_set_outputs(a, v);
#else
      altair_set_outputs(a, 0xff);
#endif
      altair_wait_step();
      host_clr_status_led_WO();
    }
}


static bool mem_is_rom(uint16_t a)
{
  for(int i=0; i<mem_get_num_roms(); i++)
    {
      uint16_t pa, pl, pe;
      mem_get_rom_info(i, NULL, &pa, &pl);
      pa &= ~0xFF;
      pe  = ((pa+pl-1)|0xFF);
      if( pa>a )
        return false;
      else if( a>=pa && a<=pe )
        return true;
    }

  return false;
}


void mem_print_layout()
{
  for(uint32_t i=0; i<0x10000; i+=0x100)
    {
      if( (i&0x3FFF) == 0 )
        {
          Serial.println();
          numsys_print_word(i);
          Serial.print(F(": "));
        }

      if( MEM_IS_WRITABLE(i) ) 
        Serial.print('.');
      else if( i>=MEMSIZE )
        Serial.print('#');
      else if( mem_is_rom(i) )
        Serial.print('P');
      else if( i>mem_ram_limit )
        Serial.print('-');
      else
        Serial.print('U');
    }

  Serial.println();
}


static void mem_protect_calc_limit()
{
  mem_protected_limit = mem_ram_limit;
  for(byte i=0; i<32; i++)
    if( mem_protected_flags[i] )
      for(byte j=0; j<8; j++)
        if( mem_protected_flags[i] & (1<<j) )
          {
            mem_protected_limit = min(mem_ram_limit, uint16_t(2048*i + 256*j));
            return;
          }
}

static void mem_protect_flag_set(uint16_t a, bool set)
{
  if( set )
    mem_protected_flags[a>>11] |=  (1<<((a>>8)&0x07));
  else
    mem_protected_flags[a>>11] &= ~(1<<((a>>8)&0x07));
}


static void mem_protect_flags_set(uint16_t start, uint16_t length, bool v)
{
  for(uint32_t p = start; p < (uint32_t) start + length; p += 0x100 )
    mem_protect_flag_set(p, v);
  mem_protect_flag_set(start+length-1, v);
}



void mem_protect(uint16_t a)
{
  mem_protect_flag_set(a, true);
  mem_protect_calc_limit();
}


void mem_unprotect(uint16_t a)
{
  if( !mem_is_rom(a) )
    {
      mem_protect_flag_set(a, false);
      mem_protect_calc_limit();
    }
}


bool mem_is_protected(uint16_t a)
{
  return !MEM_IS_WRITABLE(a) && a<=mem_ram_limit && !mem_is_rom(a);
}


bool mem_is_writable(uint16_t from, uint16_t to)
{
  for(uint32_t p = (from & ~0xff); p <= (uint32_t) to; p += 0x100 ) 
    if( !MEM_IS_WRITABLE(p) )
      return false;

  return true;
}



static void randomize(uint32_t from, uint32_t to)
{
  // note that if from/to are not on 4-byte boundaries
  // then a few bytes remain unchanged
  from = (from&3)==0 ? from/4 : from/4+1;
  to   = to/4;
  for(word i=from; i<to; i++)((uint32_t *) Mem)[i] = host_get_random();
}


static void mem_ram_init_section(uint16_t from, uint16_t to, bool clear)
{
  if( from>mem_ram_limit )
    {
      // "from" is before the RAM limit and "to" is after
      // => initialize with 0xFF
      memset(Mem+from, 0xFF, to-from+1);
    }
  else if( to<=mem_ram_limit )
    {
      // "to" is before the RAM limit
      // => initialize RAM with either 0 or random
      if( clear )
        memset(Mem+from, 0x00, to-from+1);
      else
        randomize(from, to);
    }
  else
    {
      // "from" is before the RAM limit and "to" is after
      // => initialize up to the limit with 0 or random...
      if( clear )
        memset(Mem+from, 0x00, mem_ram_limit-from+1);
      else
        randomize(from, mem_ram_limit);

      // ...and memory after the limit with 0xFF
      memset(Mem+mem_ram_limit+1, 0xFF, to-mem_ram_limit);
    }
}


void mem_ram_init(uint16_t from, uint16_t to, bool force_clear)
{
  byte i;
  uint32_t a = from;
  uint16_t pa, pl;

  // initialize RAM before and between ROMs
  for(i=0; i<mem_get_num_roms() && a<=to; i++)
    {
      mem_get_rom_info(i, NULL, &pa, &pl);
      if( (uint32_t) pa+pl>a )
        {
          if( pa>a ) mem_ram_init_section(a, min(pa-1, to), force_clear || config_clear_memory());
          a = pa+pl;
        }
    }

  // initialize RAM after last ROM
  if( a<=to ) mem_ram_init_section(a, to, force_clear || config_clear_memory());
}


void mem_set_ram_limit_usr(uint16_t a)
{
  uint16_t prev_limit = mem_ram_limit;
  mem_ram_limit = min(a, MEMSIZE-1);

  if( prev_limit < mem_ram_limit )
    {
      mem_ram_init(prev_limit+1, mem_ram_limit);

      // un-protect RAM below the new mem_ram_limit
      for(uint32_t p = prev_limit+1; p < ((uint32_t) mem_ram_limit)+1; p += 0x100 )
        mem_protect_flag_set(p, false);

      // restore protection for ROMs
      for(byte i=0; i<mem_get_num_roms(); i++)
        {
          uint16_t pa, pl;
          mem_get_rom_info(i, NULL, &pa, &pl);
          if( pa>mem_ram_limit ) 
            break;
          else if( pa+pl>prev_limit )
            mem_protect_flags_set(pa, pl, true);
        }
    }
  else if( mem_ram_limit < prev_limit )
    {
      mem_ram_init(mem_ram_limit+1, prev_limit);
      
      // write-protect RAM above the new mem_ram_limit
      // it would be easier to have an additional condition (a<=mem_ram_limit) in the
      // MEM_IS_WRITABLE macro but doing so would slow emulation as writing to memory
      // is one of the most common operations.
      for(uint32_t p = mem_ram_limit+1; p < ((uint32_t) prev_limit)+1; p += 0x100 )
        mem_protect_flag_set(p, true);
    }
  
  mem_protect_calc_limit();
}


uint16_t mem_get_ram_limit_usr()
{
  return mem_ram_limit;
}


void mem_setup()
{
  memset(mem_protected_flags, 0, 32);
  mem_ram_limit = MEMSIZE-1;
  for(uint32_t p = MEMSIZE; p < 0x10000; p += 0x100 )
    mem_protect_flag_set(p, true);
  mem_protect_calc_limit();
}


// -------------------- ROM handling


#if MAX_NUM_ROMS==0

bool mem_add_rom(uint16_t start, uint16_t length, const char *name, uint16_t flags, uint32_t config_file_offset) { return false; }
bool mem_remove_rom(byte i, bool clear) { return false; }
byte mem_get_num_roms(bool includeTemp) { return 0; }
void mem_set_rom_flags(byte i, uint16_t flags) {}
bool mem_get_rom_info(byte i, char *name, uint16_t *start, uint16_t *length, uint16_t *flags) { return false; }
uint16_t mem_get_rom_autostart_address() { return 0xFFFF; }
void mem_clear_roms() {}
void mem_reset_roms() {}

#else

byte     mem_roms_num = 0;
uint16_t mem_roms_start[MAX_NUM_ROMS];
uint16_t mem_roms_length[MAX_NUM_ROMS];
uint16_t mem_roms_flags[MAX_NUM_ROMS];
char     mem_roms_name[MAX_NUM_ROMS][9];
uint32_t mem_roms_filepos[MAX_NUM_ROMS];


bool mem_remove_rom(byte i, bool clear)
{
  if( i<mem_roms_num )
    {
      byte j;
      uint16_t start  = mem_roms_start[i], length = mem_roms_length[i];

      // remove ROM info
      for(j=i+1; j<mem_roms_num; j++)
        {
          mem_roms_start[j-1] = mem_roms_start[j];
         mem_roms_length[j-1] = mem_roms_length[j];
          mem_roms_flags[j-1] = mem_roms_flags[j];
          strcpy(mem_roms_name[j-1], mem_roms_name[j]);
        }
      mem_roms_num--;

      // remove write protection for area occupied by ROM
      mem_protect_flags_set(start, length, false);
      mem_protect_calc_limit();

      // initialize newly visible RAM 
      if( clear ) mem_ram_init(start, start+length-1);
      
      return true;
    }
  else
    return false;
}


bool mem_add_rom(uint16_t start, uint16_t length, const char *nameOpt, uint16_t flags, uint32_t filepos)
{
  int i, j, conflict = -1;
  const char *name = nameOpt == NULL ? "[?]" : nameOpt;

  for(i=0; i<mem_roms_num; i++)
    if( start <= mem_roms_start[i] )
      break;
  
  while( i>0 && ((uint32_t) mem_roms_start[i-1]+mem_roms_length[i-1] > start) && conflict<0 )
    {
      if( mem_roms_flags[i-1] & MEM_ROM_FLAG_TEMP )
        mem_remove_rom(i--, false);
      else
        conflict = i-1;
    }

  while( i<mem_roms_num && (uint32_t) start+length > mem_roms_start[i] && conflict<0 )
    {
      if( mem_roms_flags[i] & MEM_ROM_FLAG_TEMP )
        mem_remove_rom(i, false);
      else
        conflict = i;
    }

  if( start+length>MEMSIZE || conflict>=0 || mem_roms_num >= MAX_NUM_ROMS )
    {
      Serial.print(F("Error: Can't install ROM '"));
      Serial.print(name);
      Serial.print(F("' at "));
      numsys_print_word(start);
      Serial.print('-');
      numsys_print_word(start+length-1);
      if( conflict>=0 )
        {
          Serial.print(F("\r\nbecause it conflicts with existing ROM '"));
          Serial.print(mem_roms_name[conflict]);
          Serial.print(F("' at "));
          numsys_print_word(mem_roms_start[conflict]);
          Serial.print('-');
          numsys_print_word(mem_roms_start[conflict]+mem_roms_length[conflict]-1);
          Serial.println();
        }
      else if( mem_roms_num >= MAX_NUM_ROMS )
        Serial.println(F("\r\nbecause the maximum number of ROMs are already installed."));
      else
        Serial.println();

      return false;
    }

  for(j=mem_roms_num; j>i; j--)
    {
      mem_roms_start[j]  = mem_roms_start[j-1];
      mem_roms_length[j] = mem_roms_length[j-1];
      mem_roms_flags[j]  = mem_roms_flags[j-1];
      strcpy(mem_roms_name[j], mem_roms_name[j-1]);
    }
    
  mem_roms_start[i]  = start;
  mem_roms_length[i] = length;
  mem_roms_flags[i]  = flags;
  strncpy(mem_roms_name[i], name, 8);
  mem_roms_name[i][8] = 0;
  mem_roms_filepos[i] = filepos;
  mem_roms_num++;

  // write-protect area occupied by ROM
  mem_protect_flags_set(start, length, true);
  mem_protect_calc_limit();

  return true;
}


void mem_set_rom_filepos(byte i, uint32_t pos)
{
  if( i<mem_roms_num ) mem_roms_filepos[i] = pos;
}


void mem_disable_rom(byte i)
{
  if( i<mem_roms_num && !(mem_roms_flags[i]&MEM_ROM_FLAG_DISABLED) )
    {
      if( mem_roms_filepos[i]==0 && !(mem_roms_flags[i]&MEM_ROM_FLAG_TEMP) )
        Serial.println(F("[WARNING: ROMs can only be disabled if configuration was previously saved]"));
      else
        {
          uint16_t start  = mem_roms_start[i];
          uint16_t length = mem_roms_length[i];
          mem_roms_flags[i] |= MEM_ROM_FLAG_DISABLED;
          mem_protect_flags_set(start, length, false);
          mem_protect_calc_limit();
          // initialize newly visible RAM 
          mem_ram_init_section(start, start+length-1, config_clear_memory());
        }
    }
}


void mem_disable_rom(const char *name)
{
  for(byte i=0; i<mem_roms_num; i++)
    if( strncmp(mem_roms_name[i], name, 9)==0 )
      mem_disable_rom(i);
}


byte mem_get_num_roms(bool includeTemp)
{
  if( includeTemp )
    return mem_roms_num;
  else
    {
      byte n = 0;
      for(int i=0; i<mem_roms_num; i++)
        if( !(mem_roms_flags[i] & MEM_ROM_FLAG_TEMP) )
          n++;

      return n;
    }
}


uint16_t mem_get_rom_autostart_address()
{
  for(byte i=0; i<mem_roms_num; i++)
    if( mem_roms_flags[i] & MEM_ROM_FLAG_AUTOSTART )
      return mem_roms_start[i];

  return 0xFFFF;
}


void mem_set_rom_flags(byte i, uint16_t flags)
{
  if( i<mem_roms_num )
    mem_roms_flags[i] = flags;
}


bool mem_get_rom_info(byte i, char *name, uint16_t *start, uint16_t *length, uint16_t *flags)
{
  if( i<mem_roms_num )
    {
      if( name )   strcpy(name, mem_roms_name[i]);
      if( start )  *start = mem_roms_start[i];
      if( length ) *length = mem_roms_length[i];
      if( flags )  *flags = mem_roms_flags[i];
      return true;
    }
  else
    return false;
}


byte mem_find_rom(const char *name)
{
  byte n = mem_get_num_roms();
  for(byte i=0; i<mem_roms_num; i++)
    if( strncmp(name, mem_roms_name[i], 9)==0 )
      return i;

  return 0xff;
}


void mem_clear_roms()
{
  while( mem_roms_num>0 ) mem_remove_rom(0);
}


void mem_restore_roms()
{
  for(byte i=0; i<mem_roms_num; i++ )
    if( (mem_roms_flags[i]&MEM_ROM_FLAG_DISABLED) && !(mem_roms_flags[i] & MEM_ROM_FLAG_TEMP) )
      {
        if( mem_roms_filepos[i]>0 )
          {
            // restore ROM data from config file
            byte fid = filesys_open_read('C', config_get_current());
            if( fid )
              {
                filesys_seek(fid, mem_roms_filepos[i]);
                filesys_read_data(fid, Mem+mem_roms_start[i], mem_roms_length[i]);
                filesys_close(fid);
              }
          }

        mem_roms_flags[i] &= ~MEM_ROM_FLAG_DISABLED;
        mem_protect_flags_set(mem_roms_start[i], mem_roms_length[i], true);
      }

  mem_protect_calc_limit();
}


void mem_reset_roms()
{
  byte i = 0;
  while( i<mem_roms_num )
    {
      if( mem_roms_flags[i] & MEM_ROM_FLAG_TEMP )
        mem_remove_rom(i, true);
      else
        i++;
    }

  mem_restore_roms();
}

#endif
