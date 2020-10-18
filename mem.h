// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// -----------------------------------------------------------------------------

#ifndef MEM_H
#define MEM_H

#include "config.h"
#include "host.h"
#include "prog_basic.h"
#include "breakpoint.h"
#include "cpucore.h"
#include "Altair8800.h"

extern byte Mem[MEMSIZE];
extern word mem_protected_limit;

extern byte mem_protected_flags[32];

#define MEM_IS_WRITABLE(a) ((a) < mem_protected_limit || !(mem_protected_flags[(a)>>11] & (1<<(((a)>>8)&0x07))))

void mem_protect(uint16_t a);
void mem_unprotect(uint16_t a);
bool mem_is_protected(uint16_t a);
bool mem_is_writable(uint16_t from, uint16_t to);
void mem_print_layout();

#if MEMSIZE < 0x10000
// if we have less than 64k of RAM then always map ROM basic to 0xC000-0xFFFF
#define MREAD(a)    ((a)>=0xC000 ? prog_basic_read_16k(a) : ((a) < MEMSIZE ? Mem[a] : 0xFF))
#define MWRITE(a,v) {if( MEM_IS_WRITABLE(a) ) Mem[a]=v;}
#else
// If we have 64k of RAM then we just copy ROM basic to the upper 16k and write-protect
// that area.  Faster to check the address on writing than reading since there are far more
// reads than writes. Also we can skip memory bounds checking because addresses are 16 bit.
#define MREAD(a)    (Mem[a])

#if USE_DAZZLER>0
#include "dazzler.h"
#if USE_VDM1>0
#include "vdm1.h"
#define MWRITE(a,v) { dazzler_write_mem(a, v); vdm1_write_mem(a, v); if( MEM_IS_WRITABLE(a) ) Mem[a]=v; }
#else
#define MWRITE(a,v) { dazzler_write_mem(a, v); if( MEM_IS_WRITABLE(a) ) Mem[a]=v; }
#endif
#elif USE_VDM1>0
#include "vdm1.h"
#define MWRITE(a,v) { vdm1_write_mem(a, v); if( MEM_IS_WRITABLE(a) ) Mem[a]=v; }
#else
#define MWRITE(a,v) { if( MEM_IS_WRITABLE(a) ) Mem[a]=v; }
#endif

#endif

byte MEM_READ_STEP(uint16_t a);
void MEM_WRITE_STEP(uint16_t a, byte v);

// WARNING: arguments to MEM_READ and MEM_WRITE macros should not have side effects
// (e.g. MEM_READ(addr++)) => any side effects will be executed multiple times!

#if USE_REAL_MREAD_TIMING>0
inline byte MEM_READ(uint16_t a)
{
  byte res;
  if( host_read_status_led_WAIT() )
    res = MEM_READ_STEP(a);
  else
    {
      host_set_addr_leds(a);
      host_set_status_leds_READMEM();
      res = host_set_data_leds(MREAD(a));
      host_clr_status_led_MEMR();
    }
  return res;
}
#else
#define MEM_READ(a) ( host_read_status_led_WAIT() ? MEM_READ_STEP(a) : (host_set_status_leds_READMEM(),  host_set_addr_leds(a), host_set_data_leds(MREAD(a)) ))
#endif

#if SHOW_MWRITE_OUTPUT>0
inline void MEM_WRITE(uint16_t a, byte v)
{
  if( host_read_status_led_WAIT() )
    MEM_WRITE_STEP(a,v );
  else
    {
      host_set_addr_leds(a);
      host_set_data_leds(v);
      host_set_status_leds_WRITEMEM();
      MWRITE(a, v);
      host_clr_status_led_WO();
    }
}
#else
#define MEM_WRITE(a, v) if( host_read_status_led_WAIT() ) MEM_WRITE_STEP(a, v); else { host_set_status_leds_WRITEMEM(); host_set_addr_leds(a); host_set_data_leds(0xff); MWRITE(a, v); }
#endif


// manage ROMs
#define MEM_ROM_FLAG_TEMP      0x01
#define MEM_ROM_FLAG_AUTOSTART 0x04
#define MEM_ROM_FLAG_DISABLED  0x08

bool mem_add_rom(uint16_t start, uint16_t length, const char *name = NULL, uint16_t flags = 0, uint32_t filepos = 0);
void mem_set_rom_filepos(byte i, uint32_t pos);
bool mem_remove_rom(byte i, bool clear = true);
byte mem_get_num_roms(bool includeTemp = true);
void mem_set_rom_flags(byte i, uint16_t flags);
bool mem_get_rom_info(byte i, char *name = NULL, uint16_t *start = NULL, uint16_t *length = NULL, uint16_t *flags = NULL);
void mem_disable_rom(byte i);
void mem_disable_rom(const char *name);
uint16_t mem_get_rom_autostart_address();
void mem_clear_roms();
void mem_reset_roms();
void mem_restore_roms();
byte mem_find_rom(const char *name);


// set the highest address to be treated as RAM (everything above is ROM)
void     mem_set_ram_limit_usr(uint16_t a);
uint16_t mem_get_ram_limit_usr();

void mem_ram_init(uint16_t from, uint16_t to, bool force_clear = false);

void mem_setup();

#endif
