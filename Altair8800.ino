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
#include "config.h"
#include "cpucore.h"
#include "host.h"
#include "mem.h"
#include "serial.h"
#include "profile.h"
#include "breakpoint.h"
#include "disassembler.h"
#include "numsys.h"
#include "filesys.h"
#include "drive.h"
#include "timer.h"
#include "prog.h"

#define BIT(n) (1<<(n))

uint16_t cswitch = 0;
uint16_t dswitch = 0;

uint16_t p_regPC = 0xFFFF;

volatile byte     altair_interrupts_buf = 0;
volatile uint16_t altair_interrupts     = 0;
volatile byte     altair_vi_level_cur   = 0;
volatile byte     altair_vi_level       = 0;
volatile bool     altair_interrupts_enabled = false;
static   bool     altair_rtc_available  = false;
static   bool     altair_rtc_running    = false;

word status_wait = false;
bool have_ps2    = false;

void print_panel_serial(bool force = false);
void print_dbg_info();
void read_inputs_panel();
void read_inputs_serial();
void process_inputs();
void rtc_setup();

uint32_t  throttle_micros = 0;
uint16_t  throttle_delay  = 0;

#define THROTTLE_TIMER_PERIOD 25000
void update_throttle()
{
  uint32_t now   = micros();
  uint32_t ratio = (200*THROTTLE_TIMER_PERIOD) / (now-throttle_micros);

  if( ratio>201 )
    throttle_delay += (uint16_t) HOST_PERFORMANCE_FACTOR;
  else if( ratio<200 && throttle_delay>0 )
    throttle_delay -= (uint16_t) HOST_PERFORMANCE_FACTOR;

  throttle_micros = now;
}


void altair_set_outputs(uint16_t a, byte v)
{
  host_set_addr_leds(a);
  host_set_data_leds(v);
  if( MEM_IS_PROTECTED(a) )
    host_set_status_led_PROT();
  else
    host_clr_status_led_PROT();
  
  if( host_read_status_led_M1() ) print_dbg_info();
  print_panel_serial();
}


void altair_wait_reset()
{
  // set bus/data LEDs on, status LEDs off
  altair_set_outputs(0xffff, 0xff);
  host_clr_status_led_INT();
  host_set_status_led_WO();
  host_clr_status_led_STACK();
  host_clr_status_led_HLTA();
  host_clr_status_led_OUT();
  host_clr_status_led_M1();
  host_clr_status_led_INP();
  host_clr_status_led_MEMR();
  host_clr_status_led_INTE();
  host_clr_status_led_PROT();

  // wait while RESET switch is held
  while( host_read_function_switch_debounced(SW_RESET) );

  cswitch |= BIT(SW_RESET);
}


void read_inputs()
{
  cswitch = 0;
  read_inputs_panel();
  read_inputs_serial();
  print_panel_serial();
  process_inputs();
}


void process_inputs()
{  
  if( cswitch & BIT(SW_DEPOSIT) )
    {
      MWRITE(regPC, dswitch&0xff);
      altair_set_outputs(regPC, MREAD(regPC));
    }
  else if( cswitch & BIT(SW_DEPNEXT) )
    {
      regPC++;
      MWRITE(regPC, dswitch&0xff);
      altair_set_outputs(regPC, MREAD(regPC));
    }

  if( cswitch & BIT(SW_EXAMINE) )
    {
      regPC = dswitch;
      p_regPC = ~regPC;
      altair_set_outputs(regPC, MREAD(regPC));
    }
  else if( cswitch & BIT(SW_EXNEXT) )
    {
      regPC = regPC+1;
      altair_set_outputs(regPC, MREAD(regPC));
    }

  if( cswitch & BIT(SW_AUX1DOWN) )
    {
      serial_acr_check_cload_timeout();

      if( dswitch & 0x80 )
        {
          // SW7 is up => memory page operation
          host_set_status_led_HLDA();
          
          uint16_t page = dswitch & 0xff00;
          byte filenum  = dswitch & 0x003f;
          if( dswitch & 0x40 )
            {
              // SW6 is up => save memory page
              if( filesys_write_file('M', filenum, Mem+page, 256) )
                DBG_FILEOPS2(3, "saved memory page ", int(page>>8));
              else
                DBG_FILEOPS2(2, "unable to save memory page ", int(page>>8));
            }
          else
            {
              // SW6 is down => load memory page
              if( filesys_read_file('M', filenum, Mem+page, 256)==256 )
                {
                  DBG_FILEOPS2(3, "loaded memory page ", int(page>>8));
                  regPC = page;
                }
              else
                DBG_FILEOPS2(2, "file not found for memory page ", int(page>>8));
                
              altair_set_outputs(regPC, MREAD(regPC));
            }
            
          serial_update_hlda_led();
        }
      else
        {
          // SW7 is down => load program
          if( prog_load(dswitch & 0xff, &regPC, Mem) )
            {
              // program exists and is supposed to run immediately
              host_set_data_leds(MREAD(regPC));
              host_clr_status_led_WAIT();
            }
          else
            {
              // either program does not exist or is not supposed to run immediately
              p_regPC = ~regPC;
              altair_set_outputs(regPC, MREAD(regPC));
            }
        }
    }
  else if( cswitch & BIT(SW_AUX1UP) )
    {
      if( (cswitch & BIT(SW_STOP)) || host_read_function_switch_debounced(SW_STOP) )
        {
          // STOP is up => edit configuration
          config_edit();
          if( config_serial_panel_enabled() ) 
            { 
              Serial.print(F("\033[14B")); 
              print_panel_serial(true); 
            }
          p_regPC = ~regPC;
          altair_set_outputs(regPC, MREAD(regPC));
          rtc_setup();
        }
      else
        {
          // program shortcut
          byte p = config_aux1_program();
          if( p & 0x80 )
            {
              // run disk (first mount disk then install and run disk boot ROM)
              if( drive_mount(0, p & 0x7f) )
                {
                  dswitch = (dswitch & 0xff00) | 0x08;
                  cswitch = BIT(SW_AUX1DOWN);
                  process_inputs();
                }
            }
          else if( p>0 && prog_get_name(p)!=NULL )
            {
              // run program
              dswitch = (dswitch & 0xff00) | p;
              cswitch = BIT(SW_AUX1DOWN);
              process_inputs();
            }
        }
    }

  if( (cswitch & BIT(SW_AUX2DOWN)) && (dswitch&0xF000)==0x1000 )
    {
      if( (dswitch & 0xff)==0 )
        drive_dir();
      else if( drive_mount((dswitch >> 8) & 0x0f, dswitch & 0xff) )
        DBG_FILEOPS4(2, "mounted disk ", dswitch&0xff, " in drive ", (dswitch>>8) & 0x0f);
      else
        DBG_FILEOPS4(1, "error mounting disk ", dswitch&0xff, " in drive ", (dswitch>>8) & 0x0f);
    }
  else if( (cswitch & BIT(SW_AUX2UP)) && (dswitch&0xF000)==0x1000 )
    {
      if( drive_unmount((dswitch >> 8) & 0x0f) )
        DBG_FILEOPS2(2, "unmounted drive ", (dswitch>>8) & 0x0f);
      else
        DBG_FILEOPS2(1, "error unmounting drive ", (dswitch>>8) & 0x0f);
    }
  else if( cswitch & BIT(SW_AUX2UP) )
    {
      print_panel_serial();
      serial_capture_start(dswitch>>8, dswitch & 0xff);
    }
  else if( cswitch & BIT(SW_AUX2DOWN) )
    {
      print_panel_serial();
      serial_replay_start(dswitch>>8, dswitch & 0xff);
    }

  if( cswitch & BIT(SW_RESET) )
    {
      altair_wait_reset();
    }

  if( cswitch & BIT(SW_CLR) )
    {
      serial_close_files();
    }

  if( cswitch & BIT(SW_RUN) )
    {
      if( config_serial_debug_enabled() && config_serial_input_enabled() )
        Serial.print(F("\n\n--- RUNNING (press ESC twice to stop) ---\n\n"));
      host_clr_status_led_WAIT();
    }
  if( cswitch & (BIT(SW_PROTECT)) )
    {
      mem_protect(regPC);
      host_set_status_led_PROT();
    }
  if( cswitch & (BIT(SW_UNPROTECT)) )
    {
      mem_unprotect(regPC);
      host_clr_status_led_PROT();
    }
}


void altair_wait_step()
{
  cswitch &= BIT(SW_RESET); // clear everything but RESET status
  while( host_read_status_led_WAIT() && (cswitch & (BIT(SW_STEP) | BIT(SW_SLOW) | BIT(SW_RESET)))==0 )
    read_inputs();

  if( cswitch & BIT(SW_SLOW) ) delay(500);
}


void read_inputs_panel()
{
  // we react on positive edges on the function switches...
  cswitch = host_read_function_switches_edge();

  // ...except for the SLOW switch which is active as long as it is held down
  if( host_read_function_switch_debounced(SW_SLOW) ) cswitch |= BIT(SW_SLOW);

#if STANDALONE==0
  // address switches on Mega are connected to analog inputs which are free
  // floating and therefore random when not connected
  dswitch = host_read_addr_switches();
#endif
}


void read_inputs_serial()
{
  byte b;

  if( !config_serial_input_enabled() )
    return;

  int data = serial_read();
  if( data<0 )
    return;
#if STANDALONE>0
  else if( data >= '0' && data <= '9' )
    dswitch = dswitch ^ (1 << (data - '0'));
  else if( data >= 'a' && data <= 'f' )
    dswitch = dswitch ^ (1 << (data - 'a' + 10));
  else if( data == '/' )
    {
      Serial.print(F("\nSet Addr switches to: "));
      dswitch = numsys_read_word();
      Serial.println('\n');
    }
#endif
  else if( data == 'X' )
    cswitch |= BIT(SW_EXAMINE);
  else if( data == 'x' )
    cswitch |= BIT(SW_EXNEXT);
  else if( data == 'P' )
    cswitch |= BIT(SW_DEPOSIT);
  else if( data == 'p' )
    cswitch |= BIT(SW_DEPNEXT);
  else if( data == 'r' )
    cswitch |= BIT(SW_RUN);
  else if( data == 'o' || data == 27 )
    cswitch |= BIT(SW_STOP);
  else if( data == 't' )
    cswitch |= BIT(SW_STEP);
  else if( data == 'R' )
    cswitch |= BIT(SW_RESET);
  else if( data == '!' )
    cswitch |= BIT(SW_RESET) | BIT(SW_STOP);
  else if( data == 'U' )
    cswitch |= BIT(SW_AUX1UP);
  else if( data == 'u' )
    cswitch |= BIT(SW_AUX1DOWN);
  else if( data == 's' )
    cswitch |= BIT(SW_AUX2UP);
  else if( data == 'l' )
    {
      cswitch |= BIT(SW_AUX2DOWN);
      if( (dswitch & 0xF000)!=0x1000 )
	{
	  // if we're replaying (i.e. NOT mounting a disk) then we
	  // must run immediately because otherwise the debugging 
	  // console will consume the replayed characters
	  cswitch |= BIT(SW_AUX2DOWN) | BIT(SW_RUN);
	}
    }
  else if( data == 'Q' )
    cswitch |= BIT(SW_PROTECT);
  else if( data == 'q' )
    cswitch |= BIT(SW_UNPROTECT);
  else if( data == 'D' )
    {
      Serial.print(F("Start address: "));
      uint16_t addr = numsys_read_word();
      Serial.println();
      bool go = true;
      while( go )
        {
          numsys_print_word(addr); 
          Serial.print(F(": "));
          addr += disassemble(Mem, addr);
          Serial.println();
          while( !serial_available() ) delay(100);
          go = (serial_read() == 0x20);
        }
      p_regPC = ~regPC;
      print_dbg_info();
    }
  else if( data == 'M' )
    {
      Serial.print(F("Start address: "));
      uint16_t addr = numsys_read_word();
      Serial.println();
      bool go = true;
      while( go )
        {
          numsys_print_word(addr); 
          Serial.print(F(": "));
          numsys_print_mem(addr, 8, false);
          Serial.print(F("  "));
          numsys_print_mem(addr+8, 8, false);
          Serial.print(F("   "));
          for(int i=0; i<16; i++)
            {
              if(i==8) Serial.print(' ');
              Serial.write(MREAD(addr+i)<32 || MREAD(addr+i)>126 ? '.' : MREAD(addr+i));
            }
          Serial.println();
          while( !serial_available() ) delay(100);
          addr += 16;
          go = (serial_read() == 0x20);
        }
      p_regPC = ~regPC;
      print_dbg_info();
    }
  else if( data == 'L' )
    {
      Serial.print(F("\n\nStart address: "));
      uint16_t addr = numsys_read_word();
      Serial.print(F("\nNumber of bytes: "));
      uint16_t len  = numsys_read_word();
      Serial.print(F("\nData: "));
      while( len>0 )
        {
          Serial.write(' ');
          Mem[addr] = numsys_read_word();
          ++addr;
          --len;
        }
      Serial.write('\n');
      }
  else if( data == 'n' )
    {
      numsys_toggle();
      p_regPC = ~regPC;
      print_dbg_info();
    }
  else if( data == 'C' )
    {
      cswitch = BIT(SW_STOP) | BIT(SW_AUX1UP);
    }
#if MAX_BREAKPOINTS>0
  else if( data == 'B' || data == 'V' )
    {
      if( data == 'B' )
        {
          if( numBreakpoints<MAX_BREAKPOINTS )
            {
              Serial.print(F("\nAdd breakpoint at: "));
              breakpoint_add(numsys_read_word());
            }
          else
            Serial.print(F("\nToo many breakpoints!"));
        }
      else if( numBreakpoints>0 )
        breakpoint_remove_last();

      Serial.print(F("\nBreakpoints at: "));
      breakpoint_print();
      Serial.println('\n');
    }
#endif
}


void print_panel_serial(bool force)
{
  byte i, dbus;
  static uint16_t p_dswitch = 0, p_cswitch = 0, p_abus = 0xffff, p_dbus = 0xffff, p_status = 0xffff;
  uint16_t status, abus;

  if( !config_serial_panel_enabled() )
    return;

  status = host_read_status_leds();
  abus   = host_read_addr_leds();
  dbus   = host_read_data_leds();

  if( force || p_cswitch != cswitch || p_dswitch != dswitch || p_abus != abus || p_dbus != dbus || p_status != status )
    {
      Serial.print(F("\033[s\033[0;0HINTE PROT MEMR INP M1 OUT HLTA STACK WO INT  D7  D6  D5  D4  D3  D2  D1  D0\n"));

      if( status & ST_INTE  ) Serial.print(F(" *  "));    else Serial.print(F(" .  "));
      if( status & ST_PROT  ) Serial.print(F("  *  "));   else Serial.print(F("  .  "));
      if( status & ST_MEMR  ) Serial.print(F("  *  "));   else Serial.print(F("  .  "));
      if( status & ST_INP   ) Serial.print(F("  * "));    else Serial.print(F("  . "));
      if( status & ST_M1    ) Serial.print(F(" * "));     else Serial.print(F(" . "));
      if( status & ST_OUT   ) Serial.print(F("  * "));    else Serial.print(F("  . "));
      if( status & ST_HLTA  ) Serial.print(F("  *  "));   else Serial.print(F("  .  "));
      if( status & ST_STACK ) Serial.print(F("   *  "));  else Serial.print(F("   .  "));
      if( status & ST_WO    ) Serial.print(F(" * "));     else Serial.print(F(" . "));
      if( status & ST_INT   ) Serial.print(F("  *"));    else Serial.print(F("  ."));

      if( dbus&0x80 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x40 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x20 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x10 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x08 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x04 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x02 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      if( dbus&0x01 )   Serial.print(F("   *")); else Serial.print(F("   ."));
      Serial.print(("\nWAIT HLDA   A15 A14 A13 A12 A11 A10  A9  A8  A7  A6  A5  A4  A3  A2  A1  A0\n"));
      if( status & ST_WAIT ) Serial.print(F(" *  "));   else Serial.print(F(" .  "));
      if( status & ST_HLDA ) Serial.print(F("  *   ")); else Serial.print(F("  .   "));
      if( abus&0x8000 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x4000 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x2000 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x1000 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0800 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0400 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0200 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0100 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0080 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0040 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0020 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0010 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0008 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0004 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0002 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      if( abus&0x0001 ) Serial.print(F("   *")); else Serial.print(F("   ."));
      Serial.print(F("\n            S15 S14 S13 S12 S11 S10  S9  S8  S7  S6  S5  S4  S3  S2  S1  S0\n"));
      Serial.print(F("          "));
      if( dswitch&0x8000 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x4000 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x2000 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x1000 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0800 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0400 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0200 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0100 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0080 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0040 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0020 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0010 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0008 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0004 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0002 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      if( dswitch&0x0001 ) Serial.print(F("   ^")); else Serial.print(F("   v"));
      Serial.print(F("\n            Stop  Step  Examine  Deposit  Reset  Protect   Aux  Aux\n"));
      Serial.print(F("           "));
      if( cswitch & BIT(SW_STOP) )    Serial.print(F("  ^ "));       else if( cswitch & BIT(SW_RUN) )       Serial.print(F("  v "));      else Serial.print(F("  o "));
      if( cswitch & BIT(SW_STEP) )    Serial.print(F("    ^ "));     else if( cswitch & BIT(SW_SLOW) )      Serial.print(F("    v "));    else Serial.print(F("    o "));
      if( cswitch & BIT(SW_EXAMINE) ) Serial.print(F("     ^   "));  else if( cswitch & BIT(SW_EXNEXT) )    Serial.print(F("     v   ")); else Serial.print(F("     o   "));
      if( cswitch & BIT(SW_DEPOSIT) ) Serial.print(F("     ^   "));  else if( cswitch & BIT(SW_DEPNEXT) )   Serial.print(F("     v   ")); else Serial.print(F("     o   "));
      if( cswitch & BIT(SW_RESET) )   Serial.print(F("    ^  "));    else if( cswitch & BIT(SW_CLR) )       Serial.print(F("    v  "));   else Serial.print(F("    o  "));
      if( cswitch & BIT(SW_PROTECT) ) Serial.print(F("      ^  "));  else if( cswitch & BIT(SW_UNPROTECT) ) Serial.print(F("      v  ")); else Serial.print(F("      o  "));
      if( cswitch & BIT(SW_AUX1UP) )  Serial.print(F("     ^  "));   else if( cswitch & BIT(SW_AUX1DOWN) )  Serial.print(F("     v  "));  else Serial.print(F("     o  "));
      if( cswitch & BIT(SW_AUX2UP) )  Serial.print(F("  ^  "));      else if( cswitch & BIT(SW_AUX2DOWN) )  Serial.print(F("  v  "));     else Serial.print(F("  o  "));
      Serial.print(F("\n            Run         E.Next   D.Next    CLR   Unprotect\n\033[K\n\033[K\n\033[K\n\033[K\n\033[K\033[u"));
      p_cswitch = cswitch;
      p_dswitch = dswitch;
      p_abus = abus;
      p_dbus = dbus;
      p_status = status;
    }
}


void print_dbg_info()
{
  if( !config_serial_debug_enabled() || !host_read_status_led_WAIT() )
    return;

  if( regPC != p_regPC )
    {
      Serial.print(F("\n PC   = "));   numsys_print_word(regPC);
      Serial.print(F(" = ")); numsys_print_mem(regPC, 3, true); 
      Serial.print(F(" = ")); disassemble(Mem, regPC);
      Serial.print(F("\n SP   = ")); numsys_print_word(regSP);
      Serial.print(F(" = ")); numsys_print_mem(regSP, 8, true); 
      Serial.print(F("\n regA = ")); numsys_print_byte(regA);
      Serial.print(F(" regS = "));   numsys_print_byte(regS);
      Serial.print(F(" = "));
      if( regS & PS_SIGN )     Serial.print('S'); else Serial.print('.');
      if( regS & PS_ZERO )     Serial.print('Z'); else Serial.print('.');
      Serial.print('.');
      if( regS & PS_HALFCARRY ) Serial.print('A'); else Serial.print('.');
      Serial.print('.');
      if( regS & PS_PARITY )   Serial.print('P'); else Serial.print('.');
      Serial.print('.');
      if( regS & PS_CARRY )    Serial.print('C'); else Serial.print('.');

      Serial.print(F("\n regB = ")); numsys_print_byte(regB);
      Serial.print(F(" regC = "));   numsys_print_byte(regC);
      Serial.print(F(" regD = "));   numsys_print_byte(regD);
      Serial.print(F(" regE = "));   numsys_print_byte(regE);
      Serial.print(F(" regH = "));   numsys_print_byte(regH);
      Serial.print(F(" regL = "));   numsys_print_byte(regL);
      Serial.println();
    }
}



void reset(bool resetPC)
{
  host_clr_status_led_INT();
  host_set_status_led_WO();
  host_clr_status_led_STACK();
  host_clr_status_led_HLTA();
  host_clr_status_led_OUT();
  host_clr_status_led_M1();
  host_clr_status_led_INP();
  host_clr_status_led_MEMR();
  host_clr_status_led_INTE();
  host_clr_status_led_PROT();
  serial_update_hlda_led();

  if( resetPC )
    {
      regPC      = 0x0000;
      p_regPC    = 0xffff;
    }

  serial_reset();

#if STANDALONE>0
  if( cswitch & BIT(SW_STOP) )
#else
  if( host_read_function_switch(SW_STOP) )
#endif
    {
      // clear memory limit
      mem_clr_ram_limit();
      have_ps2 = false;

      // close all open files
      serial_close_files();

      // unmount all drives
      drive_reset();
    }

  altair_interrupts     = 0;
  altair_interrupts_buf = 0;
}


void switch_interrupt_handler()
{
  if( altair_interrupts & INT_SW_STOP )
    {
      altair_interrupts &= ~INT_SW_STOP;
      host_clr_status_led_MEMR();
      host_clr_status_led_WO();
      host_set_status_led_WAIT();
      if( !config_serial_debug_enabled() ) 
        {}
      else if( config_serial_panel_enabled() )
        Serial.print(F("\033[2J\033[14B\n------ STOP ------\n\n"));
      else
        Serial.print(F("\n\n------ STOP ------\n\n"));
      p_regPC = ~regPC;
    }
  else if( altair_interrupts & INT_SW_RESET )
    {
      altair_wait_reset();
      reset(true);
    }
  else
    {
      cswitch = 0;
      if( altair_interrupts & INT_SW_CLR )      { cswitch |= BIT(SW_CLR); altair_interrupts &= ~INT_SW_CLR; }
      if( altair_interrupts & INT_SW_AUX2UP )   { cswitch |= BIT(SW_AUX2UP); altair_interrupts &= ~INT_SW_AUX2UP; }
      if( altair_interrupts & INT_SW_AUX2DOWN ) { cswitch |= BIT(SW_AUX2DOWN); altair_interrupts &= ~INT_SW_AUX2DOWN; }
#if STANDALONE==0
      dswitch = host_read_addr_switches();
#endif
      process_inputs();
    }
}



void altair_rtc_interrupt()
{
  altair_interrupt(INT_RTC);
}


void rtc_setup()
{
  float rate = config_rtc_rate();
  altair_rtc_available = rate > 0.0;

  if( !altair_rtc_available )
    {
      timer_stop(TIMER_RTC);
      altair_rtc_running = false;
      //printf("RTC disabled\n");
    }
  else
    {
      timer_setup(TIMER_RTC, 1000000.0/rate, altair_rtc_interrupt);
      if( altair_rtc_running ) timer_start(TIMER_RTC, 0, true);
      //printf("RTC enabled: %f %i\n", rate, (int) (1000000.0/rate));
    }
}


void altair_vi_check_interrupt()
{
  byte level = 0xff;

  if     ( altair_interrupts_buf & config_interrupt_vi_mask[0] ) level = 0;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[1] ) level = 1;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[2] ) level = 2;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[3] ) level = 3;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[4] ) level = 4;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[5] ) level = 5;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[6] ) level = 6;
  else if( altair_interrupts_buf & config_interrupt_vi_mask[7] ) level = 7;

  if( level<altair_vi_level_cur )
    {
      altair_vi_level    = level;
      altair_interrupts |= INT_VECTOR;
    }
}


void altair_vi_out_control(byte data)
{
  // set current interrupt level for vector interrupts
  altair_vi_level_cur = (data & 8)==0 ? 0xff : (7-(data&7));
  
  // reset interrupt line from RTC 
  if( data & 0x10 )
    altair_interrupt(INT_RTC, false);

  // enable/disable real-time clock
  if( ((data & 0x40) && !altair_rtc_running && altair_rtc_available) )
    {
      altair_rtc_running = true;
      timer_start(TIMER_RTC, 0, true);
    }
  else if( !(data & 0x40) && altair_rtc_running )
    {
      altair_rtc_running = false;
      timer_stop(TIMER_RTC);
    }

  if( altair_interrupts_enabled )
    altair_vi_check_interrupt();
}


void altair_interrupt_enable()
{
  host_set_status_led_INTE();
  altair_interrupts_enabled = true;

  if( config_have_vi() )
    altair_vi_check_interrupt();
  else
    altair_interrupts = (altair_interrupts & ~INT_DEVICE) | (altair_interrupts_buf & config_interrupt_mask);
}


void altair_interrupt_disable()
{
  host_clr_status_led_INTE();
  altair_interrupts_enabled = false;
  altair_interrupts &= ~INT_DEVICE;
}


void altair_interrupt(uint16_t i, bool set)
{
  if( i & INT_DEVICE )
    {
      if( set )
        altair_interrupts_buf |=  (i & INT_DEVICE);
      else
        altair_interrupts_buf &= ~(i & INT_DEVICE);

      if( !altair_interrupts_enabled ) 
        altair_interrupts &= ~INT_DEVICE;
      else if( config_have_vi() )
        altair_vi_check_interrupt();
      else
        altair_interrupts = (altair_interrupts & ~INT_DEVICE) | (altair_interrupts_buf & config_interrupt_mask);
    }
  
  if( i & INT_SWITCH )
    altair_interrupts |= (i & INT_SWITCH);
}


static byte altair_interrupt_handler()
{
  byte opcode = 0xff;

  host_set_status_led_M1();
  host_set_status_led_INT();
  host_clr_status_led_MEMR();

  // Determine the opcode to put on the data bus (if VI enabled).
  // Must do this before calling altair_interrupt_disable, otherwise
  // the INT_VECTOR flag is cleared already
  if( altair_interrupts & INT_VECTOR )
    {
      opcode = 0307 | (altair_vi_level * 8);

      // the VI interrupt output line gets cleared as soon as the 
      // CPU acknowledges the interrupt
      altair_interrupts &= ~INT_VECTOR;
    }

  // disable interrupts now
  altair_interrupt_disable();

  if( host_read_status_led_WAIT() )
    {
      if( config_serial_debug_enabled() ) { Serial.print(F("\nInterrupt! opcode=")); numsys_print_byte(opcode); Serial.println(); }
      altair_set_outputs(regPC, opcode);
      altair_wait_step();
    }
  else
    host_set_data_leds(opcode);
  
  host_clr_status_led_HLTA();
  host_clr_status_led_INT();
  host_clr_status_led_M1();

  return opcode;
}


bool altair_isreset()
{
  return (cswitch & BIT(SW_RESET))==0;
}


void altair_hlt()
{
  host_set_status_led_HLTA();

  if( !host_read_status_led_WAIT() )
    {
      host_set_addr_leds(0xffff);
      host_set_data_leds(0xff);
      host_set_status_led_MEMR();
      host_set_status_led_WAIT();
      altair_interrupts = 0;
      while( (altair_interrupts & (~INT_SWITCH|INT_SW_RESET))==0 ) 
        { 
          // check host interrupts (e.g. switches on Mega)
          host_check_interrupts(); 

          // advance simulation time (so timers can expire)
          TIMER_ADD_CYCLES(2);
        }

      if( altair_interrupts & INT_SW_RESET )
        {
          cswitch = 0;
          while( !(cswitch & BIT(SW_RESET)) ) read_inputs_panel();
        }
      else if( altair_interrupts & INT_DEVICE )
        host_clr_status_led_WAIT();
    }
  else
    {
      host_set_status_led_MEMR();
      altair_set_outputs(0xffff, 0xff);

      altair_interrupts = 0;
      while( (altair_interrupts & (~INT_SWITCH|INT_SW_RESET))==0 )
        {
          read_inputs_panel();
          print_panel_serial();
          host_check_interrupts();

          // advance simulation time (so timers can expire)
          TIMER_ADD_CYCLES(1);
        }
    }
}


inline byte altair_read_sense_switches()
{
  byte data;

  // read SENSE switches
#if STANDALONE>0
  data = dswitch / 256;
#else
  static unsigned long debounceTimeout = 0;
  static byte debounceVal = 0;
  if( millis()>debounceTimeout )
    {
      data = host_read_sense_switches();
      debounceVal = data;
      debounceTimeout = millis() + 20;
    }
  else
    data = debounceVal;
#endif

  return data;
}


void altair_out(byte addr, byte data)
{
  host_set_status_led_OUT();
  host_set_status_led_WO();
  host_set_addr_leds(addr|addr*256);
  host_set_data_leds(0xff);

  switch( addr )
    {
    case 0000: serial_sio_out_ctrl(data); break;
    case 0001: serial_sio_out_data(data); break;

    case 0006: serial_acr_out_ctrl(data); break;
    case 0007: serial_acr_out_data(data); break;

    case 0010:
    case 0011:
    case 0012: drive_out(addr, data); break;

    case 0020: serial_2sio1_out_ctrl(data); break;
    case 0021: serial_2sio1_out_data(data); break;

    case 0022: serial_2sio2_out_ctrl(data); break;
    case 0023: serial_2sio2_out_data(data); break;

    case 0376: altair_vi_out_control(data); break;
    }
  
  if( host_read_status_led_WAIT() )
    {
      altair_set_outputs(addr | addr*256, 0xff);
      altair_wait_step();
    }

  host_clr_status_led_OUT();
}


byte altair_in(byte addr)
{
  byte data = 0;

  host_set_status_led_INP();
  host_set_addr_leds(addr | addr*256);

  // check the most common cases fist:
  //  - reading 2-SIO control register (i.e. waiting for serial input)
  //  - reading front panel switches
  switch( addr )
    {
    case 0000: data = serial_sio_in_ctrl(); break;
    case 0006: data = serial_acr_in_ctrl(); break;
    case 0020: data = serial_2sio1_in_ctrl(); break;
    case 0022: data = serial_2sio2_in_ctrl(); break;
    case 0377: data = altair_read_sense_switches(); break;
    case 0010:
    case 0011:
    case 0012: data = drive_in(addr); break;
    case 0001: data = serial_sio_in_data(); break;
    case 0007: data = serial_acr_in_data(); break;
    case 0021: data = serial_2sio1_in_data(); break;
    case 0023: data = serial_2sio2_in_data(); break;
    default:   data = 0xff; break;
    }

  if( host_read_status_led_WAIT() )
    {
      altair_set_outputs(addr| addr*256, data);
      altair_wait_step();
    }
  else
    host_set_data_leds(data);

  host_clr_status_led_INP();
  return data;
}


void setup() 
{
  int i;

  cswitch = 0;
  dswitch = 0;

  timer_setup();
  mem_setup();
  host_setup();
  filesys_setup();
  config_setup();
  drive_setup();
  serial_setup();
  profile_setup();
  rtc_setup();

  // if RESET switch is held up during powerup then use default configuration settings
  if( host_read_function_switch(SW_RESET) )
    {
      // temporarily reset configuration (also calls host_serial_setup)
      config_defaults(true);
      Serial.println(F("Configuration temporarily reset to defaults"));
      Serial.println(F("Raise and hold STOP and then raise AUX1 to enter configuration menu"));
      while( host_read_function_switch(SW_RESET) );
      delay(100);
      host_reset_function_switch_state();
    }
  else
    {
      // set up serial connection on the host
      host_serial_setup(0, config_host_serial_baud_rate(0), config_host_serial_primary()==0);
      host_serial_setup(1, config_host_serial_baud_rate(1), config_host_serial_primary()==1);
    }

  host_set_status_led_WAIT();

  // emulator extra: holding down CLR at powerup will keep all registers
  // and memory content initialized with 0. Otherwise (as would be normal 
  // with the Altair), everything is random.
  if( !host_read_function_switch(SW_CLR) && !config_clear_memory() )
    {
      regPC = host_get_random();
      regSP = host_get_random();
      regA  = host_get_random();
      regS  = host_get_random() & (PS_CARRY|PS_PARITY|PS_HALFCARRY|PS_ZERO|PS_SIGN);
      regB  = host_get_random();
      regC  = host_get_random();
      regD  = host_get_random();
      regE  = host_get_random();
      regH  = host_get_random();
      regL  = host_get_random();

      for(word i=0; i<MEMSIZE/4; i++)
        ((uint32_t *) Mem)[i] = host_get_random();
    }

  reset(false);

  if( config_serial_panel_enabled() ) Serial.print(F("\033[2J\033[14B\n"));
}



void loop() 
{
  byte opcode;

  // if we are NOT in WAIT mode then enter the main simulation loop
  if( !host_read_status_led_WAIT() )
    {
      // clear all switch-related interrupts before starting loop
      altair_interrupts &= ~INT_SWITCH;
      
      // enable/disable profiling
      profile_enable(config_profiling_enabled());

#if USE_THROTTLE>0
      if( config_throttle()<0 )
        {
          timer_setup(TIMER_THROTTLE, 0, update_throttle);
          timer_start(TIMER_THROTTLE, THROTTLE_TIMER_PERIOD, true);
          throttle_delay  = 10 * HOST_PERFORMANCE_FACTOR;
          throttle_micros = micros();
        }
      else
        throttle_delay = config_throttle() * HOST_PERFORMANCE_FACTOR;
#endif

      while( true )
        {
          // put PC on address bus LEDs
          host_set_addr_leds(regPC);

          // check for events that can't be handled by real interrupts
          // on the host (e.g. serial input on Mega)
          host_check_interrupts();

          if( altair_interrupts )
            {
              // interrupt detected
              if( altair_interrupts & INT_SWITCH )
                {
                  // switch-related interrupt (simulation handling)
                  switch_interrupt_handler();
                  if( host_read_status_led_WAIT() ) 
                    break;    // exit simulation loop
                  else
                    continue; // start over
                }
              else
                {
                  // ALTAIR interrupt
                  opcode = altair_interrupt_handler();
                }
            }
          else
            {
              // no interrupt => read opcode, put it on data bus LEDs and advance PC
              p_regPC = regPC;
              host_set_status_leds_READMEM_M1();
              host_set_addr_leds(regPC);
              opcode = MREAD(regPC);
              host_set_data_leds(opcode);
              regPC++;
              host_clr_status_led_M1();
            }

          // take a CPU step
          PROFILE_COUNT_OPCODE(opcode);
          CPU_EXEC(opcode);

          // check for breakpoint hit
          breakpoint_check(regPC);

#if USE_THROTTLE>0
          // delay execution according to current delay
          // (need the NOP, otherwise the compiler will optimize the loop away)
          for(uint32_t i=0; i<throttle_delay; i++) asm("NOP");
#endif
        }

#if USE_THROTTLE>0
      timer_stop(TIMER_THROTTLE);
#endif

      // flush any characters stuck in the serial buffer 
      // (so we don't accidentally execute commands after stopping)
      while( serial_available() ) serial_read();
    }

  if( cswitch & BIT(SW_RESET) )
    {
      reset(true);
      read_inputs();
    }

  print_dbg_info();
  p_regPC = regPC;

  host_set_status_led_M1();

  // only check for interrupts not related to front-panel switches (e.g. serial)
  host_check_interrupts();

  if( altair_interrupts & INT_DEVICE )
    { opcode = altair_interrupt_handler(); }
  else
    { opcode = MEM_READ(regPC); regPC++; }

  host_clr_status_led_M1();
  if( !(cswitch & BIT(SW_RESET)) ) 
    {
      PROFILE_COUNT_OPCODE(opcode);
      CPU_EXEC(opcode);
    }
}
