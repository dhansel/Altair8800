#include "Altair8800.h"
#include "config.h"
#include "cpucore.h"
#include "host.h"
#include "mem.h"
#include "profile.h"
#include "breakpoint.h"
#include "disassembler.h"
#include "numsys.h"
#include "filesys.h"
#include "drive.h"
#include "prog_ps2.h"
#include "prog_basic.h"
#include "prog_examples.h"
#include "prog_games.h"
#include "prog_tools.h"


#define DEBOUNCE_TIMEOUT 100

#define BIT(n) (1<<(n))

uint16_t      cswitch = 0, cswitch_currentstate = 0;
uint16_t      dswitch = 0;
uint16_t      debounceState;
unsigned long debounceTime[16];

uint16_t p_regPC = 0xFFFF;
byte serial_panel = false, serial_debug = false, serial_input = false; 
byte capture_device = 0, capture_fid = 0, tape_basic_fid = 0;
uint32_t tape_basic_last_op = 0;

word check_stop_ctr  = 0;
word status_wait = false;
word status_inte = false;
bool have_ps2    = false;
byte reg_2SIO_ctrl = 0, reg_2SIO_status = 0;

void print_panel_serial();
void print_dbg_info();
void read_inputs_panel();
void read_inputs_serial();
void process_inputs();
bool tape_basic_check_timeout();


#define DBG_FILEOPS_LEVEL 2

#if DBG_FILEOPS_LEVEL>0
#define DBG_FILEOPS(lvl, s) if(DBG_FILEOPS_LEVEL>=lvl) {Serial.print('['); Serial.print(F(s)); Serial.println(']');} else while(0)
#define DBG_FILEOPS2(lvl, s1, s2) if(DBG_FILEOPS_LEVEL>=lvl) { Serial.print('['); Serial.print(F(s1)); Serial.print(s2); Serial.println(']');} else while(0)
#define DBG_FILEOPS3(lvl, s1, s2, s3) if(DBG_FILEOPS_LEVEL>=lvl) { Serial.print('['); Serial.print(F(s1)); Serial.print(s2); Serial.print(s3); Serial.println(']');} else while(0)
#define DBG_FILEOPS4(lvl, s1, s2, s3, s4) if(DBG_FILEOPS_LEVEL>=lvl) { Serial.print('['); Serial.print(F(s1)); Serial.print(s2); Serial.print(s3); Serial.print(s4); Serial.println(']');} else while(0)
#else
#define DBG_FILEOPS(lvl, s) while(0)
#define DBG_FILEOPS2(lvl, s1, s2) while(0)
#define DBG_FILEOPS3(lvl, s1, s2, s3) while(0)
#define DBG_FILEOPS4(lvl, s1, s2, s3, s4) while(0)
#endif

#define CAPTURE_SIO     1
#define CAPTURE_2SIO1   2
#define CAPTURE_TAPE    3

void altair_set_outputs(uint16_t a, byte v)
{
  host_set_addr_leds(a);
  host_set_data_leds(v);
  print_panel_serial();
  if( MEM_IS_PROTECTED(a) )
    host_set_status_led_PROT();
  else
    host_clr_status_led_PROT();

  if( host_read_status_led_M1() ) print_dbg_info();
}


void update_hlda_led()
{
  if( capture_fid>0 )
    host_set_status_led_HLDA();
  else
    host_clr_status_led_HLDA();
}


void read_inputs()
{
  read_inputs_panel();
  read_inputs_serial();
  print_panel_serial();
  process_inputs();
}


void process_inputs()
{  
  if( cswitch & (BIT(SW_DEPOSIT) | BIT(SW_DEPNEXT)) )
    {
      if( cswitch & BIT(SW_DEPNEXT) ) regPC++;
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
      tape_basic_check_timeout();

      switch( dswitch & 0xff )
        {
        case 0x80:
        case 0x81:
          {
            host_set_status_led_HLDA();
            
            uint16_t page = dswitch & 0xff00;
            if( dswitch & 0x0001 )
              {
                if( filesys_write_file('M', page >> 8, Mem+page, 256) )
                  DBG_FILEOPS2(3, "saved memory page ", int(page>>8));
                else
                  DBG_FILEOPS2(2, "unable to save memory page ", int(page>>8));
              }
            else
              {
                if( filesys_read_file('M', page >> 8, Mem+page, 256)==256 )
                  {
                    DBG_FILEOPS2(3, "loaded memory page ", int(page>>8));
                    regPC = page;
                  }
                else
                  DBG_FILEOPS2(2, "file not found for memory page ", int(page>>8));
                
                altair_set_outputs(regPC, MREAD(regPC));
              }
            
            update_hlda_led();
            break;
          }

        case 0x00:
          {
            Serial.println();
            Serial.println(F("00000000) [print this directory]"));
            Serial.println(F("00000001) Calculator"));
            Serial.println(F("00000010) Kill-the-Bit"));
            Serial.println(F("00000011) Pong (LEDs)"));
            Serial.println(F("00000100) Pong (Terminal)"));
            Serial.println(F("00000101) 4k Basic (A11 up)"));
#if !defined(__AVR_ATmega2560__)
            Serial.println(F("00000110) MITS Programming System II (A15 up, A11 up, A9 up)"));
            Serial.println(F("00000111) ALTAIR Turnkey Monitor"));
#endif
            Serial.println(F("00001000) Music ('Daisy')"));
            Serial.println(F("00001001) CPU Diagnostic"));
            Serial.println(F("00001010) CPU Exerciser"));
            Serial.println(F("00001011) Status lights test"));
            Serial.println(F("00001100) Serial echo using IRQ"));
            Serial.println(F("01000000) [manage file system]"));
            Serial.println(F("10000000) [load memory page]"));
            Serial.println(F("10000001) [save memory page]"));
            break;
          }

        case 0x01:
          {
            regPC = prog_tools_copy_calc(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x02:
          {
            regPC = prog_games_copy_killbits(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x03:
          {
            regPC = prog_games_copy_pong(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x04:
          {
            regPC = prog_games_copy_pongterm(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x05:
          {
            regPC = prog_basic_copy_4k(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();

            // 4k BASIC will get into an infinite loop if a full 64k RAM are
            // available => purposely reduce the RAM size by 1 byte
            mem_set_ram_limit(0xfffe);
            break;
          }

#if !defined(__AVR_ATmega2560__)
        case 0x06:
          {
            if( capture_fid>0 && (capture_device!=CAPTURE_TAPE && capture_fid!=0xff) )
              DBG_FILEOPS(2, "cannot mount PS2 tape (other operation in progress)");
            else 
              {
                if( !have_ps2 )
                  {
                    regPC = prog_ps2_copy_monitor(Mem);
                    host_clr_status_led_WAIT();
                    host_set_data_leds(MREAD(regPC));
                    have_ps2 = true;
                  }

                // (re-) mount the PS2 tape
                prog_ps2_read_start();
                DBG_FILEOPS(3, "mounting PS2 tape");
                capture_device = CAPTURE_TAPE;
                capture_fid    = 0xff;
                update_hlda_led();
              }
            
            break;
          }

        case 0x07:
          {
            regPC = prog_tools_copy_turnmon(Mem);
            host_set_data_leds(MREAD(regPC));
            if( regPC<MEMSIZE ) 
              { host_clr_status_led_WAIT(); }
            else
              altair_set_outputs(regPC, MREAD(regPC));
            break;
          }

        case 0x0d:
          {
            regPC = prog_tools_copy_diskboot(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            //altair_set_outputs(regPC, MREAD(regPC));
            // disk boot rom starts at 0xff00 so RAM goes up to 0xfeff
            mem_set_ram_limit(0xfeff);
            break;
          }
#endif

        case 0x08:
          {
            regPC = prog_games_copy_daisy(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x09:
          {
            regPC = prog_tools_copy_diag(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x0a:
          {
            regPC = prog_tools_copy_exerciser(Mem);
            host_set_data_leds(MREAD(regPC));
            host_clr_status_led_WAIT();
            break;
          }

        case 0x0b:
          {
            regPC = prog_tools_copy_statustest(Mem);
            altair_set_outputs(regPC, MREAD(regPC));
            host_set_status_led_WAIT();
            p_regPC = ~regPC;
            break;
          }

        case 0x0c:
          {
            regPC = prog_tools_copy_serialirqtest(Mem);
            altair_set_outputs(regPC, MREAD(regPC));
            host_set_status_led_WAIT();
            p_regPC = ~regPC;
            break;
          }

        case 0x40:
          {
            filesys_manage();
            break;
          }
        }
    } 
  else if( cswitch & BIT(SW_AUX1UP) )
    {
      // ROM BASIC starts at 0xC000 so RAM goes up to 0xBFFF
      mem_set_ram_limit(0xbfff);

      regPC = 0xc000;
      p_regPC = ~regPC;
#if MEMSIZE>=0x10000
      prog_basic_copy_16k(Mem);
#endif
      host_set_data_leds(MREAD(regPC));
      host_clr_status_led_WAIT();
    }

  if( (cswitch & BIT(SW_AUX2DOWN)) && (dswitch&0x1000) )
    {
      if( (dswitch & 0xff)==0 )
        drive_dir();
      else if( drive_mount((dswitch >> 8) & 0x0f, dswitch & 0xff) )
        DBG_FILEOPS4(2, "mounted disk ", dswitch&0xff, " in drive ", (dswitch>>8) & 0x0f);
      else
        DBG_FILEOPS4(1, "error mounting disk ", dswitch&0xff, " in drive ", (dswitch>>8) & 0x0f);
    }
  else if( (cswitch & BIT(SW_AUX2UP)) && (dswitch&0x1000) )
    {
      if( drive_unmount((dswitch >> 8) & 0x0f) )
        DBG_FILEOPS2(2, "unmounted drive ", (dswitch>>8) & 0x0f);
      else
        DBG_FILEOPS2(1, "error unmounting drive ", (dswitch>>8) & 0x0f);
    }
  else if( cswitch & BIT(SW_AUX2UP) )
    {
      print_panel_serial();
      tape_basic_check_timeout();

      if( capture_fid>0 )
        {
          // end capture (0xff is special fid for BASIC example playback)
          if( capture_fid < 0xff && filesys_is_write(capture_fid) )
            {
              filesys_close(capture_fid);
              capture_fid = 0;
              DBG_FILEOPS(3, "ending capture");
            }
          else
            DBG_FILEOPS(1, "cannot start capture (replay operation in progress)");
        }
      else
        {
          capture_device = 0;
          if( (dswitch & 0xE000) == 0x8000 )
            { capture_device = CAPTURE_2SIO1; DBG_FILEOPS(3, "capturing from 88-2SIO"); }
          else if( (dswitch & 0xE000) == 0x4000 )
            { capture_device = CAPTURE_TAPE; DBG_FILEOPS(3, "capturing from TAPE"); }
          if( (dswitch & 0xE000) == 0x2000 )
            { capture_device = CAPTURE_SIO; DBG_FILEOPS(3, "capturing from 88-SIO"); }
          
          if( capture_device!=0 )
            {
              // start capture
              capture_fid = filesys_open_write('S', dswitch & 0xff);
              if( capture_fid )
                DBG_FILEOPS2(3, "starting data capture, file ", dswitch & 0xff);
              else
                DBG_FILEOPS(1, "unable to start capturing (storage full?)");
            }
          else
            DBG_FILEOPS(2, "invalid capture device");
        }
      
      update_hlda_led();
    }
  else if( cswitch & BIT(SW_AUX2DOWN) )
    {
      print_panel_serial();
      tape_basic_check_timeout();

      if( capture_fid==0xff )
        {
          if( capture_device == CAPTURE_TAPE )
            DBG_FILEOPS(3, "ejecting PS2 tape");
          else
            DBG_FILEOPS(3, "stopping BASIC example load");

          capture_fid = 0;
        }
      else if( capture_fid>0 )
        {
          if( filesys_is_read(capture_fid) )
            {
              DBG_FILEOPS(3, "stopping data replay");
              filesys_close(capture_fid);
              capture_fid = 0;
            }
          else
            DBG_FILEOPS(1, "unable to replay data (capture operation in progress)");
        }
      else if( capture_fid>0 )
        DBG_FILEOPS(1, "unable to load example (other capture/replay operation in progress)");
      else
        {
          capture_device = 0;
          if( (dswitch & 0xE000) == 0x8000 )
            { capture_device = CAPTURE_2SIO1; DBG_FILEOPS(3, "replaying to 88-2SIO"); }
          else if( (dswitch & 0xE000) == 0x4000 )
            { capture_device = CAPTURE_TAPE; DBG_FILEOPS(3, "replaying to TAPE"); }
          if( (dswitch & 0xE000) == 0x2000 )
            { capture_device = CAPTURE_SIO; DBG_FILEOPS(3, "replaying to 88-SIO"); }
          
          if( capture_device>0 )
            {
              capture_fid = filesys_open_read('S', dswitch&0xff);
              if( capture_fid>0 )
                DBG_FILEOPS2(3, "replaying captured data, file ", int(dswitch&0xff));
              else
                DBG_FILEOPS2(2, "unable to replay captured data (file not found), file ", int(dswitch&0xff));
            }
          else if( (dswitch & 0xE000)==0 )
            {
              if( prog_examples_read_start((dswitch&0xff)) )
                {
                  capture_fid    = 0xff;
                  capture_device = CAPTURE_2SIO1;
                  DBG_FILEOPS2(3, "loading example ", int(dswitch&0xff));
                }
              else
                DBG_FILEOPS2(2, "example does not exist: ", int(dswitch&0xff));
            }
          else
            DBG_FILEOPS(2, "invalid replay device");
        }

      update_hlda_led();
    }

  if( cswitch & BIT(SW_CLR) )
    {
      if( capture_fid>0 )
        {
          filesys_close(capture_fid);
          capture_fid = 0;
        }

      update_hlda_led();
    }

  if( cswitch & BIT(SW_RUN) )
    {
      if( serial_debug && serial_input )
        Serial.print(F("\n\n--- RUNNING (press ESC to stop) ---\n\n"));
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


void read_momentary_input_panel(byte inputNum)
{
  uint16_t inputBit, v;

  // read pin state, inverting result (internal pullup)
  inputBit = 1 << inputNum;
  v = host_read_function_switch(inputNum) ? inputBit : 0;

  // debounce
  if( millis()>debounceTime[inputNum] )
    {
      if( v != (debounceState & inputBit) )
        {
          debounceState = v ? (debounceState | inputBit) : (debounceState & ~inputBit);
          debounceTime[inputNum] = millis() + DEBOUNCE_TIMEOUT;
        }
    }
  else
    v = debounceState & inputBit;

  // edge trigger
  cswitch &= ~inputBit;
  if( inputBit == BIT(SW_SLOW) )
    {
      // "Slow" switch is not just a trigger
      cswitch |= v;
    }
  else if( inputBit == BIT(SW_RESET) )
    {
      // "Reset" switch triggers on high-low flank
      // and sets all outputs high as long as it is held up
      if( v )
        {
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
          //host_clr_status_led_HLDA();
          while( millis()<debounceTime[inputNum] || host_read_function_switch(inputNum) );
          debounceState &= ~inputBit;
          debounceTime[inputNum] = millis() + DEBOUNCE_TIMEOUT;
          altair_set_outputs(0x0000, 0x00);
          host_set_status_led_WAIT();
          cswitch |= BIT(SW_RESET);
        }
      else
        cswitch &= ~BIT(SW_RESET);
    }
  else if( v && !(cswitch_currentstate & v) )
    {
      cswitch |= v;
      cswitch_currentstate |= v;
    }
  else
    cswitch_currentstate = (cswitch_currentstate & ~inputBit) ^ v;
}


void read_inputs_panel()
{
  cswitch = 0;
  byte i;
  for(i=0; i<16; i++) 
    read_momentary_input_panel(i);

#if STANDALONE==0
  dswitch = host_read_addr_switches();
#endif
}


void read_inputs_serial()
{
  byte b;

  if( !serial_input )
    return;

  int data = Serial.read();
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
    cswitch |= BIT(SW_AUX2DOWN);
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
          while( !Serial.available() ) delay(100);
          go = (Serial.read() == 0x20);
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
          while( !Serial.available() ) delay(100);
          addr += 16;
          go = (Serial.read() == 0x20);
        }
      p_regPC = ~regPC;
      print_dbg_info();
    }
  else if( data == 'L' )
    {
      uint16_t addr = numsys_read_word();
      uint16_t len  = numsys_read_word();
      while( len>0 )
        {
          Mem[addr] = numsys_read_word();
          ++addr;
          --len;
        }
    }
  else if( data == 'n' )
    {
      numsys_toggle();
      p_regPC = ~regPC;
      print_dbg_info();
    }
  else if( data == 'F' )
    prof_toggle();
#if MAX_BREAKPOINTS>0
  else if( data == 'B' || data == 'C' )
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


void print_panel_serial()
{
  byte i, dbus;
  static uint16_t p_dswitch = 0, p_cswitch = 0, p_abus = 0xffff, p_dbus = 0xffff, p_status = 0xffff;
  uint16_t status, abus;

  if( !serial_panel )
    return;

  status = host_read_status_leds();
  abus   = host_read_addr_leds();
  dbus   = host_read_data_leds();

  if( p_cswitch != cswitch || p_dswitch != dswitch || p_abus != abus || p_dbus != dbus || p_status != status )
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
      Serial.print(F("\033[u"));
      p_cswitch = cswitch;
      p_dswitch = dswitch;
      p_abus = abus;
      p_dbus = dbus;
      p_status = status;
      //Serial.print("Status=");Serial.print(status); Serial.print("\n");
      //for(i=8; i<12; i++) {Serial.print(2+i); Serial.print(" "); Serial.print(status & (1<<i)); Serial.print("\n");}
    }
}


void print_dbg_info()
{
  if( !serial_debug || !host_read_status_led_WAIT() )
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
      /*Serial.print("\n FACC (0x016f) = "); numsys_print_mem(0x016F,  4, true);
      Serial.print("\n FBUF (0x0174) = "); numsys_print_mem(0x0174, 12, true);
      Serial.print("\n LINE (0x0113) = "); numsys_print_mem(0x0113, 16, true);*/
      Serial.println();
      //for(b=0; b<54; b++){ Serial.print(b); Serial.print(" "); Serial.print(digitalPinToPort(b)); Serial.print(" "); numsys_print_byte(digitalPinToBitMask(b)); Serial.println(""); }
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
  update_hlda_led();

  if( resetPC )
    {
      regPC      = 0x0000;
      p_regPC    = 0xffff;
    }

  reg_2SIO_ctrl   = 0;
  reg_2SIO_status = 0;

#if STANDALONE>0
  if( cswitch & BIT(SW_STOP) )
#else
  if( host_read_function_switch(SW_STOP) )
#endif
    {
      // erase memory
      //for(int i=0; i<MEMSIZE; i++) Mem[i] = 0;
      mem_clr_ram_limit();
      have_ps2 = false;

      // close all open files
      if( capture_fid>0 && capture_fid<0xff ) filesys_close(capture_fid);
      if( tape_basic_fid>0 ) filesys_close(tape_basic_fid);
      capture_fid = 0;
      tape_basic_fid = 0;
      update_hlda_led();

      // unmount all drives
      drive_reset();
    }

#if STANDALONE>0
  serial_debug = true;
  //serial_panel = true;
  serial_input = true;
#else
  uint16_t b = host_read_addr_switches();
  serial_debug = b & 0x80 ? true : false;
  //serial_panel = b & 0x40 ? true : false;
  serial_input = b & 0x20 ? true : false;
#endif
}


void check_stop()
{
  // only read inputs if STOP or RESET are on or ESC was sent via serial
  if( host_stop_request_check() || (serial_input && Serial.available() && (Serial.peek()==27)) )
    {
      if( serial_input && Serial.available() )
        {
          read_inputs_serial();
        }
      else if( host_stop_request_check()==128 )
        {
          // Stop request was caused by an ESC that was read in the altair_in function
          cswitch = BIT(SW_STOP);
        }
      else if( host_read_function_switch(SW_RESET) )
        {
          while( !(cswitch & BIT(SW_RESET)) )
            read_inputs_panel(); 
        }
      else
        {
          int i;
          
          for(i=0; i<16; i++)
            if( (cswitch_currentstate & BIT(i)) && (millis()>debounceTime[i]) )
              cswitch_currentstate &= ~BIT(i);
          
          read_inputs_panel();
          
          for(i=0; i<16; i++)
            if( (cswitch_currentstate & BIT(i)) )
              debounceTime[i] = millis()+500;
        }

      host_stop_request_clear();

      if( cswitch & BIT(SW_RESET) )
        {
          reset(true);
          host_clr_status_led_WAIT();
          cswitch &= ~BIT(SW_RESET);
        }
      else if( cswitch & BIT(SW_STOP) )
        {
          host_clr_status_led_MEMR();
          host_clr_status_led_WO();
          host_set_status_led_WAIT();
          if( !serial_debug ) 
            {}
          else if( serial_panel )
            Serial.print(F("\033[2J\033[14B\n------ STOP ------\n\n"));
          else
            Serial.print(F("\n\n------ STOP ------\n\n"));

          p_regPC = ~regPC;
        }
      else
        process_inputs();
    }
}


byte serial_irq_handler()
{
  byte opcode = 0xff;
  host_serial_irq_clear();
  reg_2SIO_status = 1;
  
  // interrupt => get opcode from IRQ_handler
  host_set_status_led_M1();

  host_clr_status_led_INTE();
  host_set_status_led_INT();
  host_clr_status_led_MEMR();

  if( host_read_status_led_WAIT() )
    {
      altair_set_outputs(regPC, opcode);
      if( serial_debug ) { Serial.print(F("\nInterrupt! opcode=")); numsys_print_byte(opcode); Serial.println(); }

      if( serial_input )
        {
          // unfortunately the character(s) sent over serial would just be consumed
          // by our serial_input() routine (and could possibly execute a command there)
          // so the best we can do is to just need to flush them now :(
          while( Serial.available() ) Serial.read();
        }
      
      altair_wait_step();
    }
  else
    host_set_data_leds(opcode);

  host_serial_irq_clear();
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
  if( !host_read_status_led_WAIT() )
    {
      host_set_addr_leds(0xffff);
      host_set_data_leds(0xff);
      host_set_status_led_HLTA();
      host_set_status_led_MEMR();
      host_set_status_led_WAIT();
      while( !host_read_function_switch(SW_RESET) && (Serial.peek()!=27) && !host_serial_irq_check() );

      if( host_read_function_switch(SW_RESET) )
        {
          cswitch = 0;
          while( !(cswitch & BIT(SW_RESET)) ) read_inputs_panel();
          host_set_status_led_WAIT();
        }
      else if( Serial.peek()==27 ) 
        read_inputs_serial();

      host_clr_status_led_HLTA();
      if( host_serial_irq_check() ) host_clr_status_led_WAIT();
    }
  else
    {
      host_set_status_led_HLTA();
      host_set_status_led_MEMR();
      altair_set_outputs(0xffff, 0xff);
      
      cswitch = 0;
      while( (cswitch&BIT(SW_RESET))==0 && !host_serial_irq_check() )
        {
          read_inputs_panel();
          //read_inputs_serial();
          print_panel_serial();
        }

      host_clr_status_led_HLTA();
    }

  PROF_ADD_CYCLES(7);
}


bool tape_basic_check_timeout()
{
  if( tape_basic_fid>0 && millis() > tape_basic_last_op+100 )
    {
      // if the last write or read from BASIC was more than .1 seconds ago
      // then this is a new read/write operation => close the previous file
      filesys_close(tape_basic_fid);
      tape_basic_fid = 0;
      DBG_FILEOPS(4, "closing tape file due to timeout");
      return true;
    }
  
  return false;
}  


byte tape_read_next_byte()
{
  static byte tape_fname = 0;
  byte data;
  bool go = true;

  if( capture_fid>0 && capture_device==CAPTURE_TAPE && (capture_fid==0xff || filesys_is_read(capture_fid)) )
    {
      if( capture_fid==0xff )
        {
          /* reading Programming System II data from tape (endless loop) */
          prog_ps2_read_next(&data);
          go = false;
        }
      else if( !filesys_read_char(capture_fid, &data) )
        {
          DBG_FILEOPS(4, "no more captured data");
          //filesys_close(capture_fid);
          //capture_fid = 0;
          //update_hlda_led();
          data = 0;
        }
    }
  else if( regPC==0xE2A0 )
    {
      // This is ALTAIR Extended BASIC loading from tape
      
      // check for timeout from previous operation
      tape_basic_check_timeout();
  
      // if we were writing before, close the file now
      if( tape_basic_fid>0 && !filesys_is_read(tape_basic_fid) )
        {
          filesys_close(tape_basic_fid);
          tape_basic_fid = 0;
        }

      // no file is open: either we closed it due to timeout or
      // there was a FILE NOT FOUND error earlier. In either case,
      // we need to start searching from the first file again.
      if( tape_basic_fid==0 )
        tape_fname = 0;

      while( go )
        {
          if( host_stop_request_check() )
            break;
          else if( tape_basic_fid>0 )
            {
              if( filesys_read_char(tape_basic_fid, &data) )
                go = false;
              else
                {
                  filesys_close(tape_basic_fid);
                  tape_basic_fid = 0;
                }
            }

          if( go )
            {
              while( tape_basic_fid==0 && tape_fname<96 )
                {
                  tape_basic_fid = filesys_open_read('B', 32+tape_fname);
                  if( tape_basic_fid ) DBG_FILEOPS2(4, "reading BASIC CSAVE file: ", char(32+tape_fname));
                  tape_fname++;
                }
              
              if( tape_basic_fid==0 )
                {
                  // ALTAIR BASIC would wait forever and require the user to
                  // reset the computer if a file is not found. We catch that
                  // condition here for convenience. We're setting the PC to 
                  // 0xC0A0 because it will be increased by one at the the end
                  // of the IN instruction.
                  Serial.println(F("FILE NOT FOUND"));
                  regPC = 0xC0A0;
                  break;
                }
            }
        }

      tape_basic_last_op = millis();
    }

  return data;
}


void tape_write_next_byte(byte data)
{
  static byte leadchar = 0, leadcount = 0, endcount = 0;

  // check for timeout from previous operation
  tape_basic_check_timeout();
  
  if( capture_fid>0 && capture_device==CAPTURE_TAPE && filesys_is_write(capture_fid) )
    {
      if( !filesys_write_char(capture_fid, data) )
        {
          DBG_FILEOPS(1, "capture storage exhausted");
          //filesys_close(capture_fid);
          //capture_fid  = 0;
          //update_hlda_led();
        }
    }
  else if( regPC == 0xE2AF )
    {
      // This is ALTAIR Extended BASIC saving to tape

      // if we were reading before, close the file now
      if( tape_basic_fid>0 && !filesys_is_write(tape_basic_fid) )
        {
          filesys_close(tape_basic_fid);
          tape_basic_fid = 0;
        }
  
      if( tape_basic_fid==0 )
        {
          if( leadcount==0 && data==0xd3 )
            leadcount = 1;
          else if( data == 0xd3 ) 
            leadcount++;
          else 
            {
              if( leadcount>3 )
                {
                  tape_basic_fid = filesys_open_write('B', data);
                  if( tape_basic_fid ) DBG_FILEOPS2(4, "writing BASIC CSAVE file: ", char(data));
                  for(byte i=0; i<leadcount; i++) filesys_write_char(tape_basic_fid, 0xd3);
                  filesys_write_char(tape_basic_fid, data);
                }
              leadcount = 0;
            }
        }
      else
        {
          filesys_write_char(tape_basic_fid, data);
      
          if( data == 0x00 )
            endcount++;
          else
            endcount = 0;
          
          if( endcount==10 )
            {
              filesys_close(tape_basic_fid);
              tape_basic_fid = 0;
              endcount = 0;
            }
        }

      tape_basic_last_op = millis();
    }
}


void altair_out(byte addr, byte data)
{
  host_set_status_led_OUT();
  host_set_status_led_WO();
  host_set_data_leds(0xff);

  switch( addr )
    {
    case 0000:
      {
        // write to control register of 88-SIO
        break;
      }

    case 0001:
      {
        // write data to 88-SIO
        if( capture_fid>0 && (capture_device==CAPTURE_SIO) && filesys_is_write(capture_fid) )
          {
            if( !filesys_write_char(capture_fid, data) )
              {
                DBG_FILEOPS(1, "capture storage exhausted");
                //filesys_close(capture_fid);
                //capture_fid  = 0;
                //update_hlda_led();
              }
            else
              DBG_FILEOPS2(5, "88-SIO writing captured data: ", int(data));
          }
        
        break;
      }

    case 0006:
      {
        // write to control register of tape interface
        break;
      }

    case 0007:
      {
        //printf("TAPE port data write at %04x: %02x (%c)\n", regPC, data, data>=32 ? data : '.');
        tape_write_next_byte(data);
        DBG_FILEOPS2(5, "88-SIO writing captured data: ", int(data));
        break;
      }

    case 0010:
    case 0011:
    case 0012:
      {
        drive_out(addr, data);
        break;
      }

    case 0020:
      {
        // write to control register of first serial device of 88-2SIO

        if( !(reg_2SIO_ctrl & 0x80) && (data & 0x80) )
          DBG_FILEOPS(4, "ENABLING interrupts on 2SIO");
        else if( (reg_2SIO_ctrl & 0x80) && !(data & 0x80) )
          DBG_FILEOPS(4, "disabling interrupts on 2SIO");

        reg_2SIO_ctrl   = data;
        reg_2SIO_status = 0;
        break;
      }
      
    case 0021:
      {
        // write data to first serial device of 88-2SIO
        if( regPC == 0x0380 || regPC == 0x071C )
          {
            // ALTAIR BASIC I/O routine has "OUT" instruction at 0x037F
            // MITS Programming System II has OUT instruction at 0x071B

            // only use lower 7 bits
            data &= 0x7f;
            
            // translate '_' to backspace
            if( data=='_' ) data = 127;
          }
        else if( regPC == 0x00AC )
          {
            // ALTAIR EXTENDED BASIC I/O routine has "OUT" instruction at 0x00AB

            // translate '_' to backspace
            if( data=='_' ) data = 127;
          }

        Serial.write(data);
        if( !host_read_status_led_WAIT() && capture_fid>0 && (capture_device==CAPTURE_2SIO1) && filesys_is_write(capture_fid) )
          {
            if( !filesys_write_char(capture_fid, data) )
              {
                DBG_FILEOPS(1, "capture storage exhausted");
                //filesys_close(capture_fid);
                //capture_fid  = 0;
                //update_hlda_led();
              }
            else
              DBG_FILEOPS2(5, "88-2SIO writing captured data: ", int(data));
          }
        
        break;
      }
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

  // check the most common cases fist:
  //  - reading 2-SIO control register (i.e. waiting for serial input)
  //  - reading front panel switches
  switch( addr )
    {
    case 0020:
      {
        // read control register of first serial device of 88-2SIO
        if( capture_device==CAPTURE_2SIO1 && capture_fid>0 && (capture_fid==0xff || filesys_is_read(capture_fid)) )
          {
            if( capture_fid==0xff )
              {
                // we're playing back a basic example, those are always 0-terminated
                // and capture_fid==0xff would not hold if we had
                // reached the end
                data |= 0x01;
              }
            else
              {
                if( !filesys_eof(capture_fid) )
                  data |= 0x01;
              }
          }
        else if( Serial.available() )
          data |= 0x01;

        if( capture_device==CAPTURE_2SIO1 && capture_fid>0 && capture_fid<0xff && filesys_is_write(capture_fid) )
          {
            if( !filesys_eof(capture_fid) )
              data |= 0x02;
          }
        else if( Serial.availableForWrite() )
          data |= 0x02;

        break;
      }

    case 0377:
      {
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
        break;
      }

    case 0000:
      {
        // read control register of 88-SIO
        if( capture_device==CAPTURE_SIO && capture_fid>0 && capture_fid<0xff )
          {
            if( !filesys_is_write(capture_fid) || filesys_eof(capture_fid) )
              data |= 0x80;

            if( !filesys_is_read(capture_fid) || filesys_eof(capture_fid) )
              data |= 0x01;
          }
        else
          data |= 0x81;

        break;
      }

    case 0001:
      {
        // read data from 88-SIO
        if( capture_device==CAPTURE_SIO && capture_fid>0 && capture_fid<0xff && filesys_is_read(capture_fid) )
          {
            if( !filesys_read_char(capture_fid, &data) )
              {
                DBG_FILEOPS(4, "no more data for replay");
                data = 0;
              }
            else
              DBG_FILEOPS2(5, "88-SIO reading captured data: ", int(data));
          }
        
        break;
      }

    case 0006:
      {
        if( capture_device==CAPTURE_TAPE && capture_fid>0 )
          {
            if( capture_fid == 0xff )
              {
                // This is us loading the Programming System II tape
                // => we can always read (continuous loop) but not write
                data |= 0x80;
              }
            else
              {
                if( !filesys_is_write(capture_fid) || filesys_eof(capture_fid) )
                  data |= 0x80;
                
                if( !filesys_is_read(capture_fid) || filesys_eof(capture_fid) )
                  data |= 0x01;
              }
          }
        else if( regPC==0xE299 || regPC==0xE2A7 )
          {
            // This is ALTAIR Extended BASIC loading from or saving to tape
            // our BASIC CSAVE/CLOAD tape emulation is a continuous loop so we always
            // have data available (i.e. bit 0 NOT set) 
            data = 0x00;
          }
        else
          data = 0x81;
        
        break;
      }
      
    case 0007:
      {
        data = tape_read_next_byte();
        DBG_FILEOPS2(5, "tape reading captured data: ", int(data));
        break;
      }

    case 0010:
    case 0011:
    case 0012:
      {
        data = drive_in(addr);
        break;
      }

    case 0021:
      {
        // read data from first serial device of 88-2SIO
        if( capture_device==CAPTURE_2SIO1 && capture_fid>0 && (capture_fid==0xff || filesys_is_read(capture_fid)) )
          {
            if( capture_fid==0xff )
              {
                // special fid for BASIC example replay
                if( !prog_examples_read_next(&data) )
                  {
                    capture_fid = 0;
                    DBG_FILEOPS(4, "done loading");
                    update_hlda_led();
                  }
              }
            else
              {
                if( !filesys_read_char(capture_fid, &data) )
                  {
                    DBG_FILEOPS(4, "no more data for replay");
                    data = 0;
                  }
                else
                  DBG_FILEOPS2(5, "88-2SIO reading captured data: ", int(data));
              }
          }
        else
          {
            if( Serial.available() ) 
              {
                data = Serial.read();
                if( data==27 && serial_input ) host_stop_request_set(128);
              }

            if( regPC == 0x038A || regPC == 0x0726 )
              {
                // ALTAIR BASIC I/O routine has "IN" instruction at 0x0389
                // MITS Programming System II has "IN" instruction at 0x0725

                // only use upper-case letters
                if( data>96 && data<123 ) data -= 32;
                
                // translate backspace to underscore
                if( data==8 || data==127 ) data = '_';
              }
            else if( regPC == 0x00A0 )
              {
                // ALTAIR EXTENDED BASIC I/O routine has "IN" instruction at 0x009F

                // translate backspace to underscore
                if( data==8 || data==127 ) data = '_';
              }
          }
        fflush(stdout);
        
        reg_2SIO_status = 0;
        break;
      }
    }

  if( host_read_status_led_WAIT() )
    {
      altair_set_outputs(addr| addr*256, data);
      altair_wait_step();
    }

  host_clr_status_led_INP();
  return data;
}

     
void setup() 
{
  int i;

#if STANDALONE>0
  Serial.begin(115200);
#else
  Serial.begin(9600);
#endif
  Serial.setTimeout(10000);

  cswitch = 0;
  debounceState = 0;
  for(i=0; i<16; i++) debounceTime[i] = 0;

  mem_setup();
  host_setup();
  filesys_setup();
  drive_setup();

  host_set_status_led_WAIT();

  // emulator extra: holding down CLR at powerup will keep all registers
  // and memory content initialized with 0. Otherwise (as would be normal 
  // with the Altair), everything is random.
  if( !host_read_function_switch(SW_CLR) )
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

      for(uint32_t i=0; i<MEMSIZE/4; i++)
        ((uint32_t *) Mem)[i] = host_get_random();
    }

  reset(false);

  if( serial_panel ) Serial.print(F("\033[2J\033[14B\n"));
}


void loop() 
{
  byte opcode;

  check_stop_ctr = 10000;
  while( !host_read_status_led_WAIT() )
    {
      // put PC on address bus LEDs
      host_set_addr_leds(regPC);

      if( host_serial_irq_check() )
        opcode = serial_irq_handler();
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
      PROF_COUNT_OPCODE(opcode);
      CPU_EXEC(opcode);

      // check for breakpoint hit
      breakpoint_check(regPC);

      // profile
      prof_check();
      
      // check for stop request
      if( --check_stop_ctr==0 )
        {
          check_stop();
          check_stop_ctr = 10000;
        }
    }

  if( cswitch & BIT(SW_RESET) )
    {
      reset(true);
      read_inputs();
    }

  print_dbg_info();
  p_regPC = regPC;

  host_set_status_led_M1();

  if( host_serial_irq_check() )
    opcode = serial_irq_handler();
  else
    { opcode = MEM_READ(regPC); regPC++; }

  host_clr_status_led_M1();
  if( !(cswitch & BIT(SW_RESET)) ) 
    {
      PROF_COUNT_OPCODE(opcode);
      CPU_EXEC(opcode);
    }
}
