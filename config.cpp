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
#include "mem.h"
#include "serial.h"
#include "filesys.h"
#include "numsys.h"
#include "drive.h"
#include "prog.h"

#define  BAUD_110    0
#define  BAUD_150    1
#define  BAUD_300    2
#define  BAUD_600    3
#define  BAUD_1200   4
#define  BAUD_2400   5
#define  BAUD_4800   6
#define  BAUD_9600   7
#define  BAUD_19200  8
#define  BAUD_38400  9
#define  BAUD_57600  10
#define  BAUD_115200 11

// config_flags:
// xxxxxxxx xxxxxxxt ttttRRRR xVCDIPFT
// T = Throttle
// t = Throttle delay if throttle is enabled (0=auto)
// F = Profile
// P = Serial Panel
// I = Serial Input
// D = Serial Debug
// C = Clear memory on powerup
// V = VI board installed
// R = RTC rate
uint32_t config_flags;


// cofig_serial_settings:
// xxxxxxxx xxxxxxxx xxxxxxxI bbbbBBBB
// BBBB = baud rate for Serial  host interface (see baud rates above)
// bbbb = baud rate for Serial1 host interface (see baud rates above)
// I    = select primary serial interface (0=Serial, 1=Serial1)
// x    = unused
uint32_t config_serial_settings, new_config_serial_settings;


// config_serial_device_settings[0-3]
// xxxxxxxx xxxxxxxx TT77UUMM CNNNBBBB
// BBBB = baud rate for serial playback (see baud rates above)
// NNN  = NULs to send after a carriage return when playing back examples
// C    = trap CLOAD/CSAVE in extended BASIC (for CSM_ACR device only)
// MM   = map device to host interface (00=NONE, 01=primary, 02=secondary)
// UU   = only uppercase for inputs (00=off, 01=on, 10=autodetect)
// 77   = use 7 bit for serial outputs (00=off [use 8 bit], 01=on, 10=autodetect)
// TT   = translate backspace to (00=off, 01=underscore, 10=autodetect, 11=rubout)
uint32_t config_serial_device_settings[4];


// masks defining which interrupts (INT_*) are at which vector interrupt levels
byte config_interrupt_vi_mask[8];


// mask defining whch interrupts (INT_*) are connected if VI board is not installed
byte config_interrupt_mask;


// program to be run when AUX1 is raised
byte config_aux1_prog;


// --------------------------------------------------------------------------------


inline uint32_t get_bits(uint32_t v, byte i, byte n)
{
  return (v >> i) & ((1<<n)-1);
}


inline uint32_t set_bits(uint32_t v, byte i, byte n, uint32_t nv)
{
  uint32_t mask = ((1<<n)-1) << i;
  return (v & ~mask) | ((nv << i) & mask);
}


static uint32_t toggle_bits(uint32_t v, byte i, byte n, byte min = 0x00, byte max = 0xff)
{
  byte b = get_bits(v, i, n) + 1;
  return set_bits(v, i, n, b>max ? min : (b<min ? min : b));
}


static uint32_t config_baud_rate(byte b)
{
  uint32_t res;
  switch(b)
    {
    case BAUD_110   : res = 110;    break;
    case BAUD_150   : res = 150;    break;
    case BAUD_300   : res = 300;    break;
    case BAUD_600   : res = 600;    break;
    case BAUD_1200  : res = 1200;   break;
    case BAUD_2400  : res = 2400;   break;
    case BAUD_4800  : res = 4800;   break;
    case BAUD_9600  : res = 9600;   break;
    case BAUD_19200 : res = 19200;  break;
    case BAUD_38400 : res = 38400;  break;
    case BAUD_57600 : res = 57600;  break;
    case BAUD_115200: 
    default         : res = 115200; break;
    }

  return res;
}


float config_rtc_rate()
{
  float res = 0.0;
  
  byte value = get_bits(config_flags, 8, 4);
  if( value & 0x08 )
    {
      switch( value & 0x07 )
        {
        case 0: res =     0.06; break;
        case 1: res =     0.60; break;
        case 2: res =     6.00; break;
        case 3: res =    10.00; break;
        case 4: res =    60.00; break;
        case 5: res =   100.00; break;
        case 6: res =  1000.00; break;
        case 7: res = 10000.00; break;
        }
    }

  return res;
}


#if USE_THROTTLE>0
int config_throttle()
{
  if( config_flags & CF_THROTTLE )
    {
      int i = get_bits(config_flags, 12, 5);
      if( i==0 )
        return -1; // auto
      else
        return i;  // manual
    }
  else
    return 0; // off
}
#endif

byte config_aux1_program()
{
  return config_aux1_prog;
}

byte config_serial_backspace(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 14, 2);
}

byte config_serial_7bit(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 12, 2);
}

byte config_serial_ucase(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 10, 2);
}

bool config_serial_trap_CLOAD()
{
  return get_bits(config_serial_device_settings[CSM_ACR], 7, 1);
}

uint32_t config_serial_playback_baud_rate(byte dev)
{
  return config_baud_rate(get_bits(config_serial_device_settings[dev], 0, 4));
}

byte config_serial_playback_example_nuls(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 4, 3);
}

byte config_serial_map_sim_to_host(byte dev)
{
  switch( get_bits(config_serial_device_settings[dev], 8, 2) )
    {
    case 1: return   config_host_serial_primary();
    case 2: return 1-config_host_serial_primary();
    }

  return 0xff;
}

byte config_host_serial_primary()
{
  return get_bits(config_serial_settings, 8, 1);
}

uint32_t config_host_serial_baud_rate(byte iface)
{
  return config_baud_rate(get_bits(config_serial_settings, iface==0 ? 0 : 4, 4));
}


uint32_t config_host_serial_baud_rate(uint32_t settings, byte iface)
{
  return config_baud_rate(get_bits(settings, iface==0 ? 0 : 4, 4));
}


// --------------------------------------------------------------------------------


static void set_cursor(byte row, byte col)
{
  Serial.print(F("\033["));
  Serial.print(row);
  Serial.print(';');
  Serial.print(col);
  Serial.print(F("H\033[K"));
}


static void print_flag(int value, byte row=0, byte col=0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);
  Serial.print((config_flags&value)!=0 ? F("yes") : F("no"));
}


static void print_vi_flag()
{
  Serial.print((config_flags&CF_HAVE_VI)!=0 ? F("Use Vector Interrupt board") : F("Interrupts connected directly to CPU"));
}


static void print_host_serial_baud_rate(byte iface, byte row=0, byte col=0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);
  Serial.print(config_host_serial_baud_rate(new_config_serial_settings, iface)); 

  if( config_host_serial_baud_rate(new_config_serial_settings, iface) != config_host_serial_baud_rate(iface) )
    {
      Serial.print(F(" (current: ")); 
      Serial.print(config_host_serial_baud_rate(iface));
      Serial.print(')');
    }
}


static void print_throttle(byte row = 0, byte col = 0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);

  int i = config_throttle();
  if     ( i<0  ) Serial.print(F("auto adjust"));
  else if( i==0 ) Serial.print(F("off"));
  else            Serial.print(i);
}


static void print_serial_device_sim(byte dev)
{
  switch( dev )
    {
    case CSM_SIO:   Serial.print(F("SIO")); break;
    case CSM_ACR:   Serial.print(F("ACR")); break;
    case CSM_2SIO1: Serial.print(F("2-SIO port 1")); break;
    case CSM_2SIO2: Serial.print(F("2-SIO port 2")); break;
    }
}


static void print_host_primary_interface_aux(byte iface)
{
  switch( iface )
    {
    case 0: Serial.print(F("Serial (USB, pin 0/1)")); break;
    case 1: Serial.print(F("Serial1 (pin 18/19)")); break;
    }
}


static void print_host_primary_interface(byte row = 0, byte col = 0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);
  print_host_primary_interface_aux(get_bits(new_config_serial_settings, 8, 1));

  if( get_bits(new_config_serial_settings, 8, 1) != config_host_serial_primary() )
    {
      Serial.print(F(" (current: ")); 
      print_host_primary_interface_aux(config_host_serial_primary());
      Serial.print(')');
    }
}


static void print_serial_flag(uint32_t settings, byte pos, byte bits = 2)
{
  switch( get_bits(settings, pos, bits) )
    {
    case CSF_OFF:  Serial.print(F("off"));        break;
    case CSF_ON:   Serial.print(F("on"));         break;
    case CSF_AUTO: Serial.print(F("autodetect")); break;
    }
}


static void print_serial_flag_backspace(uint32_t settings)
{
  switch( get_bits(settings, 14, 2) )
    {
    case CSFB_NONE:       Serial.print(F("off"));            break;
    case CSFB_UNDERSCORE: Serial.print(F("underscore (_)")); break;
    case CSFB_RUBOUT:     Serial.print(F("rubout (127)"));   break;
    case CSFB_AUTO:       Serial.print(F("autodetect"));     break;
    }
}


static void print_serial_device_mapped_to(uint32_t settings)
{
  switch( get_bits(settings, 8, 2) )
    {
#if defined(__SAM3X8E__)
    case 0: Serial.print("Not mapped"); break;
    case 1: Serial.print("Primary serial host interface"); break;
    case 2: Serial.print("Secondary serial host interface"); break;
#else
    case 0: Serial.print(F("Not mapped")); break;
    case 1: Serial.print(F("Serial")); break;
#endif
    }
}


static void print_rtc_frequency()
{
  float rate = config_rtc_rate();
  if( rate==0.0 )
    Serial.print(F("Disabled"));
  else
    { Serial.print(rate); Serial.print(F(" Hz")); }
}


static void print_interrupt_conn(byte mask, byte b)
{
  if( config_flags&CF_HAVE_VI )
    {
      if( b == 0xff )
        Serial.print(F("Not connected"));
      else
        { Serial.print(F("VI")); Serial.print(b); }
    }
  else
    {
      if( config_interrupt_mask & mask )
        Serial.print(F("Connected"));
      else
        Serial.print(F("Not connected"));
    }
}


static void print_aux1_program(byte row=0, byte col=0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);

  byte b = config_aux1_program();
  if( b & 0x80 )
    {
      const char *disk = drive_disk_description(b&0x7f);
      if( disk==NULL )
        Serial.print(F("none"));
      else
        Serial.print(disk);
    }
  else if( b > 0 && prog_get_name(b)!=NULL )
    Serial.print(FP(prog_get_name(b)));
  else
    Serial.print(F("none"));
}


// --------------------------------------------------------------------------------


static bool apply_host_serial_settings(uint32_t settings)
{
  config_serial_settings = settings;
  host_serial_setup(0, config_host_serial_baud_rate(0), config_host_serial_primary()==0);
  host_serial_setup(1, config_host_serial_baud_rate(1), config_host_serial_primary()==1);
}


static void toggle_host_serial_baud_rate(byte iface, byte row, byte col)
{
  byte min_rate;

#ifdef HOST_PC_H
  new_config_serial_settings = toggle_bits(new_config_serial_settings, iface==0 ? 0 : 4, 4, BAUD_110, BAUD_115200);
  apply_host_serial_settings(new_config_serial_settings);
#else
  new_config_serial_settings = toggle_bits(new_config_serial_settings, iface==0 ? 0 : 4, 4, BAUD_1200, BAUD_115200);
#endif

  print_host_serial_baud_rate(iface, row, col);
}


static void toggle_host_primary_interface(byte row, byte col)
{
  new_config_serial_settings = toggle_bits(new_config_serial_settings, 8, 1);
  print_host_primary_interface(row, col);
}


static void toggle_rtc_rate()
{
  byte value = get_bits(config_flags, 8, 4);
  if( (value & 0x08)==0 )
    value = 0x08;
  else if( value==0x0f )
    value = 0x00;
  else
    value++;

  config_flags = set_bits(config_flags, 8, 4, value);
}


static void toggle_throttle(byte row, byte col)
{
  int i = config_throttle();
  if     ( i<0  ) i =  1;
  else if( i==0 ) i = -1;
  else if( i>30 ) i = 0;
  else            i = i + 1;

  config_flags = set_bits(config_flags,  0, 1, i!=0);
  config_flags = set_bits(config_flags, 12, 5, i<0 ? 0 : i);

  set_cursor(row, col);
  print_throttle();
}


static byte toggle_interrupt_conn(byte mask, byte c)
{
  if( config_flags&CF_HAVE_VI )
    {
      if( c == 0xff )
        c = 0;
      else if( c>=7 )
        c = 0xff;
      else
        c++;
    }
  else
    config_interrupt_mask = (config_interrupt_mask & ~mask) | (~config_interrupt_mask & mask);

  return c;
}


static uint32_t toggle_serial_flag(uint32_t settings, byte pos, bool haveAuto = true)
{
  byte b = get_bits(settings, pos, 2);
  switch( b )
    {
    case CSF_OFF:  b = CSF_ON;   break;
    case CSF_ON:   b = haveAuto ? CSF_AUTO : CSF_OFF; break;
    case CSF_AUTO: b = CSF_OFF;  break;
    default:       b = haveAuto ? CSF_AUTO : CSF_OFF; break;
    }

  return set_bits(settings, pos, 2, b);
}


static uint32_t toggle_serial_flag_backspace(uint32_t settings)
{
  byte b = get_bits(settings, 14, 2);
  switch( b )
    {
    case CSFB_NONE:       b = CSFB_UNDERSCORE; break;
    case CSFB_UNDERSCORE: b = CSFB_RUBOUT; break;
    case CSFB_RUBOUT:     b = CSFB_AUTO; break;
    case CSFB_AUTO:       b = CSFB_NONE; break;
    }
  
  return set_bits(settings, 14, 2, b);
}


static byte find_disk(byte n)
{
  byte i=n;
  do
    {
      if( drive_disk_filename(i)!=NULL ) return i;
      i++;
    }
  while( i>0 );

  return 0;
}


static void toggle_aux1_program(byte row, byte col)
{
  byte b = config_aux1_prog;
  
  if( b & 0x80 )
    {
      b = find_disk((b & 0x7f)+1);
      if( b>0 ) b |= 0x80;
    }
  else if( prog_get_name(b+1)==NULL  )
    {
      b = find_disk(1);
      if( b>0 ) b |= 0x80;
    }
  else
    b = b + 1;

  config_aux1_prog = b;
  print_aux1_program(row, col);
}


static void toggle_flag(uint32_t value, byte row, byte col)
{
  config_flags = (config_flags & ~value) | (~(config_flags & value) & value);
  print_flag(value, row, col);
}


static bool apply_host_serial_settings()
{
  char c;
  uint32_t old_config_serial_settings = config_serial_settings;
  apply_host_serial_settings(new_config_serial_settings);

  Serial.println(F("\n[will revert to old settings if no answer within 20 seconds]"));
  Serial.print(F("Keep new host interface settings (y/n)? "));
  uint32_t timeout = millis() + 20000;
  c = 0;
  do { delay(50); c = serial_read(); } while( c!='y' && c!='n' && millis()<timeout );
  Serial.println(c);
  if( c!='y' ) 
    { 
      apply_host_serial_settings(old_config_serial_settings);
      if( c!='n' ) 
        {
          delay(100);
          Serial.println(F("\nSettings were not confirmed withing 20 seconds."));
          delay(2000);

          // flush serial input that may have accumulated while waiting
          while( serial_read()>=0 );
        }
      return false;
    }

  return true;
}


// --------------------------------------------------------------------------------


static bool save_config(byte fileno)
{
  // better to write all data at once (will overwrite instead
  // of deleting/creating the file)
  byte s = sizeof(uint32_t);
  byte data[6*s+10];
  memcpy(data+0*s,   &config_flags, s);
  memcpy(data+1*s,   &new_config_serial_settings, s);
  memcpy(data+2*s,    config_serial_device_settings, 4*s);
  memcpy(data+6*s,    config_interrupt_vi_mask, 8);
  memcpy(data+6*s+8, &config_interrupt_mask, 1);
  memcpy(data+6*s+9, &config_aux1_prog, 1);
  return filesys_write_file('C', fileno, (void *) data, 6*s+10);
}


static bool load_config(byte fileno)
{
  bool ok = false;
  byte fid = filesys_open_read('C', fileno);
  if( fid )
    {
      // initialize all settings with defaults so configuration settings
      // missing at the end of the file will just be at their defaults
      // (helpful when reading an old config file after adding new settings)
      config_defaults(false);

      byte s = sizeof(uint32_t);
      filesys_read_data(fid, &config_flags, s);
      filesys_read_data(fid, &new_config_serial_settings, s);
      filesys_read_data(fid, config_serial_device_settings, 4*s);
      filesys_read_data(fid, config_interrupt_vi_mask, 8);
      filesys_read_data(fid, &config_interrupt_mask, 1);
      filesys_read_data(fid, &config_aux1_prog, 1);
      
      filesys_close(fid);
#if STANDALONE>0
      config_flags |= CF_SERIAL_INPUT;      
#endif
#ifdef HOST_PC_H
      apply_host_serial_settings(new_config_serial_settings);
#endif
      ok = true;
    }

  return ok;
}


// --------------------------------------------------------------------------------


byte find_vi_conn(byte interrupt)
{
  for(int i=0; i<8; i++)
    if( config_interrupt_vi_mask[i] & interrupt )
      return i;

  return 0xff;
}


void config_edit_interrupts()
{
  byte conn_sio   = find_vi_conn(INT_SIO);
  byte conn_acr   = find_vi_conn(INT_ACR);
  byte conn_2sio1 = find_vi_conn(INT_2SIO1);
  byte conn_2sio2 = find_vi_conn(INT_2SIO2);
  byte conn_rtc   = find_vi_conn(INT_RTC);
  byte conn_drive = find_vi_conn(INT_DRIVE);

  bool go = true;
  while( go )
    {
      Serial.print(F("\033[2J\033[0;0H\n"));
      Serial.println(F("Configure interrupt settings"));
      Serial.print(F("\n(R)eal-Time Clock             : ")); print_rtc_frequency(); Serial.println();
      Serial.print(F("(V)ector Interrupt board      : ")); print_vi_flag(); Serial.println();

      Serial.print(F("\n(1) Disk drive interrupt      : ")); print_interrupt_conn(INT_DRIVE, conn_drive); Serial.println();
      Serial.print(F("(2) Real-Time Clock interrupt : ")); print_interrupt_conn(INT_RTC, conn_rtc); Serial.println();
      Serial.print(F("(3) 88-2SIO port 1 interrupt  : ")); print_interrupt_conn(INT_2SIO1, conn_2sio1); Serial.println();
      Serial.print(F("(4) 88-2SIO port 2 interrupt  : ")); print_interrupt_conn(INT_2SIO2, conn_2sio2); Serial.println();
      Serial.print(F("(5) 88-SIO interrupt          : ")); print_interrupt_conn(INT_SIO, conn_sio); Serial.println();
      Serial.print(F("(6) 88-ACR interrupt          : ")); print_interrupt_conn(INT_ACR, conn_acr); Serial.println();

      Serial.println(F("\nE(x)it to main menu"));

      Serial.print(F("\n\nCommand: "));
      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'R':
        case 'r':
        case 'C': 
        case 'c': toggle_rtc_rate(); break;
        case 'V':
        case 'v': toggle_flag(CF_HAVE_VI, 0, 0); break;

        case '1': conn_drive = toggle_interrupt_conn(INT_DRIVE, conn_drive); break;
        case '2': conn_rtc   = toggle_interrupt_conn(INT_RTC, conn_rtc);   break;
        case '3': conn_2sio1 = toggle_interrupt_conn(INT_2SIO1, conn_2sio1); break;
        case '4': conn_2sio2 = toggle_interrupt_conn(INT_2SIO2, conn_2sio2); break;
        case '5': conn_sio   = toggle_interrupt_conn(INT_SIO, conn_sio);   break;
        case '6': conn_acr   = toggle_interrupt_conn(INT_ACR, conn_acr);   break;

        case 27:
        case 'x': go = false; break;
        }
    }

  for(int i=0; i<8; i++)  config_interrupt_vi_mask[i] = 0;
  if( conn_sio   < 0xff ) config_interrupt_vi_mask[conn_sio]   |= INT_SIO;
  if( conn_acr   < 0xff ) config_interrupt_vi_mask[conn_acr]   |= INT_ACR;
  if( conn_2sio1 < 0xff ) config_interrupt_vi_mask[conn_2sio1] |= INT_2SIO1;
  if( conn_2sio2 < 0xff ) config_interrupt_vi_mask[conn_2sio1] |= INT_2SIO2;
  if( conn_rtc   < 0xff ) config_interrupt_vi_mask[conn_rtc]   |= INT_RTC;
  if( conn_drive < 0xff ) config_interrupt_vi_mask[conn_drive] |= INT_DRIVE;
}


// --------------------------------------------------------------------------------


void config_edit_serial_device(byte dev)
{
  uint32_t settings = config_serial_device_settings[dev];

  while( true )
    {
      Serial.print(F("\033[2J\033[0;0H\n"));
      Serial.print(F("Configure serial device ")); print_serial_device_sim(dev); Serial.println();
      Serial.print(F("\nMap to host (i)nterface    : ")); print_serial_device_mapped_to(settings); Serial.println();
      Serial.print(F("Simulated (b)aud rate      : ")); Serial.println(config_baud_rate(get_bits(settings, 0, 4)));
      Serial.print(F("Example playback (N)ULs    : ")); Serial.println(get_bits(settings, 4, 3));
      Serial.print(F("Use (7) bits               : ")); print_serial_flag(settings, 12); Serial.println();
      Serial.print(F("Serial input (u)ppercase   : ")); print_serial_flag(settings, 10); Serial.println();
      Serial.print(F("Translate (B)ackspace to   : ")); print_serial_flag_backspace(settings); Serial.println();
      if( dev==CSM_ACR )
        { Serial.print(F("Enable CLOAD/CSAVE (t)raps : ")); print_serial_flag(settings, 7, 1); Serial.println(); }

      Serial.println(F("\nE(x)it to main menu"));

      Serial.print(F("\n\nCommand: "));
      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
#if defined(__SAM3X8E__)
        case 'i': settings = toggle_bits(settings, 8, 2, 0, 2); break;
#else
        case 'i': settings = toggle_bits(settings, 8, 2, 0, 1); break;
#endif
        case 'b': settings = toggle_bits(settings, 0, 4, BAUD_110, BAUD_19200); break;
        case 'u': settings = toggle_serial_flag(settings, 10); break;
        case 't': settings = toggle_bits(settings, 7, 1); break;
        case '7': settings = toggle_serial_flag(settings, 12); break;
        case 'B': settings = toggle_serial_flag_backspace(settings); break;
        case 'N': settings = toggle_bits(settings, 4, 3); break;

        case 27:
        case 'x': 
          config_serial_device_settings[dev] = settings;
          serial_timer_interrupt_setup(dev);
          return;
        }
    }
}


// --------------------------------------------------------------------------------


void config_edit()
{
  new_config_serial_settings = config_serial_settings;

  // flush serial input
  while( serial_read()>=0 );
  
  bool redraw = true;

  byte row, col, r_profile, r_throttle, r_panel, r_input, r_debug, r_clearmem, r_aux1, r_interrupt, r_baud0, r_baud1, r_primary, r_cmd;
  while( true )
    {
      char c;

      if( redraw )
        {
          row = 2;
          col = 31;
          Serial.print(F("\033[2J\033[0;0H\n"));
      
          Serial.print(F("Enable pro(f)iling          : ")); print_flag(CF_PROFILE); Serial.println(); r_profile = row++;
#if USE_THROTTLE>0
          Serial.print(F("Set (t)hrottle delay        : ")); print_throttle(); Serial.println(); r_throttle = row++;
#endif
          Serial.print(F("Enable serial (p)anel       : ")); print_flag(CF_SERIAL_PANEL); Serial.println(); r_panel = row++;
#if STANDALONE==0
          Serial.print(F("Enable serial (i)nput       : ")); print_flag(CF_SERIAL_INPUT); Serial.println(); r_input = row++;
#endif
          Serial.print(F("Enable serial (d)ebug       : ")); print_flag(CF_SERIAL_DEBUG); Serial.println(); r_debug = row++;
          Serial.print(F("Clear (m)emory on powerup   : ")); print_flag(CF_CLEARMEM); Serial.println(); r_clearmem = row++;
          Serial.print(F("A(u)x1 shortcut program     : ")); print_aux1_program(); Serial.println(); r_aux1 = row++;
          Serial.print(F("Configure (I)nterrupts      : ")); print_vi_flag(); Serial.println(); r_interrupt = row++;
          Serial.print(F("Host Serial (b)aud rate     : ")); print_host_serial_baud_rate(0); Serial.println(); r_baud0 = row++;
#if defined(__SAM3X8E__)
          Serial.print(F("Host Serial1 baud (r)ate    : ")); print_host_serial_baud_rate(1); Serial.println(); r_baud1 = row++;
          Serial.print(F("(P)rimary host serial       : ")); print_host_primary_interface(); Serial.println(); r_primary = row++;
#endif
          Serial.println();
          Serial.print(F("(1) Configure SIO           : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_SIO]); Serial.println();
          Serial.print(F("(2) Configure ACR           : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_ACR]); Serial.println();
          Serial.print(F("(3) Configure 2SIO port 1   : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO1]); Serial.println();
          Serial.print(F("(4) Configure 2SIO port 2   : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO2]); Serial.println();
          row += 5;

          Serial.println();
          Serial.println(F("(M)anage Filesystem"));
#ifndef HOST_PC_H
          Serial.println(F("(A)pply host serial settings")); row++;
#endif
          Serial.println(F("(C)lear memory"));
          Serial.println(F("(S)ave configuration"));
          Serial.println(F("(L)oad configuration"));
          Serial.println(F("(R)eset to defaults"));
          Serial.println(F("\nE(x)it"));
          row += 8;

          Serial.print(F("\n\nCommand: ")); r_cmd = row+2;
        }
      else
        set_cursor(r_cmd, 10);

      while( !serial_available() ) delay(50);
      c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      redraw = true;
      switch( c )
        {
        case 'f': toggle_flag(CF_PROFILE, r_profile, col); redraw = false; break;
#if USE_THROTTLE>0
        case 't': toggle_throttle(r_throttle, col); redraw = false; break;
#endif
        case 'p': toggle_flag(CF_SERIAL_PANEL, r_panel, col); redraw = false; break;
#if STANDALONE==0
        case 'i': toggle_flag(CF_SERIAL_INPUT, r_input, col); redraw = false; break;
#endif
        case 'd': toggle_flag(CF_SERIAL_DEBUG, r_debug, col); redraw = false; break;
        case 'm': toggle_flag(CF_CLEARMEM, r_clearmem, col); redraw = false; break;
        case 'u': toggle_aux1_program(r_aux1, col); redraw = false; break;

        case 'b': toggle_host_serial_baud_rate(0, r_baud0, col); redraw = false; break;
        case 'r': toggle_host_serial_baud_rate(1, r_baud1, col); redraw = false; break;
        case 'P': toggle_host_primary_interface(r_primary, col); redraw = false; break;

        case '1': config_edit_serial_device(CSM_SIO); break;
        case '2': config_edit_serial_device(CSM_ACR); break;
        case '3': config_edit_serial_device(CSM_2SIO1); break;
        case '4': config_edit_serial_device(CSM_2SIO2); break;
        case 'I': config_edit_interrupts(); break;
          
        case 'M': filesys_manage(); break;
        case 'A': apply_host_serial_settings(); break;
        case 'C': 
          {
            mem_clr_ram_limit();
            for(int i=0; i<MEMSIZE; i++) Mem[i] = 0; 
            break;
          }
        case 'S': 
          {
            Serial.print(F("\n\nSave as config # (0=default): "));
            byte i = numsys_read_word();
            Serial.println();
            if( !save_config(i & 0xff) )
              Serial.println(F("Saving failed. Capture/replay in progress?"));
            break;
          }
        case 'L':
          {
            Serial.print(F("\n\nLoad config #: "));
            byte i = numsys_read_word();
            Serial.println();
            if( !load_config(i & 0xff) )
              Serial.println(F("Load failed. File does not exist?"));
            break;
          }
        case 'R': config_defaults(false); break;

        case 27:
        case 'x':
          {
            if( (config_serial_settings&0x1FF) != (new_config_serial_settings&0x1FF) )
              {
                Serial.print(F("\nApply new host serial settings (y/n/ESC)? "));
                do { delay(50); c = serial_read(); } while( c!='y' && c!='n' && c!=27 );
                Serial.println(c);
                if( c==27 || (c=='y' && !apply_host_serial_settings()) )
                  continue;
              }

            Serial.print(F("\033[2J"));
            return;
          }
        }
    }
}


void config_defaults(bool apply)
{
  // default settings:
  // - SERIAL_DEBUG, SERIAL_INPUT, SERIAL_PANEL enabled if in STANDALONE mode, otherwise disabled
  // - Profiling disabled
  // - Throttling enabled (on Due)

  config_flags = 0;
#if STANDALONE>0
  config_flags |= CF_SERIAL_DEBUG;
  config_flags |= CF_SERIAL_INPUT;
  config_flags |= CF_SERIAL_PANEL;
#endif
  config_flags |= CF_THROTTLE;

  new_config_serial_settings  = 0;
  new_config_serial_settings |= (BAUD_115200 << 0); // Serial interface (USB): 115200 baud
  new_config_serial_settings |= (BAUD_9600   << 4); // Serial1 interface: 9600 baud
  new_config_serial_settings |= (0           << 8); // USB Serial is primary interface

  uint32_t s = 0;
  s |= (BAUD_1200 <<  0); // serial playback baud rate: 1200
  s |= (4         <<  4); // 4 NUL characters after newline
  s |= (0         <<  8); // not mapped to any host interface
  s |= (2         << 10); // autodetect uppercase inputs
  s |= (2         << 12); // autodetect 7 bit 
  s |= (2         << 14); // autodetect backspace translation

  for(byte dev=0; dev<4; dev++)
    config_serial_device_settings[dev] = s;

  config_serial_device_settings[CSM_SIO]   |= (1 << 8); // map to SIO to primary host interface
  config_serial_device_settings[CSM_2SIO1] |= (1 << 8); // map to 2SIO-1 to primary host interface
  config_serial_device_settings[CSM_ACR]   |= (1 << 7); // enable CLOAD traps

  config_interrupt_vi_mask[0] = INT_DRIVE;
  config_interrupt_vi_mask[1] = INT_RTC;
  config_interrupt_vi_mask[2] = INT_2SIO1 | INT_2SIO2;
  config_interrupt_vi_mask[3] = 0;
  config_interrupt_vi_mask[4] = 0;
  config_interrupt_vi_mask[5] = 0;
  config_interrupt_vi_mask[6] = 0;
  config_interrupt_vi_mask[7] = INT_SIO | INT_ACR;

  config_interrupt_mask = INT_SIO | INT_2SIO1;
  
  config_aux1_prog = prog_find("16k ROM Basic");

  if( apply ) apply_host_serial_settings(new_config_serial_settings);
}


void config_setup()
{
  config_defaults(true);
  if( load_config(0) )
    apply_host_serial_settings(new_config_serial_settings);
}
