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
#include "hdsk.h"
#include "prog.h"
#include "sdmanager.h"

#define CONFIG_FILE_VERSION 3

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

#if HOST_NUM_SERIAL_PORTS>5
#error "Maximum number of host serial interfaces supported is 5"
#endif

#if USE_SECOND_2SIO>0
#define NUM_SERIAL_DEVICES 6
#else
#define NUM_SERIAL_DEVICES 4
#endif

// config_flags:
// vvvvvvvv mmmpphrt ttttRRRR dVCDIPFT
// T = Throttle
// t = Throttle delay if throttle is enabled (0=auto)
// F = Profile
// P = Serial Panel
// I = Serial Input
// D = Serial Debug
// C = Clear memory on powerup
// V = VI board installed
// R = RTC rate
// d = force real-time mode for disk drives
// p = printer type (00=NONE, 01=Okidata, 02=C700)
// m = map printer to host interface (00=NONE, 01=primary, 02=secondary)
// r = real-time mode for printer
// h = real-time mode for hard drives
// v = config file version
uint32_t config_flags;


// cofig_serial_settings:
// xxxxxxxx 44443333 2222xPPP 11110000
// 0000 = baud rate for fist   host interface (see baud rates above)
// 1111 = baud rate for second host interface (see baud rates above)
// 2222 = baud rate for third  host interface (see baud rates above)
// 3333 = baud rate for fourth host interface (see baud rates above)
// 4444 = baud rate for fifth  host interface (see baud rates above)
// PPP  = primary serial interface (maximum number depends on host)
// x    = unused
uint32_t config_serial_settings, new_config_serial_settings;


// cofig_serial_settings2:
// xxxxxxxB BPPSBBPP SBBPPSBB PPSBBPPS
// for all 5 host interfaces:
// BB = number of bits (0=5, 1=6, 2=7, 3=8)
// PP = parity         (0=none, 1=even, 2=odd)
// S  = stop bits      (0=1, 1=2)
uint32_t config_serial_settings2, new_config_serial_settings2;


// config_serial_device_settings[0-3]
// xxxxxxxx xxxxMMMR TT77UUxx CNNNBBBB
// BBBB = baud rate for serial playback (see baud rates above)
// NNN  = NULs to send after a carriage return when playing back examples
// C    = trap CLOAD/CSAVE in extended BASIC (for CSM_ACR device only)
// MMM  = map device to host interface (000=NONE, 001=first, 010=second, 011=third, 100=fourth, 101=fifth, 111=primary)
// UU   = only uppercase for inputs (00=off, 01=on, 10=autodetect)
// 77   = use 7 bit for serial outputs (00=off [use 8 bit], 01=on, 10=autodetect)
// TT   = translate backspace to (00=off, 01=underscore, 10=autodetect, 11=delete)
// R    = force realtime operation (use baud rate even if not using interrupts)
uint32_t config_serial_device_settings[NUM_SERIAL_DEVICES];


// masks defining which interrupts (INT_*) are at which vector interrupt levels
uint32_t config_interrupt_vi_mask[8];


// mask defining whch interrupts (INT_*) are connected if VI board is not installed
uint32_t config_interrupt_mask;


// program to be run when AUX1 is raised
byte config_aux1_prog;


// amount of RAM installed
uint32_t config_mem_size;


// --------------------------------------------------------------------------------


inline uint32_t get_bits(uint32_t v, byte i, byte n)
{
  return (v >> ((uint32_t) i)) & ((1ul<<n)-1);
}


inline uint32_t set_bits(uint32_t v, byte i, byte n, uint32_t nv)
{
  uint32_t mask = ((1ul<<n)-1) << i;
  return (v & ~mask) | ((nv << i) & mask);
}


static uint32_t toggle_bits(uint32_t v, byte i, byte n, byte min = 0x00, byte max = 0xff)
{
  byte b = get_bits(v, i, n) + 1;
  return set_bits(v, i, n, b>max ? min : (b<min ? min : b));
}

static byte config_baud_rate_bits(byte iface)
{
  byte n = 0;
  switch( iface )
    {
    case 0: n = 0;  break;
    case 1: n = 4;  break;
    case 2: n = 12; break;
    case 3: n = 16; break;
    case 4: n = 20; break;
    }

  return n;
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
        case 0: res =     0.06f; break;
        case 1: res =     0.60f; break;
        case 2: res =     6.00f; break;
        case 3: res =    10.00f; break;
        case 4: res =    60.00f; break;
        case 5: res =   100.00f; break;
        case 6: res =  1000.00f; break;
        case 7: res = 10000.00f; break;
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


byte config_serial_backspace(byte dev, uint16_t PC)
{
  byte b = get_bits(config_serial_device_settings[dev], 14, 2);

  if( b==CSFB_AUTO )
    {
      if( PC == 0x038A || PC == 0x0380 || PC == 0x00A0 || PC == 0x00AC )
        {
          // ALTAIR 4k BASIC I/O routine has "IN" instruction at 0x389 and "OUT" instruction at 0x037F
          // ALTAIR EXTENDED BASIC I/O routine has "IN" instruction at 0x09f and "OUT" instruction at 0x00AB
          b = CSFB_UNDERSCORE;
        }
      else if( PC == 0x0726 || PC == 0x08e5 )
        {
          // MITS Programming System II has "IN" instruction at 0x0725 and 0x07E4
          b = CSFB_DELETE;
        }
      else 
        b = CSFB_NONE;
    }
  
  return b;
}


bool config_serial_ucase(byte dev, uint16_t PC)
{
  byte b = get_bits(config_serial_device_settings[dev], 10, 2);
  if( b==CSF_AUTO )
    {
      // ALTAIR 4k BASIC I/O routine has "IN" instruction at 0x0389
      // MITS Programming System II has "IN" instruction at 0x0725 and 0x07E4
      return (PC == 0x038A) || (PC == 0x0726) || (PC == 0x08e5);
    }
  else
    return b==CSF_ON;
}


bool config_serial_7bit(byte dev, uint16_t PC)
{
  byte b = get_bits(config_serial_device_settings[dev], 12, 2);
  if( b==CSF_AUTO )
    {
      // ALTAIR 4k BASIC I/O routine has "OUT" instruction at 0x037F
      // ALTAIR EXTENDED BASIC I/O routine has "OUT" instruction at 0x00AB
      // MITS Programming System II has "OUT" instruction at 0x071B
      return (PC == 0x0380) || (PC == 0x00AC) || (PC == 0x071C);
    }
  else
    return b==CSF_ON;
}


bool config_serial_trap_CLOAD()
{
  return get_bits(config_serial_device_settings[CSM_ACR], 7, 1)>0;
}


uint32_t config_serial_playback_baud_rate(byte dev)
{
  return config_baud_rate(get_bits(config_serial_device_settings[dev], 0, 4));
}

byte config_serial_playback_example_nuls(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 4, 3);
}

byte config_map_device_to_host_interface(byte s)
{
  if( s==7 )
    return config_host_serial_primary();
  else if( s<=HOST_NUM_SERIAL_PORTS )
    return s-1;
  else
    return 0xff;
}

byte config_serial_map_sim_to_host(byte dev)
{
  return config_map_device_to_host_interface(get_bits(config_serial_device_settings[dev], 17, 3));
}

bool config_serial_realtime(byte dev)
{
  return get_bits(config_serial_device_settings[dev], 16, 1) ? true : false;
}

byte config_host_serial_primary()
{
  return get_bits(config_serial_settings, 8, 3);
}

uint32_t config_host_serial_baud_rate(uint32_t settings, byte iface)
{
  return config_baud_rate(get_bits(settings, config_baud_rate_bits(iface), 4));
}


uint32_t config_host_serial_config(uint32_t settings2, byte iface)
{
  byte v = get_bits(settings2, iface * 5, 5);
  switch( v )
    {
    case 0x00: return SERIAL_5N1;
    case 0x01: return SERIAL_5N2;
    case 0x02: return SERIAL_5E1;
    case 0x03: return SERIAL_5E2;
    case 0x04: return SERIAL_5O1;
    case 0x05: return SERIAL_5O2;

    case 0x08: return SERIAL_6N1;
    case 0x09: return SERIAL_6N2;
    case 0x0A: return SERIAL_6E1;
    case 0x0B: return SERIAL_6E2;
    case 0x0C: return SERIAL_6O1;
    case 0x0D: return SERIAL_6O2;
  
    case 0x10: return SERIAL_7N1;
    case 0x11: return SERIAL_7N2;
    case 0x12: return SERIAL_7E1;
    case 0x13: return SERIAL_7E2;
    case 0x14: return SERIAL_7O1;
    case 0x15: return SERIAL_7O2;

    case 0x18: return SERIAL_8N1;
    case 0x19: return SERIAL_8N2;
    case 0x1A: return SERIAL_8E1;
    case 0x1B: return SERIAL_8E2;
    case 0x1C: return SERIAL_8O1;
    case 0x1D: return SERIAL_8O2;

    // fall back to default 8N1 settings
    default  : return SERIAL_8N1;
    }
}


uint32_t config_host_serial_config(byte iface)
{
  return config_host_serial_config(config_serial_settings2, iface);
}


uint32_t config_host_serial_baud_rate(byte iface)
{
  return config_host_serial_baud_rate(config_serial_settings, iface);
}


byte config_printer_map_to_host_serial()
{
  return config_map_device_to_host_interface(get_bits(config_flags, 21, 3));
}


byte config_printer_type()
{
  return get_bits(config_flags, 19, 2);
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


static void print_mem_size(uint32_t s, byte row=0, byte col=0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);

  if( (s&0x3FF)==0 )
    {
      Serial.print(s/1024); 
      Serial.println(F(" KB     "));
    }
  else
    {
      Serial.print(s); 
      Serial.println(F(" bytes     "));
    }
}


static void print_flag(uint32_t data, uint32_t value, byte row=0, byte col=0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);
  Serial.print((data&value)!=0 ? F("yes") : F("no"));
}


static void print_flag(uint32_t value, byte row=0, byte col=0)
{
  print_flag(config_flags, value, row, col);
}


static void print_vi_flag()
{
  Serial.print((config_flags&CF_HAVE_VI)!=0 ? F("Use Vector Interrupt board") : F("Interrupts connected directly to CPU"));
}


static void print_host_serial_config(uint32_t settings2, byte iface)
{
  byte v = get_bits(settings2, iface * 5, 5);
  Serial.print(get_bits(settings2, iface * 5 + 3, 2)+5);
  switch( get_bits(settings2, iface * 5 + 1, 2) )
    {
    case 0 : Serial.print('N'); break;
    case 1 : Serial.print('E'); break;
    case 2 : Serial.print('O'); break;
    case 3 : Serial.print('?'); break;
    }
  Serial.print(get_bits(settings2, iface * 5, 1)+1);
}


static void print_host_serial_config(byte iface, byte row, byte col)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);
  Serial.print(config_host_serial_baud_rate(new_config_serial_settings, iface)); 
  Serial.print(F(" baud"));
  if( host_serial_port_has_configs(iface) )
    {
      Serial.print(' ');
      print_host_serial_config(new_config_serial_settings2, iface);
    }

  if( config_host_serial_baud_rate(new_config_serial_settings, iface) != config_host_serial_baud_rate(iface) 
      ||
      get_bits(new_config_serial_settings2, iface * 5, 5) != get_bits(config_serial_settings2, iface * 5, 5) )
    {
      Serial.print(F(" (current: ")); 
      Serial.print(config_host_serial_baud_rate(iface));
      if( host_serial_port_has_configs(iface) )
        {
          Serial.print(' ');
          print_host_serial_config(config_serial_settings2, iface);
        }
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
  Serial.print(host_serial_port_name(iface));
}


static void print_host_primary_interface(byte row = 0, byte col = 0)
{
  if( row!=0 || col!=0 ) set_cursor(row, col);
  print_host_primary_interface_aux(get_bits(new_config_serial_settings, 8, 3));

  if( get_bits(new_config_serial_settings, 8, 3) != config_host_serial_primary() )
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
    case CSFB_DELETE:     Serial.print(F("delete (127)"));   break;
    case CSFB_AUTO:       Serial.print(F("autodetect"));     break;
    }
}


static void print_device_mapped_to(byte s)
{
  if( s==0 )
    Serial.print(F("Not mapped")); 
  else if( s==7 )
    {
      Serial.print(F("Primary interface ("));
      print_host_primary_interface();
      Serial.print(')');
    }
  else
    Serial.print(host_serial_port_name(s-1));
}


static void print_serial_device_mapped_to(uint32_t settings)
{
  print_device_mapped_to(get_bits(settings, 17, 3));
}


static void print_printer_mapped_to()
{
  print_device_mapped_to(get_bits(config_flags, 21, 3));
}


static void print_rtc_frequency()
{
  float rate = config_rtc_rate();
  if( rate==0.0 )
    Serial.print(F("Disabled"));
  else
    { Serial.print(rate); Serial.print(F(" Hz")); }
}


static void print_interrupt_conn(uint32_t mask, byte b)
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
      const char *image;

      if( b & 0x40 )
        image = hdsk_get_image_description(b&0x3f);
      else
        image = drive_get_image_description(b&0x3f);

      if( image==NULL )
        Serial.print(F("none"));
      else
        Serial.print(image);
    }
  else if( b > 0 && prog_get_name(b)!=NULL )
    Serial.print(FP(prog_get_name(b)));
  else
    Serial.print(F("none"));
}


static void print_drive_mounted()
{
  byte n = 0;
  for(byte i=0; i<NUM_DRIVES; i++)
    if( drive_get_mounted_image(i)>0 ) n++;

  Serial.print(n); Serial.print(F(" mounted"));
}


static void print_drive_mounted_image(byte d)
{
  if( d==0 )
    Serial.print(F("none"));
  else if( drive_get_image_filename(d)==NULL )
    { Serial.print(F("empty disk #")); numsys_print_byte(d); }
  else
    Serial.print(drive_get_image_description(d));
}


static void print_hdsk_mounted()
{
  byte n = 0;
  for(byte i=0; i<NUM_HDSK_UNITS; i++)
    for(byte j=0; j<4; j++)
      if( hdsk_get_mounted_image(i, j)>0 ) n++;

  Serial.print(n); Serial.print(F(" mounted"));
}


static void print_hdsk_mounted_image(byte d)
{
  if( d==0 )
    Serial.print(F("none"));
  else if( hdsk_get_image_filename(d)==NULL )
    { Serial.print(F("empty disk #")); numsys_print_byte(d); }
  else
    Serial.print(hdsk_get_image_description(d));
}


static void print_printer_type()
{
  switch( config_printer_type() )
    {
    case 0: Serial.print(F("None")); break;
    case 1: Serial.print(F("Okidata")); break;
    case 2: Serial.print(F("C700")); break;
    }
}


static void print_parity(byte p)
{
  switch( p ) 
    {
    case 0  : Serial.print(F("None")); break;
    case 1  : Serial.print(F("Even")); break;
    case 2  : Serial.print(F("Odd "));  break;
    default : Serial.print(F("??? "));  break;
    }
}


// --------------------------------------------------------------------------------


static void apply_host_serial_settings(uint32_t settings, uint32_t settings2)
{
  config_serial_settings  = settings;
  config_serial_settings2 = settings2;
  for(byte i=0; i<HOST_NUM_SERIAL_PORTS; i++)
    host_serial_setup(i, config_host_serial_baud_rate(i), 
                      config_host_serial_config(settings2, i),
                      config_host_serial_primary()==i);
}


static byte toggle_host_serial_baud_rate(byte iface, byte n)
{
  uint32_t min, max;
  if( host_serial_port_baud_limits(iface, &min, &max) )
    {
      do 
        { n = (n + 1) % 12; }
      while( config_baud_rate(n) < min || config_baud_rate(n) > max );
      
#if defined(__SAM3X8E__)
      // connecting to Arduino Due at 1200 baud over USB will cause it to erase its flash memory
      // and go into programming mode => skip 1200 baud setting for USB interface
      if( iface==0 && n==BAUD_1200 ) n++;
#endif
    }

  return n;
}


static void toggle_host_primary_interface(byte row, byte col)
{
  new_config_serial_settings = toggle_bits(new_config_serial_settings, 8, 3, 0, HOST_NUM_SERIAL_PORTS-1);
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


static byte toggle_interrupt_conn(uint32_t mask, byte c, byte row, byte col)
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

  set_cursor(row, col);
  print_interrupt_conn(mask, c); 
  return c;
}


static uint32_t toggle_map_to_serial(uint32_t settings, byte start_bit)
{
  byte i = get_bits(settings, start_bit, 3);
  if( i==HOST_NUM_SERIAL_PORTS )
    i = 0;
#if HOST_NUM_SERIAL_PORTS>1
  else if( i==0 )
    i = 7;
  else if( i==7 )
    i = 1;
#endif
  else
    i++;
    
  return set_bits(settings, start_bit, 3, i);
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
    case CSFB_UNDERSCORE: b = CSFB_DELETE; break;
    case CSFB_DELETE:     b = CSFB_AUTO; break;
    case CSFB_AUTO:       b = CSFB_NONE; break;
    }
  
  return set_bits(settings, 14, 2, b);
}


static byte find_floppy_image(byte n)
{
  byte i=n;
  do
    {
      if( drive_get_image_filename(i)!=NULL ) return i;
      i++;
    }
  while( i>0 );

  return 0;
}


static byte find_hdsk_image(byte n)
{
  byte i=n;
  do
    {
      if( hdsk_get_image_filename(i)!=NULL ) return i;
      i++;
    }
  while( i>0 );

  return 0;
}


static void toggle_aux1_program(byte row, byte col)
{
  byte b = config_aux1_prog;
  bool found = false;

  while( !found )
    {
      if( !found && (b & 0xC0)==0x00 )
        {
          // look for next integrated program
          b = b + 1;
          if( prog_get_name(b) )
            found = true;
          else
            b = 0x80;
        }

      if( !found && (b & 0xC0)==0x80 )
        {
          // look for next floppy disk image
          b = find_floppy_image((b & 0x3f)+1) | 0x80;
          if( b>0x80 ) 
            found = true;
          else
            b = 0xC0;
        }

      if( !found && (b & 0xC0)==0xC0 )
        {
          // look for next hard disk image
          b = find_hdsk_image((b & 0x3f)+1) | 0xC0;
          if( b>0xC0 ) 
            found = true;
          else
            b = 0;
        }
    }

  config_aux1_prog = b;
  print_aux1_program(row, col);
}


static void toggle_flag(uint32_t value, byte row, byte col)
{
  config_flags = (config_flags & ~value) | (~(config_flags & value) & value);
  print_flag(config_flags, value, row, col);
}


static bool apply_host_serial_settings()
{
  char c;
  uint32_t old_config_serial_settings = config_serial_settings;
  uint32_t old_config_serial_settings2 = config_serial_settings2;

#if HOST_NUM_SERIAL_PORTS>1
  if( config_host_serial_primary() != get_bits(new_config_serial_settings, 8, 3) )
    Serial.println("\r\nChange must be confirmed by answering 'y' through new primary interface.\r\n"
                   "[will revert back to this interface if new interface is not confirmed within 30 seconds]");
  else
#endif
  Serial.println(F("\r\n[will revert to old settings if new settings not confirmed within 30 seconds]\r\n"));
  Serial.flush();

  apply_host_serial_settings(new_config_serial_settings, new_config_serial_settings2);

  uint32_t timeout = millis() + 30000, timeout2 = 0;
  c = 0;
  do
    {
      if( millis()>timeout2 )
        {
          Serial.print(F("\r\nKeep new host interface settings (y/n)? "));
          timeout2 = millis() + 2000;
        }

      delay(50);
      c = serial_read();
    }
  while( c!='y' && c!='n' && millis()<timeout );
  Serial.println(c);
  if( c!='y' )
    { 
      apply_host_serial_settings(old_config_serial_settings, old_config_serial_settings2);
      if( c!='n' ) 
        {
          delay(100);
          Serial.println(F("\r\nSettings were not confirmed within 30 seconds."));
          delay(3000);

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
  byte data[(12+NUM_SERIAL_DEVICES)*sizeof(uint32_t)+1+1+NUM_DRIVES+1+NUM_HDSK_UNITS*4+1+1];

  // merge version number into config_flags
  config_flags = (config_flags & 0x00ffffff) | (((uint32_t) CONFIG_FILE_VERSION) << 24);

  word n = 0;
  memcpy(data+n, &config_flags, s); n+=s;
  memcpy(data+n, &new_config_serial_settings, s); n+=s;
  memcpy(data+n, &new_config_serial_settings2, s); n+=s;
  data[n] = NUM_SERIAL_DEVICES; n++;
  memcpy(data+n,  config_serial_device_settings, NUM_SERIAL_DEVICES*s); n+=NUM_SERIAL_DEVICES*s;
  memcpy(data+n,  config_interrupt_vi_mask, 8*s); n+=8*s;
  memcpy(data+n, &config_interrupt_mask, s); n+=s;

  data[n++] = config_aux1_prog;
  
  data[n++] = NUM_DRIVES;
  for(byte i=0; i<NUM_DRIVES; i++) data[n++] = drive_get_mounted_image(i);

  data[n++] = NUM_HDSK_UNITS;
  for(byte i=0; i<NUM_HDSK_UNITS; i++) 
    for(byte j=0; j<4; j++)
      data[n++] = hdsk_get_mounted_image(i, j);

  data[n++] = config_mem_size/256;
  return filesys_write_file('C', fileno, (void *) data, n);
}


static bool load_config(byte fileno)
{
  bool ok = false;
  byte i, j, n, d, fid = filesys_open_read('C', fileno);
  if( fid )
    {
      // initialize all settings with defaults so configuration settings
      // missing at the end of the file will just be at their defaults
      // (helpful when reading an old config file after adding new settings)
      config_defaults(false);

      byte s = sizeof(uint32_t);
      filesys_read_data(fid, &config_flags, s);
      
      // highest 8 bits are file version
      byte v = config_flags >> 24;

      filesys_read_data(fid, &new_config_serial_settings, s);

      if( v<3 )
        {
          // config file before version 3 does not include serial port configuration
          // => default to 8N1
          new_config_serial_settings2 = 0;
          for(i=0; i<HOST_NUM_SERIAL_PORTS; i++) 
            new_config_serial_settings2 |= 0x18 << (i*5);
        }
      else
        filesys_read_data(fid, &new_config_serial_settings2, s);

      n = 4;
      // in config file version 2, the number of serial devices is in the file, 
      // before it was fixed at 4
      if( v>=2 ) filesys_read_data(fid, &n, 1);
      filesys_read_data(fid, config_serial_device_settings, min(NUM_SERIAL_DEVICES, n)*s);

      // skip extra serial device data in config
      if( NUM_SERIAL_DEVICES<n )
        for(i=NUM_SERIAL_DEVICES*s; i<n*s; i++)
          filesys_read_data(fid, &d, 1);

      if( v==0 )
        {
          // in config file version 0 the interrupt masks are 8 bits
          byte b[9];
          filesys_read_data(fid, b, 9);
          for(i=0; i<8; i++) config_interrupt_vi_mask[i] = b[i];
          config_interrupt_mask = b[8];
        }
      else
        {
          // in config file version 1 and later the interrupt masks are 32 bits
          filesys_read_data(fid, config_interrupt_vi_mask, 8*s);
          filesys_read_data(fid, &config_interrupt_mask, s);
        }

      // AUX1 UP shortcut program
      filesys_read_data(fid, &config_aux1_prog, 1);
      
      // disk drive settings
      if( filesys_read_data(fid, &n, 1)!=1 ) n = 0;
      for(i=0; i<n; i++) 
        if( filesys_read_data(fid, &d, 1)==1 && i<NUM_DRIVES )
          {
            if( d>0 )
              drive_mount(i, d);
            else
              drive_unmount(i);
          }
      drive_set_realtime((config_flags & CF_DRIVE_RT)!=0);

      // hard disk settings
      if( filesys_read_data(fid, &n, 1)!=1 ) n = 0;
      for(i=0; i<n; i++) 
        for(j=0; j<4; j++) 
          if( filesys_read_data(fid, &d, 1)==1 && i<NUM_HDSK_UNITS )
            {
              if( d>0 )
                hdsk_mount(i, j, d);
              else
                hdsk_unmount(i, j);
            }
      hdsk_set_realtime((config_flags & CF_HDSK_RT)!=0);

      // memory (RAM) size
      if( filesys_read_data(fid, &d, 1) )
        config_mem_size = (d==0) ? 0x10000 : (d*256);
      else
        config_mem_size = MEMSIZE;
      
      filesys_close(fid);

#if defined(__AVR_ATmega2560__)
      if( v==0 )
        {
          // on MEGA, early versions accidentally set bits 16-31 on serial settings
          for(byte dev=0; dev<4; dev++)
            if( (config_serial_device_settings[dev] & 0xFFFF0000)==0xFFFF0000 )
              config_serial_device_settings[dev] &= 0x0000FFFF;
        }
#endif

      if( v<2 )
        {
          // in version 2, serial device to host serial mapping changed:
          // previous: bits  8- 9 (00=NONE, 01=primary, 02=secondary)
          // now:      bits 17-19 (000=NONE, 001=first, 010=second, 011=third, 100=fourth, 101=fifth, 111=primary)
          for(i=0; i<4; i++)
            {
              byte map = get_bits(config_serial_device_settings[i], 8, 2);
              config_serial_device_settings[i] = set_bits(config_serial_device_settings[i],  8, 2, 0);
              config_serial_device_settings[i] = set_bits(config_serial_device_settings[i], 17, 3, map==1 && HOST_NUM_SERIAL_PORTS>1 ? 7 : map);
            }

          // in version 2, printer to host serial mapping changed:
          // previous: bits 17-18 (realtime mode flags bits 21-22)
          // now     : bits 21-23 (realtime mode flags bits 17-18)
          byte map   = get_bits(config_flags, 17, 2);
          byte flags = get_bits(config_flags, 21, 2);
          config_flags = set_bits(config_flags, 17, 2, flags);
          config_flags = set_bits(config_flags, 21, 3, map==1 ? 7 : map);
          
          // version 2 introduces more host serial interfaces: set them all to 9600 baud
          for(i=2; i<HOST_NUM_SERIAL_PORTS; i++) new_config_serial_settings |= (BAUD_9600 << (config_baud_rate_bits(i)));
        }
      else
        {
          // make sure nothing is mapped to an illegal host serial port
          for(i=0; i<NUM_SERIAL_DEVICES; i++) 
            if( config_serial_map_sim_to_host(i) >= HOST_NUM_SERIAL_PORTS )
              config_serial_device_settings[i] = set_bits(config_serial_device_settings[i], 17, 3, 0);              
        }

#if STANDALONE>0
      config_flags |= CF_SERIAL_INPUT;      
#endif
      ok = true;
    }

  return ok;
}


// --------------------------------------------------------------------------------

#if USE_PRINTER>0

void config_edit_printer()
{
  byte row, col, r_type, r_iface, r_force, r_cmd;
  bool go = true, redraw = true;

  while( go )
    {
      if( redraw )
        {
          Serial.print(F("\033[2J\033[0;0H\n"));
          
          row = 4;
          col = 30;
          Serial.println(F("Configure printer settings"));
          Serial.print(F("\n(P)rinter type             : ")); r_type = row++; print_printer_type(); Serial.println();
          Serial.print(F("Map printer to (i)nterface : ")); r_iface = row++; print_printer_mapped_to(); Serial.println();
          Serial.print(F("(F)orce real-time mode     : ")); r_force = row++; print_flag(CF_PRINTER_RT); Serial.println();
          
          Serial.println(F("\nE(x)it to main menu"));
          Serial.print(F("\n\nCommand: ")); r_cmd = row + 4;
          redraw = false;
        }
      else
        set_cursor(r_cmd, 10);
      
      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'P': 
          config_flags = toggle_bits(config_flags, 19, 2, 0, 2);
          set_cursor(r_type, col);
          print_printer_type(); 
          break;

        case 'i':
          config_flags = toggle_map_to_serial(config_flags, 21);
          set_cursor(r_iface, col);
          print_printer_mapped_to(); 
          break;

        case 'F': toggle_flag(CF_PRINTER_RT, r_force, col); break;
        case 27:
        case 'x': go = false; break;
        }
    }
}

#endif

// --------------------------------------------------------------------------------

#if NUM_DRIVES>0
void config_edit_drives()
{
  bool go = true;
  byte i, mounted[NUM_DRIVES];

  for(i=0; i<NUM_DRIVES; i++) mounted[i] = drive_get_mounted_image(i);

  byte row, col, r_realtime, r_drives[NUM_DRIVES], r_cmd;
  row = 4;
  col = 32;
  Serial.print(F("\033[2J\033[0;0H\n"));

  Serial.println(F("Configure disk drive settings"));
  Serial.print(F("\n(F)orce real-time mode : ")); print_flag(CF_DRIVE_RT); Serial.println(); r_realtime = row++;
  
  for(i=0; i<NUM_DRIVES; i++)
    {
      Serial.print(F("Drive (")); 
      if( i<10 ) Serial.write(48+i); else Serial.write(87+i);
      Serial.print(F(") mounted disk image : "));
      print_drive_mounted_image(mounted[i]);
      Serial.println();
      r_drives[i] = row++;
    }

  Serial.println(F("\nE(x)it to main menu")); row+=4;
  Serial.print(F("\n\nCommand: ")); r_cmd = row++;

  while( go )
    {
      set_cursor(r_cmd, 10);
      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'F': toggle_flag(CF_DRIVE_RT, r_realtime, 26); break;
        case 27:
        case 'x': go = false; break;

        default:
          {
            int d = -1;
            if( c>='0' && c<='9' ) 
              d = c-48;
            else if( c>='a' && c<='f' )
              d = c-87;

            if( d>=0 && d<=NUM_DRIVES )
              {
                byte next = find_floppy_image(mounted[d]+1);
                if( drive_get_mounted_image(d)>0 && 
                    drive_get_image_filename(drive_get_mounted_image(d))==NULL &&
                    mounted[d] < drive_get_mounted_image(d) &&
                    (next       > drive_get_mounted_image(d) || next<mounted[d]) )
                  next = drive_get_mounted_image(d);

                mounted[d] = next;
                set_cursor(r_drives[d], col);
                print_drive_mounted_image(mounted[d]);
              }
          }
        }
    }

  drive_set_realtime((config_flags & CF_DRIVE_RT)!=0);
  for(i=0; i<NUM_DRIVES; i++) 
    if( mounted[i] != drive_get_mounted_image(i) )
      drive_mount(i, mounted[i]);
}
#endif


// --------------------------------------------------------------------------------

#if NUM_HDSK_UNITS>0
void config_edit_hdsk()
{
  bool go = true;
  byte i, j;

  byte mounted[NUM_HDSK_UNITS*4];
  for(i=0; i<NUM_HDSK_UNITS; i++)
    for(j=0; j<4; j++)
      mounted[i*4+j] = hdsk_get_mounted_image(i, j);

  byte row, col, r_drives[NUM_HDSK_UNITS*4], r_cmd, r_realtime;
  row = 4;
  col = 33;
  Serial.print(F("\033[2J\033[0;0H\n"));

  Serial.println(F("Configure hard disk settings"));
  Serial.print(F("\n(F)orce real-time mode : ")); print_flag(CF_HDSK_RT); Serial.println(); r_realtime = row++;

  for(i=0; i<NUM_HDSK_UNITS; i++)
    for(j=0; j<4; j++)
      {
        byte n = i*4+j;
        Serial.print(F("("));
        if( n<10 ) Serial.write(48+n); else Serial.write(87+n);
        Serial.print(F(") Hard disk"));
#if NUM_HDSK_UNITS>1
        Serial.print(F(" unit ")); 
        Serial.print(i+1);
        col=40;
#endif
        Serial.print(F(" platter "));
        Serial.print(j);
        Serial.print(F(" image : "));
        print_hdsk_mounted_image(mounted[i*4+j]);
        Serial.println();
        r_drives[n] = row++;
      }

  Serial.println(F("\n(R)eset hard disk controller")); row+=3;
  
  Serial.println(F("\nE(x)it to main menu")); row+=3;
  Serial.print(F("\n\nCommand: ")); r_cmd = row++;

  while( go )
    {
      set_cursor(r_cmd, 10);
      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'F': 
          toggle_flag(CF_HDSK_RT, r_realtime, 26); 
          break;

        case 'R': 
          Serial.print(F("\n\nResetting hard disk controller..."));
          hdsk_reset(); 
          delay(1000);
          Serial.print(F("\033[33D\033[K"));
          break;

        case 27:
        case 'x': go = false; break;

        default:
          {
            int d = -1;
            if( c>='0' && c<='9' ) 
              d = c-48;
            else if( c>='a' && c<='f' )
              d = c-87;

            if( d>=0 && d<4*NUM_HDSK_UNITS )
              {
                byte u = d/4, p = d % 4;
                byte next = find_hdsk_image(mounted[d]+1);
                if( hdsk_get_mounted_image(u, p)>0 && 
                    hdsk_get_image_filename(hdsk_get_mounted_image(u, p))==NULL &&
                    mounted[d] < hdsk_get_mounted_image(u, p) &&
                    (next      > hdsk_get_mounted_image(u, p) || next<mounted[d]) )
                  next = hdsk_get_mounted_image(u, p);

                mounted[d] = next;
                set_cursor(r_drives[d], col);
                print_hdsk_mounted_image(mounted[d]);
              }
          }
        }
    }

  hdsk_set_realtime((config_flags & CF_HDSK_RT)!=0);
  for(i=0; i<NUM_HDSK_UNITS; i++)
    for(j=0; j<4; j++)
      if( mounted[i*4+j] != hdsk_get_mounted_image(i, j) )
        hdsk_mount(i, j, mounted[i*4+j]);
}
#endif


// --------------------------------------------------------------------------------


byte find_vi_conn(uint32_t interrupt)
{
  for(int i=0; i<8; i++)
    if( config_interrupt_vi_mask[i] & interrupt )
      return i;

  return 0xff;
}


void config_edit_interrupts()
{
  byte row, col, r_rtcf, r_vint, r_sio, r_acr, r_2sio1, r_2sio2, r_2sio3, r_2sio4, r_rtc, r_lpc, r_drive, r_hdsk, r_cmd;
  byte conn_sio   = find_vi_conn(INT_SIO);
  byte conn_acr   = find_vi_conn(INT_ACR);
  byte conn_2sio1 = find_vi_conn(INT_2SIO1);
  byte conn_2sio2 = find_vi_conn(INT_2SIO2);
  byte conn_2sio3 = find_vi_conn(INT_2SIO3);
  byte conn_2sio4 = find_vi_conn(INT_2SIO4);
  byte conn_rtc   = find_vi_conn(INT_RTC);
  byte conn_lpc   = find_vi_conn(INT_LPC);
  byte conn_drive = find_vi_conn(INT_DRIVE);
  byte conn_hdsk  = find_vi_conn(INT_HDSK);

  bool go = true, redraw = true;
  while( go )
    {
      if( redraw )
        {
          row = 4;
          col = 34;
          Serial.print(F("\033[2J\033[0;0H\n"));
          Serial.println(F("Configure interrupt settings"));
          Serial.print(F("\n(R)eal-Time Clock              : ")); r_rtcf = row++; print_rtc_frequency(); Serial.println();
          Serial.print(F("(V)ector Interrupt board       : ")); r_vint = row++; print_vi_flag(); Serial.println();

          Serial.println(); row++;
          Serial.print(F("(0) Real-Time Clock interrupt  : ")); r_rtc = row++; print_interrupt_conn(INT_RTC, conn_rtc); Serial.println();
          Serial.print(F("(1) 88-SIO interrupt           : ")); r_sio = row++; print_interrupt_conn(INT_SIO, conn_sio); Serial.println();
          Serial.print(F("(2) 88-ACR interrupt           : ")); r_acr = row++; print_interrupt_conn(INT_ACR, conn_acr); Serial.println();
          Serial.print(F("(3) 88-LPC interrupt           : ")); r_lpc = row++; print_interrupt_conn(INT_LPC, conn_lpc); Serial.println();
          Serial.print(F("(4) 88-2SIO port 1 interrupt   : ")); r_2sio1 = row++; print_interrupt_conn(INT_2SIO1, conn_2sio1); Serial.println();
          Serial.print(F("(5) 88-2SIO port 2 interrupt   : ")); r_2sio2 = row++; print_interrupt_conn(INT_2SIO2, conn_2sio2); Serial.println();
#if USE_SECOND_2SIO>0
          Serial.print(F("(6) 88-2SIO-2 port 1 interrupt : ")); r_2sio3 = row++; print_interrupt_conn(INT_2SIO3, conn_2sio3); Serial.println();
          Serial.print(F("(7) 88-2SIO-2 port 2 interrupt : ")); r_2sio4 = row++; print_interrupt_conn(INT_2SIO4, conn_2sio4); Serial.println();
#endif
#if NUM_DRIVES>0
          Serial.print(F("(8) Disk drive interrupt       : ")); r_drive = row++; print_interrupt_conn(INT_DRIVE, conn_drive); Serial.println();
#endif
#if NUM_HDSK_UNITS>0
          Serial.print(F("(9) 88-HDSK interrupt          : ")); r_hdsk = row++; print_interrupt_conn(INT_HDSK, conn_hdsk); Serial.println();
#endif
          
          Serial.println(F("\nE(x)it to main menu"));
          
          Serial.print(F("\n\nCommand: ")); r_cmd = row+4;
          redraw = false;
        }
      else
        set_cursor(r_cmd, 10);

      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'R':
        case 'r':
        case 'C': 
        case 'c': 
          {
            toggle_rtc_rate(); 
            set_cursor(r_rtcf, col);
            print_rtc_frequency(); 
            break;
          }

        case 'V':
        case 'v': toggle_flag(CF_HAVE_VI, 0, 0); redraw = true; break;

        case '0': conn_rtc   = toggle_interrupt_conn(INT_RTC, conn_rtc, r_rtc, col);     break;
        case '1': conn_sio   = toggle_interrupt_conn(INT_SIO, conn_sio, r_sio, col);     break;
        case '2': conn_acr   = toggle_interrupt_conn(INT_ACR, conn_acr, r_acr, col);     break;
        case '3': conn_lpc   = toggle_interrupt_conn(INT_LPC, conn_lpc, r_lpc, col);     break;
        case '4': conn_2sio1 = toggle_interrupt_conn(INT_2SIO1, conn_2sio1, r_2sio1, col); break;
        case '5': conn_2sio2 = toggle_interrupt_conn(INT_2SIO2, conn_2sio2, r_2sio2, col); break;
#if USE_SECOND_2SIO>0
        case '6': conn_2sio3 = toggle_interrupt_conn(INT_2SIO3, conn_2sio3, r_2sio3, col); break;
        case '7': conn_2sio4 = toggle_interrupt_conn(INT_2SIO4, conn_2sio4, r_2sio4, col); break;
#endif
        case '8': conn_drive = toggle_interrupt_conn(INT_DRIVE, conn_drive, r_drive, col); break;
        case '9': conn_hdsk  = toggle_interrupt_conn(INT_HDSK, conn_hdsk, r_hdsk, col);   break;

        case 27:
        case 'x': go = false; break;
        }
    }

  for(int i=0; i<8; i++)  config_interrupt_vi_mask[i] = 0;
  if( conn_sio   < 0xff ) config_interrupt_vi_mask[conn_sio]   |= INT_SIO;
  if( conn_acr   < 0xff ) config_interrupt_vi_mask[conn_acr]   |= INT_ACR;
  if( conn_2sio1 < 0xff ) config_interrupt_vi_mask[conn_2sio1] |= INT_2SIO1;
  if( conn_2sio2 < 0xff ) config_interrupt_vi_mask[conn_2sio1] |= INT_2SIO2;
  if( conn_2sio3 < 0xff ) config_interrupt_vi_mask[conn_2sio3] |= INT_2SIO3;
  if( conn_2sio4 < 0xff ) config_interrupt_vi_mask[conn_2sio4] |= INT_2SIO4;
  if( conn_rtc   < 0xff ) config_interrupt_vi_mask[conn_rtc]   |= INT_RTC;
  if( conn_drive < 0xff ) config_interrupt_vi_mask[conn_drive] |= INT_DRIVE;
  if( conn_lpc   < 0xff ) config_interrupt_vi_mask[conn_lpc]   |= INT_LPC;
  if( conn_hdsk  < 0xff ) config_interrupt_vi_mask[conn_hdsk]  |= INT_HDSK;
}


// --------------------------------------------------------------------------------


void config_edit_serial_device(byte dev)
{
  uint32_t settings = config_serial_device_settings[dev];
  byte row, col, r_iface, r_baud, r_force, r_nuls, r_7bits, r_ucase, r_bspace, r_traps, r_cmd;
  bool redraw = true;

  while( true )
    {
      if( redraw )
        {
          row = 4;
          col = 30;
          Serial.print(F("\033[2J\033[0;0H\n"));
          Serial.print(F("Configure serial device ")); print_serial_device_sim(dev); Serial.println();
          Serial.print(F("\nMap to host (i)nterface    : ")); r_iface = row++; print_serial_device_mapped_to(settings); Serial.println();
          Serial.print(F("Simulated (b)aud rate      : ")); r_baud = row++; Serial.println(config_baud_rate(get_bits(settings, 0, 4)));
          Serial.print(F("(F)orce baud rate          : ")); r_force = row++; print_flag(settings, 1ul<<16, 0, 0); Serial.println();
          Serial.print(F("Example playback (N)ULs    : ")); r_nuls = row++; Serial.println(get_bits(settings, 4, 3));
          Serial.print(F("Use (7) bits               : ")); r_7bits = row++; print_serial_flag(settings, 12); Serial.println();
          Serial.print(F("Serial input (u)ppercase   : ")); r_ucase = row++; print_serial_flag(settings, 10); Serial.println();
          Serial.print(F("Translate (B)ackspace to   : ")); r_bspace = row++; print_serial_flag_backspace(settings); Serial.println();
          if( dev==CSM_ACR )
            { Serial.print(F("Enable CLOAD/CSAVE (t)raps : ")); r_traps = row++; print_serial_flag(settings, 7, 1); Serial.println(); }
          
          Serial.println(F("\nE(x)it to main menu"));
          
          Serial.print(F("\n\nCommand: ")); r_cmd = row+4;
          redraw = false;
        }
      else
        set_cursor(r_cmd, 10);

      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'i':
          {
            bool ok = true;
            if( get_bits(settings, 17, 3)==7 )
              {
                ok = false;
                for(byte d=0; d<4 && !ok; d++)
                  if( d!=dev && get_bits(config_serial_device_settings[d], 17, 3)==7 )
                    ok = true;

                if( !ok )
                  {
                    Serial.println(F("\n\nCan not change mapping. At least one device must"));
                    Serial.println(F("be mapped to the host's (primary) serial interface."));
                    redraw = true;
                    delay(4000);
                  }
              }

            if( ok )
              {
                settings = toggle_map_to_serial(settings, 17);
                set_cursor(r_iface, col); 
                print_serial_device_mapped_to(settings);
              }

            break;
          }

        case 'b': 
          {
            settings = toggle_bits(settings, 0, 4, BAUD_110, BAUD_19200); 
            set_cursor(r_baud, col); 
            Serial.print(config_baud_rate(get_bits(settings, 0, 4)));
            break;
          }

        case 'F': 
          {
            settings = toggle_bits(settings, 16, 1); 
            print_flag(settings, 1ul<<16, r_force, col); 
            break;
          }

        case 'N': 
          {
            settings = toggle_bits(settings, 4, 3); 
            set_cursor(r_nuls, col); 
            Serial.println(get_bits(settings, 4, 3));
            break;
          }

        case '7': 
          {
            settings = toggle_serial_flag(settings, 12); 
            set_cursor(r_7bits, col); 
            print_serial_flag(settings, 12); 
            break;
          }

        case 'u': 
          {
            settings = toggle_serial_flag(settings, 10); 
            set_cursor(r_ucase, col); 
            print_serial_flag(settings, 10); 
            break;
          }

        case 'B': 
          {
            settings = toggle_serial_flag_backspace(settings); 
            set_cursor(r_bspace, col); 
            print_serial_flag_backspace(settings); 
            break;
          }

        case 't': 
          {
            settings = toggle_bits(settings, 7, 1); 
            set_cursor(r_traps, col); 
            print_serial_flag(settings, 7, 1); 
            break;
          }

        case 27:
        case 'x': 
          config_serial_device_settings[dev] = settings;
          serial_timer_interrupt_setup(dev);
          return;
        }
    }
}


// --------------------------------------------------------------------------------


void config_host_serial_details(byte iface)
{
  bool go = true, redraw = true;
  byte row, col, r_baud, r_bits, r_parity, r_stop, r_cmd;

  byte baud   = get_bits(new_config_serial_settings, config_baud_rate_bits(iface), 4);
  byte bits   = get_bits(new_config_serial_settings2, iface*5+3, 2) + 5;
  byte parity = get_bits(new_config_serial_settings2, iface*5+1, 2);
  byte stop   = get_bits(new_config_serial_settings2, iface*5  , 1) + 1;

  row = 4;
  col = 15;
  while( go )
    {
      if( redraw )
        {
          Serial.print(F("\033[2J\033[0;0H\n"));
#if HOST_NUM_SERIAL_PORTS>1
          Serial.print(F("Configure host serial settings for interface: "));
          Serial.print(host_serial_port_name(iface)); 
          Serial.println();
#else
          Serial.println(F("Configure host serial settings"));
#endif
          Serial.print(F("\n(B)aud rate : ")); Serial.println(config_baud_rate(baud)); r_baud = row++;
          Serial.print(F("(b)its      : ")); Serial.println(bits); r_bits = row++;
          Serial.print(F("(P)arity    : ")); print_parity(parity); Serial.println(); r_parity = row++;
          Serial.print(F("(S)top bits : ")); Serial.println(stop); r_stop = row++;

          Serial.println(F("\nE(x)it to parent menu"));
          row += 2;
          
          Serial.print(F("\n\nCommand: ")); r_cmd = row+2;
          redraw = false;
        }
      else
        set_cursor(r_cmd, 10);

      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
        case 'B': 
          {
            baud = toggle_host_serial_baud_rate(iface, baud);
            set_cursor(r_baud, col);
            Serial.print(config_baud_rate(baud)); Serial.print(F("    "));
            break;
          }

        case 'b':
          {
            bits = bits+1; 
            if( bits>8 ) bits = 5;
            set_cursor(r_bits, col);
            Serial.print(bits);
            break;
          }

        case 'P':
          {
            parity = (parity+1) % 3;
            set_cursor(r_parity, col);
            print_parity(parity); 
            break;
          }

        case 'S':
          {
            stop = stop + 1;
            if( stop>2 ) stop = 1;
            set_cursor(r_stop, col);
            Serial.print(stop);
            break;
          }            

        case 27:
        case 'x': 
          {
            go = false; 
            break;
          }
        }
    }

  byte config = (bits-5) * 8 + (parity * 2) + (stop-1);
  new_config_serial_settings  = set_bits(new_config_serial_settings, config_baud_rate_bits(iface), 4, baud);
  new_config_serial_settings2 = set_bits(new_config_serial_settings2, iface*5, 5, config);
}


// --------------------------------------------------------------------------------


void config_host_serial()
{
  int i;
  bool go = true, redraw = true;
  byte row, col, r_baud[HOST_NUM_SERIAL_PORTS], r_primary, r_cmd;

  col = 0;
#if HOST_NUM_SERIAL_PORTS>1
  for(i=0; i<HOST_NUM_SERIAL_PORTS; i++)
    col = max(col, strlen(host_serial_port_name(i)));
  col += 4;
#endif
  col += 28;

  while( go )
    {
      if( redraw )
        {
          Serial.print(F("\033[2J\033[0;0H\n"));
          Serial.println(F("Configure host serial settings\n"));
          
          row = 4;
          for(i=0; i<HOST_NUM_SERIAL_PORTS; i++)
            {
              r_baud[i] = row++;
              Serial.print('('); Serial.print(i); Serial.print(F(") Host Serial")); 
#if HOST_NUM_SERIAL_PORTS>1
              Serial.print(i);
              Serial.print(F(" (")); Serial.print(host_serial_port_name(i)); Serial.print(')');
#endif
              set_cursor(r_baud[i], col-12);
              Serial.print(F(" settings : "));
              print_host_serial_config(i, 0, 0);
              Serial.println();
            }
          
#if HOST_NUM_SERIAL_PORTS>1
          Serial.print(F("\n(P)rimary host serial : ")); print_host_primary_interface(); row++; r_primary = row++; Serial.println();
#endif

          Serial.println(F("\n(A)pply host serial settings")); row+=2;

          Serial.println(F("\nE(x)it to main menu"));
          row += 2;
          
          Serial.print(F("\n\nCommand: ")); r_cmd = row+2;
          redraw = false;
        }
      else
        set_cursor(r_cmd, 10);

      while( !serial_available() ) delay(50);
      char c = serial_read();
      if( c>31 && c<127 ) Serial.println(c);

      switch( c )
        {
#if HOST_NUM_SERIAL_PORTS>1
        case 'P': toggle_host_primary_interface(r_primary, 25); Serial.print(F("\033[K")); break;
#endif

        case 'A': apply_host_serial_settings(); redraw = true; break;

        case 27:
        case 'x': 
          {
            go = false; 
            break;
          }

        default:
          {
            i = c-'0';
            if( i>=0 && i<HOST_NUM_SERIAL_PORTS )
              {
                if( host_serial_port_has_configs(i) )
                  {
                    config_host_serial_details(i);
                    redraw = true;
                  }
                else
                  {
                    byte baud = get_bits(new_config_serial_settings, config_baud_rate_bits(i), 4);
                    baud = toggle_host_serial_baud_rate(i, baud);
                    new_config_serial_settings = set_bits(new_config_serial_settings, config_baud_rate_bits(i), 4, baud);
                    print_host_serial_config(i, r_baud[i], col);
                  }
              }
            break;
          }
        }
    }
}


// --------------------------------------------------------------------------------


void config_edit()
{
  new_config_serial_settings  = config_serial_settings;
  new_config_serial_settings2 = config_serial_settings2;

  // flush serial input
  while( serial_read()>=0 );

  bool redraw = true;

  config_mem_size = ((uint32_t) mem_get_ram_limit_usr())+1;
  byte row, col, r_profile, r_throttle, r_panel, r_debug, r_clearmem, r_aux1, r_cmd, r_memsize, r_input;
  while( true )
    {
      char c;

      if( redraw )
        {
          row = 2;
          col = 31;
          Serial.print(F("\033[2J\033[0;0H"));
          Serial.println();

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
          Serial.print(F("RAM size (+/-)              : ")); print_mem_size(config_mem_size, row, col); r_memsize = row++;
          Serial.print(F("A(u)x1 shortcut program     : ")); print_aux1_program(); Serial.println(); r_aux1 = row++;
          Serial.print(F("Configure host (s)erial     : ")); 
#if HOST_NUM_SERIAL_PORTS>1
          Serial.print(F("Primary interface is ")); print_host_primary_interface(); Serial.println(); row++;
#else
          print_host_serial_config(0, 0, 0); Serial.println(); row++;
#endif

          Serial.println(); row++;
          Serial.print(F("(1) Configure SIO           : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_SIO]); Serial.println(); row++;
          Serial.print(F("(2) Configure ACR           : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_ACR]); Serial.println(); row++;
          Serial.print(F("(3) Configure 2SIO port 1   : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO1]); Serial.println(); row++;
          Serial.print(F("(4) Configure 2SIO port 2   : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO2]); Serial.println(); row++;
#if USE_SECOND_2SIO>0
          Serial.print(F("(5) Configure 2SIO-2 port 1 : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO3]); Serial.println(); row++;
          Serial.print(F("(6) Configure 2SIO-2 port 2 : ")); print_serial_device_mapped_to(config_serial_device_settings[CSM_2SIO4]); Serial.println(); row++;
#endif
#if USE_PRINTER>0
          Serial.print(F("(P) Configure printer       : ")); 
          print_printer_type();
          if( config_printer_type()!=CP_NONE ) { Serial.print(F(" on ")); print_printer_mapped_to(); }
          Serial.println(); row++;
#endif
#if NUM_DRIVES>0
          Serial.print(F("(D) Configure disk drives   : ")); print_drive_mounted(); Serial.println(); row++;
#endif
#if NUM_HDSK_UNITS>0
          Serial.print(F("(H) Configure hard disks    : ")); print_hdsk_mounted(); Serial.println(); row++;
#endif
          Serial.print(F("(I) Configure interrupts    : ")); print_vi_flag(); Serial.println(); row++;
          row += 1;

          Serial.println();
          Serial.println(F("(M)anage Filesystem"));
#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0
          if( host_have_sd_card() )
            { Serial.println(F("(F)ile manager for SD card")); row++; }
#endif
          Serial.println(F("(C)lear memory"));
          Serial.println(F("(S)ave configuration"));
          Serial.println(F("(L)oad configuration"));
          Serial.println(F("(R)eset to defaults"));
          Serial.println(F("\nE(x)it"));
          row += 7;

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
        case 's': config_host_serial(); break;
          
        case '1': config_edit_serial_device(CSM_SIO); break;
        case '2': config_edit_serial_device(CSM_ACR); break;
        case '3': config_edit_serial_device(CSM_2SIO1); break;
        case '4': config_edit_serial_device(CSM_2SIO2); break;
#if USE_SECOND_2SIO>0
        case '5': config_edit_serial_device(CSM_2SIO3); break;
        case '6': config_edit_serial_device(CSM_2SIO4); break;
#endif
#if USE_PRINTER>0
        case 'P': config_edit_printer(); break;
#endif

        case 'I': config_edit_interrupts(); break;
#if NUM_DRIVES>0
        case 'D': config_edit_drives(); break;
#endif
#if NUM_HDSK_UNITS>0
        case 'H': config_edit_hdsk(); break;
#endif
        case 'h': 
          {
            Serial.println(F("\n\n"));
            host_system_info();
            Serial.print(F("\n\nPress any key to continue..."));
            while( !serial_available() ); 
            while( serial_available() ) serial_read();
            break;
          }
          
        case '-': 
          {
            if( config_mem_size > 1024 )
              config_mem_size -= 1024;
            else if( config_mem_size > 256 )
              config_mem_size -= 256;
            else
              config_mem_size = MEMSIZE;

            print_mem_size(config_mem_size, r_memsize, col);
            redraw = false;
            break;
          }

        case '+': 
          {
            if( config_mem_size < 1024 )
              config_mem_size += 256;
            else if( config_mem_size < MEMSIZE )
              config_mem_size += 1024;
            else
              config_mem_size = 256;

            print_mem_size(config_mem_size, r_memsize, col);
            redraw = false;
            break;
          }

        case 'M': filesys_manage(); break;

#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0
        case 'F': if( host_have_sd_card() ) sd_manager(); break;
#endif

        case 'C': 
          {
            for(uint16_t i=0; i<mem_ram_limit; i++) Mem[i] = 0; 
            break;
          }

        case 'S': 
          {
            Serial.print(F("\n\nSave as config # (0=default): "));
            bool esc = false;
            byte i = (byte) numsys_read_word(&esc);
            Serial.println();
            if( !esc )
              if( !save_config(i & 0xff) )
                Serial.println(F("Saving failed. Capture/replay in progress?"));

            break;
          }
        case 'L':
          {
            Serial.print(F("\n\nLoad config #: "));
            bool esc = false;
            byte i = (byte) numsys_read_word(&esc);
            Serial.println();
            if( !esc )
              if( !load_config(i & 0xff) )
                {
                  Serial.println(F("Load failed. File does not exist?"));
                  delay(2000);
                }
            break;
          }
        case 'R': config_defaults(false); break;

        case 27:
        case 'x':
          {
            bool exit = false;
            if( (config_serial_settings&0xFFF7FF) != (new_config_serial_settings&0xFFF7FF) ||
                (config_serial_settings2) != (new_config_serial_settings2) )
              {
                Serial.print(F("\nApply new host serial settings (y/n/ESC)? "));
                do { delay(50); c = serial_read(); } while( c!='y' && c!='n' && c!=27 );
                Serial.println(c);
                if( c=='n' || (c=='y' && apply_host_serial_settings()) )
                  exit = true;
              }
            else 
              exit = true;

            if( exit )
              {
                mem_set_ram_limit_usr(config_mem_size-1);
                Serial.print(F("\033[2J\033[0;0H"));
                return;
              }
            break;
          }

        default:
          redraw = false;
          break;
        }
    }
}


void config_defaults(bool apply)
{
  byte i, j;
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
  new_config_serial_settings |= (0 << 8); // USB Programming port is primary interface

  // USB ports 115200 baud, serial interfaces 9600 baud
  for(i=0; i<HOST_NUM_SERIAL_PORTS; i++) 
    if( strstr(host_serial_port_name(i), "USB") || HOST_NUM_SERIAL_PORTS==1 )
      new_config_serial_settings |= (BAUD_115200 << (config_baud_rate_bits(i)));
    else
      new_config_serial_settings |= (BAUD_9600 << (config_baud_rate_bits(i)));

  // serial configuration defaults to 8N1
  new_config_serial_settings2 = 0;
  for(i=0; i<HOST_NUM_SERIAL_PORTS; i++) 
    new_config_serial_settings2 |= 0x18ul << (i*5);

  uint32_t s = 0;
  s |= (BAUD_1200 <<  0); // serial playback baud rate: 1200
  s |= (4         <<  4); // 4 NUL characters after newline
  s |= (CSF_AUTO  << 10); // autodetect uppercase inputs
  s |= (CSF_AUTO  << 12); // autodetect 7 bit 
  s |= (CSFB_NONE << 14); // no backspace translation
  s |= (0l        << 17); // not mapped to any host interface

  for(byte dev=0; dev<NUM_SERIAL_DEVICES; dev++)
    config_serial_device_settings[dev] = s;

#if HOST_NUM_SERIAL_PORTS>1
  config_serial_device_settings[CSM_SIO]   |= (7l << 17); // map to SIO to primary host interface
  config_serial_device_settings[CSM_2SIO1] |= (7l << 17); // map to 2SIO-1 to primary host interface
#else
  config_serial_device_settings[CSM_SIO]   |= (1l << 17); // map to SIO to host interface
  config_serial_device_settings[CSM_2SIO1] |= (1l << 17); // map to 2SIO-1 to host interface
#endif
  config_serial_device_settings[CSM_ACR]   |= (1 << 7);  // enable CLOAD traps

  config_interrupt_vi_mask[0] = INT_DRIVE;
  config_interrupt_vi_mask[1] = INT_RTC;
  config_interrupt_vi_mask[2] = INT_2SIO1 | INT_2SIO2;
  config_interrupt_vi_mask[3] = INT_2SIO3 | INT_2SIO4;
  config_interrupt_vi_mask[4] = 0;
  config_interrupt_vi_mask[5] = 0;
  config_interrupt_vi_mask[6] = 0;
  config_interrupt_vi_mask[7] = 0;

  config_interrupt_mask = INT_SIO | INT_2SIO1;
  
  config_aux1_prog = prog_find("16k ROM Basic");

  drive_set_realtime((config_flags & CF_DRIVE_RT)!=0);
  hdsk_set_realtime((config_flags & CF_DRIVE_RT)!=0);
  for(i=0; i<NUM_DRIVES; i++) drive_unmount(i);
  for(i=0; i<NUM_HDSK_UNITS; i++) 
    for(j=0; j<4; j++)
      hdsk_unmount(i, j);

  // maximum amount of RAM supported by host
  config_mem_size = MEMSIZE; 

  if( apply ) 
    {
      apply_host_serial_settings(new_config_serial_settings, new_config_serial_settings2);
      mem_set_ram_limit_usr(config_mem_size-1);
    }
}


void config_setup(byte n)
{
  config_defaults(true);
  if( load_config(n) )
    {
      apply_host_serial_settings(new_config_serial_settings, new_config_serial_settings2);
      mem_set_ram_limit_usr(config_mem_size-1);
    }
}
