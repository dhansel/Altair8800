// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2017 David Hansel
// Copyright (C) 2020 Dirk Herrendoerfer
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

// NOTE:
// Teensy host support implemented by Dirk Herrendoerfer. For the necessary
// hardware setup see: https://easyeda.com/dherrendoerfer/altair8800again


#ifdef __MK66FX1M0__ // Teensy 3.6


//Define this, if you are using the AIO board. This board has a different LED numbering
#define AIO 1

#include <Arduino.h>
#include "Altair8800.h"
#include "config.h"
#include "host_teensy36.h"
#include "mem.h"
#include "cpucore.h"
#include "serial.h"
#include "timer.h"
#include "dazzler.h"

#if USE_DAZZLER>0
#error Teensy currently does not support DAZZLER emulation
#endif

#if USE_VDM1>0
#error Teensy currently does not support VDM-1 emulation
#endif

#if USE_IO_BUS>0
#error Teensy does not support I/O bus
#endif

#if USE_REAL_MREAD_TIMING>0
#error Teensy port does not support USE_REAL_MREAD_TIMING
#endif

// Altair8800Again! video out
#include <uVGA.h>
#define UVGA_180M_360X300
#include <uVGA_valid_settings.h>
UVGA_STATIC_FRAME_BUFFER(uvga_fb);
uVGA uvga;

// Altair8800Again! keyboard
#include "host_teensy_kbd.h"
#include "host_teensy_kbd_charmap.h"

#include "host_teensy_vt100.h"

#include <SPI.h>
#include <SdFat.h>

// un-define Serial which was #define'd to SwitchSerialClass in switch_serial.h
// otherwise we get infinite loops when calling Serial.* functions below
#undef Serial

extern "C" char* sbrk(int incr);
int FreeRam() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

static SdFatSdio SD;

#define min(x, y) ((x)<(y) ? (x) : (y))
#define max(x, y) ((x)>(y) ? (x) : (y))

// Altair8800Again! LEDs
#include <WS2812Serial.h>

static IntervalTimer updateTimer;

volatile uint8_t kbd_state = 10;
volatile uint8_t switch_bank0,switch_bank1,switch_bank2,switch_bank3;
volatile uint8_t state_bank0,state_bank1,state_bank2,state_bank3;
volatile uint8_t rising_edge_bank2,rising_edge_bank3;
volatile boolean switch_change;

#ifdef AIO
#define NUMBER_OF_LEDS 36 
#else
#define NUMBER_OF_LEDS 42
#endif
#define LEDS_PIN 10

DMAMEM byte displayMemory[NUMBER_OF_LEDS*12]; // 12 bytes per LED
byte drawingMemory[NUMBER_OF_LEDS*3];         //  3 bytes per LED
WS2812Serial leds(NUMBER_OF_LEDS, displayMemory, drawingMemory, LEDS_PIN, WS2812_GRB);

volatile uint16_t addr_led_local;
volatile uint16_t status_led_local;
volatile uint16_t data_led_local;
#ifdef LEDLOW
volatile uint16_t addr_led_local_low;
volatile uint16_t status_led_local_low;
volatile uint16_t data_led_local_low;
#endif

// Display the system status on VGA ?
volatile boolean vga_sysmon = true; 


#ifdef AIO
static const uint8_t address_to_LED[] = {18,19,20,21,22,23,24,25,26,    27,28,29,30,31,32,    33};
static const uint8_t data_to_LED[]    = {17,16,15,14,13,12,11,10};
static const uint8_t status_to_LED[]  = {9,8,7,6,5,4,3,2,1,0,34,35};
#else
static const uint8_t address_to_LED[] = {21,22,23,24,25,26,27,28,29,    31,32,33,34,35,36,    38};
static const uint8_t data_to_LED[]    = {20,19,18,17,16,15,14,13};
static const uint8_t status_to_LED[]  = {9,8,7,6,5,4,3,2,1,0,40,41};
#endif


// Include audio tape support 
#include "host_teensy_tape.h"


/*
  NOTE:
  Change -Os to -O3 (to switch optimization from size to performance) in:
  c:\Users\[user]\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.9\platform.txt

  ---- front panel connections by function:

  For pins that are not labeled on the board with their digital number
  the board label is given in []

  Function switches:
     RUN          => DATA7 @ ROW3
     STOP         => DATA7 @ ROW4
     STEP         => DATA6 @ ROW4
     SLOW         => DATA6 @ ROW3
     EXAMINE      => DATA5 @ ROW4
     EXAMINE NEXT => DATA5 @ ROW4
     DEPOSIT      => DATA4 @ ROW4
     DEPOSIT NEXT => DATA4 @ ROW3
     RESET        => DATA3 @ ROW4
     CLR          => DATA3 @ ROW3
     PROTECT      => DATA2 @ ROW4
     UNPROTECT    => DATA2 @ ROW3
     AUX1 UP      => DATA1 @ ROW4
     AUX1 DOWN    => DATA1 @ ROW3
     AUX2 UP      => DATA0 @ ROW4
     AUX2 DOWN    => DATA3 @ ROW3

   Address switches:
     SW0...7      => DATA & ROW1
     SW8...15     => DATA @ ROW2

   Bus LEDs:
     A0..7        => 21 - 28
     A8..15       => 30, 32 - 37, 39
     D0..8        => 20 - 13

   Status LEDs:
     INT          => 9
     WO           => 8
     STACK        => 7
     HLTA         => 6
     OUT          => 5
     M1           => 4
     INP          => 3
     MEMR         => 2
     PROT         => 1
     INTE         => 0
     WAIT         => 41
     HLDA         => 40


  ---- front panel connections by Teensy pin:

  DATA is GPIO Port B: (not all in bit order)
    0  => 16 (in)
    1  => 17 (in)
    2  => 19 (in)
    3  => 18 (in)
    16  => 0 (in)
    17  => 1 (in)
    18  => 31 (in)
    19  => 32 (in)

   ROW selects are:
    0 (ROW1) => 33 (out)
    1 (ROW2) => 24 (out)
    2 (ROW3) => 3 (out)
    3 (ROW4) => 4 (out)

*/


//------------------------------------------------------------------------------------------------------


uint16_t host_read_addr_switches()
{
  return (switch_bank1<<8) | switch_bank0;
}


//------------------------------------------------------------------------------------------------------


class HLDAGuard
{
public:
  HLDAGuard()  { m_hlda = (host_read_status_leds() & ST_HLDA)!=0; }
  ~HLDAGuard() { if( m_hlda ) host_set_status_led_HLDA(); else host_clr_status_led_HLDA(); }

private:
  bool m_hlda;
};

uint32_t sd_storagesize = 512*1024;

#define MOVE_BUFFER_SIZE 1024
byte moveBuffer[MOVE_BUFFER_SIZE];

static bool use_sd = false;
static File storagefile;

bool host_storage_init(bool write)
{
  host_storage_close();

  storagefile = SD.open("STORAGE.DAT", write ? FILE_WRITE : FILE_READ);
  if( storagefile )
    return true;
  else
    return false;
}


void host_storage_close()
{
  if( storagefile ) storagefile.close();
}


void host_storage_invalidate()
{
  host_storage_close();
  SD.remove("STORAGE.BAK");
  SD.rename("STORAGE.DAT", "STORAGE.BAK");
}


static void host_storage_write_sd(const void *data, uint32_t addr, uint32_t len)
{
  HLDAGuard hlda;
  if( host_filesys_file_seek(storagefile, addr) )
    {
      storagefile.write((byte *) data, len);
      storagefile.flush();
    }
}


static void host_storage_read_sd(void *data, uint32_t addr, uint32_t len)
{
  HLDAGuard hlda;
  if( storagefile.seek(addr) )
    storagefile.read((byte *) data, len);
}


void host_storage_write(const void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    host_storage_write_sd(data, addr, len);
}

void host_storage_read(void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    host_storage_read_sd(data, addr, len);
}


void host_storage_move(uint32_t to, uint32_t from, uint32_t len)
{
  uint32_t i;
  if( from < to )
    {
      for(i=0; i+MOVE_BUFFER_SIZE<len; i+=MOVE_BUFFER_SIZE)
        {
          host_storage_read(moveBuffer, from+len-i-MOVE_BUFFER_SIZE, MOVE_BUFFER_SIZE);
          host_storage_write(moveBuffer, to+len-i-MOVE_BUFFER_SIZE, MOVE_BUFFER_SIZE);
        }

      if( i<len )
        {
          host_storage_read(moveBuffer, from, len-i);
          host_storage_write(moveBuffer, to, len-i);
        }
    }
  else
    {
      for(i=0; i+MOVE_BUFFER_SIZE<len; i+=MOVE_BUFFER_SIZE)
        {
          host_storage_read(moveBuffer, from+i, MOVE_BUFFER_SIZE);
          host_storage_write(moveBuffer, to+i, MOVE_BUFFER_SIZE);
        }

      if( i<len )
        {
          host_storage_read(moveBuffer, from+i, len-i);
          host_storage_write(moveBuffer, to+i, len-i);
        }
    }
}


//------------------------------------------------------------------------------------------------------


File host_filesys_file_open(const char *filename, bool write)
{
  HLDAGuard hlda;
  return SD.open(filename, write ? FILE_WRITE : FILE_READ);
}


uint32_t host_filesys_file_read(File &f, uint32_t len, void *buffer)
{
  HLDAGuard hlda;
  return f.read((uint8_t *) buffer, len);
}


uint32_t host_filesys_file_write(File &f, uint32_t len, const void *buffer)
{
  HLDAGuard hlda;
  return f.write((const uint8_t *) buffer, len);
}


uint32_t host_filesys_file_set(File &f, uint32_t len, byte b)
{
  HLDAGuard hlda;
  uint32_t res = 0;

  // write data in MOVE_BUFFER_SIZE chunks
  memset(moveBuffer, b, min(len, MOVE_BUFFER_SIZE));
  for(uint32_t i=0; i<len; i+=MOVE_BUFFER_SIZE)
    res += f.write(moveBuffer, min(len-i, MOVE_BUFFER_SIZE));

  return res;
}


void host_filesys_file_flush(File &f)
{
  HLDAGuard hlda;
  f.flush();
}


bool host_filesys_file_seek(File &f, uint32_t pos)
{
  HLDAGuard hlda;

  f.seek(pos);
  if( f.position()<pos && !f.isReadOnly() )
    {
      // if we are seeking past the end of a writable
      // file then expand its size accordingly
      host_filesys_file_set(f, pos-f.position(), 0);
    }

  return f.position()==pos;
}


uint32_t host_filesys_file_pos(File &f)
{
  HLDAGuard hlda;
  return f.position();
}


bool host_filesys_file_eof(File &f)
{
  HLDAGuard hlda;
  return f.isReadOnly() ? f.available()==0 : false;
}


void host_filesys_file_close(File &f)
{
  HLDAGuard hlda;
  f.close();
}


uint32_t host_filesys_file_size(const char *filename)
{
  HLDAGuard hlda;
  int res = -1;

  File f = SD.open(filename, FILE_READ);
  if( f )
    {
      res = f.size();
      f.close();
    }

  return res;
}


bool host_filesys_file_exists(const char *filename)
{
  HLDAGuard hlda;
  return SD.exists(filename);
}


bool host_filesys_file_remove(const char *filename)
{
  HLDAGuard hlda;
  return SD.remove(filename);
}


bool host_filesys_file_rename(const char *from, const char *to)
{
  HLDAGuard hlda;
  return SD.rename(from, to);
}


File host_filesys_dir_open()
{
  HLDAGuard hlda;
  return SD.open("/");
}


const char *host_filesys_dir_nextfile(File &d)
{
  HLDAGuard hlda;
  static char buffer[15];

  while( true )
    {
      File entry = d.openNextFile();
      if( entry )
        {
          if( entry.isFile() )
            {
              entry.getSFN(buffer);
              entry.close();
              return buffer;
            }

          entry.close();
        }
      else
        return NULL;
    }
}


void host_filesys_dir_rewind(File &d)
{
  HLDAGuard hlda;
  d.rewindDirectory();
}


void host_filesys_dir_close(File &d)
{
  HLDAGuard hlda;
  d.close();
}


bool host_filesys_ok()
{
  return use_sd;
}


//------------------------------------------------------------------------------------------------------

void host_serial_setup(byte iface, uint32_t baud, uint32_t config, bool set_primary_interface)
{
  // switch the primary serial interface (if requested)
  if( set_primary_interface ) SwitchSerial.select(iface); 

  if( iface==1 )
    {
      Serial1.begin(baud, config);
      Serial1.setTimeout(10000);
    }
  if( iface==2 )
    {
      if (set_primary_interface) {
        vga_sysmon = false;
        uvga.println("\nMonitor disabled, terminal is primary");
      }
    }
  if( iface==3 )
    {
      //Tape
      
    }
}

void host_serial_end(byte i)
{
  switch( i )
    {
    case 1: Serial1.end(); break;
    }
}

int host_serial_available(byte i)
{
  switch( i )
    {
    case 0: return Serial.available(); break;
    case 1: return Serial1.available(); break;
    case 2: return uKbdC_available(); break;
    case 3: return a_serial_available(); break;
    }

 return 0;
}

int host_serial_available_for_write(byte i)
{
  switch( i )
    {
    case 0: return Serial.availableForWrite(); break;
    case 1: return Serial1.availableForWrite(); break;
    case 2: return 16; break;
    case 3: return a_serial_write_avail(); break;
    }

 return 0;
}

int host_serial_peek(byte i)
{
  switch( i )
    {
    case 0: return Serial.peek(); break;
    case 1: return Serial1.peek(); break;
    }

 return -1;
}

int host_serial_read(byte i)
{
  switch( i )
    {
    case 0: return Serial.read(); break;
    case 1: return Serial1.read(); break;
    case 2: return uKbdC_read(); break;
    case 3: return a_serial_read(); break;
    }

  return -1;
}

void host_serial_flush(byte i)
{
  switch( i )
    {
    case 0: Serial.flush(); break;
    case 1: Serial1.flush(); break;
    case 2: break;
    case 3: break; 
    }
}

size_t host_serial_write(byte i, uint8_t b)
{
  switch( i )
    {
    //case 0: uVt_write(b);return Serial.write(b); break;
    case 0: return Serial.write(b); break;
    case 1: return Serial1.write(b); break;
    case 2: if (vga_sysmon)
              vga_sysmon = false;
            return uVt_write(b); 
            break;
    case 3: return a_serial_write(b); break;
    }

  return 0;
}

size_t host_serial_write(byte i, const char *buf, size_t n)
{
  size_t a;
  switch( i )
    {
    case 0:
      {
        a = Serial.availableForWrite();
        if( a<n ) n = a;
        for(a=0; a<n; a++) Serial.write(buf[a]);          
        return a;
      }

    case 1:
      {
        a = Serial1.availableForWrite();
        if( a<n ) n = a;
        for(a=0; a<n; a++) Serial1.write(buf[a]);
        return a;
      }
    case 2:
      {
        if (vga_sysmon)
          vga_sysmon = false;

        for(a=0; a<n; a++) uVt_write(buf[a]);
        return a;
      }
    case 3:
      {
        a = a_serial_write_avail();
        if( a<n ) n = a;
        for(a=0; a<n; a++) a_serial_write(buf[a]);
        return a;
      }
    }

  return 0;
}

bool host_serial_ok(byte i)
{
  switch( i )
    {
    case 0: return (bool) Serial; break;
    case 1: return (bool) Serial1; break;
    case 2: return true; break;
    case 3: return true; break;
    }

  return false;
  }


const char *host_serial_port_name(byte i)
{
  switch(i)
    {
    case 0: return "USB Programming Port";
    case 1: return "Serial1 (pin 26/27)";
    case 2: return "VGA Terminal";
    case 3: return "Teensy Tape emulation";
    default: return "???";
    }
}


bool host_serial_port_baud_limits(byte i, uint32_t *min, uint32_t *max)
{
  switch(i)
    {
    case 0: *min = 600;    *max = 1050000; break;
    case 1: *min = 110;    *max = 1050000; break;
    case 2: *min = 110;    *max = 1050000; break;
    case 3: *min = 300;    *max = 300; break;
    default: return false;
    }

  return true;
}


bool host_serial_port_has_configs(byte i)
{
  return i==1 || i==3 || i==4;
}

void host_check_interrupts() 
{ 
  if( Serial.available() ) 
    serial_receive_host_data(0, Serial.read());

  if( Serial1.available() ) 
    serial_receive_host_data(1, Serial1.read());

  if( uKbdC_available() ) 
    serial_receive_host_data(2, uKbdC_read());

  if( a_serial_available() ) 
    serial_receive_host_data(3, a_serial_read());

}

void host_serial_interrupts_pause()
{
  return;
}

void host_serial_interrupts_resume()
{
  return;
}

// --------------------------------------------------------------------------------------------------
// Host Terminal functions

void host_vga_write(uint8_t c)
{
  uvga.write(c);  
}

void host_vga_setpos(uint8_t x, uint8_t y)
{
  uvga.moveCursor(x, y);
}

void host_vga_clear()
{
  uvga.clear();
}


// --------------------------------------------------------------------------------------------------

static const uint16_t function_switch_id[16] = {0x80,0x8000,0x4000,0x40,0x2000,0x20,0x1000,0x10,0x800,0x8,0x400,0x4,0x200,0x2,0x100,0x1};

static const uint8_t function_switch_bank2_irq[16] = { INT_SW_AUX2DOWN>>24, 0, 0, INT_SW_CLR>>24, 0, 0, 0, 0 };
static const uint8_t function_switch_bank3_irq[16] = { INT_SW_AUX2UP>>24, 0, 0, INT_SW_RESET>>24, 0, 0, 0, INT_SW_STOP>>24};

bool host_read_function_switch(byte i)
{
  int index = function_switch_id[i];
  if (index > 0xff)
    return (switch_bank3 & (index>>8));

  return (switch_bank2 & (index));
}

void do_altair_interrupt_bank2(byte i)
{
  if (function_switch_bank2_irq[i] != 0)
    altair_interrupt(function_switch_bank2_irq[i]<<24);
}

void do_altair_interrupt_bank3(byte i)
{
  if (function_switch_bank3_irq[i] != 0)
    altair_interrupt(function_switch_bank3_irq[i]<<24);
}


bool host_read_function_switch_debounced(byte i)
{
  return host_read_function_switch(i);
}


bool host_read_function_switch_edge(byte i)
{
  //
  // Note: reading an edge switch event also clears the bit
  //
  int index = function_switch_id[i];
  boolean ret;
  if (index > 0xff){
    ret = (rising_edge_bank3 & (index>>8));
    if (ret)
      rising_edge_bank3 &= ~(index>>8);
    return ret;
  }

  ret = (rising_edge_bank2 & (index));
  if (ret)
    rising_edge_bank2 &= ~(index);
  return ret;
}


uint16_t host_read_function_switches_edge()
{
  uint16_t res = 0;

  for (int i=0 ; i<16 ; i++) {
    if (host_read_function_switch_edge(i))
      res |= 1<<i;
  }
  return res;
}


void host_reset_function_switch_state()
{
  switch_change = false;
  rising_edge_bank2 = 0;
  rising_edge_bank3 = 0;
}


uint8_t host_read_sense_switches()
{
  return switch_bank1;
}


// --------------------------------------------------------


void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len)
{
  memcpy(dst, src, len);
}


uint32_t host_get_random()
{
  delayMicroseconds(1);
  return (((uint32_t) random(0,65535)) * 65536l | random(0,65535));
}


bool host_is_reset()
{
  return host_read_function_switch(SW_RESET);
}


void host_system_info()
{
  SwitchSerial.println("Host is Teensy\n");
  SwitchSerial.println("Teensy host support implemented by Dirk Herrendoerfer:");
  SwitchSerial.println("https://github.com/dherrendoerfer/altair-8800-again");

  SwitchSerial.print("Free RAM     : "); SwitchSerial.println(FreeRam());
  SwitchSerial.print("Data storage : ");
#if USE_HOST_FILESYS>0
  SwitchSerial.print("SD card file system");
  if( SD.card()->errorCode()==SD_CARD_ERROR_NONE )
    { SwitchSerial.print(" ("); SwitchSerial.print(SD.card()->cardSize() / (2*1024)); SwitchSerial.println("M)"); }
  else
    { SwitchSerial.print(" (card error: 0x"); SwitchSerial.print(SD.card()->errorCode(), HEX); SwitchSerial.println(")"); }
#else
  SwitchSerial.print(storagefile ? "STORAGE.DAT file on SD card" : "flash memory");
  SwitchSerial.print(" ("); SwitchSerial.print(sd_storagesize / 1024); SwitchSerial.print("K)");
#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0
  if( SD.card()->errorCode()!=SD_CARD_ERROR_NONE )
    { SwitchSerial.print(" (SD card error: 0x"); SwitchSerial.print(SD.card()->errorCode(), HEX); SwitchSerial.print(")"); }
  SwitchSerial.println();
#endif
#endif
}

// -----------------------------------------------------------------------------


static void updateAddressLEDs()
{
  for (int i=0;i<16;i++) {
    uint8_t led=address_to_LED[i];
    if (addr_led_local & (1<<i))
      leds.setPixel(led, LEDCOLOR);
    else
#ifdef LEDLOW    
      if (addr_led_local_low & (1<<i))
        leds.setPixel(led, LEDLOW);
      else
#endif
        leds.setPixel(led, LEDOFF);
  }
#ifdef LEDLOW    
  addr_led_local_low = 0;
#endif
}

static void updateDataLEDs()
{
  for (int i=0;i<8;i++) {
    uint8_t led=data_to_LED[i];
    if (data_led_local & (1<<i))
      leds.setPixel(led,LEDCOLOR);
    else
#ifdef LEDLOW    
      if (data_led_local_low & (1<<i))
        leds.setPixel(led,LEDLOW);
      else
#endif
        leds.setPixel(led,LEDOFF);
  }
#ifdef LEDLOW
  data_led_local_low = 0;
#endif
}

static void updateStatusLEDs()
{
  for (int i=0;i<13;i++) {
    uint8_t led=status_to_LED[i];
    if (status_led_local & (1<<i))
      leds.setPixel(led,LEDCOLOR);
    else
#ifdef LEDLOW    
      if (status_led_local_low & (1<<i))
        leds.setPixel(led, LEDLOW);
      else
#endif
      leds.setPixel(led,LEDOFF);
  }
#ifdef LEDLOW    
  status_led_local_low = 0;
#endif
}

// -----------------------------------------------------------------------------
void monitor_panel()
{
  uvga.clear();
  uvga.println("                ALTAIR 8800");
  uvga.println("     ----STATUS-----      ------DATA-----");
  uvga.println("");
  uvga.println(" I P M I M O H S I W      D D D D D D D D");
  uvga.println(" N R E N 1 U L T N O      7 6 5 4 3 2 1 0");
  uvga.println(" T O M P   T T A T");
  uvga.println(" E T R       A C");
  uvga.println("               K ");
  uvga.println("");
  uvga.println("        --------------ADDRESS------------");
  uvga.println("");
  uvga.println(" W H    A A A A A A A A   A A A A A A A A");
  uvga.println(" A L    1 1 1 1 1 1 9 8   7 6 5 4 3 2 1 0");
  uvga.println(" I D    5 4 3 2 1 0");
  uvga.println(" T A");
  uvga.println("");
  uvga.println("        -------------SWITCHES------------");
  uvga.println("");
  uvga.println("        S S S S S S S S   S S S S S S S S");
  uvga.println("        1 1 1 1 1 1 9 8   7 6 5 4 3 2 1 0");
  uvga.println("        5 4 3 2 1 0");
  uvga.println("");
  uvga.println("");
  uvga.println(" Tape stats: recv    err_frm  err_tim");
}

static void m_led(uint8_t posx, uint8_t posy, uint16_t on)
{
  uint16_t x = 3 + 8*posx;  
  uint16_t y = 3 + 8*posy;

  if (on) {
    uvga.drawPixelNoSync(x,y,0xE0);
    uvga.drawPixelNoSync(x+1,y,0xE0);
    uvga.drawPixelNoSync(x,y+1,0xE0);
    uvga.drawPixelNoSync(x+1,y+1,0xE0);
    uvga.drawPixelNoSync(x+2,y,0xE0);
    uvga.drawPixelNoSync(x+2,y+1,0xE0);
  }
  else {
    uvga.drawPixelNoSync(x,y,0x00);
    uvga.drawPixelNoSync(x+1,y,0x00);
    uvga.drawPixelNoSync(x,y+1,0x00);
    uvga.drawPixelNoSync(x+1,y+1,0x00);
    uvga.drawPixelNoSync(x+2,y,0x00);
    uvga.drawPixelNoSync(x+2,y+1,0x00);
  }
}
static const uint8_t m_status_to_LED[] = {19,17,15,13,11,9,7,5,3,1,3,1};
static const uint8_t m_address_to_LED[] = {40,38,36,34,32,30,28,26,  22,20,18,16,14,12,10,8};

static void m_led_update_status_LEDs()
{
  uint8_t i;
  for (i=0;i<10;i++)
    m_led(m_status_to_LED[i], 2, status_led_local & (1<<i));
  for (i=10;i<12;i++)
    m_led(m_status_to_LED[i], 10, status_led_local & (1<<i));

  for (i=0;i<8;i++) {
    m_led(m_address_to_LED[i], 2, data_led_local & (1<<i));
    m_led(m_address_to_LED[i], 10, addr_led_local & (1<<i));
    m_led(m_address_to_LED[i+8], 10, addr_led_local & (1<<(i+8)));
  }

}
static void m_led_update_switch_LEDs()
{
  uint8_t i;
  for (i=0;i<8;i++) {
    m_led(m_address_to_LED[i], 17, switch_bank0 & (1<<i));
    m_led(m_address_to_LED[i+8], 17, switch_bank1 & (1<<i));
  }
}

// -----------------------------------------------------------------------------

// ---- The panel works differently with Teensy: since we don't have enough pins and
//   ports to connect every LED and SWITCH, we multiplex the switches and use addressable
//   LEDs to display. This is being driven by a timer-fed interrupt
//   The pannel uses a 4x8 matrix to read the SWITCHES:
//   4 rows are selected and pulled to ground one-by-one and connect commonly to 8 switches.
//   8 switches are connected to 8 input ports on the teensy. These are read when a row is
//   selected. Diodes must be used on each line from switch to port to prevent ghosting.
//   ROW1 is ADDRESS0-7
//   ROW2 is ADDRESS8-15 
//   ROW3 is FUNCTION SWITCHES DOWN (AUX2, AUX1, PROTECT, RESET, DEPOSIT, EXAMINE, SINGLE_STEP, STOP)
//   ROW4 is FUNCTION SWITCHES UP 

void panelUpdate()
{
  uint32_t tmp;
  switch(kbd_state) {
    case 10: //Setup: Set all ROWs high
      digitalWriteFast(33,HIGH);
      digitalWriteFast(24,HIGH);
      digitalWriteFast(3,HIGH);
      digitalWriteFast(4,HIGH);
      kbd_state=0;
      break;    
    case 0: //ROW 1 LOW
      digitalWriteFast(4,HIGH); digitalWriteFast(33,LOW);
      break;
    case 1: //READ Switches 0-7
      tmp=~GPIOB_PDIR;
      switch_bank0 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      if (state_bank0 != switch_bank0) {
        state_bank0 = switch_bank0;
        switch_change = true;  
      }
      break;
    case 2: //ROW 2 LOW
      digitalWriteFast(33,HIGH); digitalWriteFast(24,LOW);
      break;
    case 3: //READ Switches 8-15
      tmp=~GPIOB_PDIR;
      switch_bank1 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      if (state_bank1 != switch_bank1) {
        state_bank1 = switch_bank1;
        switch_change = true;  
      }
      break;
    case 4: //ROW 2 LOW
      digitalWriteFast(24,HIGH); digitalWriteFast(3,LOW);
      break;
    case 5: //READ Function Switches 0-7 Down position
      tmp=~GPIOB_PDIR;
      switch_bank2 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      rising_edge_bank2 = 0; 
      if (state_bank2 != switch_bank2) {
        for (int i=0;i<8;i++) { 
          if((switch_bank2 & (1<<i)) != (state_bank2 & (1<<i))){
            if (switch_bank2 & (1<<i)) {
              rising_edge_bank2 += 1<<i;
              do_altair_interrupt_bank2(i);
            }
          }
        }
        state_bank2 = switch_bank2;
        switch_change = true;  
      }
      break;
    case 6: //ROW 2 LOW
      digitalWriteFast(3,HIGH); digitalWriteFast(4,LOW);
      break;
    case 7: //READ Function Switches 0-7 Down position
      tmp=~GPIOB_PDIR;
      switch_bank3 = ((tmp & 0x030000)>>12) | ((tmp & 0x0C00)>>4) | (tmp & 0x0f);
      rising_edge_bank3 = 0;
      if (state_bank3 != switch_bank3) {
        for (int i=0;i<8;i++) { 
          if((switch_bank3 & (1<<i)) != (state_bank3 & (1<<i))) {
            if (switch_bank3 & (1<<i)) {
              rising_edge_bank3 += 1<<i;
              do_altair_interrupt_bank3(i);
            }
          }
        }
        state_bank3 = switch_bank3;
        switch_change = true;  
      }
      break;
  }  
  kbd_state++;
  if (kbd_state == 8) {
    kbd_state=0;

    if (vga_sysmon) {
      m_led_update_switch_LEDs();     
      host_vga_setpos(13,24);
      uvga.print(a_stat_read);
      host_vga_setpos(21,24);
      uvga.println(a_stat_frame_err);
      host_vga_setpos(30,24);
      uvga.println(a_stat_timeout_err);
    }

    return;
  }

  //This is the update for every cycle
  updateStatusLEDs();
  updateAddressLEDs();
  updateDataLEDs();
  leds.show();
  if(vga_sysmon)
    m_led_update_status_LEDs();

  //read the keyboard, then send the event to the charmapper
  //if an event is completed it becomes available in uKbdC_available();
  if (uKbd_available()){
    uint8_t code;
    code = uKbd_read();
    uKbdC_send_scancode(code);
  }
  
  //Tape DEBUG
//  if ( a_serial_available())
//    SwitchSerial.print((char)a_serial_read());

}

// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------

// Make sure we don't have an object in the middle of the 
// boundary change
bool reserveSramBoundary() {
  void* locks[1000];
  uint8_t ct = 0;

  while(ct<1000) {
    locks[ct] = malloc(64);
  
    if ((uintptr_t)locks[ct] >= 0x20000000u ){
      for (int i=0; i<ct-1; i++){
        free(locks[i]);
      }
      return true;
      break;
    }
    ct++;
    delay(50);
  }

  for (int i=0; i<ct; i++){
    free(locks[i]);
  }

  return false;
}

void host_setup()
{
  int ret;
  if(reserveSramBoundary())
  {
    if (Serial) SwitchSerial.println("mem-quirk");    
  }

  //Keyboard init
  if (uKbd_start()){
    if (Serial) SwitchSerial.println("no keyboard");
  }

  //VGA init
  uvga.set_static_framebuffer(uvga_fb);
  ret=uvga.begin(&modeline);
  if (ret != 0) {
    if (Serial) SwitchSerial.println("VGA init failed.");
    vga_sysmon = false;
  }
  else {
    monitor_panel();
  }

  //On-board LED
  pinMode(13,OUTPUT);
  
  //Reroute serial1
  pinMode(26,OUTPUT);
  pinMode(27,INPUT);
  Serial1.end(); //Stop, so it can be rerouted
  Serial1.setTX(26);
  Serial1.setRX(27);

  // Set in/out for pins
  //PORT A, Switch rows
  pinMode(33,OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  //Port B, Switch inputs
  pinMode(16,INPUT_PULLUP);
  pinMode(17,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(18,INPUT_PULLUP);
  pinMode(0,INPUT_PULLUP);
  pinMode(1,INPUT_PULLUP);
  pinMode(31,INPUT_PULLUP);
  pinMode(32,INPUT_PULLUP);

  //Analog/Sound out
  analogWriteResolution(12);
  analogWrite(A21,2048);

  if (Serial) SwitchSerial.println("PRE Timer");

  //Start update timer
  updateTimer.begin(panelUpdate, 10000);

  if (Serial) SwitchSerial.println("PRE Tape");
  //Start tape update timer
  tapeTimer.priority(32);
  tapeTimer.begin(tapeUpdate, 50);

  if (Serial) SwitchSerial.println("PRE LED");
  //Init non-blocking WS2812b lib
  leds.begin();

  if (Serial) SwitchSerial.println("PRE SD");
#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0 || USE_HOST_FILESYS>0
  // check if SD card available (send "chip select" signal to HLDA status light)
  HLDAGuard hlda;
  if( SD.begin()) {
    // storing configurations etc directly on SD card
    use_sd = host_storage_init(true);
  }
#endif

  // Give random a spin
  randomSeed(analogRead(A22));
}

#endif
