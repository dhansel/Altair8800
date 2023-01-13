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

#ifdef __SAM3X8E__

#include <Arduino.h>
#include <DueFlashStorage.h>
#include "Altair8800.h"
#include "config.h"
#include "host_due.h"
#include "mem.h"
#include "cpucore.h"
#include "serial.h"
#include "timer.h"
#include "dazzler.h"
#include "soft_uart.h"

#include <SPI.h>
#include <SdFat.h>

static SdFat SD;

#define min(x, y) ((x)<(y) ? (x) : (y))
#define max(x, y) ((x)>(y) ? (x) : (y))


// The SerialUSB implementation (serial via native USB port) sends the data for
// each "write" call in its own frame. Since most data in the simulator is sent
// byte-by-byte, each byte is sent in a separate USB frame. That is not so much
// a problem for hosts supporting High Speed (USB 2.0) but for hosts that only
// support Full Speed mode, waiting for a new frame for each byte of data to send
// can get slow (similar to a 2400 baud connection). Enabling USE_NATIVE_USB_TX_OPTIMIZATION
// will buffer characters sent while waiting for the next USB frame and then send 
// them all together.
// IMPORTANT: this MUST be enabled when using PIC32-based Dazzler or VDM-1 over USB
// because the SerialUSB implementation does not properly support full-speed devices,
// specifically it always sends data in 512-byte blocks (high-speed size) instead of
// using 64-byte block sizes for full-speed devices.
#define USE_NATIVE_USB_TX_OPTIMIZATION


// un-define Serial which was #define'd to SwitchSerialClass in switch_serial.h
// otherwise we get infinite loops when calling Serial.* functions below
#undef Serial


/*
  NOTE:
  Change -Os to -O3 (to switch optimization from size to performance) in:
  c:\Users\[user]\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.9\platform.txt

  ---- front panel connections by function:

  For pins that are not labeled on the board with their digital number
  the board label is given in []

  Function switches:
     RUN          => D20 (PIOB12)
     STOP         => D21 (PIOB13)
     STEP         => D54 [A0] (PIOA16)
     SLOW         => D55 [A1] (PIOA24)
     EXAMINE      => D56 [A2] (PIOA23)
     EXAMINE NEXT => D57 [A3] (PIOA22)
     DEPOSIT      => D58 [A4] (PIOA6)
     DEPOSIT NEXT => D59 [A5] (PIOA4)
     RESET        => D52 (PIOB21)
     CLR          => D53 (PIOB14)
     PROTECT      => D60 [A6] (PIOA3)
     UNPROTECT    => D61 [A7] (PIOA2)
     AUX1 UP      => D30 (PIOD9)
     AUX1 DOWN    => D31 (PIOA7)
     AUX2 UP      => D32 (PIOD10)
     AUX2 DOWN    => D33 (PIOC1)

   Address switches:
     SW0...7      => D62 [A8], D63 [A9], D64 [A10], D65 [A11], D66 [DAC0], D67 [DAC1], D68 [CANRX], D69 [CANTX]
     SW8...15     => D17,D16,D23,D24,D70[SDA1],D71[SCL1],D42,D43  (PIOA, bits 12-15,17-20)

   Bus LEDs:
     A0..7        => 34, 35, ..., 41          (PIOC, bits 2-9)
     A8..15       => 51, 50, ..., 44          (PIOC, bits 12-19)
     D0..8        => 25,26,27,28,14,15,29,11  (PIOD, bits 0-7)

   Status LEDs:
     INT          => D2  (PIOB25)
     WO           => D3  (PIOC28)
     STACK        => D4  (PIOC26)
     HLTA         => D5  (PIOC25)
     OUT          => D6  (PIOC24)
     M1           => D7  (PIOC23)
     INP          => D8  (PIOC22)
     MEMR         => D9  (PIOC21)
     INTE         => D12 (PIOD8)
     PROT         => D13 (PIOB27)
     WAIT         => D10 (PIOC29)
     HLDA         => D22 (PIOB26)


  ---- front panel connections by Arduino pin:

    D0  => Serial0 RX (in)
    D1  => Serial0 TX (out)
    D2  => INT        (out)
    D3  => WO         (out)
    D4  => STACK      (out)
    D5  => HLTA       (out)
    D6  => OUT        (out)
    D7  => M1         (out)
    
    D8  => INP        (out)
    D9  => MEMR       (out)
    D10 => WAIT       (out)
    D11 => D7         (out)
    D12 => INTE       (out)
    D13 => PROT       (out)

    D14 => D4         (out)
    D15 => D5         (out)
    D16 => SW9        (in)
    D17 => SW8        (in)
    D18 => Serial1 TX (out)
    D19 => Serial1 RX (in)
    D20 => RUN        (in)
    D21 => STOP       (in)
    
    D22 => HLDA       (out)
    D23 => SW10       (in)
    D24 => SW11       (in)
    D25 => D0         (out)
    D26 => D1         (out)
    D27 => D2         (out)
    D28 => D3         (out)
    D29 => D6         (out)
    D30 => AUX1 UP    (in)
    D31 => AUX1 DOWN  (in)
    D32 => AUX2 UP    (in)
    D33 => AUX2 DOWN  (in)
    D34 => A0         (out)
    D35 => A1         (out)
    D36 => A2         (out)
    D37 => A3         (out)
    D38 => A4         (out)
    D39 => A5         (out)
    D40 => A6         (out)
    D41 => A7         (out)
    D42 => SW14       (in)
    D43 => SW15       (in)
    D44 => A15        (out)
    D45 => A14        (out)
    D46 => A13        (out)
    D47 => A12        (out)
    D48 => A11        (out)
    D49 => A10        (out)
    D50 => A9         (out)
    D51 => A8         (out)
    D52 => RESET      (in)
    D53 => CLR        (in)

    The following are not labeled as digital pins on the board
    (i.e. not labeled Dxx) but can be used as digital pins.
    The board label for the pins is shown in parentheses.

    D54 (A0)    => STEP         (in)
    D55 (A1)    => SLOW         (in)
    D56 (A2)    => EXAMINE      (in)
    D57 (A3)    => EXAMINE NEXT (in)
    D58 (A4)    => DEPOSIT      (in)
    D59 (A5)    => DEPOSIT NEXT (in)
    D60 (A6)    => PROTECT      (in)
    D61 (A7)    => UNPROTECT    (in)
    D62 (A8)    => SW0          (in)
    D63 (A9)    => SW1          (in)
    D64 (A10)   => SW2          (in)
    D65 (A11)   => SW3          (in)
    D66 (DAC0)  => SW4          (in)
    D67 (DAC1)  => SW5          (in)
    D68 (CANRX) => SW6          (in)
    D69 (CANTX) => SW7          (in)

    D70 (SDA1)  => SW12         (in)
    D71 (SCL1)  => SW13         (in)

  ---- front panel connections by Processor register:
 
  PIOA:
    0 => SW7          (in)
    1 => SW6          (in)
    2 => UNPROTECT    (in)     
    3 => PROTECT      (in)     
    4 => DEPOSIT NEXT (in)     
    6 => DEPOSIT      (in)     
    7 => AUX1 DOWN    (in)
   12 => SW8          (in)     
   13 => SW9          (in)     
   14 => SW10         (in)     
   15 => SW11         (in)
   16 => STEP         (in)     
   17 => SW12         (in)
   18 => SW13         (in)
   19 => SW14         (in)
   20 => SW15         (in)
   22 => EXAMINE NEXT (in)     
   23 => EXAMINE      (in)     
   24 => SLOW         (in)     

  PIOB:
   12 => RUN       (in)
   13 => STOP      (in)
   14 => CLR       (in)
   15 => SW4       (in)
   16 => SW5       (in)
   17 => SW0       (in)
   18 => SW1       (in)
   19 => SW2       (in)
   20 => SW3       (in)
   21 => RESET     (in)
   25 => INT       (out)
   26 => HLDA      (out)
   17 => PROT      (out)

  PIOC:
    1 => AUX2 DOWN (in)
    2 => A0        (out)
    3 => A1        (out)
    4 => A2        (out)
    5 => A3        (out)
    6 => A4        (out)
    7 => A5        (out)
    8 => A6        (out)
    9 => A7        (out)
   12 => A8        (out)
   13 => A9        (out)
   14 => A10       (out)
   15 => A11       (out)
   16 => A12       (out)
   17 => A13       (out)
   18 => A14       (out)
   19 => A15       (out)
   21 => MEMR      (out)
   22 => INP       (out)
   23 => M1        (out)
   24 => OUT       (out)
   25 => HLTA      (out)
   26 => STACK     (out)
   28 => WO        (out)
   29 => WAIT      (out)

  PIOD:
    0 => D0        (out)
    1 => D1        (out)
    2 => D2        (out)
    3 => D3        (out)
    4 => D4        (out)
    5 => D5        (out)
    6 => D6        (out)
    7 => D7        (out)
    8 => INTE      (out)
    9 => AUX1 UP   (in)
   10 => AUX2 UP   (in)
*/


#define GETBIT(reg, regbit, v) (REG_PIO ## reg ## _PDSR & (1<<(regbit)) ? v : 0)
#define SETBIT(v, vbit, reg, regbit) if( v & vbit ) REG_PIO ## reg ## _SODR = 1<<regbit; else REG_PIO ## reg ## _CODR = 1<<regbit


uint16_t host_read_status_leds()
{
  uint16_t res = 0;
  res |= GETBIT(B, 25, ST_INT);
  res |= GETBIT(C, 28, ST_WO);
  res |= GETBIT(C, 26, ST_STACK);
  res |= GETBIT(C, 25, ST_HLTA);
  res |= GETBIT(C, 24, ST_OUT);
  res |= GETBIT(C, 23, ST_M1);
  res |= GETBIT(C, 22, ST_INP);
  res |= GETBIT(C, 21, ST_MEMR);
  res |= GETBIT(D,  8, ST_INTE);
  res |= GETBIT(B, 27, ST_PROT);
  res |= GETBIT(C, 10, ST_WAIT);
  res |= GETBIT(B, 26, ST_HLDA);
  return res;
}


byte host_read_data_leds()
{
  // D0..8 => PIOD, bits 0-7
  return REG_PIOD_PDSR & 0xff;
}


uint16_t host_read_addr_leds()
{
  // A0..7  => PIOC, bits 2-9
  // A8..15 => PIOC, bits 12-19
  word w = REG_PIOC_PDSR;
  return ((w & 0x000ff000) >> 4) | ((w & 0x000003fc) >> 2);
}


//------------------------------------------------------------------------------------------------------


uint16_t host_read_addr_switches()
{
  uint16_t v = 0;
  if( !digitalRead(62) ) v |= 0x01;
  if( !digitalRead(63) ) v |= 0x02;
  if( !digitalRead(64) ) v |= 0x04;
  if( !digitalRead(65) ) v |= 0x08;
  if( !digitalRead(66) ) v |= 0x10;
  if( !digitalRead(67) ) v |= 0x20;
  if( !digitalRead(68) ) v |= 0x40;
  if( !digitalRead(69) ) v |= 0x80;
  return v | (host_read_sense_switches() * 256);
}


//------------------------------------------------------------------------------------------------------

volatile static bool host_timer_running[9];
volatile static TimerFnTp host_timer_fn[9];

void TC0_Handler() { TC_GetStatus(TC0, 0); host_timer_fn[0](); }
void TC1_Handler() { TC_GetStatus(TC0, 1); host_timer_fn[1](); }
void TC2_Handler() { TC_GetStatus(TC0, 2); host_timer_fn[2](); }
void TC3_Handler() { TC_GetStatus(TC1, 0); host_timer_fn[3](); }
#if USE_SERIAL_ON_RXLTXL==0
// if the additional software serial interface on RXL/TXL is enabled then
// TC5 is used by that interface so we can not define a TC5_Handler 
// function here.  Timer 5 can not be used.
void TC4_Handler() { TC_GetStatus(TC1, 1); host_timer_fn[4](); }
#endif
#if USE_SERIAL_ON_A6A7==0
// if the additional software serial interface on A6/A7 is enabled then
// TC5 is used by that interface so we can not define a TC5_Handler 
// function here.  Timer 5 can not be used.
void TC5_Handler() { TC_GetStatus(TC1, 2); host_timer_fn[5](); }
#endif
void TC6_Handler() { TC_GetStatus(TC2, 0); host_timer_fn[6](); }
void TC7_Handler() { TC_GetStatus(TC2, 1); host_timer_fn[7](); }
void TC8_Handler() { TC_GetStatus(TC2, 2); host_timer_fn[8](); }


bool host_interrupt_timer_running(byte tid)
{
  return host_timer_running[tid];
}


void host_interrupt_timer_start(byte tid, uint32_t period_us = 0)
{
  if( host_timer_fn[tid]!=NULL )
    {
      host_timer_running[tid] = true; 
      switch( tid / 3 )
        {
        case 0: 
          if( period_us>0 ) TC_SetRC(TC0, tid % 3, period_us * 2.625);
          TC_Start(TC0, tid % 3); 
          break;

        case 1: 
          if( period_us>0 ) TC_SetRC(TC0, tid % 3, period_us * 2.625);
          TC_Start(TC1, tid % 3); 
          break;

        case 2: 
          if( period_us>0 ) TC_SetRC(TC0, tid % 3, period_us * 2.625);
          TC_Start(TC2, tid % 3); 
          break;
        }
    }
}


void host_interrupt_timer_stop(byte tid)
{
  switch( tid / 3 )
    {
    case 0: TC_Stop(TC0, tid % 3); break;
    case 1: TC_Stop(TC1, tid % 3); break;
    case 2: TC_Stop(TC2, tid % 3); break;
    }

  host_timer_running[tid] = false; 
}


void host_interrupt_timer_setup(byte tid, uint32_t period_us, TimerFnTp f)
{
  byte chid = tid % 3;
  byte clid = tid / 3;

  Tc *TC = NULL;
  switch( clid )
    {
    case 0 : TC = TC0; break;
    case 1 : TC = TC1; break;
    case 2 : TC = TC2; break;
    }
  
  if( TC==NULL ) return;
  
  // turn on the timer clock in the power management controller
  pmc_set_writeprotect(false);     // disable write protection for pmc registers
  switch( tid )
  {
  case 0 : pmc_enable_periph_clk(ID_TC0); break;
  case 1 : pmc_enable_periph_clk(ID_TC1); break;
  case 2 : pmc_enable_periph_clk(ID_TC2); break;
  case 3 : pmc_enable_periph_clk(ID_TC3); break;
  case 4 : pmc_enable_periph_clk(ID_TC4); break;
  case 5 : pmc_enable_periph_clk(ID_TC5); break;
  case 6 : pmc_enable_periph_clk(ID_TC6); break;
  case 7 : pmc_enable_periph_clk(ID_TC7); break;
  case 8 : pmc_enable_periph_clk(ID_TC8); break;
  }

  // we want wavesel 01 with RC (clock #0, channel 0)
  // TC_CMR_TCCLKS_TIMER_CLOCK3 specifies a base frequency of 2.625MHz
  // this gives a timer range from 0.38us to 1636s with a resolution of 0.38 us
  TC_Configure(TC, chid, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3); 

  // enable timer interrupts on the timer
  TC->TC_CHANNEL[chid].TC_IER=TC_IER_CPCS;   // IER = interrupt enable register
  TC->TC_CHANNEL[chid].TC_IDR=~TC_IER_CPCS;  // IDR = interrupt disable register

  // Enable the interrupt in the nested vector interrupt controller
  switch( tid )
  {
  case 0 : NVIC_EnableIRQ(TC0_IRQn); break;
  case 1 : NVIC_EnableIRQ(TC1_IRQn); break;
  case 2 : NVIC_EnableIRQ(TC2_IRQn); break;
  case 3 : NVIC_EnableIRQ(TC3_IRQn); break;
  case 4 : NVIC_EnableIRQ(TC4_IRQn); break;
  case 5 : NVIC_EnableIRQ(TC5_IRQn); break;
  case 6 : NVIC_EnableIRQ(TC6_IRQn); break;
  case 7 : NVIC_EnableIRQ(TC7_IRQn); break;
  case 8 : NVIC_EnableIRQ(TC8_IRQn); break;
  }
  
  // set the timer period. CLOCK3 is 2.625 MHz so if we set the
  // timer to period_us*2.625 then timer will go off after period_us microseconds
  TC_SetRC(TC, chid, period_us * 2.625);
  host_timer_running[tid] = false;
  host_timer_fn[tid] = f;
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


// The Due has 512k FLASH memory (addresses 0x00000-0x7ffff).
// We use 16k (0x4000 bytes) for storage
// DueFlashStorage address 0 is the first address of the second memory bank,
// i.e. 0x40000. We add 0x3C000 so we use at 0x7C000-0x7ffff
// => MUST make sure that our total program size (shown in Arduine IDE after compiling)
//    is less than 507903 (0x7Bfff)! Otherwise we would overwrite our own program when
//    saving memory pages.
#define FLASH_STORAGE_OFFSET 0x3C000
DueFlashStorage dueFlashStorage;
uint32_t due_storagesize = 0x4000;

#define MOVE_BUFFER_SIZE 1024
byte moveBuffer[MOVE_BUFFER_SIZE];

static bool use_sd = false;
static File storagefile;

bool host_storage_init(bool write)
{
  HLDAGuard hlda;
  host_storage_close();

  storagefile = SD.open("STORAGE.DAT", write ? FILE_WRITE : FILE_READ);
  if( storagefile ) 
    {
      // when using the storage file we can provide more memory than with FLASH
      due_storagesize = 512*1024;
      return true;
    }
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


static void host_storage_write_flash(const void *data, uint32_t addr, uint32_t len)
{
  uint32_t offset = addr & 3;
  if( offset != 0)
    {
      byte buf[4];
      uint32_t alignedAddr = addr & 0xfffffffc;
      memcpy(buf, dueFlashStorage.readAddress(FLASH_STORAGE_OFFSET + alignedAddr), 4);
      memcpy(buf+offset, data, min(4-offset, len));
      dueFlashStorage.write(FLASH_STORAGE_OFFSET + alignedAddr, buf, 4);
      if( offset + len > 4 )
        dueFlashStorage.write(FLASH_STORAGE_OFFSET + alignedAddr + 4, ((byte *) data) + (4-offset), len - (4-offset));
    }
  else
    dueFlashStorage.write(FLASH_STORAGE_OFFSET + addr, (byte *) data, len);
}


static void host_storage_read_flash(void *data, uint32_t addr, uint32_t len)
{
  memcpy(data, dueFlashStorage.readAddress(FLASH_STORAGE_OFFSET + addr), len);
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
  else
    host_storage_write_flash(data, addr, len);
}

void host_storage_read(void *data, uint32_t addr, uint32_t len)
{
  if( storagefile )
    host_storage_read_sd(data, addr, len);
  else
    host_storage_read_flash(data, addr, len);
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
              entry.getSFN(buffer,sizeof(buffer));
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


// pointer to Arduino (native) USB ISR
extern void (*gpf_isr)(void);

// our local copy of the original USB ISR pointer
void (*gpf_isr_orig)(void) = NULL;

// do we want to handle received data in interrupts (i.e. call serial_receive_host_data whenever
// data arrives)?
static bool usb_receive_interrupt_enabled = false;

// receive interrupt handler routine (defined below)
void host_serial_receive_start_interrupt_if2();

#ifdef USE_NATIVE_USB_TX_OPTIMIZATION
// USB transmit buffering enabled (see comment at #define on top of file)

volatile size_t  fifo_size = 0, fifo_len = 0;
volatile uint8_t sofcount = 0;

static void usb_isr()
{
  bool isReceive = Is_udd_out_received(CDC_RX) && Is_udd_endpoint_interrupt(CDC_RX);

  if( Is_udd_reset() )
    {
      fifo_size = 0;
      fifo_len  = 0;
    }

  // if there is data waiting to be sent then send it now
  if( fifo_len>0 && (Is_udd_sof()||Is_udd_msof()) && Is_udd_sof_interrupt_enabled() )
    {
      if( sofcount>0 ) --sofcount;

      if( sofcount==0 && udd_nb_busy_bank(CDC_TX)==0 )
        {
          udd_ack_fifocon(CDC_TX);
          udd_disable_msof_interrupt();
          udd_disable_sof_interrupt();
          udd_ack_sof();
          udd_ack_msof();
          fifo_len  = 0;
          sofcount  = 2;
        }
    }

  // call original USB interrupt handler
  gpf_isr_orig();

  // if interrupt was due to an OUT (receive data) packet then
  // call our receive interrupt handler
  if( isReceive && usb_receive_interrupt_enabled )
    host_serial_receive_start_interrupt_if2();
}


static size_t usb_write(const char *buf, size_t len)
{
  size_t nbytes;

  udd_disable_msof_interrupt();
  udd_disable_sof_interrupt();

  // wait until FIFO is available
  while( !Is_udd_fifocon(CDC_TX) );

  // determine packet size (can't do it on reset because it is not guaranteed
  // that we have registered our interrupt function at that point)
  if( fifo_size==0 )
    fifo_size = ((UOTGHS->UOTGHS_SR & UOTGHS_SR_SPEED_Msk) == UOTGHS_SR_SPEED_HIGH_SPEED) ? 512 : 64;
  
  // copy data to FIFO
  volatile uint8_t *ptr_dest = ((volatile uint8_t *) &udd_get_endpoint_fifo_access8(CDC_TX)) + fifo_len;
  if( len==1 )
    {
      // most common and trivial case - note that there's always room
      // for at least one byte in the FIFO at this point
      *ptr_dest = *buf;
      fifo_len++;
      nbytes = 1;
    }
  else
    {
      nbytes = min(len, fifo_size-fifo_len);
      volatile uint8_t *ptr_end = ptr_dest + nbytes;
      while( ptr_dest!=ptr_end ) *ptr_dest++ = *buf++;
      fifo_len += nbytes;
    }

  if( fifo_len==fifo_size )
    {
      // FIFO is full => send it now
      udd_ack_fifocon(CDC_TX);
      fifo_len  = 0;
      sofcount  = 2;
    }
  else
    {
      // in full speed mode wait 1-2 SOF frames (1-2ms) before
      // sending the FIFO - this gives us some time to collect
      // more data and not waste a whole frame sending a single byte
      // in high speed we just wait until the next micro-SOF
      udd_ack_sof();
      udd_ack_msof();
      udd_enable_sof_interrupt();
      udd_enable_msof_interrupt();
    }

  return nbytes;
}

inline size_t usb_write(uint8_t b)
{
  return usb_write((const char *) &b, 1);
}


inline int usb_available_for_write()
{
  return fifo_size-fifo_len;
}


#else
// USB transmit buffering disabled (see comment at #define on top of file)

void usb_isr()
{
  bool isReceive = Is_udd_out_received(CDC_RX);

  // call original USB interrupt handler
  gpf_isr_orig();

  // if interrupt was due to an OUT (receive data) packet then call the simulator receive interrupt
  if( usb_receive_interrupt_enabled && isReceive )
    host_serial_receive_start_interrupt_if2();
}

inline size_t usb_write(byte b)
{
  return SerialUSB.write(&b, 1);
}

static size_t usb_write(const char *buf, size_t len)
{
  return SerialUSB.write(buf, len);
}

inline int usb_available_for_write()
{
  return SerialUSB.availableForWrite(); 
}

#endif


//------------------------------------------------------------------------------------------------------


host_serial_receive_callback_tp serial_receive_callback[HOST_NUM_SERIAL_PORTS];

host_serial_receive_callback_tp host_serial_set_receive_callback(byte iface, host_serial_receive_callback_tp f)
{
  host_serial_receive_callback_tp old_f = NULL;

  if( iface>=0 && iface<HOST_NUM_SERIAL_PORTS )
    {
      old_f = serial_receive_callback[iface];
      serial_receive_callback[iface] = f;
    }

  return old_f;
}


static void host_serial_receive_finished_interrupt_if0()
{
  // a complete character should have been received on pin 0/1 (USB programming port)
  // forward it to the emulated serial card. If no data is available then wait one more
  // timer interval to see if something arrives.
  static bool oneMore = false;
  if( Serial.available() )
    { 
      serial_receive_callback[0](0, Serial.read()); 
      oneMore = true; 
    }
  else if( !oneMore )
    host_interrupt_timer_stop(8);
  else
    oneMore = false;
}


static void host_serial_receive_start_interrupt_if0()
{
  // we have seen a signal change on the RX serial line of pin 0/1 (USB programming port)
  // so a serial character is being received => wait until it is finished
  if( !host_interrupt_timer_running(8) )
    host_interrupt_timer_start(8);
}


static void host_serial_receive_finished_interrupt_if1()
{
  // a complete character should have been received on pin 18/19 serial port
  // forward it to the emulated serial card. If no data is available then wait one more
  // timer interval to see if something arrives.
  static bool oneMore = false;
  if( Serial1.available() )
    {
      serial_receive_callback[1](1, Serial1.read());
      oneMore = true; 
    }
  else if( !oneMore )
    host_interrupt_timer_stop(7);
  else
    oneMore = false;
}


static void host_serial_receive_start_interrupt_if1()
{
  // we have seen a signal change on the RX serial line on pin 18/19 serial port
  // so a serial character is being received => wait until it is finished
  if( !host_interrupt_timer_running(7) )
    host_interrupt_timer_start(7);
}


static void host_serial_receive_finished_interrupt_if2()
{
  // keep forwarding data received on Native USB 
  // (one character at a time according to baud rate) 
  // to emulated serial card until no more data
  if( SerialUSB.available() )
    serial_receive_callback[2](2, SerialUSB.read());
  else 
    host_interrupt_timer_stop(6);
}


void host_serial_receive_start_interrupt_if2()
{
  // received data on Native USB port
  if( !host_interrupt_timer_running(6) )
    {
      // forward first byte to emulated serial card
      if( SerialUSB.available() )
        serial_receive_callback[2](2, SerialUSB.read());
      
      // if there is more to forward then schedule timer (according to baud rate)
      if( SerialUSB.available() )
        host_interrupt_timer_start(6);
    }
}


#if USE_SERIAL_ON_A6A7>0
// define a software UART on interrupt 5 with 16 byte
// transmit and receive buffers
serial_tc5_declaration(16,16);

void CAN1_Handler()
{
  // forward received data to the emulated serial card
  while( serial_tc5.available() )
    {
      int d = serial_tc5.read();
      if( d>=0 ) serial_receive_callback[3](3, d);
    }
}

static void host_serial_receive_finished_interrupt_if3()
{
  // This function is set to be called from the soft_uart code after a new
  // character has been received - from within the timer interrupt handler.
  // If we call serial_receive_callback directly from here then
  // the time spent in that function will (at higher baud rates)
  // prevent the soft_uart interrupt from happening again (since
  // the interrupt function is still active), causing receive errors.
  // So instead we schedule the (otherwise unused) CAN1 interrupt (which
  // is set up with a lower priority) to actually handle the character.
  // This frees the soft_uart timer interrupt to be handled again.
  NVIC_SetPendingIRQ(CAN1_IRQn);
}

#endif


#if USE_SERIAL_ON_RXLTXL>0
// define a software UART on interrupt 4 with 16 byte
// transmit and receive buffers
serial_tc4_declaration(16,16);

void CAN0_Handler()
{
  // forward received data to the emulated serial card
  while( serial_tc4.available() )
    {
      int d = serial_tc4.read();
      if( d>=0 ) serial_receive_callback[HOST_NUM_SERIAL_PORTS-1](HOST_NUM_SERIAL_PORTS-1, d);
    }
}

static void host_serial_receive_finished_interrupt_if4()
{
  // This function is set to be called from the soft_uart code after a new
  // character has been received - from within the timer interrupt handler.
  // If we call serial_receive_callback directly from here then
  // the time spent in that function will (at higher baud rates)
  // prevent the soft_uart interrupt from happening again (since
  // the interrupt function is still active), causing receive errors.
  // So instead we schedule the (otherwise unused) CAN0 interrupt (which
  // is set up with a lower priority) to actually handle the character.
  // This frees the soft_uart timer interrupt to be handled again.
  NVIC_SetPendingIRQ(CAN0_IRQn);
}

#endif


static byte num_bits(uint32_t config)
{
  switch( config )
    {
    case SERIAL_5N1: return 7;
    case SERIAL_6N1: return 8;
    case SERIAL_7N1: return 9;
    case SERIAL_8N1: return 10;
    case SERIAL_5N2: return 8;
    case SERIAL_6N2: return 9;
    case SERIAL_7N2: return 10;
    case SERIAL_8N2: return 11;
    case SERIAL_5E1: return 8;
    case SERIAL_6E1: return 9;
    case SERIAL_7E1: return 10;
    case SERIAL_8E1: return 11;
    case SERIAL_5E2: return 9;
    case SERIAL_6E2: return 10;
    case SERIAL_7E2: return 11;
    case SERIAL_8E2: return 12;
    case SERIAL_5O1: return 8;
    case SERIAL_6O1: return 9;
    case SERIAL_7O1: return 10;
    case SERIAL_8O1: return 11;
    case SERIAL_5O2: return 9;
    case SERIAL_6O2: return 10;
    case SERIAL_7O2: return 11;
    case SERIAL_8O2: return 12;
    }

  return 10;
}


void host_serial_setup(byte iface, uint32_t baud, uint32_t config, bool set_primary_interface)
{
  byte rxPin, timer;
  void (*fnStarting)() = NULL, (*fnFinished)() = NULL;

  if( iface==0 )
    {
      rxPin = 0; 
      timer = 8; 
      fnStarting = host_serial_receive_start_interrupt_if0; 
      fnFinished = host_serial_receive_finished_interrupt_if0; 
    }
  else if( iface==1 )
    { 
      rxPin = 19; 
      timer = 7; 
      fnStarting = host_serial_receive_start_interrupt_if1; 
      fnFinished = host_serial_receive_finished_interrupt_if1; 
    }
  else if( iface==2 )
    {
      // hook into the SerialUSB interrupt service routine
      if( gpf_isr_orig==NULL ) gpf_isr_orig = gpf_isr;
      gpf_isr = usb_isr;
      usb_receive_interrupt_enabled = true;

      timer = 6;
      fnFinished = host_serial_receive_finished_interrupt_if2;
    }

  if( fnStarting!=NULL )
    {
      // detach interrupt (if it was already set)
      detachInterrupt(rxPin);

      // interrupt to see activity on serial RX pin
      attachInterrupt(digitalPinToInterrupt(rxPin), fnStarting, FALLING);
    }

  if( fnFinished!=NULL )
    {
      // stop timer interrupt (if running)
      if( host_interrupt_timer_running(timer) ) host_interrupt_timer_stop(timer);
      
      // When receiving data via UART or USART we would LIKE to receive an
      // interrupt when a character was received, so it can be forwarded to
      // the emulated serial card.  The SAM3X processor does of course provide
      // such an interrupt BUT the Arduino library consumes that interrupt
      // without providing a means to hook into the handler or a way to override
      // it. So it seems there is no way for our code to receive an interrupt
      // when a character was received. So we need a workaround:
      // Trigger an interrupt on he falling edge of the RX line (start bit) and
      // within that interrupt, start a timer that (based on the baud rate) produces 
      // another interrupt after the byte has been received. 

      int stopBitStart = ((num_bits(config)-1)*1000000)/baud;
      int stopBitEnd   = ((num_bits(config))*1000000)/baud;

      // Set the timeout such that the timer interrupt occurs NO EARLIER than
      // 20us after the beginning of the (second) stop bit. That timing ensures that
      // the byte has been properly received before our interrupt. If the bits 
      // are long enough then schedule it 25us before the end of the stop bit.
      // That leaves enough time to process the received byte before the next
      // one starts. For reference, at 9600 baud one bit is ~104us long.
      host_interrupt_timer_setup(timer, max(stopBitStart+20, stopBitEnd-25), fnFinished);
    }

  // switch the primary serial interface (if requested)
  if( set_primary_interface ) SwitchSerial.select(iface); 

  if( iface==0 )
    { 
      Serial.begin(baud); 
      Serial.setTimeout(10000); 
    }
  else if( iface==1 )
    { 
      Serial1.begin(baud);
      if( config==SERIAL_8N1 || config==SERIAL_8O1 || config==SERIAL_8E1 )
        Serial1.begin(baud, (UARTClass::UARTModes) config);
      else
        Serial1.begin(baud, (USARTClass::USARTModes) config);

      Serial1.setTimeout(10000); 
    }
  else if( iface==2 )
    { 
      SerialUSB.begin(baud); 
      SerialUSB.setTimeout(10000); 
    }
#if USE_SERIAL_ON_A6A7>0
  else if( iface==3 )
    { 
      serial_tc5.begin(60, 61, baud, config);
      serial_tc5.setTimeout(10000); 

      // see comment in function: host_serial_receive_finished_interrupt_if3
      NVIC_SetPriority(CAN1_IRQn, 10);
      NVIC_EnableIRQ(CAN1_IRQn);

      // rx_handler will be called (from within serial_tc5's ISR) after 
      // a complete byte has been received
      serial_tc5.set_rx_handler(host_serial_receive_finished_interrupt_if3);
    }
#endif
#if USE_SERIAL_ON_RXLTXL>0
  else if( iface==(HOST_NUM_SERIAL_PORTS-1) )
    { 
      serial_tc4.begin(72, 73, baud, config);
      serial_tc4.setTimeout(10000); 

      // see comment in function: host_serial_receive_finished_interrupt_if4
      NVIC_SetPriority(CAN0_IRQn, 10);
      NVIC_EnableIRQ(CAN0_IRQn);

      // rx_handler will be called (from within serial_tc4's ISR) after 
      // a complete byte has been received
      serial_tc4.set_rx_handler(host_serial_receive_finished_interrupt_if4);
    }
#endif
}


void host_serial_interrupts_pause()
{
  if( Serial )  detachInterrupt(0);
  if( Serial1 ) detachInterrupt(19);
  usb_receive_interrupt_enabled = false;
#if USE_SERIAL_ON_A6A7>0
  if( serial_tc5 ) serial_tc5.set_rx_handler(NULL);
#endif  
#if USE_SERIAL_ON_RXLTXL>0
  if( serial_tc4 ) serial_tc4.set_rx_handler(NULL);
#endif  
}


void host_serial_interrupts_resume()
{
  if( Serial )  attachInterrupt(0, host_serial_receive_start_interrupt_if0, FALLING);
  if( Serial1 ) attachInterrupt(19, host_serial_receive_start_interrupt_if1, FALLING);
  usb_receive_interrupt_enabled = true;
#if USE_SERIAL_ON_A6A7>0
  if( serial_tc5 ) serial_tc5.set_rx_handler(host_serial_receive_finished_interrupt_if3);
#endif  
#if USE_SERIAL_ON_RXLTXL>0
  if( serial_tc4 ) serial_tc4.set_rx_handler(host_serial_receive_finished_interrupt_if4);
#endif  
}


void host_serial_end(byte i)
{
  switch( i )
    {
    case 0: Serial.end(); break;
    case 1: Serial1.end(); break;
    case 2: SerialUSB.end(); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: serial_tc5.end(); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): serial_tc4.end(); break;
#endif
    }
}

int host_serial_available(byte i)
{
  switch( i )
    {
    case 0: return Serial.available(); break;
    case 1: return Serial1.available(); break;
    case 2: return SerialUSB.available(); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return serial_tc5.available(); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.available(); break;
#endif
    }

 return 0;
}

int host_serial_available_for_write(byte i)
{
  switch( i )
    {
    case 0: return Serial.availableForWrite(); break;
    case 1: return Serial1.availableForWrite(); break;
    case 2: return usb_available_for_write(); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return serial_tc5.availableForWrite(); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.availableForWrite(); break;
#endif
    }

 return 0;
}

int host_serial_peek(byte i)
{
  switch( i )
    {
    case 0: return Serial.peek(); break;
    case 1: return Serial1.peek(); break;
    case 2: return SerialUSB.peek(); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return serial_tc5.peek(); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.peek(); break;
#endif
    }

 return -1;
}

int host_serial_read(byte i)
{
  switch( i )
    {
    case 0: return Serial.read(); break;
    case 1: return Serial1.read(); break;
    case 2: return SerialUSB.read(); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return serial_tc5.read(); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.read(); break;
#endif
    }

  return -1;
}

void host_serial_flush(byte i)
{
  switch( i )
    {
    case 0: Serial.flush(); break;
    case 1: Serial1.flush(); break;
    case 2: SerialUSB.flush(); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: serial_tc5.flush(); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.flush(); break;
#endif
    }
}

size_t host_serial_write(byte i, uint8_t b)
{
  switch( i )
    {
    case 0: return Serial.write(b); break;
    case 1: return Serial1.write(b); break;
    case 2: return usb_write(b); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return serial_tc5.write(b); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.write(b); break;
#endif
    }

  return 0;
}

size_t host_serial_write(byte i, const char *buf, size_t n)
{
  switch( i )
    {
    case 0: 
      {
        size_t i;
        // do not allow serial interrupts while writing the data, otherwise
        // the serial transmit buffer can get out of sync and skip data bytes
        // (bug in Arduino UART serial implementation)
        noInterrupts();
        i = Serial.availableForWrite();
        if( i<n ) n = i;
        for(i=0; i<n; i++) Serial.write(buf[i]); 
        interrupts();
        return i;
      }

    case 1: 
      {
        size_t i;
        // do not allow serial interrupts while writing the data, otherwise
        // the serial transmit buffer can get out of sync and skip data bytes
        // (bug in Arduino UART serial implementation)
        noInterrupts();
        i = Serial1.availableForWrite();
        if( i<n ) n = i;
        for(i=0; i<n; i++) Serial1.write(buf[i]); 
        interrupts();
        return i;
      }

    case 2: return usb_write(buf, n); break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return serial_tc5.write(buf, n); break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return serial_tc4.write(buf, n); break;
#endif
    }

  return 0;
}


bool host_serial_ok(byte i)
{ 
  switch( i )
    {
    case 0: return (bool) Serial; break;
    case 1: return (bool) Serial1; break;
    case 2: return (bool) SerialUSB; break;
#if USE_SERIAL_ON_A6A7>0
    case 3: return (bool) serial_tc5; break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return (bool) serial_tc4; break;
#endif
    }

  return false;
  }


const char *host_serial_port_name(byte i)
{
  switch(i)
    {
    case 0: return "USB Programming Port";
    case 1: return "Serial (pin 18/19)";
    case 2: return "USB Native Port";
#if USE_SERIAL_ON_A6A7>0
    case 3: return "Serial (pin A6/A7)";
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): return "Serial (RXL/TXL)";
#endif
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
#if USE_SERIAL_ON_A6A7>0
    case 3: *min = 110;    *max =   38400; break;
#endif
#if USE_SERIAL_ON_RXLTXL>0
    case (HOST_NUM_SERIAL_PORTS-1): *min = 110; *max =  38400; break;
#endif
    default: return false;
    }

  return true;
}


bool host_serial_port_has_configs(byte i)
{
  return i==1 || i==3 || i==4;
}


// --------------------------------------------------------------------------------------------------


volatile static uint16_t switches_pulse = 0;
volatile static uint16_t switches_debounced = 0;
static uint32_t debounceTime[16];
static const byte function_switch_pin[16] = {20,21,54,55,56,57,58,59,52,53,60,61,30,31,32,33};
static const uint8_t function_switch_irq[16] = {0, INT_SW_STOP>>24, 0, 0, 0, 0, 0, 0, 
                                                INT_SW_RESET>>24, INT_SW_CLR>>24, 0, 0, 0, 0, 
                                                INT_SW_AUX2UP>>24, INT_SW_AUX2DOWN>>24};


bool host_read_function_switch(byte i)
{
  return !digitalRead(function_switch_pin[i]);
}


bool host_read_function_switch_debounced(byte i)
{
  return (switches_debounced & (1<<i)) ? true : false;
}


bool host_read_function_switch_edge(byte i)
{
  uint16_t bitval = 1<<i;
  bool b = switches_pulse & bitval ? true : false;
  if( b ) switches_pulse &= ~bitval;
  return b;
}


uint16_t host_read_function_switches_edge()
{
  uint16_t res = switches_pulse;
  switches_pulse &= ~res;
  return res;
}


void host_reset_function_switch_state()
{
  switches_debounced = 0;
  switches_pulse     = 0;

  for(int i=0; i<16; i++)
    {
      debounceTime[i]=millis() + 50;
      if( !digitalRead(function_switch_pin[i]) ) switches_debounced |= (1<<i);
    }
}


static void switch_interrupt(int i)
{
  if( millis()>debounceTime[i] )
    {
      uint16_t bitval = 1<<i;

      bool d1 = !digitalRead(function_switch_pin[i]);
      bool d2 = (switches_debounced & bitval) ? true : false;

      if( d1 && !d2 ) 
        {
          switches_debounced |= bitval;
          switches_pulse |= bitval;
          if( function_switch_irq[i]>0 ) altair_interrupt(function_switch_irq[i]<<24);
          debounceTime[i] = millis() + 50;
        }
      else if( !d1 && d2 ) 
        {
          switches_debounced &= ~bitval;
          switches_pulse &= ~bitval;
          debounceTime[i] = millis() + 50;
        }
    }
}


static void switch_interrupt_0()  { switch_interrupt(0);  }
static void switch_interrupt_1()  { switch_interrupt(1);  }
static void switch_interrupt_2()  { switch_interrupt(2);  }
static void switch_interrupt_3()  { switch_interrupt(3);  }
static void switch_interrupt_4()  { switch_interrupt(4);  }
static void switch_interrupt_5()  { switch_interrupt(5);  }
static void switch_interrupt_6()  { switch_interrupt(6);  }
static void switch_interrupt_7()  { switch_interrupt(7);  }
static void switch_interrupt_8()  { switch_interrupt(8);  }
static void switch_interrupt_9()  { switch_interrupt(9);  }
static void switch_interrupt_10() { switch_interrupt(10); }
static void switch_interrupt_11() { switch_interrupt(11); }
static void switch_interrupt_12() { switch_interrupt(12); }
static void switch_interrupt_13() { switch_interrupt(13); }
static void switch_interrupt_14() { switch_interrupt(14); }
static void switch_interrupt_15() { switch_interrupt(15); }


static void switches_setup()
{
  host_reset_function_switch_state();
  attachInterrupt(function_switch_pin[ 0], switch_interrupt_0,  CHANGE);
  attachInterrupt(function_switch_pin[ 1], switch_interrupt_1,  CHANGE);
  attachInterrupt(function_switch_pin[ 2], switch_interrupt_2,  CHANGE);
  attachInterrupt(function_switch_pin[ 3], switch_interrupt_3,  CHANGE);
  attachInterrupt(function_switch_pin[ 4], switch_interrupt_4,  CHANGE);
  attachInterrupt(function_switch_pin[ 5], switch_interrupt_5,  CHANGE);
  attachInterrupt(function_switch_pin[ 6], switch_interrupt_6,  CHANGE);
  attachInterrupt(function_switch_pin[ 7], switch_interrupt_7,  CHANGE);
  attachInterrupt(function_switch_pin[ 8], switch_interrupt_8,  CHANGE);
  attachInterrupt(function_switch_pin[ 9], switch_interrupt_9,  CHANGE);
#if !(USE_SERIAL_ON_A6A7>0)
  attachInterrupt(function_switch_pin[10], switch_interrupt_10, CHANGE);
  attachInterrupt(function_switch_pin[11], switch_interrupt_11, CHANGE);
#endif
  attachInterrupt(function_switch_pin[12], switch_interrupt_12, CHANGE);
  attachInterrupt(function_switch_pin[13], switch_interrupt_13, CHANGE);
  attachInterrupt(function_switch_pin[14], switch_interrupt_14, CHANGE);
  attachInterrupt(function_switch_pin[15], switch_interrupt_15, CHANGE);
}


// --------------------------------------------------------


signed char isinput[] = 
  {
   -1, // D0  => Serial0 RX (don't set)
   -1, // D1  => Serial0 TX (don't set)
    0, // D2  => INT
    0, // D3  => WO
    0, // D4  => STACK
    0, // D5  => HLTA
    0, // D6  => OUT
    0, // D7  => M1
    0, // D8  => INP
    0, // D9  => MEMR
    0, // D10 => WAIT
    0, // D11 => D7
    0, // D12 => INTE
    0, // D13 => PROT
    0, // D14 => D4
    0, // D15 => D5
    1, // D16 => SW9
    1, // D17 => SW8
   -1, // D18 => Serial1 TX (don't set)
   -1, // D19 => Serial1 RX (don't set)
    1, // D20 => RUN
    1, // D21 => STOP
    0, // D22 => HLDA
    1, // D23 => SW10
    1, // D24 => SW11
    0, // D25 => D0
    0, // D26 => D1
    0, // D27 => D2
    0, // D28 => D3
    0, // D29 => D6
    1, // D30 => AUX1 UP
    1, // D31 => AUX1 DOWN
    1, // D32 => AUX2 UP
    1, // D33 => AUX2 DOWN
    0, // D34 => A0
    0, // D35 => A1
    0, // D36 => A2
    0, // D37 => A3
    0, // D38 => A4
    0, // D39 => A5
    0, // D40 => A6
    0, // D41 => A7
    1, // D42 => SW14
    1, // D43 => SW15
    0, // D44 => A15
    0, // D45 => A14
    0, // D46 => A13
    0, // D47 => A12
    0, // D48 => A11
    0, // D49 => A10
    0, // D50 => A9
    0, // D51 => A8
    1, // D52 => RESET
    1, // D53 => CLR
    1, // D54 (A0)    => STEP
    1, // D55 (A1)    => SLOW
    1, // D56 (A2)    => EXAMINE
    1, // D57 (A3)    => EXAMINE NEXT
    1, // D58 (A4)    => DEPOSIT
    1, // D59 (A5)    => DEPOSIT NEXT
#if USE_SERIAL_ON_A6A7>0
   -1, // D60 (A6)    => serial_tc5 RX (don't set)
   -1, // D61 (A7)    => serial_tc5 TX (don't set)
#else
    1, // D60 (A6)    => PROTECT
    1, // D61 (A7)    => UNPROTECT
#endif
    1, // D62 (A8)    => SW0
    1, // D63 (A9)    => SW1
    1, // D64 (A10)   => SW2
    1, // D65 (A11)   => SW3
    1, // D66 (DAC0)  => SW4
    1, // D67 (DAC1)  => SW5
    1, // D68 (CANRX) => SW6
    1, // D69 (CANTX) => SW7
    1, // D70 (SCL1)  => SW13
    1, // D71 (SDA1)  => SW12
   -1, // D72 (RXL)   => serial_tc4 RX (don't set)
   -1, // D73 (TXL)   => serial_tc4 TX (don't set)
  };


void host_copy_flash_to_ram(void *dst, const void *src, uint32_t len)
{
  memcpy(dst, src, len);
}


uint32_t host_get_random()
{
  delayMicroseconds(1);
  return (uint32_t) trng_read_output_data(TRNG);
}

static uint32_t lamppins[]=
    { 12, 13, 9, 8, 7, 6, 5, 4, 3, 2, 11, 29, 15, 14, 28, 27, 26, 25, 10, 22, 44, 45, 46, 47, 48, 49, 50, 51, 41, 40, 39, 38, 37, 36, 35, 34 };  // DUE

#define LAMPCT (sizeof(lamppins)/sizeof(lamppins[0]))

void host_lamp_test(void)  
{
  unsigned i,j=0;
  int lastpin=-1;
  Serial.println();
  Serial.println("Lamp test in progress...");
  for (i=0;i<LAMPCT;i++)   // all on
    digitalWrite(lamppins[i],HIGH);
  delay(10000);     // 10s
  for (i=0;i<LAMPCT;i++)  // all off
    digitalWrite(lamppins[i],LOW);
  for (i=0;i<LAMPCT*2;i++)  // go forward and backward
  {
    if (lastpin!=-1) digitalWrite(lamppins[lastpin],LOW);  // turn off last light
    if (j==LAMPCT) j--;   // adjust for bounce at the end
    digitalWrite(lamppins[j],HIGH);   // light it up
    lastpin=j;   // remember which one to turn off next time
    if (i>=LAMPCT) j--; else j++;  // left or right
    delay(250);
  }  
  digitalWrite(lamppins[0],LOW);  // turn off the last one
  // good place to stick one or more on for extended troubleshooting
  //digitalWrite(44,HIGH);
}


#include <malloc.h>
extern char _end;
extern "C" char *sbrk(int i);
char *ramstart=(char *)0x20070000;
char *ramend=(char *)0x20088000;

void host_system_info()
{
  SwitchSerial.println("Host is Arduino Due\n");

  struct mallinfo mi=mallinfo();
  char *heapend=sbrk(0);
  register char * stack_ptr asm("sp");

  SwitchSerial.print("RAM Start        : 0x"); SwitchSerial.println((unsigned long)ramstart, HEX);
  SwitchSerial.print("Data/Bss end     : 0x"); SwitchSerial.println((unsigned long)&_end, HEX);
  SwitchSerial.print("Heap End         : 0x"); SwitchSerial.println((unsigned long)heapend, HEX);
  SwitchSerial.print("Stack Pointer    : 0x"); SwitchSerial.println((unsigned long)stack_ptr, HEX);
  SwitchSerial.print("RAM End          : 0x"); SwitchSerial.println((unsigned long)ramend, HEX);
  SwitchSerial.print("Heap RAM Used    : "); SwitchSerial.println(mi.uordblks);
  SwitchSerial.print("Program RAM Used : "); SwitchSerial.println(&_end - ramstart);
  SwitchSerial.print("Stack RAM Used   : "); SwitchSerial.println(ramend - stack_ptr);
  SwitchSerial.print("Free RAM         : "); SwitchSerial.println(stack_ptr - heapend + mi.fordblks);
  SwitchSerial.print("Data storage     : ");

#if USE_HOST_FILESYS>0
  SwitchSerial.print("SD card file system");
  HLDAGuard hlda;
  if( SD.card()->errorCode()==SD_CARD_ERROR_NONE )
    { SwitchSerial.print(" ("); SwitchSerial.print(SD.card()->cardSize() / (2*1024)); SwitchSerial.println("M)"); }
  else
    { SwitchSerial.print(" (card error: 0x"); SwitchSerial.print(SD.card()->errorCode(), HEX); SwitchSerial.println(")"); }
#else
  SwitchSerial.print(storagefile ? "STORAGE.DAT file on SD card" : "flash memory");
  SwitchSerial.print(" ("); SwitchSerial.print(due_storagesize / 1024); SwitchSerial.print("K)");
#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0
  if( SD.card()->errorCode()!=SD_CARD_ERROR_NONE )
    { SwitchSerial.print(" (SD card error: 0x"); SwitchSerial.print(SD.card()->errorCode(), HEX); SwitchSerial.print(")"); }
  SwitchSerial.println();
#endif
#endif
}


void host_setup()
{
  // define digital input/output pin direction
  for(int i=0; i<74; i++)
    if( isinput[i]>=0 )
      pinMode(i, isinput[i] ? INPUT_PULLUP : OUTPUT);

  // enable pull-up resistor on RX1, otherwise some serial devices
  // (e.g. serial-to-bluetooth) won't work
  pinMode(19, INPUT_PULLUP);
  
  // attach interrupts
  switches_setup();

  // initialize interrupt timers
  for(byte tid=0; tid<9; tid++)
    {
      host_timer_running[tid] = false;
      host_timer_fn[tid] = NULL;
    }

  // set mask for bits that will be written to via REG_PIOX_OSDR
  REG_PIOC_OWDR = 0xFFF00C03;  // address bus (16 bit)
  REG_PIOD_OWDR = 0xFFFFFF00;  // data bus (8 bit)

  // init random number generator
  pmc_enable_periph_clk(ID_TRNG);
  trng_enable(TRNG);
  trng_read_output_data(TRNG);

#if NUM_DRIVES>0 || NUM_HDSK_UNITS>0 || USE_HOST_FILESYS>0
  // check if SD card available (send "chip select" signal to HLDA status light)
  // SdInfo.h in the SdFat library says: Set SCK rate to F_CPU/3 (SPI_DIV3_SPEED) for Due
  // (84MHZ/3 = 28MHz). If that fails try 4MHz and if that fails too then fall back to 250Khz.
  // If neither of those work then something is seriously wrong.
  if( SD.begin(22, SPI_DIV3_SPEED) || SD.begin(22, SD_SCK_MHZ(4)) || SD.begin(22, SD_SCK_HZ(250000)) )
    {
#if USE_HOST_FILESYS>0
      // storing configurations etc directly on SD card
      use_sd = true;
#else
      // not using host file system => open storage (file) for writing
      use_sd = host_storage_init(true);
#endif
    }
  host_clr_status_led_HLDA();
#endif

  // set serial receive callbacks to default
  for(byte i=0; i<HOST_NUM_SERIAL_PORTS; i++)
    host_serial_set_receive_callback(i, serial_receive_host_data);
}


#endif
