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

#include "dazzler.h"
#include "mem.h"
#include "cpucore.h"
#include "host.h"
#include "serial.h"
#include "timer.h"
#include "numsys.h"

#if USE_DAZZLER==0

void dazzler_out_ctrl(byte v) {}
void dazzler_out_pict(byte v) {}
byte dazzler_in(byte port) { return 0xff; }
void dazzler_set_iface(byte iface) {}
byte dazzler_get_iface() { return 0xff; }
void dazzler_setup() {}

#else

#define DAZZLER_HOST_VERSION 0x01

#define DAZ_MEMBYTE   0x10
#define DAZ_FULLFRAME 0x20
#define DAZ_CTRL      0x30
#define DAZ_CTRLPIC   0x40
#define DAZ_VERSION   0xF0

#define DAZ_JOY1      0x10
#define DAZ_JOY2      0x20
#define DAZ_KEY       0x30
#define DAZ_VSYNC     0x40

#define BUFFER1       0x00
#define BUFFER2       0x08

#define DEBUGLVL 0

byte dazzler_iface = 0xff;
int  dazzler_client_version = -1;
uint32_t dazzler_vsync_cycles = 0;

uint16_t dazzler_mem_addr1, dazzler_mem_addr2, dazzler_mem_start, dazzler_mem_end, dazzler_mem_size;
volatile byte d7a_port[5];


static void dazzler_send(const byte *data, uint16_t size)
{
  if( dazzler_iface<0xff )
    {
      size_t n, ptr = 0;
      while( size>0 )
        {
          n = host_serial_write(dazzler_iface, ((const char *) data)+ptr, size);
          ptr  += n;
          size -= n;
        }
    }
}


static void dazzler_send_fullframe(uint8_t buffer_flag, uint16_t addr)
{
  // send full picture memory
  byte b = DAZ_FULLFRAME | buffer_flag | (dazzler_mem_size > 512 ? 1 : 0);
  host_serial_write(dazzler_iface, b);
  dazzler_send(Mem+addr, dazzler_mem_size > 512 ? 2048 : 512);
}


static void dazzler_send_frame(int buffer_flag, uint16_t addr_old, uint16_t addr_new)
{
  uint8_t b[3];
  int n = 0, d = addr_new-addr_old, end = addr_new + dazzler_mem_size;

  // check how many bytes actually differ between the 
  // old and new memory locations (if a program is
  // switching quickly between two locations then likely
  // not a large number of pixels/bytes will differ)
  for(int i=addr_new; i<=end; i++)
    if( Mem[i] != Mem[i-d] )
      n++;

  if( n*3 < dazzler_mem_size )
    {
      // it takes less data to send a diff than the full frame
      for(int i=addr_new; i<end; i++)
        if( Mem[i] != Mem[i-d] )
          {
            b[0] = DAZ_MEMBYTE | buffer_flag | (((i-addr_new) & 0x0700)/256);
            b[1] = i & 255;
            b[2] = Mem[i];
            dazzler_send(b, 3);
          }
    }
    else
    {
      // sending full frame is shorter
      dazzler_send_fullframe(buffer_flag, addr_new);
    }
}


void dazzler_write_mem_do(uint16_t a, byte v)
{
  byte b[3];

#if DEBUGLVL>0
  printf("dazzler_write_mem(%04x, %02x)\n", a, v);
#endif

  // buffer 1 must be in use if we get here
  if( ((uint16_t) (a-dazzler_mem_addr1))<dazzler_mem_size )
    {
      b[0] = DAZ_MEMBYTE | BUFFER1 | (((a-dazzler_mem_addr1) & 0x0700)/256);
      b[1] = a & 0xFF;
      b[2] = v;
      dazzler_send(b, 3);
    }

  // buffer 2 may or may not be in use
  if( dazzler_mem_addr2!=0xFFFF && ((uint16_t) (a-dazzler_mem_addr2))<dazzler_mem_size )
    {
      b[0] = DAZ_MEMBYTE | BUFFER2 | (((a-dazzler_mem_addr2) & 0x0700)/256);
      b[1] = a & 0xFF;
      b[2] = v;
      dazzler_send(b, 3);
    }
}


void dazzler_out_ctrl(byte v)
{
  byte b[3];
  b[0] = DAZ_CTRL;

  // client version 0 expects CTRL before FULLFRAME data
  if( dazzler_client_version<1 ) { b[1] = v; dazzler_send(b, 2); }

#if DEBUGLVL>0
  {
    static byte prev = 0xff;
    if( v!=prev ) {printf("dazzler_out_ctrl(%02x)\n", v); prev = v; }
  }
#endif

  if( dazzler_client_version<0 )
    {
      // client version not yet determined => send host version to client
      b[0] = DAZ_VERSION | (DAZZLER_HOST_VERSION&0x0F);
      dazzler_send(b, 1);

      // wait to receive version response from client
      // client response is received via interrupt in dazzler_receive()
      unsigned long t = millis();
      while( millis()-t < 10 && dazzler_client_version<0 ) host_check_interrupts();

      // if client does not respond within 10ms then it is either not present
      // or version 0 (which did not understand DAZ_VERSION and ignored it)
      if( dazzler_client_version<0 ) dazzler_client_version=0;
    }

  // D7: 1=enabled, 0=disabled
  // D6-D0: bits 15-9 of dazzler memory address
  uint16_t a = (v & 0x7f) << 9;
  if( (v & 0x80)==0 )
    {
      // dazzler is turned off
      dazzler_mem_addr1 = 0xFFFF;
      dazzler_mem_addr2 = 0xFFFF;
      v = 0x00;
    }
  else if( dazzler_client_version<1 )
    {
      // Dazzler client version 0 only has one buffer
      // (expects "v" value unchanged)
      if( a!=dazzler_mem_addr1 )
        {
          dazzler_send_frame(BUFFER1, dazzler_mem_addr1, a);
          dazzler_mem_addr1 = a;
        }
    }
  else if( a==dazzler_mem_addr1 )
    {
      // buffer for this address range is already initialized
      v = 0x80;
    }
  else if( a==dazzler_mem_addr2 )
    {
      // buffer for this address range is already initialized
      v = 0x81;
    }
  else if( dazzler_mem_addr1==0xFFFF )
    {
      // new address range, buffer 1 is available => use it
      dazzler_mem_addr1 = a;
      dazzler_send_fullframe(BUFFER1, dazzler_mem_addr1);
      v = 0x80;
    }
  else if( dazzler_mem_addr2==0xFFFF )
    {
      // new address range, buffer 2 is available => use it
      dazzler_mem_addr2 = a;
      dazzler_send_fullframe(BUFFER2, dazzler_mem_addr2);
      v = 0x81;
    }
  else 
    {
      // new address range, both buffers are in use => pick one to overwrite
      static bool first = true;
      if( first )
        {
          dazzler_send_frame(BUFFER1, dazzler_mem_addr1, a);
          dazzler_mem_addr1 = a;
          v = 0x80;
        }
      else
        {
          dazzler_send_frame(BUFFER2, dazzler_mem_addr2, a);
          dazzler_mem_addr2 = a;
          v = 0x81;
        }

      first = !first;
    }

  // determine rough address range (for memory writes)
  if( dazzler_mem_addr1==0xFFFF )
    {
      // no buffer occupied
      dazzler_mem_start = 0x0000;
      dazzler_mem_end   = 0x0000;
    }
  else if( dazzler_mem_addr2==0xFFFF )
    {
      // only buffer 1 occupied
      dazzler_mem_start = dazzler_mem_addr1;
      dazzler_mem_end   = dazzler_mem_start + dazzler_mem_size;
    }
  else
    {
      // both buffers occupied
      dazzler_mem_start = min(dazzler_mem_addr1, dazzler_mem_addr2);
      dazzler_mem_end   = max(dazzler_mem_addr1, dazzler_mem_addr2) + dazzler_mem_size;
    }

  // client version 1 and later can have CTRLPIC after FULLFRAME data
  // (avoids initial display of garbage memory)
  if( dazzler_client_version>=1 ) { b[1] = v; dazzler_send(b, 2); }
}


void dazzler_out_pict(byte v)
{
  // D7: not used
  // D6: 1=resolution x4 (single color), 0=normal resolution (multi-color)
  // D5: 1=2k memory, 0=512byte memory
  // D4: 1=color, 0=monochrome
  // D3-D0: color info for x4 high res mode

#if DEBUGLVL>0
  static byte prev = 0xff;
  if( v!=prev ) {printf("dazzler_out_pict(%02x)\n", v); prev = v; }
#endif

  byte b[2];
  b[0] = DAZ_CTRLPIC;
  b[1] = v;

  // client version 0 expects CTRLPIC before FULLFRAME data
  if( dazzler_client_version<1 ) dazzler_send(b, 2);

  uint16_t s = v & 0x20 ? 2048 : 512;
  if( s > dazzler_mem_size )
    {
      if( dazzler_mem_end>0 ) dazzler_mem_end += s-dazzler_mem_size;

      // switching to a bigger memory size => send memory again
      dazzler_mem_size = s;
      if( dazzler_mem_addr1!=0xFFFF )
        dazzler_send_fullframe(BUFFER1, dazzler_mem_addr1);
      if( dazzler_mem_addr2!=0xFFFF )
        dazzler_send_fullframe(BUFFER2, dazzler_mem_addr2);
    }
  else
    dazzler_mem_size = s;

  // client version 1 and later can have CTRLPIC after FULLFRAME data
  // (avoids initial display of garbage memory if memory size increased)
  if( dazzler_client_version>=1 ) dazzler_send(b, 2);
}


inline void set_d7a_port(byte p, byte v)
{
#if DEBUGLVL>0
  printf("set_d7a_port(%i, %02x)\n", 0030+p, v);
#endif
  d7a_port[p]=v;
}


void dazzler_receive(byte iface, byte data)
{
  static byte state=0;

#if DEBUGLVL>0
  Serial.print("dazzler_receive: "); Serial.println(data, HEX);
#endif

  switch( state )
    {
    case 0:
      {
        state = data & 0xf0;
        switch( state )
          {
          case DAZ_JOY1:
            set_d7a_port(0, (d7a_port[0] & 0xF0) | (data & 0x0F));
            break;
            
          case DAZ_JOY2:
            set_d7a_port(0, (d7a_port[0] & 0x0F) | ((data & 0x0F)*16));
            break;

          case DAZ_KEY:
            break;

          case DAZ_VERSION:
            dazzler_client_version = data & 0x0F;
            state = 0;
            break;

          case DAZ_VSYNC:
            dazzler_vsync_cycles = timer_get_cycles();
            state = 0;
            break;

          default:
            state = 0;
          }
        break;
      }

    case DAZ_JOY1:
      set_d7a_port(1, data);
      state++;
      break;

    case DAZ_JOY1+1:
      set_d7a_port(2, data);
      state = 0;
      break;

    case DAZ_JOY2:
      set_d7a_port(3, data);
      state++;
      break;

    case DAZ_JOY2+1:
      set_d7a_port(4, data);
      state = 0;
      break;

    case DAZ_KEY:
      int i = config_serial_map_sim_to_host(CSM_SIO);
      if( i<0xff ) serial_receive_host_data(i, data);
      state = 0;
      break;
    }
}


byte dazzler_in(byte port)
{
  byte v = 0;

  if( port==0016 )
    {
      // the values here are approximated and certainly not
      // synchronized with the actual picture on the client
      // we just provide them here so programs waiting for the
      // signals don't get stuck.
      // If I understand correctly, the Dazzler does not
      // interlace two fields and instead counts each field
      // (half-frame) as a full frame (of 262 lines). Therefore 
      // its frame rate is 29.97 * 2 = 59.94Hz, i.e. 16683us per frame,
      // i.e. 33367 cycles per frame.
      const uint32_t cycles_per_frame = 33367;
      const uint32_t cycles_per_line  = cycles_per_frame/262;

      // determine position within frame
      uint32_t c = timer_get_cycles() % cycles_per_frame;

      // bits 0-5 are unused
      v = 0xff;

      // bit 6: is low for 4ms (8000 cycles) between frames
      // (we pull it low at the beginning of a frame)
      // NOTE: bit 7 is also low during this period
      if( dazzler_client_version==0 )
        {
          // client version 0 does not report VSYNC => compute an aproximation
          if( c<8000 ) v &= ~0xC0;
        }
      else
        {
          // clients version 1 and later send VSYNC => keep bit 6 low
          // for 8000 cycles after we receive it
          if( timer_get_cycles()-dazzler_vsync_cycles < 8000 )
            v &= ~0xC0;
        }

      // bit 7: low for odd line, high for even line
      if( (c/cycles_per_line)&1 ) v &= ~0x80;
    }
  else if( port>=0030 && port<0035 )
    {
      // D+7A I/O board
      // The D+7A board was not part of the Dazzler but we include
      // it in the Dazzler emulation to support joysticks
      v = d7a_port[port-0030];
    }
  
#if DEBUGLVL>1
  printf("%04x: dazzler_in(%i)=%02x\n", regPC, port, v);
#endif

  return v;
}


void dazzler_set_iface(byte iface)
{
  static host_serial_receive_callback_tp fprev = NULL;

  if( iface != dazzler_iface )
    {
      // if we had an interface set, restore that interface's previous receive callback
      if( dazzler_iface<0xff ) host_serial_set_receive_callback(dazzler_iface, fprev);
      
      // set receive callback
      dazzler_iface = iface;
      dazzler_client_version = -1;
      fprev = host_serial_set_receive_callback(dazzler_iface, dazzler_receive);
      
#if DEBUGLVL>0
      if( iface==0xff ) Serial.println("Dazzler disabled"); else {Serial.print("Dazzler on interface:"); Serial.println(iface);}
#endif
    }
}


byte dazzler_get_iface()
{
  return dazzler_iface;
}


void dazzler_setup()
{
  dazzler_mem_size  = 2048;
  dazzler_mem_addr1 = 0xFFFF;
  dazzler_mem_addr2 = 0xFFFF;
  dazzler_mem_start = 0x0000;
  dazzler_mem_end   = 0x0000;
  for(int i=0; i<5; i++) d7a_port[i]=0xff;
  dazzler_client_version = -1;

  dazzler_set_iface(config_dazzler_interface());
}

#endif
