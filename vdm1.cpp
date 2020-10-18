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

#include "vdm1.h"
#include "mem.h"
#include "cpucore.h"
#include "host.h"
#include "serial.h"
#include "timer.h"
#include "io.h"
#include "cdrive.h"

#if USE_VDM1==0

void vdm1_out(byte v) {}
void vdm1_set_iface(byte iface) {}
void vdm1_register_ports() {}
void vdm1_setup() {}
byte vdm1_get_iface() { return 0xff; }

#else

#define VDM_MEMBYTE   0x10
#define VDM_FULLFRAME 0x20
#define VDM_CTRL      0x30
#define VDM_DIP       0x40

#define VDM_CONNECT   0x10
#define VDM_KEY       0x30

#define DEBUGLVL 0

static byte vdm_iface = 0xff;
static int  vdm_connected = 0;
static byte vdm_ctrl = 0x00;
static byte vdm_dip = 0;
static byte vdm_keyboard_ctrl = 0xFF;
static byte vdm_keyboard_data = 0xFF;
uint16_t vdm1_mem_start, vdm1_mem_end;

static void vdm1_send_dip();
static void vdm1_send_ctrl();
static void vdm1_send_fullframe();


static void vdm1_connect()
{
  vdm_connected = 1;
  vdm_keyboard_ctrl = 1;
  vdm1_send_dip();
  vdm1_send_ctrl();
  vdm1_send_fullframe();
}


static void vdm1_send(const byte *data, uint16_t size)
{
  if( vdm_iface<0xff && vdm_connected!=0 )
    {
#if DEBUGLVL>1
      printf("VDM sending: ");
      for(int i=0; i<size; i++) printf("%02X ", data[i]);
      printf("\n");
#endif

      if( vdm_connected<0 )
        {
          // if the VDM-1 client has connected but has not been initialized
          // yet then do that before sending the data
          vdm1_connect();
          delay(100);
        }
      
      size_t n, ptr = 0;
      while( size>0 )
        {
          n = host_serial_write(vdm_iface, ((const char *) data)+ptr, size);
          ptr  += n;
          size -= (uint16_t) n;
        }
    }
}


static void vdm1_send_dip()
{
  byte b[2];
  b[0] = VDM_DIP;
  b[1] = vdm_dip;
  vdm1_send(b, 2);
}


static void vdm1_send_ctrl()
{
  byte b[2];
  b[0] = VDM_CTRL;
  b[1] = vdm_ctrl;
  vdm1_send(b, 2);
}


static void vdm1_send_fullframe()
{
  // send full picture memory
  byte b = VDM_FULLFRAME;
  host_serial_write(vdm_iface, b);
  vdm1_send(Mem+vdm1_mem_start, 1024);
}


void vdm1_write_mem_(uint16_t a, byte v)
{
  // when vdm1 is off, vdm1_mem_end=0 so we never
  // get here (due to condition in vdm1_write_mem macro in vdm1.h)

#if DEBUGLVL>0
  printf("vdm1_write_mem(%04x, %02x)\n", a, v);
#endif

  if( Mem[a] != v )
    {
      a -= vdm1_mem_start;
      byte b[3];
      b[0] = VDM_MEMBYTE | ((a & 0x0300)/256) ;
      b[1] = a & 255;
      b[2] = v;
      vdm1_send(b, 3);
    }
}


void vdm1_out(byte port, byte v)
{
#if DEBUGLVL>0
  printf("vdm1_out(%02x)\n", v);
#endif

  if( v != vdm_ctrl )
    {
      vdm_ctrl = v;
      vdm1_send_ctrl();
    }
}


void vdm1_set_dip(byte v)
{
#if DEBUGLVL>0
  printf("vdm1_set_dip(%02x)\n", v);
#endif
  vdm_dip = v;
  vdm1_send_dip();
}


void vdm1_set_address(uint16_t a)
{
  vdm1_mem_start = a;
  vdm1_mem_end   = vdm1_mem_start + 1024;
  vdm1_send_fullframe();
}


void vdm1_receive(byte iface, byte data)
{
  static byte state=0;

#if DEBUGLVL>0
  Serial.print("vdm1_receive: "); Serial.println(data, HEX);
#endif

  switch( state )
    {
    case 0:
      {
        state = data & 0xf0;
        switch( state )
          {
          case VDM_KEY:
            break;

          case VDM_CONNECT:
            vdm_connected = -1;

            // start a timer that will finish the initialization sequence
            // by sending the current information to the VDM1 client.
            // note that this timer will only run when the Altair is in "run" mode.
            // we can not finish the sequence right here because vdm1_receive is
            // called from within the host's receive interrupt.
            timer_start(TIMER_VDM1, 1000);

            state = 0;
            break;

          default:
            state = 0;
          }
        break;
      }

    case VDM_KEY:
      {
        vdm_keyboard_ctrl = 0;
        vdm_keyboard_data = data;

        byte dev = config_vdm1_keyboard_device();
        if( dev<0xff ) serial_receive_data(dev, data);

        state = 0;
        break;
      }
    }
}


byte vdm1_keyboard_in_ctrl(byte port)
{
  // control port (channel A) of a Protec 3P+S card jumpered such
  // that the latched XDA signal (data available) is available at
  // bit 0 (active low). Output will be 0 if keyboard input from 
  // the VDM-1 client is available, otherwise 1.

  // The 3P+S was not part of the VDM-1 but the parallel port keyboard
  // emulation is included here since it was often used together
  // with the VDM-1.
  return vdm_keyboard_ctrl;
}


byte vdm1_keyboard_in_data(byte port)
{
  // data port (channel B) of a Protec 3+S card. Keyboard data from 
  // the VDM-1 client will be available at this port.

  // The 3P+S was not part of the VDM-1 but the parallel port keyboard
  // emulation is included here since it was often used together
  // with the VDM-1.
  vdm_keyboard_ctrl = 1;
  return vdm_keyboard_data;
}


void vdm1_register_ports()
{
  bool mapped = config_vdm1_interface()!=0xff;
  io_register_port_out(0xC8, mapped ? vdm1_out : NULL);

  mapped = config_vdm1_keyboard_device()!=0xff;
  io_register_port_inp(0x04, mapped ? vdm1_keyboard_in_ctrl : NULL);
  io_register_port_inp(0x05, mapped ? vdm1_keyboard_in_data : NULL);
  
#if NUM_CDRIVES>0
  // both VDM1 keyboard support and Cromemco disk controller use port 4.  
  // We give the VDM1 keyboard priority to use that port but when the VDM1 
  // keyboard gets disabled we re-initialize the Cromemco controller so 
  // it can grab the port.
  if( !mapped ) cdrive_register_ports();
#endif
}


void vdm1_set_iface(byte iface)
{
  static host_serial_receive_callback_tp fprev = NULL;

  if( iface != vdm_iface )
    {
      // if we had an interface set, restore that interface's previous receive callback
      if( vdm_iface<0xff ) host_serial_set_receive_callback(vdm_iface, fprev);
      
      // set receive callback
      vdm_iface = iface;
      fprev = host_serial_set_receive_callback(vdm_iface, vdm1_receive);
      
#if DEBUGLVL>0
      if( iface==0xff ) 
        Serial.println("VDM-1 disabled"); 
      else 
        {Serial.print("VDM-1 on interface: "); Serial.println(host_serial_port_name(iface));}
      delay(500);
#endif

      vdm1_register_ports();
    }
}


byte vdm1_get_iface()
{
  return vdm_iface;
}


static void vdm1_timer()
{
  if( vdm_connected<0 ) vdm1_connect();
}


void vdm1_setup()
{
  vdm1_set_dip(config_vdm1_dip());
  vdm1_set_address(config_vdm1_address());
  vdm1_set_iface(config_vdm1_interface());
  timer_setup(TIMER_VDM1, 0, vdm1_timer);
  vdm_connected = 0;
  vdm_keyboard_ctrl = 0xFF;
  vdm_keyboard_data = 0xFF;
}

#endif
