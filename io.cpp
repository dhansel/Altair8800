// -----------------------------------------------------------------------------
// Altair 8800 Simulator
// Copyright (C) 2020 David Hansel
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

#include "io.h"
#include "config.h"
#include "host.h"
#include "numsys.h"

//#define DEBUG

// Ports used by emulated devices (ports with "*" are available on Arduino MEGA)
// 00-01  * MITS 88-SIO
// 02     * printer (OkiData, C700 or generic)
// 03     * printer (OkiData, C700 or generic, output only)
// 04       Cromemco disk controller
// 04-05    ProTec VDM1 keyboard (input only)
// 06-07  * MITS 88-ACR
// 08-0A    MITS 88-DCDD disk controller
// 0E       Cromemco Dazzler
// 0F       Cromemco Dazzler (output only)
// 10-11  * MITS 88-2SIO port A
// 12-13  * MITS 88-2SIO port B
// 14-15  * second MITS 88-2SIO port A
// 16-17  * second MITS 88-2SIO port B
// 18       Cromemco D+7A board (Dazzler support, input only)
// 19-1C    Cromemco D+7A board (Dazzler support)
// 1D-1F    Cromemco D+7A board (Dazzler support, output only)
// 30-34    Cromemco disk controller
// 40       Cromemco disk controller (output only)
// A0-A8    MITS hard disk controller (4PIO interface board)
// C8       ProTec VDM-1 (output only)
// F0       Cromemco disk controller (input only)
// F8-FD    Tarbell disk controller
// FE     * MITS 88-VI interrupt controller (output only)
// FF     * front-panel sense switches (input only)

static byte io_unused_inp(byte port)
{
#if USE_IO_BUS>0
  // The I/O bus uses the pins connected to the D0-7 LEDs for input
  // (they get switched to input mode when INP LED is turned on).
  return host_read_data_bus();
#else
  return 0xff;
#endif
}


static void io_unused_out(byte port, byte data)
{
}

#if !defined(__AVR_ATmega2560__)

// On platforms other than the Arduine MEGA we use call tables to jump to
// the I/O functions of the different devices. A device calls io_register_port_*
// for each port it needs to use. This improves performance, especially if many
// devices are supported.

static IOFUN_INP portfun_inp[256];
static IOFUN_OUT portfun_out[256];


byte io_inp(byte port)
{
#if USE_IO_BUS>0
  // Wait while WAIT signal is asserted by an external device
  // before reading the actual data
  while( host_read_status_WAIT() );
#endif
  return portfun_inp[port](port);
}


void io_out(byte port, byte data)
{
  portfun_out[port](port, data);
#if USE_IO_BUS>0
  // Wait while WAIT signal is asserted by an external
  // device before continuing
  while( host_read_status_WAIT() );
#endif
}


void io_register_port_inp(byte port, IOFUN_INP f)
{
  if( f==NULL ) f = io_unused_inp;

  if( f != portfun_inp[port] )
    {
#ifdef DEBUG
      if( f==io_unused_inp ) Serial.print("UN-");
      Serial.print("registering INP port 0x"); 
      numsys_print_byte_hex(port); Serial.println();
#endif
      portfun_inp[port] = f;
    }
}


void io_register_port_out(byte port, IOFUN_OUT f)
{
  if( f==NULL ) f = io_unused_out;

  if( f != portfun_out[port] )
    {
#ifdef DEBUG
      if( f==io_unused_out ) Serial.print("UN-");
      Serial.print("registering OUT port 0x"); 
      numsys_print_byte_hex(port); Serial.println();
#endif
      portfun_out[port] = f;
    }
}


void io_print_registered_ports()
{
  int i;
  Serial.print(F("Registered ports for INP:"));
  for(i=0; i<256; i++)
    if( portfun_inp[i]!=io_unused_inp )
      { Serial.print(" "); numsys_print_byte_hex(i); }
  Serial.println();

  Serial.print(F("Registered ports for OUT:"));
  for(i=0; i<256; i++)
    if( portfun_out[i]!=io_unused_out )
      { Serial.print(" "); numsys_print_byte_hex(i); }
  Serial.println();
}


void io_setup()
{
  for(int i=0; i<256; i++)
    {
      portfun_inp[i] = io_unused_inp;
      portfun_out[i] = io_unused_out;
    }
}

#else // -------------------------------------------------------------------------------


// Arduino MEGA has only 6k of RAM and we want to use as much as possible
// for emulated RAM => do not waste 1024+ bytes to register ports for the
// few devices that are supported on the MEGA and instead call the functions
// directly

// defined in serial.cpp
byte serial_sio_in_ctrl(byte port);
byte serial_sio_in_data(byte port);
void serial_sio_out_ctrl(byte port, byte data);
void serial_sio_out_data(byte port, byte data);
byte serial_acr_in_ctrl(byte port);
byte serial_acr_in_data(byte port);
void serial_acr_out_ctrl(byte port, byte data);
void serial_acr_out_data(byte port, byte data);
byte serial_2sio_in_ctrl(byte port);
byte serial_2sio_in_data(byte port);
void serial_2sio_out_ctrl(byte port, byte data);
void serial_2sio_out_data(byte port, byte data);

// defined in printer.cpp
byte printer_in_ctrl(byte port);
byte printer_in_data(byte port);
void printer_out_ctrl(byte port, byte data);
void printer_out_data(byte port, byte data);

// defined in Altair8800.ino
void altair_vi_out_control(byte port, byte data);
byte altair_read_sense_switches(byte port);


byte io_inp(byte port)
{
  switch( port )
    {
    case 0x00: return serial_sio_in_ctrl(port); 
    case 0x01: return serial_sio_in_data(port); 
#if USE_PRINTER>0
    case 0x02: return printer_in_ctrl(port);
    case 0x03: return printer_in_data(port);
#endif
    case 0x06: return serial_acr_in_ctrl(port);
    case 0x07: return serial_acr_in_data(port); 
    case 0x10:
    case 0x12: return serial_2sio_in_ctrl(port);
    case 0x11:
    case 0x13: return serial_2sio_in_data(port); 
#if USE_SECOND_2SIO>0
    case 0x14:
    case 0x16: return serial_2sio_in_ctrl(port);
    case 0x15:
    case 0x17: return serial_2sio_in_data(port); 
#endif      
    case 0xff: return altair_read_sense_switches(port);
    default:   return io_unused_inp(port);
    }
}


void io_out(byte port, byte data)
{
  switch( port )
    {
    case 0x00: serial_sio_out_ctrl(port, data); break;
    case 0x01: serial_sio_out_data(port, data); break;
#if USE_PRINTER>0
    case 0x02: printer_out_ctrl(port, data); break;
    case 0x03: printer_out_data(port, data); break;
#endif
    case 0x06: serial_acr_out_ctrl(port, data); break;
    case 0x07: serial_acr_out_data(port, data); break;
    case 0x10:
    case 0x12: serial_2sio_out_ctrl(port, data); break;
    case 0x11:
    case 0x13: serial_2sio_out_data(port, data); break;
#if USE_SECOND_2SIO>0
    case 0x14:
    case 0x16: serial_2sio_out_ctrl(port, data); break;
    case 0x15:
    case 0x17: serial_2sio_out_data(port, data); break;
#endif
    case 0xFE: altair_vi_out_control(port, data); break;
    default: io_unused_out(port, data);
    }
}


// not used for MEGA
void io_register_port_inp(byte port, IOFUN_INP f) {}
void io_register_port_out(byte port, IOFUN_OUT f) {}
void io_print_registered_ports() {}
void io_setup() {}

#endif
